/*
 * Copyright (C) 2020-2021, HENSOLDT Cyber GmbH
 * SPDX-License-Identifier: GPL-2.0-only
 */

/*
 * Copyright 2019, Data61
 * Commonwealth Scientific and Industrial Research Organisation (CSIRO)
 * ABN 41 687 119 230.
 *
 * This software may be distributed and modified according to the terms of
 * the GNU General Public License version 2. Note that NO WARRANTY is provided.
 * See "LICENSE_GPLv2.txt" for details.
 *
 * @TAG(DATA61_GPL)
 */

#include "OS_Dataport.h"

#include <stdbool.h>
#include <string.h>

#include <camkes.h>
#include <camkes/dma.h>
#include <camkes/io.h>
#include <camkes/irq.h>

#include <platsupport/io.h>
#include <platsupport/irq.h>
#include <ethdrivers/raw.h>
#include <ethdrivers/helpers.h>
#include <ethdrivers/plat/eth_plat.h>
#include <sel4utils/sel4_zf_logif.h>

#define RX_BUFS          256
#define CLIENT_RX_BUFS   128
#define CLIENT_TX_BUFS   128
#define DMA_BUF_SIZE    2048

typedef struct
{
    dma_addr_t* dma;
    int len;
} rx_frame_t; // Clients share a pool of RX frames

typedef struct
{
    dma_addr_t dma;
    int len;
} tx_frame_t; // Each client has a pool of TX frames

typedef struct
{
    /* keeps track of the head of the queue */
    unsigned int pending_rx_head;
    /* keeps track of the tail of the queue */
    unsigned int pending_rx_tail;
    /*
     * this is a cyclic queue of RX buffers pending to be read by a client,
     * the head represents the first buffer in the queue, and the tail the last
     */
    rx_frame_t pending_rx[CLIENT_RX_BUFS];

    /* keeps track of how many TX buffers are in use */
    unsigned int num_tx;
    /* the allocated TX buffers for the client */
    tx_frame_t tx_mem[CLIENT_TX_BUFS];
    /*
     * this represents the pool of buffers that can be used for TX,
     * this array is a sliding array in that num_tx acts a pointer to
     * separate between buffers that are in use and buffers that are
     * not in use. E.g. 'o' = free, 'x' = in use
     *  -------------------------------------
     *  | o | o | o | o | o | o | x | x | x |
     *  -------------------------------------
     *                          ^
     *                        num_tx
     */
    tx_frame_t* pending_tx[CLIENT_TX_BUFS];

    /* mac address for this client */
    uint8_t mac[6];
} client_t;


typedef struct
{
// The marker stating wether the initialization was successful must be
// flagged volatile when strictly following the C rules. A hypothetical highly
// advanced optimizer could turn global variable accesses into constants, if it
// concludes the global state is always well known. Also, there is no rule in C
// that global variables must be synced with memory content on function entry
// and exit - it is just something that happen due to practical reasons. There
// is not even a rule that functions must be preserved and can't be inlined,
// which would eventually allow caching global variables easily. Furthermore, C
// also does not know threads nor concurrent execution of functions, but both
// have a string impact on global variables.
// Using volatile here guarantees at least, that accesses to global variables
// are accessing the actual memory in the given order stated in the program and
// there is no caching or constant folding that removes memory accesses. That is
// the best we can do to avoid surprises at higher optimization levels.
    volatile bool done_init;
    struct eth_driver* eth_driver;
    unsigned int num_rx_bufs;
    dma_addr_t rx_bufs[RX_BUFS];
    dma_addr_t* rx_buf_pool[RX_BUFS];
    client_t client;
} zynq_nic_ctx_t;


//------------------------------------------------------------------------------
// global variables
static zynq_nic_ctx_t zynq_nic_ctx;

//------------------------------------------------------------------------------
// add DMA memory to RX pool
static void add_to_rx_buf_pool(
    zynq_nic_ctx_t* nic_ctx,
    dma_addr_t* dma)
{
    assert(nic_ctx);
    assert(dma);

    unsigned int idx = nic_ctx->num_rx_bufs;
    dma_addr_t* dma_slot = &nic_ctx->rx_bufs[idx];
    *dma_slot = *dma;
    nic_ctx->rx_buf_pool[idx] = dma_slot;
    (nic_ctx->num_rx_bufs)++;
}


//------------------------------------------------------------------------------
// add DMA memory from TX pool
static void add_to_client_tx_buf_pool(
    client_t* client,
    dma_addr_t* dma)
{
    assert(client);
    assert(dma);

    unsigned int idx = client->num_tx;
    tx_frame_t* tx_frame = &client->tx_mem[idx];
    tx_frame->dma = *dma;
    tx_frame->len = DMA_BUF_SIZE;
    client->pending_tx[idx] = tx_frame;
    (client->num_tx)++;
}


//------------------------------------------------------------------------------
// get DMA memory from RX pool
static dma_addr_t* get_from_rx_buf_pool(
    zynq_nic_ctx_t* nic_ctx)
{
    assert(nic_ctx);

    if (0 == nic_ctx->num_rx_bufs)
    {
        LOG_ERROR("Invalid number of buffers");
        return NULL;
    }

    (nic_ctx->num_rx_bufs)--;
    return nic_ctx->rx_buf_pool[nic_ctx->num_rx_bufs];
}


//------------------------------------------------------------------------------
// return DMA memory to RX pool
static void return_to_rx_buf_pool(
    zynq_nic_ctx_t* nic_ctx,
    dma_addr_t* dma)
{
    assert(nic_ctx);
    assert(dma);

    nic_ctx->rx_buf_pool[nic_ctx->num_rx_bufs] = dma;
    (nic_ctx->num_rx_bufs)++;
}


//------------------------------------------------------------------------------
static void cb_eth_tx_complete(
    void* cb_cookie,
    void* cookie)
{
    assert(cb_cookie);
    zynq_nic_ctx_t* nic_ctx = (zynq_nic_ctx_t*)cb_cookie;
    assert(&zynq_nic_ctx == nic_ctx);

    client_t* client = &(nic_ctx->client);

    client->pending_tx[client->num_tx] = (tx_frame_t*)cookie;
    (client->num_tx)++;
}


//------------------------------------------------------------------------------
static uintptr_t cb_eth_allocate_rx_buf(
    void* cb_cookie,
    size_t buf_size,
    void** cookie)
{
    assert(cb_cookie);
    zynq_nic_ctx_t* nic_ctx = (zynq_nic_ctx_t*)cb_cookie;
    assert(&zynq_nic_ctx == nic_ctx);

    if (buf_size > DMA_BUF_SIZE)
    {
        LOG_ERROR("Requested size doesn't fit in buffer");
        return 0;
    }

    dma_addr_t* dma = get_from_rx_buf_pool(nic_ctx);
    if (!dma)
    {
        LOG_ERROR("DMA pool empty");
        return 0;
    }

    *cookie = dma;
    return dma->phys;
}


//------------------------------------------------------------------------------
static void cb_eth_rx_complete(
    void* cb_cookie,
    unsigned int num_bufs,
    void** cookies,
    unsigned int* lens)
{
    assert(cb_cookie);
    zynq_nic_ctx_t* nic_ctx = (zynq_nic_ctx_t*)cb_cookie;
    assert(&zynq_nic_ctx == nic_ctx);

    client_t* client = &(nic_ctx->client);

    if (num_bufs != 1)
    {
        LOG_ERROR("Trying to write %d buffers, can only do one", num_bufs);
    }
    else if (((client->pending_rx_head + 1) % CLIENT_RX_BUFS)
             == client->pending_rx_tail)
    {
        LOG_ERROR("RX buffer overflow");
    }
    else
    {
        rx_frame_t* rx_frame = &client->pending_rx[client->pending_rx_head];

        // Cookie points to one of the dma pool elements (rx_bufs[] and
        // rx_buf_pool[]), and we store this pointer in the rx_frame so that the
        // return_to_rx_buf_pool() can return it back to the dma pool once the
        // data is processed in the client_rx_data(). After the dma structure is
        // back on the pool it can be reused as another cookie.
        //
        // See return_to_rx_buf_pool() and get_from_rx_buf_pool() for more
        // details.
        rx_frame->dma        = (dma_addr_t*)(cookies[0]);
        rx_frame->len        = lens[0];

        client->pending_rx_head =
            (client->pending_rx_head + 1) % CLIENT_RX_BUFS;

        nic_event_hasData_emit();

        return;
    }

    /* abort and put all the bufs back */
    for (unsigned int i = 0; i < num_bufs; i++)
    {
        dma_addr_t* dma = (dma_addr_t*)(cookies[i]);
        return_to_rx_buf_pool(nic_ctx, dma);
    }
}


//------------------------------------------------------------------------------
/** If eth frames have been received by the driver, copy a single frame from
 * the driver's buffer (rx_bufs), into the dataport of the caller of this
 * function.
 *
 * @param[out] len The size in bytes of the eth frame.
 * @param[out] framesRemaining Flag indicating if further frames are available
 * to be read.
 * @return     OS_SUCCESS   A frame was placed in the dataport. len and
 *             framesRemaining are updated accordingly.
 *             OS_ERROR_NOT_INITIALIZED The device hasn't finished initializing.
 *             The call should be retried.
 *             OS_ERROR_NO_DATA No data is available to be read.
 */
OS_Error_t nic_rpc_rx_data(
    size_t* pLen,
    size_t* framesRemaining)
{
    if (!zynq_nic_ctx.done_init)
    {
        LOG_ERROR("Device not initialized");
        return OS_ERROR_NOT_INITIALIZED;
    }

    client_t* client = &zynq_nic_ctx.client;
    if (client->pending_rx_head == client->pending_rx_tail)
    {
        // Ideally, the network stack does not poll the driver and we end up
        // here only in very few cases. Practically, we see this message a lot
        // and this pollutes the logs. This needs further investigation, until
        // then we don't print anything here.
        //   LOG_INFO("no RX data, client should wait for notification");
        return OS_ERROR_NO_DATA;
    }

    rx_frame_t* rx = &client->pending_rx[client->pending_rx_tail];

    /* ToDo: Instead of copying the DMA buffer into the shared dataport memory,
     *       we should share the ring buffer elements with the network stack to
     *       use a zero-copy approach.
     */
    memcpy(nic_to_port, rx->dma->virt, rx->len);
    *pLen = rx->len;

    client->pending_rx_tail = (client->pending_rx_tail + 1) % CLIENT_RX_BUFS;
    if (client->pending_rx_tail == client->pending_rx_head)
    {
        *framesRemaining = 0;
    }
    else
    {
        *framesRemaining = 1;
    }

    return_to_rx_buf_pool(&zynq_nic_ctx, rx->dma);

    return OS_SUCCESS;
}


//------------------------------------------------------------------------------
/**
 * @param[in] len The size in bytes of the eth frame
 * @return OS_SUCCESS A frame has been enqueued to be send*
 *         OS_ERROR_NOT_INITIALIZED The device hasn't finished initializing.
 *         The call should be retried.
 *         OS_ERROR_INVALID_PARAMETER The length requested is invalid.
 *         OS_ERROR_TRY_AGAIN Frame couldn't be enqueued and has to be sent
 *                              again.
 */
OS_Error_t nic_rpc_tx_data(size_t* pLen)
{
    if (!zynq_nic_ctx.done_init)
    {
        LOG_ERROR("Device not initialized");
        return OS_ERROR_NOT_INITIALIZED;
    }

    size_t len = *pLen;
    // packet must at least contain dest MAC and source MAC
    if (len < 12)
    {
        ZF_LOGW("invalid packet size %zu", len);
        return OS_ERROR_INVALID_PARAMETER;
    }

    if (len > DMA_BUF_SIZE)
    {
        ZF_LOGW(
            "truncate packet size %zu to max supported %d",
            len,
            DMA_BUF_SIZE);

        len = DMA_BUF_SIZE;
    }

    struct eth_driver* eth_driver = zynq_nic_ctx.eth_driver;
    client_t* client = &zynq_nic_ctx.client;

    /* drop packet if TX queue is full */
    if (0 == client->num_tx)
    {
        LOG_ERROR("TX queue is full");
        return OS_ERROR_TRY_AGAIN;
    }

    (client->num_tx)--;
    tx_frame_t* tx_buf = client->pending_tx[client->num_tx];

    /* copy the packet over */
    memcpy(tx_buf->dma.virt, nic_from_port, len);

    /* set source MAC */
    memcpy(
        &((char*)tx_buf->dma.virt)[6],
        client->mac,
        sizeof(client->mac));

    /* queue up transmit */

    int err = eth_driver->i_fn.raw_tx(
                  eth_driver,
                  1,
                  (uintptr_t*) & (tx_buf->dma.phys),
                  (unsigned int*)&len,
                  tx_buf);

    if (ETHIF_TX_ENQUEUED != err)
    {
        /* TX failed, free internal TX buffer. Client my retry transmission */
        LOG_ERROR("Failed to enqueue tx packet, code %d", err);
        (client->num_tx)++;
        return OS_ERROR_TRY_AGAIN;
    }

    return OS_SUCCESS;
}


//------------------------------------------------------------------------------
OS_Error_t nic_rpc_get_mac_address(void)
{
    client_t* client = &zynq_nic_ctx.client;
    memcpy((uint8_t*)nic_to_port, client->mac, sizeof(client->mac));

    return OS_SUCCESS;
}


//------------------------------------------------------------------------------
static int cb_eth_interface_found(
    void*  cookie,
    void*  interface_instance,
    char** properties)
{
    assert(cookie);
    assert(interface_instance);

    zynq_nic_ctx_t* nic_ctx = (zynq_nic_ctx_t*)cookie;
    assert(&zynq_nic_ctx == nic_ctx);
    struct eth_driver* eth_driver = interface_instance;

    /* remember the instance */
    nic_ctx->eth_driver = eth_driver;

    return PS_INTERFACE_FOUND_MATCH;
}


//------------------------------------------------------------------------------
// Module initialization
//------------------------------------------------------------------------------

// We registered this function via the macro CAMKES_POST_INIT_MODULE_DEFINE(),
// but actually it's called as the last thing in the CAmkES pre_init() handler
// implemented by seL4SingleThreadedComponent.template.c function. This means
// we cannot do much interaction with other components here.
int server_init(
    ps_io_ops_t* io_ops)
{
    LOG_ERROR("Called server_init() in ethdriver.c");
    /* this calls cb_eth_interface_found() with the interface instance */
    int error = ps_interface_find(
                    &io_ops->interface_registration_ops,
                    PS_ETHERNET_INTERFACE,
                    cb_eth_interface_found,
                    &zynq_nic_ctx);

    if (error)
    {
        LOG_ERROR("Unable to find an ethernet device, code %d", error);
        return -1;
    }

    /* cb_eth_interface_found() has set this up */
    struct eth_driver* eth_driver = zynq_nic_ctx.eth_driver;
    assert(eth_driver);
    /* cb_cookie is passed to each of the callbacks below */
    eth_driver->cb_cookie = &zynq_nic_ctx;

    static const struct raw_iface_callbacks ethdriver_callbacks =
    {
        .tx_complete = cb_eth_tx_complete,
        .rx_complete = cb_eth_rx_complete,
        .allocate_rx_buf = cb_eth_allocate_rx_buf
    };
    eth_driver->i_cb = ethdriver_callbacks;

    client_t* client = &zynq_nic_ctx.client;

    /* preallocate buffers */
    LOG_INFO("allocate RX DMA buffers: %u x %zu (=%zu) byte",
             RX_BUFS, DMA_BUF_SIZE, RX_BUFS * DMA_BUF_SIZE);
    for (unsigned int i = 0; i < RX_BUFS; i++)
    {
        /* Note that the parameters "cached" and "alignment" of this helper
         * function are in the opposite order than in ps_dma_alloc()
         */
        dma_addr_t dma = dma_alloc_pin(
                             &(io_ops->dma_manager),
                             DMA_BUF_SIZE,
                             0, // uncached
                             4); // alignment
        if (!dma.phys)
        {
            LOG_ERROR("Failed to allocate DMA of size %zu for RX buffer #%d ",
                      DMA_BUF_SIZE, i);
            return -1;
        }
        memset(dma.virt, 0, DMA_BUF_SIZE);
        add_to_rx_buf_pool(&zynq_nic_ctx, &dma);
    }

    LOG_INFO("allocate TX DMA buffers: %u x %zu (=%zu) byte",
             CLIENT_TX_BUFS, DMA_BUF_SIZE, CLIENT_TX_BUFS * DMA_BUF_SIZE);
    for (unsigned int i = 0; i < CLIENT_TX_BUFS; i++)
    {
        /* Note that the parameters "cached" and "alignment" of this helper
         * function are in the opposite order than in ps_dma_alloc()
         */
        dma_addr_t dma = dma_alloc_pin(
                             &(io_ops->dma_manager),
                             DMA_BUF_SIZE,
                             0, // uncached
                             4); // alignment
        if (!dma.phys)
        {
            LOG_ERROR("Failed to allocate DMA of size %zu for TX buffer #%d ",
                      DMA_BUF_SIZE, i);
            return -1;
        }
        memset(dma.virt, 0, DMA_BUF_SIZE);
        add_to_client_tx_buf_pool(client, &dma);
    }

    /* get MAC from hardware and remember it */
    uint8_t hw_mac[6];
    eth_driver->i_fn.get_mac(eth_driver, hw_mac);
    memcpy(client->mac, hw_mac, sizeof(client->mac));

    eth_driver->i_fn.raw_poll(eth_driver);

    zynq_nic_ctx.done_init = true;

    return 0;
}


//------------------------------------------------------------------------------
// this is called when the CAmkES component starts
int do_env_init(
    ps_io_ops_t* io_ops)
{
    memset(&zynq_nic_ctx, 0, sizeof(zynq_nic_ctx));

    return 0;
}


//------------------------------------------------------------------------------
CAMKES_ENV_INIT_MODULE_DEFINE(ethdriver_do_env_init, do_env_init)
CAMKES_POST_INIT_MODULE_DEFINE(ethdriver_run, server_init);
