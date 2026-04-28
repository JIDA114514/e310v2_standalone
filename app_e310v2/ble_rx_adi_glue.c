#include "ble_rx_adi_glue.h"

#include <stdbool.h>

#include "axi_dmac.h"
#include "app_config.h"

#ifdef XILINX_PLATFORM
#include <xil_cache.h>
#endif

int ble_adi_dma_capture(void *dma_ctx, void *dst, uint32_t bytes,
                        uint32_t timeout_ms)
{
    uint32_t status;
    struct axi_dma_transfer transfer;
    ble_adi_dma_ctx_t *ctx = (ble_adi_dma_ctx_t *)dma_ctx;

    if (!ctx || !ctx->rx_dmac || !dst || bytes == 0u)
        return -1;

    transfer.size = bytes;
    transfer.transfer_done = false;
    transfer.cyclic = NO;
    transfer.src_addr = 0u;
    transfer.dest_addr = (uint32_t)(uintptr_t)dst;

    status = axi_dmac_transfer_start(ctx->rx_dmac, &transfer);
    if (status < 0)
        return (int)status;

    status = axi_dmac_transfer_wait_completion(ctx->rx_dmac, timeout_ms);
    if (status < 0)
        return (int)status;

    return 0;
}

void ble_adi_cache_invalidate(uintptr_t addr, uint32_t bytes)
{
#ifdef XILINX_PLATFORM
    Xil_DCacheInvalidateRange(addr, bytes);
#else
    (void)addr;
    (void)bytes;
#endif
}
