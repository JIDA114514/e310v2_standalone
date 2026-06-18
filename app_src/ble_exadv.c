#include <stdint.h>
#include <stdio.h>

#include "ad9361_api.h"
#include "axi_dac_core.h"
#include "axi_dmac.h"
#include "ble_exadv.h"
#include "no_os_delay.h"
#include "xtime_l.h"

#include "../../../../../../python/std_ble/ble_exadv_waveform_30_72M.h"

#ifdef XILINX_PLATFORM
extern void Xil_DCacheFlush(void);
extern void Xil_DCacheFlushRange(uintptr_t adr, uint32_t len);
#endif

#define BLE_EXADV_WORD_RATE_HZ (61440000ULL)
#define BLE_EXADV_SPIN_WAIT_US (50u)
#define BLE_EXADV_LO_SWITCH_GUARD_US (20u)
#define BLE_EXADV_MIN_AUX_FRAME_SPACE_US (300u)
#define BLE_EXADV_LO_SWITCH_EST_US (700u)
#define BLE_EXADV_SECONDARY_START_LEAD_US (0u)

#ifndef BLE_EXADV_TIMING_DEBUG
#define BLE_EXADV_TIMING_DEBUG (0)
#endif

extern struct ad9361_rf_phy *ad9361_phy;
extern struct axi_dmac *tx_dmac;

static uint8_t ble_exadv_active = 0u;
static uint32_t ble_exadv_aux_offset_us = BLE_EXADV_AUX_OFFSET_US;
static uint32_t ble_exadv_interval_us = BLE_EXADV_INTERVAL_US;
static uint32_t ble_exadv_primary_total_us = 0u;
static uint32_t ble_exadv_primary_pad_us = 0u;
static uint32_t ble_exadv_secondary_total_us = 0u;
static uint32_t ble_exadv_secondary_start_lead_us = 0u;
static XTime ble_exadv_next_event = 0;

static XTime ble_exadv_ticks_from_us(uint32_t us)
{
    return (XTime)(((uint64_t)COUNTS_PER_SECOND * us + 999999ULL) / 1000000ULL);
}

static uint32_t ble_exadv_words_to_us(uint32_t words)
{
    return (uint32_t)(((uint64_t)words * 1000000ULL + BLE_EXADV_WORD_RATE_HZ - 1ULL) /
                      BLE_EXADV_WORD_RATE_HZ);
}

static XTime ble_exadv_time_after(XTime start, uint32_t delay_us)
{
    return start + ble_exadv_ticks_from_us(delay_us);
}

static XTime ble_exadv_time_before(XTime start, uint32_t advance_us)
{
    XTime ticks = ble_exadv_ticks_from_us(advance_us);

    return start > ticks ? start - ticks : 0;
}

static uint32_t ble_exadv_ticks_to_us(XTime ticks)
{
    uint64_t us = ((uint64_t)ticks * 1000000ULL + COUNTS_PER_SECOND - 1ULL) /
                  COUNTS_PER_SECOND;

    if (us > 0xFFFFFFFFULL)
        us = 0xFFFFFFFFULL;
    return (uint32_t)us;
}

#if BLE_EXADV_TIMING_DEBUG
static uint32_t ble_exadv_elapsed_us(XTime start, XTime end)
{
    return end > start ? ble_exadv_ticks_to_us(end - start) : 0u;
}
#endif

static uint32_t ble_exadv_late_us(XTime deadline)
{
    XTime now;

    XTime_GetTime(&now);
    if (now <= deadline)
        return 0u;
    return ble_exadv_ticks_to_us(now - deadline);
}

static uint32_t ble_exadv_udelay_until(XTime deadline)
{
    XTime now;
    XTime spin_ticks = ble_exadv_ticks_from_us(BLE_EXADV_SPIN_WAIT_US);

    XTime_GetTime(&now);
    if (now >= deadline)
        return ble_exadv_late_us(deadline);

    if ((deadline - now) > spin_ticks) {
        XTime spin_start = deadline - spin_ticks;
        uint32_t delay_us = ble_exadv_ticks_to_us(spin_start - now);

        if (delay_us > 0u)
            no_os_udelay(delay_us);
    }

    do {
        XTime_GetTime(&now);
    } while (now < deadline);

    return ble_exadv_late_us(deadline);
}

static uint32_t ble_exadv_min_interval_us(void)
{
    return ble_exadv_aux_offset_us + ble_exadv_secondary_total_us +
           ble_exadv_primary_pad_us;
}

static int ble_exadv_prepare_tx_path(void)
{
    int ret = 0;

    ret |= axi_dac_set_datasel(ad9361_phy->tx_dac, -1, AXI_DAC_DATA_SEL_DMA);
    ret |= no_os_gpio_set_value(ad9361_phy->gpio_desc_tx1_ctrl_h, 0);
    ret |= no_os_gpio_set_value(ad9361_phy->gpio_desc_tx1_ctrl_l, 1);
    ret |= no_os_gpio_set_value(ad9361_phy->gpio_desc_tx2_ctrl_h, 0);
    ret |= no_os_gpio_set_value(ad9361_phy->gpio_desc_tx2_ctrl_l, 1);
    ret |= ad9361_set_tx_rf_port_output(ad9361_phy, TXB);

    return ret < 0 ? -1 : 0;
}

static void ble_exadv_flush_iq(void)
{
#ifdef XILINX_PLATFORM
    Xil_DCacheFlush();
    Xil_DCacheFlushRange((uintptr_t)ble_exadv_primary_iq_ch37,
                         BLE_EXADV_PRIMARY_WORDS * sizeof(uint32_t));
    Xil_DCacheFlushRange((uintptr_t)ble_exadv_secondary_iq_ch3,
                         BLE_EXADV_SECONDARY_WORDS * sizeof(uint32_t));
#endif
}

static int ble_exadv_start_dma(const uint32_t *iq_words, uint32_t word_count)
{
    struct axi_dma_transfer transfer;

    transfer.size = word_count * sizeof(uint32_t);
    transfer.transfer_done = 0;
    transfer.cyclic = NO;
    transfer.src_addr = (uintptr_t)iq_words;
    transfer.dest_addr = 0;

    axi_dac_write(ad9361_phy->tx_dac, AXI_DAC_REG_SYNC_CONTROL, AXI_DAC_SYNC);
    return axi_dmac_transfer_start(tx_dmac, &transfer);
}

static int ble_exadv_set_primary_lo(void)
{
    int ret = ad9361_set_tx_lo_freq(ad9361_phy, BLE_EXADV_PRIMARY_FREQ_HZ);

    if (ret < 0)
        printf("ble exadv primary lo tune failed\n");
    else
        no_os_mdelay(1);

    return ret;
}

static int ble_exadv_set_secondary_lo(void)
{
#if BLE_EXADV_SECONDARY_FREQ_HZ == BLE_EXADV_PRIMARY_FREQ_HZ
    return 0;
#else
    int ret = ad9361_set_tx_lo_freq(ad9361_phy, BLE_EXADV_SECONDARY_FREQ_HZ);

    if (ret < 0)
        printf("ble exadv secondary lo tune failed\n");

    return ret;
#endif
}

void ble_exadv_stop(uint8_t restore_dds)
{
    ble_exadv_active = 0u;
    ble_exadv_next_event = 0;

    if (tx_dmac != NULL)
        axi_dmac_transfer_stop(tx_dmac);

    if (restore_dds && ad9361_phy != NULL && ad9361_phy->tx_dac != NULL)
        axi_dac_set_datasel(ad9361_phy->tx_dac, -1, AXI_DAC_DATA_SEL_DDS);
}

int ble_exadv_start(uint32_t aux_offset_us, uint32_t interval_us)
{
    return ble_exadv_start_with_timing(aux_offset_us, interval_us, 0u);
}

int ble_exadv_start_with_timing(uint32_t aux_offset_us, uint32_t interval_us,
                                uint32_t secondary_start_lead_us)
{
    uint32_t max_secondary_start_lead_us;

    if (aux_offset_us == 0u)
        aux_offset_us = BLE_EXADV_AUX_OFFSET_US;
    if (interval_us == 0u)
        interval_us = BLE_EXADV_INTERVAL_US;

    if (tx_dmac == NULL || ad9361_phy == NULL || ad9361_phy->tx_dac == NULL) {
        printf("ble exadv tx init error\n");
        return -1;
    }

    ble_exadv_stop(0);

    ble_exadv_aux_offset_us = aux_offset_us;
    ble_exadv_primary_total_us = ble_exadv_words_to_us(BLE_EXADV_PRIMARY_WORDS);
    ble_exadv_primary_pad_us = ble_exadv_primary_total_us > BLE_EXADV_PRIMARY_AIR_US ?
                               ble_exadv_primary_total_us - BLE_EXADV_PRIMARY_AIR_US : 0u;
    ble_exadv_secondary_total_us = ble_exadv_words_to_us(BLE_EXADV_SECONDARY_WORDS);

    if (ble_exadv_aux_offset_us <
        BLE_EXADV_PRIMARY_AIR_US + BLE_EXADV_MIN_AUX_FRAME_SPACE_US) {
        printf("BLE EXT ADV error: aux_delay_us must be at least primary_air + T_MAFS (%lu us)\n",
               (unsigned long)(BLE_EXADV_PRIMARY_AIR_US +
                               BLE_EXADV_MIN_AUX_FRAME_SPACE_US));
        return -1;
    }

    if (ble_exadv_aux_offset_us <
        BLE_EXADV_PRIMARY_AIR_US + BLE_EXADV_LO_SWITCH_GUARD_US +
        BLE_EXADV_LO_SWITCH_EST_US) {
        printf("BLE EXT ADV warning: aux_delay_us leaves less than %lu us for LO retune\n",
               (unsigned long)BLE_EXADV_LO_SWITCH_EST_US);
    }

    max_secondary_start_lead_us = ble_exadv_aux_offset_us > ble_exadv_primary_total_us ?
                                  ble_exadv_aux_offset_us - ble_exadv_primary_total_us : 0u;
    if (secondary_start_lead_us > max_secondary_start_lead_us) {
        printf("BLE EXT ADV warning: secondary_start_lead clamped from %lu us to %lu us\n",
               (unsigned long)secondary_start_lead_us,
               (unsigned long)max_secondary_start_lead_us);
        secondary_start_lead_us = max_secondary_start_lead_us;
    }
    ble_exadv_secondary_start_lead_us = secondary_start_lead_us;

    if (ble_exadv_aux_offset_us != BLE_EXADV_AUX_OFFSET_US) {
        printf("BLE EXT ADV error: AuxPtr is generated for %lu us; regenerate waveform for %lu us\n",
               (unsigned long)BLE_EXADV_AUX_OFFSET_US,
               (unsigned long)ble_exadv_aux_offset_us);
        return -1;
    }

    if (interval_us < ble_exadv_min_interval_us())
        interval_us = ble_exadv_min_interval_us();
    ble_exadv_interval_us = interval_us;

    if (ble_exadv_prepare_tx_path() < 0) {
        printf("ble exadv tx path setup failed\n");
        ble_exadv_stop(1);
        return -1;
    }
    ble_exadv_flush_iq();
    axi_dmac_transfer_stop(tx_dmac);
    if (ble_exadv_set_primary_lo() < 0) {
        ble_exadv_stop(1);
        return -1;
    }

    ble_exadv_active = 1u;
    XTime_GetTime(&ble_exadv_next_event);

    printf("BLE EXT ADV armed: aux_delay=%lu us interval=%lu us primary_pad=%lu us secondary_lead=%lu us\n",
           (unsigned long)ble_exadv_aux_offset_us,
           (unsigned long)ble_exadv_interval_us,
           (unsigned long)ble_exadv_primary_pad_us,
           (unsigned long)ble_exadv_secondary_start_lead_us);
    return 0;
}

static int ble_exadv_emit_once(void)
{
    XTime event_start;
    XTime primary_end;
    XTime primary_dma_done;
    XTime secondary_due;
    XTime secondary_trigger;
    XTime secondary_start;
    XTime now;
#if BLE_EXADV_TIMING_DEBUG
    XTime primary_dma_begin;
    XTime secondary_lo_begin;
    XTime secondary_lo_end;
    XTime secondary_dma_begin;
    XTime secondary_dma_end;
    XTime primary_lo_begin;
    XTime primary_lo_end;
    uint32_t primary_guard_late_us;
    uint32_t secondary_wait_late_us;
    uint32_t secondary_start_late_us;
#endif

    axi_dmac_transfer_stop(tx_dmac);
#if BLE_EXADV_TIMING_DEBUG
    XTime_GetTime(&primary_dma_begin);
#endif
    if (ble_exadv_start_dma(ble_exadv_primary_iq_ch37, BLE_EXADV_PRIMARY_WORDS) < 0) {
        printf("ble exadv primary dma start failed\n");
        return -1;
    }

    XTime_GetTime(&event_start);
    primary_end = ble_exadv_time_after(event_start, BLE_EXADV_PRIMARY_AIR_US);
    primary_dma_done = ble_exadv_time_after(event_start, ble_exadv_primary_total_us);
    secondary_due = ble_exadv_time_after(event_start, ble_exadv_aux_offset_us);
    secondary_trigger = ble_exadv_time_before(secondary_due, ble_exadv_secondary_start_lead_us);

#if BLE_EXADV_TIMING_DEBUG
    primary_guard_late_us = ble_exadv_udelay_until(ble_exadv_time_after(primary_end, BLE_EXADV_LO_SWITCH_GUARD_US));
#else
    ble_exadv_udelay_until(ble_exadv_time_after(primary_end, BLE_EXADV_LO_SWITCH_GUARD_US));
#endif
#if BLE_EXADV_TIMING_DEBUG
    XTime_GetTime(&secondary_lo_begin);
#endif
    if (ble_exadv_set_secondary_lo() < 0)
        return -1;
#if BLE_EXADV_TIMING_DEBUG
    XTime_GetTime(&secondary_lo_end);
#endif

    /*
     * AuxOffset is measured from the start of the ADV_EXT_IND that contains
     * AuxPtr. The generated 1 ms primary zero padding remains inside that wait
     * window while the AD9363 retunes; stop it before the secondary deadline so
     * the critical path only submits the secondary DMA.
     */
    ble_exadv_udelay_until(primary_dma_done);
    axi_dmac_transfer_stop(tx_dmac);

#if BLE_EXADV_TIMING_DEBUG
    secondary_wait_late_us = ble_exadv_udelay_until(secondary_trigger);
#else
    ble_exadv_udelay_until(secondary_trigger);
#endif
#if BLE_EXADV_TIMING_DEBUG
    XTime_GetTime(&secondary_dma_begin);
#endif
    if (ble_exadv_start_dma(ble_exadv_secondary_iq_ch3, BLE_EXADV_SECONDARY_WORDS) < 0) {
        printf("ble exadv secondary dma start failed\n");
        return -1;
    }

    XTime_GetTime(&secondary_start);
#if BLE_EXADV_TIMING_DEBUG
    secondary_dma_end = secondary_start;
    secondary_start_late_us = ble_exadv_late_us(secondary_due);
#endif
    ble_exadv_udelay_until(ble_exadv_time_after(secondary_start, ble_exadv_secondary_total_us));
    axi_dmac_transfer_stop(tx_dmac);

    ble_exadv_next_event = ble_exadv_time_after(event_start, ble_exadv_interval_us);
#if BLE_EXADV_TIMING_DEBUG
    XTime_GetTime(&primary_lo_begin);
#endif
    if (ble_exadv_set_primary_lo() < 0)
        return -1;
#if BLE_EXADV_TIMING_DEBUG
    XTime_GetTime(&primary_lo_end);
    printf("BLE EXT ADV timing: prim_dma=%lu us sec_lo=%lu us sec_wait_late=%lu us sec_dma=%lu us sec_late=%lu us prim_guard_late=%lu us prim_lo=%lu us\n",
           (unsigned long)ble_exadv_elapsed_us(primary_dma_begin, event_start),
           (unsigned long)ble_exadv_elapsed_us(secondary_lo_begin, secondary_lo_end),
           (unsigned long)secondary_wait_late_us,
           (unsigned long)ble_exadv_elapsed_us(secondary_dma_begin, secondary_dma_end),
           (unsigned long)secondary_start_late_us,
           (unsigned long)primary_guard_late_us,
           (unsigned long)ble_exadv_elapsed_us(primary_lo_begin, primary_lo_end));
#endif
    XTime_GetTime(&now);
    while (ble_exadv_next_event <= now)
        ble_exadv_next_event = ble_exadv_time_after(ble_exadv_next_event, ble_exadv_interval_us);

    return 0;
}

void ble_exadv_task_tick(void)
{
    XTime now;

    if (!ble_exadv_active)
        return;

    if (tx_dmac == NULL || ad9361_phy == NULL || ad9361_phy->tx_dac == NULL) {
        ble_exadv_stop(1);
        return;
    }

    XTime_GetTime(&now);
    if (now < ble_exadv_next_event)
        return;

    if (ble_exadv_emit_once() < 0)
        ble_exadv_stop(1);
}

void ble_exadv_tx_demo(double *param, char param_no)
{
    uint32_t aux_offset_us = BLE_EXADV_AUX_OFFSET_US;
    uint32_t interval_us = BLE_EXADV_INTERVAL_US;
    uint32_t secondary_start_lead_us = 0u;

    if (param_no > 3) {
        printf("ble_exadv_tx?: expected at most 3 params: aux_delay_us interval_us secondary_start_lead_us\n");
        return;
    }

    if (param_no >= 1 && param[0] > 0.0)
        aux_offset_us = (uint32_t)param[0];
    if (param_no >= 2 && param[1] > 0.0)
        interval_us = (uint32_t)param[1];
    if (param_no >= 3 && param[2] > 0.0)
        secondary_start_lead_us = (uint32_t)param[2];

    printf("BLE EXT ADV cmd: param_no=%d aux=%lu interval=%lu lead=%lu raw=[%lu,%lu,%lu]\n",
           (long)param_no,
           (unsigned long)aux_offset_us,
           (unsigned long)interval_us,
           (unsigned long)secondary_start_lead_us,
           (unsigned long)param[0],
           (unsigned long)param[1],
           (unsigned long)param[2]);

    if (secondary_start_lead_us > aux_offset_us) {
        printf("ble_exadv_tx?: secondary_start_lead_us must be <= aux_delay_us\n");
        return;
    }

    (void)ble_exadv_start_with_timing(aux_offset_us, interval_us,
                                      secondary_start_lead_us);
}
