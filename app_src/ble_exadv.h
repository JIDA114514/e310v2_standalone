#ifndef BLE_EXADV

#define BLE_EXADV

#include <stdint.h>
//#include "../../../../../../python/std_ble/ble_exadv_waveform_30_72M.h"

#define BLE_EXADV_PRIMARY_FREQ_HZ (2402000000ULL)
#define BLE_EXADV_SECONDARY_FREQ_HZ (2402000000ULL)
#define BLE_EXADV_AUX_OFFSET_US (1500u)
#define BLE_EXADV_INTERVAL_US (20000u)
#define BLE_EXADV_PRIMARY_AIR_US (184u)
#define BLE_EXADV_PRIMARY_WORDS (72746u)
#define BLE_EXADV_SECONDARY_WORDS (78152u)

// extern const uint32_t ble_exadv_primary_iq_ch37[BLE_EXADV_PRIMARY_WORDS] __attribute__((aligned(64)));
// extern const uint32_t ble_exadv_secondary_iq_ch3[BLE_EXADV_SECONDARY_WORDS] __attribute__((aligned(64)));

int ble_exadv_start(uint32_t aux_offset_us, uint32_t interval_us);
int ble_exadv_start_with_timing(uint32_t aux_offset_us, uint32_t interval_us,
                                uint32_t secondary_start_lead_us);
void ble_exadv_stop(uint8_t restore_dds);
void ble_exadv_task_tick(void);
void ble_exadv_tx_demo(double *param, char param_no);

#endif
