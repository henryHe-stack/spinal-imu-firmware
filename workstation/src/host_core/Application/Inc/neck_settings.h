#ifndef NECK_SETTINGS_H
#define NECK_SETTINGS_H
#include <stdint.h>
#include <stddef.h>
/* Two 1 KiB flash pages reserved by linker, 64-byte append-only records.
 * Commit halfword is written last; erase only page NOT holding newest record.
 * No neutral reference, calibration approval or sample time is persisted. */
typedef struct { uint32_t version, cutoff_millihz, hold_ms, thresholds_mdeg[4]; } NeckSettings;
typedef struct {
    void *user;
    int (*read)(void *,uint32_t offset,uint8_t *bytes,size_t length);
    int (*program)(void *,uint32_t offset,const uint8_t *bytes,size_t length);
    int (*erase)(void *,uint32_t page);
} NeckSettingsIo;
typedef struct {
    NeckSettingsIo io;
    NeckSettings settings;
    uint32_t sequence,boot,config,latest_offset;
    uint8_t loaded;
} NeckSettingsStore;
void neck_settings_default(NeckSettings *settings);
int neck_settings_valid(const NeckSettings *settings);
/* key 1=cutoff mHz, 2=hold ms, 3..6=flex warn/danger,lateral warn/danger mdeg */
int neck_settings_set(NeckSettings *settings,uint32_t key,uint32_t value);
int neck_settings_open(NeckSettingsStore *store,const NeckSettingsIo *io);
int neck_settings_save(NeckSettingsStore *store,const NeckSettings *settings,uint32_t config);
/* MUST succeed before sending any packet in a new MCU boot. Exhaustion/errors
 * are fail-closed. Unique only while reserved flash survives reflashing. */
int neck_settings_reserve_boot(NeckSettingsStore *store);
#endif
