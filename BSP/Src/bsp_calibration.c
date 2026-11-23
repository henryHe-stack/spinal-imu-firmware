#include "bsp_calibration.h"
#include "app_config.h"
#include "protocol_service.h"
#include "stm32f1xx_hal.h"
#include <string.h>

#define SLOT0 0x0800F800UL
#define SLOT1 0x0800FC00UL
#define RECORD_SIZE 88U
/* 0..3 magic, 4 format, 5 mode, 6 axis map, 7 sign, 8..11 generation,
 * 12..77 offsets, 78..81 reserved, 82..83 CRC, 84..85 commit, 86..87 reserved. */
static bool Valid(uint32_t address)
{
    const uint8_t *p = (const uint8_t *)address;
    return memcmp(p,"BNC1",4)==0 && p[4]==1 && p[5]==APP_BNO055_OPERATION_MODE
        && p[6]==APP_BNO055_AXIS_MAP && p[7]==APP_BNO055_AXIS_SIGN
        && p[84]==0xA5 && p[85]==0x5A
        && ProtocolService_Crc16(p,82)==((uint16_t)p[82] | (uint16_t)p[83]<<8);
}
static uint32_t Generation(uint32_t addr)
{ uint32_t value; memcpy(&value,(const void *)(addr+8),4); return value; }
static uint32_t Latest(void)
{
    bool a=Valid(SLOT0), b=Valid(SLOT1);
    if (!a) return b ? SLOT1 : 0;
    if (!b) return SLOT0;
    return (int32_t)(Generation(SLOT1)-Generation(SLOT0))>0 ? SLOT1 : SLOT0;
}
AppStatus BspCalibration_Load(uint8_t offsets[3][22])
{
    if (!offsets) return APP_STATUS_INVALID_ARGUMENT;
    uint32_t slot=Latest();
    if (!slot) return APP_STATUS_DEVICE_NOT_FOUND;
    memcpy(offsets,(const void *)(slot+12),66);
    return APP_STATUS_OK;
}
AppStatus BspCalibration_Save(const uint8_t offsets[3][22])
{
    if (!offsets) return APP_STATUS_INVALID_ARGUMENT;
    uint8_t record[RECORD_SIZE]; memset(record,0xFF,sizeof(record));
    uint32_t previous=Latest(), target=previous==SLOT0 ? SLOT1 : SLOT0;
    uint32_t generation=previous ? Generation(previous)+1U : 1U;
    memcpy(record,"BNC1",4); record[4]=1; record[5]=APP_BNO055_OPERATION_MODE;
    record[6]=APP_BNO055_AXIS_MAP; record[7]=APP_BNO055_AXIS_SIGN;
    memcpy(record+8,&generation,4); memcpy(record+12,offsets,66);
    uint16_t crc=ProtocolService_Crc16(record,82); record[82]=(uint8_t)crc; record[83]=(uint8_t)(crc>>8);
    record[84]=0xA5; record[85]=0x5A;
    if (HAL_FLASH_Unlock()!=HAL_OK) return APP_STATUS_IO_ERROR;
    FLASH_EraseInitTypeDef erase={0}; uint32_t page_error;
    erase.TypeErase=FLASH_TYPEERASE_PAGES; erase.PageAddress=target; erase.NbPages=1;
    HAL_StatusTypeDef status=HAL_FLASHEx_Erase(&erase,&page_error);
    /* Write commit marker last; loss of power preserves the other slot. */
    for (unsigned i=0; i<84 && status==HAL_OK; i+=2)
        status=HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD,target+i,(uint16_t)record[i] | (uint16_t)record[i+1]<<8);
    if (status==HAL_OK) status=HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD,target+84,0x5AA5);
    HAL_FLASH_Lock();
    return status==HAL_OK && Valid(target) ? APP_STATUS_OK : APP_STATUS_IO_ERROR;
}
