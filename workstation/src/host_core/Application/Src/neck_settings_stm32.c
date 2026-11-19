#include "neck_settings_stm32.h"
#include "main.h"
#include <string.h>
#define STORAGE_BASE 0x0800F800u
static int flash_read(void *user,uint32_t offset,uint8_t *bytes,size_t size)
{
    (void)user;
    if(!bytes || offset>2048 || size>2048-offset) return 0;
    memcpy(bytes,(const void *)(uintptr_t)(STORAGE_BASE+offset),size); return 1;
}
static int flash_program(void *user,uint32_t offset,const uint8_t *bytes,size_t size)
{
    size_t i; int ok=1; (void)user;
    if(!bytes || (offset&1) || (size&1) || offset>2048 || size>2048-offset) return 0;
    if(HAL_FLASH_Unlock()!=HAL_OK) return 0;
    for(i=0;i<size;i+=2) {
        uint32_t address=STORAGE_BASE+offset+(uint32_t)i;
        uint16_t value=(uint16_t)bytes[i]|((uint16_t)bytes[i+1]<<8);
        /* Recovery may revisit an already committed erase marker. Do not
         * reprogram an identical halfword (or erased padding). */
        if(*(const volatile uint16_t *)(uintptr_t)address==value) continue;
        if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD,address,value)!=HAL_OK) { ok=0; break; }
    }
    if(HAL_FLASH_Lock()!=HAL_OK) ok=0;
    return ok;
}
static int flash_erase(void *user,uint32_t page)
{
    FLASH_EraseInitTypeDef erase; uint32_t error=0; int ok; (void)user;
    if(page>1 || HAL_FLASH_Unlock()!=HAL_OK) return 0;
    memset(&erase,0,sizeof(erase)); erase.TypeErase=FLASH_TYPEERASE_PAGES;
    erase.PageAddress=STORAGE_BASE+page*1024; erase.NbPages=1;
    ok=HAL_FLASHEx_Erase(&erase,&error)==HAL_OK;
    if(HAL_FLASH_Lock()!=HAL_OK) ok=0;
    return ok;
}
NeckSettingsIo neck_settings_stm32_io(void)
{ NeckSettingsIo io={0,flash_read,flash_program,flash_erase}; return io; }
