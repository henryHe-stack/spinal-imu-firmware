#include "sensor_service.h"
#include "app_config.h"
#include "bsp_calibration.h"
#include <string.h>

static const uint8_t channels[3] = {APP_IMU_C0_TCA_CHANNEL,APP_IMU_C4_TCA_CHANNEL,APP_IMU_C7_TCA_CHANNEL};
AppStatus SensorService_Init(SensorService *self, BspI2c *bus, BspTime *timebase)
{
    if (!self || !bus || !timebase) return APP_STATUS_INVALID_ARGUMENT;
    memset(self,0,sizeof(*self)); self->bus=bus; self->timebase=timebase;
    uint8_t offsets[3][22]; bool restored=BspCalibration_Load(offsets)==APP_STATUS_OK;
    for (unsigned i=0;i<3;++i)
    {
        Bno055_InitObject(&self->sensors[i],bus,timebase,&self->mux,APP_BNO055_ADDRESS_7BIT,channels[i],(AppImuNodeId)i);
        if (restored) { memcpy(self->sensors[i].calibration,offsets[i],22); self->sensors[i].calibration_available=true; }
        Bno055_Configure(&self->sensors[i]);
    }
    self->initialized=Tca9548a_Init(&self->mux,bus,APP_TCA9548A_ADDRESS_7BIT)==APP_STATUS_OK;
    self->next_recovery_ms=HAL_GetTick()+APP_SENSOR_RETRY_MS;
    return self->initialized ? APP_STATUS_OK : APP_STATUS_IO_ERROR;
}
AppStatus SensorService_ReadCycle(SensorService *self, AppSampleFrame *frame)
{
    if (!self || !frame || !self->bus) return APP_STATUS_INVALID_ARGUMENT;
    memset(frame,0,sizeof(*frame)); frame->sample_count=3; frame->sequence=self->next_sequence++;
    uint32_t now=HAL_GetTick(); unsigned valid=0;
    for (unsigned i=0;i<3;++i)
    {
        AppImuSample *sample=&frame->samples[i];
        sample->node_id=i; sample->tca_channel=channels[i]; sample->flags=APP_SAMPLE_FLAG_SENSOR_ERROR;
        sample->timestamp_start_us=sample->timestamp_end_us=sample->timestamp_mid_us=BspTime_NowUs(self->timebase);
        if (!self->initialized) continue;
        Bno055_Service(&self->sensors[i]);
        if (self->sensors[i].configured && Bno055_ReadSample(&self->sensors[i],sample)==APP_STATUS_OK) valid++;
    }
    /* One timed recovery of the common bus. Reinitialize mux cache because its
     * hardware selection may differ after reset; all node delays are independent. */
    if (!valid && (int32_t)(now-self->next_recovery_ms)>=0)
    {
        self->next_recovery_ms=now+APP_SENSOR_RETRY_MS;
        BspI2c_Recover(self->bus);
        self->initialized=Tca9548a_Init(&self->mux,self->bus,APP_TCA9548A_ADDRESS_7BIT)==APP_STATUS_OK;
    }
    return valid ? APP_STATUS_OK : APP_STATUS_IO_ERROR;
}
AppStatus SensorService_ExecuteCommand(SensorService *self, const AppControlCommand *command)
{
    if (!self || !command) return APP_STATUS_INVALID_ARGUMENT;
    if (command->id==APP_COMMAND_REINITIALIZE_SENSORS)
    {
        for (unsigned i=0;i<3;++i) Bno055_Configure(&self->sensors[i]);
        return APP_STATUS_OK; /* Accepted; readiness is asynchronous. */
    }
    if (command->id==APP_COMMAND_SAVE_CALIBRATION)
    {
        uint8_t offsets[3][22];
        for (unsigned i=0;i<3;++i)
        {
            AppStatus status=Bno055_CaptureCalibration(&self->sensors[i]);
            if (status!=APP_STATUS_OK) return status;
            memcpy(offsets[i],self->sensors[i].calibration,22);
        }
        return BspCalibration_Save(offsets);
    }
    return APP_STATUS_NOT_IMPLEMENTED; /* Neutral/neck algorithm remains separate. */
}
