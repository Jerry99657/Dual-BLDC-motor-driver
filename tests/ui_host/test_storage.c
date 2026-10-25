#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <stddef.h>
#include "app_storage.h"
#include "w25q64.h"
#include "main.h"

static uint8_t flash[3*W25Q64_SECTOR_SIZE];
static uint32_t tick, busy_until;
static unsigned sync_calls;
static bool fail_program, corrupt_read, storage_blocked;
bool MT6701_2_StorageBlocked(void) { return storage_blocked; }
bool MT6701_2_FlashBegin(void) { return true; }
void MT6701_2_FlashEnd(void) {}
uint32_t HAL_GetTick(void) { return tick; }
void HAL_Delay(uint32_t ms) { assert(ms==2U); tick+=ms; }
void MT6701_2_Service(uint32_t now) { (void)now; }
bool W25Q_Init(void) { return true; }
uint32_t W25Q_ReadID(void) { return 0xEF4017; }
uint32_t W25Q_LastFailure(void) { return 0x06010402U; }
bool W25Q_LastFailureBusy(void) { return false; }
uint8_t W25Q_LastStatus(void) { return 1; }
bool W25Q_Read(uint32_t addr, uint8_t *data, uint32_t len) {
    assert((int32_t)(tick-busy_until)>=0);
    assert(addr+len<=sizeof(flash)); memcpy(data,flash+addr,len);
    if(corrupt_read) data[0]^=1;
    return true;
}
bool W25Q_PollBusy(bool *busy) { *busy=(int32_t)(tick-busy_until)<0; return true; }
bool W25Q_BeginSectorErase(uint32_t addr) {
    assert((int32_t)(tick-busy_until)>=0);
    assert(addr%W25Q64_SECTOR_SIZE==0 && addr+W25Q64_SECTOR_SIZE<=sizeof(flash));
    memset(flash+addr,255,W25Q64_SECTOR_SIZE); busy_until=tick+40; return true;
}
bool W25Q_BeginPageProgram(uint32_t addr,const uint8_t *data,uint16_t len) {
    assert((int32_t)(tick-busy_until)>=0);
    assert(addr+len<=sizeof(flash) && addr%256+len<=256);
    if(fail_program) return false;
    for(unsigned i=0;i<len;++i) flash[addr+i]&=data[i];
    busy_until=tick+3; return true;
}
bool W25Q_SectorErase(uint32_t addr) {
    ++sync_calls; if(!W25Q_BeginSectorErase(addr)) return false;
    tick=busy_until; return true;
}
bool W25Q_PageProgram(uint32_t addr,const uint8_t *data,uint16_t len) {
    ++sync_calls; if(!W25Q_BeginPageProgram(addr,data,len)) return false;
    tick=busy_until; return true;
}
static void service(unsigned ms) {
    for(unsigned i=0;i<ms;++i) { ++tick; AppStorage_Service(tick); }
}
static bool result(void) {
    bool success; assert(AppStorage_PollSaveResult(&success)); return success;
}
static uint32_t crc32(const void *data,unsigned len) {
    const uint8_t *bytes=data; uint32_t crc=0xFFFFFFFFU;
    for(unsigned i=0;i<len;++i) {
        crc^=bytes[i];
        for(unsigned bit=0;bit<8;++bit) crc=(crc>>1)^((crc&1U)?0xEDB88320U:0U);
    }
    return ~crc;
}
int main(void) {
    memset(flash,255,sizeof(flash)); assert(AppStorage_Init());
    assert(AppStorage_GetSaveState()==APP_SAVE_IDLE);
    assert(sync_calls==0);
    assert(((AppStorage_Get()->flags & APP_SPEED_LIMIT_MASK)>>APP_SPEED_LIMIT_SHIFT)==100);
    assert(!(AppStorage_Get()->flags & APP_FLAG_MOTOR_4015));
    AppStorage_Get()->flags |= APP_FLAG_MOTOR_4015;
    AppStorage_Get()->speed_x10=123;
    AppStorage_Get()->flags=(AppStorage_Get()->flags & ~APP_SPEED_LIMIT_MASK) |
                            (123UL<<APP_SPEED_LIMIT_SHIFT);
    AppStorage_Get()->speed_step_x10=6;
    AppStorage_Get()->torque_step_x100=7;
    AppStorage_Get()->position_step_x100=4;
    assert(AppStorage_RequestSave()); assert(AppStorage_GetSaveState()==APP_SAVE_PENDING);
    storage_blocked=true; service(500); assert(AppStorage_GetSaveState()==APP_SAVE_DEFERRED);
    storage_blocked=false; service(10); assert(AppStorage_GetSaveState()==APP_SAVE_WRITING);
    bool success; assert(!AppStorage_PollSaveResult(&success));
    assert(!AppStorage_RequestSave()); /* snapshot in flight is immutable */
    AppStorage_Get()->speed_x10=456;
    AppStorage_Get()->flags=(AppStorage_Get()->flags & ~APP_SPEED_LIMIT_MASK) |
                            (234UL<<APP_SPEED_LIMIT_SHIFT);
    service(50); assert(result()); assert(AppStorage_GetSaveState()==APP_SAVE_SAVED);
    assert(AppStorage_Get()->speed_x10==456); /* live edit survives completion */
    assert(((AppStorage_Get()->flags & APP_SPEED_LIMIT_MASK)>>APP_SPEED_LIMIT_SHIFT)==234);
    service(10); assert(sync_calls==0);
    assert(AppStorage_Init()); assert(AppStorage_Get()->speed_x10==123);
    assert(AppStorage_Get()->flags & APP_FLAG_MOTOR_4015);
    assert(((AppStorage_Get()->flags & APP_SPEED_LIMIT_MASK)>>APP_SPEED_LIMIT_SHIFT)==123);
    assert(((AppStorage_Get()->flags & APP_POSITION_SPEED_MASK)>>APP_POSITION_SPEED_SHIFT)==30);
    assert(AppStorage_Get()->speed_step_x10==6 && AppStorage_Get()->torque_step_x100==7 && AppStorage_Get()->position_step_x100==4);
    AppStorage_Get()->speed_x10=789; assert(AppStorage_RequestSave());
    service(60); assert(result()); service(10);
    assert(AppStorage_Init()); assert(AppStorage_Get()->speed_x10==789);
    /* Interrupted/incomplete replacement leaves previous valid A/B copy. */
    AppStorage_Get()->speed_x10=321; assert(AppStorage_RequestSave());
    service(10); tick=busy_until; assert(AppStorage_Init());
    assert(AppStorage_Get()->speed_x10==789);
    fail_program=true; assert(AppStorage_RequestSave()); service(100); assert(!result());
    assert(AppStorage_GetSaveState()==APP_SAVE_FAILED);
    service(100); assert(AppStorage_GetSaveState()==APP_SAVE_FAILED);
    assert(!strcmp(AppStorage_SaveErrorText(),"Save WRITE"));
    fail_program=false; assert(AppStorage_RequestSave()); service(50); assert(result()); service(10);
    corrupt_read=true; assert(AppStorage_RequestSave()); service(100); assert(!result());
    assert(!strcmp(AppStorage_SaveErrorText(),"Save VERIFY"));
    corrupt_read=false; service(20); assert(AppStorage_Init());
    /* Queued logs rotate across sector boundary without synchronous waits. */
    for(unsigned i=0;i<150;++i) {
        AppStorage_Log(APP_LOG_TARGET_CHANGED,i); service(50);
    }
    assert(sync_calls==0);
    AppStorage_ResetDefaults(); assert(AppStorage_Get()->speed_x10==0);
    assert(!(AppStorage_Get()->flags & APP_FLAG_MOTOR_4015));
    assert(((AppStorage_Get()->flags & APP_SPEED_LIMIT_MASK)>>APP_SPEED_LIMIT_SHIFT)==100);
    assert(AppStorage_RequestSave()); service(60); assert(result());
    assert(AppStorage_Get()->speed_step_x10==5 && AppStorage_Get()->torque_step_x100==5 && AppStorage_Get()->position_step_x100==5);
    /* Both configured speed-limit endpoints survive the real NOR save/reload. */
    const unsigned limits[]={1,600};
    for(unsigned i=0;i<2;++i) {
        AppStorage_Get()->flags=(AppStorage_Get()->flags & ~APP_SPEED_LIMIT_MASK) |
                                ((uint32_t)limits[i]<<APP_SPEED_LIMIT_SHIFT);
        assert(AppStorage_RequestSave()); service(60); assert(result()); service(20);
        assert(AppStorage_Init());
        assert(((AppStorage_Get()->flags & APP_SPEED_LIMIT_MASK)>>APP_SPEED_LIMIT_SHIFT)==limits[i]);
    }
    /* Extended position speed must preserve the independent flags and survive
     * real NOR save/reload, including the old six-bit boundary. */
    const unsigned positions[]={1,60,63,64,100,120,600};
    for(unsigned i=0;i<sizeof(positions)/sizeof(positions[0]);++i) {
        AppStorage_Get()->flags=APP_FLAG_CURRENT_LOOP|APP_FLAG_LED_ENABLE|APP_FLAG_MOTOR_4015|
            (600UL<<APP_SPEED_LIMIT_SHIFT)|AppSettings_EncodePositionSpeed(positions[i]);
        AppStorage_Get()->axis2_flags=(120UL<<APP_SPEED_LIMIT_SHIFT)|AppSettings_EncodePositionSpeed(120);
        assert(AppStorage_RequestSave()); service(60); assert(result()); service(20);
        assert(AppStorage_Init());
        assert(AppSettings_GetPositionSpeed(AppStorage_Get()->flags)==positions[i]);
        assert(AppSettings_GetPositionSpeed(AppStorage_Get()->axis2_flags)==120);
        assert((AppStorage_Get()->flags & APP_SPEED_LIMIT_MASK)==(600UL<<APP_SPEED_LIMIT_SHIFT));
        assert((AppStorage_Get()->flags & 3U)==3U && (AppStorage_Get()->flags & APP_FLAG_MOTOR_4015));
    }
    /* Original version 1 settings remain readable after adding step fields. */
    struct Legacy {
        uint32_t magic; uint16_t version,size; uint32_t seq;
        int32_t speed,voltage,position; uint32_t flags,brightness,crc;
    } legacy={0x464F4331UL,1,sizeof(legacy),21,-123,250,125,
              APP_FLAG_LED_ENABLE|(31UL<<APP_POSITION_SPEED_SHIFT),35,0};
    legacy.crc=crc32(&legacy,offsetof(struct Legacy,crc));
    service(20); memset(flash,255,sizeof(flash)); memcpy(flash,&legacy,sizeof(legacy));
    assert(AppStorage_Init());
    const AppSettings *migrated=AppStorage_Get();
    assert(migrated->version==5 && migrated->speed_x10==-123 && migrated->voltage_x100==250 && migrated->led_brightness==35);
    assert(migrated->speed_step_x10==5 && migrated->torque_step_x100==5 && migrated->position_step_x100==5);
    assert((migrated->flags & APP_SPEED_LIMIT_MASK)==0); /* UI supplies default for old records. */
    assert(!(migrated->flags & APP_FLAG_MOTOR_4015));
    assert(AppStorage_RequestSave()); service(60); assert(result()); service(20);
    assert(AppStorage_Init() && AppStorage_Get()->speed_x10==-123);
    assert((AppStorage_Get()->flags & APP_SPEED_LIMIT_MASK)==0); /* Existing version 2 layout. */
    struct Legacy2 {
        uint32_t magic; uint16_t version,size; uint32_t seq;
        int32_t speed,voltage,position; uint32_t flags,brightness;
        int32_t speed_step,torque_step,position_step; uint32_t crc;
    } legacy2={0x464F4331UL,2,sizeof(legacy2),32,456,300,-100,
               APP_FLAG_MOTOR_4015|APP_FLAG_CURRENT_LOOP,65,8,9,10,0};
    legacy2.crc=crc32(&legacy2,offsetof(struct Legacy2,crc));
    service(20); memset(flash,255,sizeof(flash)); memcpy(flash,&legacy2,sizeof(legacy2));
    assert(AppStorage_Init()); migrated=AppStorage_Get();
    assert(migrated->version==5 && migrated->speed_x10==456 && migrated->position_x100==-100);
    assert((migrated->flags & APP_FLAG_MOTOR_4015) && migrated->torque_step_x100==9);
    assert(migrated->gear_ratio_x100==100 && migrated->control_config==0);
    assert(migrated->axis2_speed_step_x10==5 && migrated->axis2_speed_x10==0);
    assert(AppSettings_GetIbusLimit(migrated->control_config)==APP_IBUS_DEFAULT_MA);
    assert(AppSettings_GetIbusLimit(63U<<APP_CONFIG_IBUS_SHIFT)==APP_IBUS_DEFAULT_MA);
    AppStorage_Get()->control_config=18 | APP_CONFIG_FOCUS_M2 | APP_CONFIG_SIXSTEP | APP_CONFIG_ESC | (27UL<<APP_CONFIG_IBUS_SHIFT) | (100UL<<APP_CONFIG_THROTTLE_SHIFT); AppStorage_Get()->gear_ratio_x100=-125;
    AppStorage_Get()->axis2_speed_x10=432; AppStorage_Get()->axis2_position_step_x100=12;
    assert(AppStorage_RequestSave()); service(80); assert(result());
    assert(AppStorage_Init()); migrated=AppStorage_Get();
    assert(migrated->control_config==(18 | APP_CONFIG_FOCUS_M2 | APP_CONFIG_SIXSTEP | APP_CONFIG_ESC | (27UL<<APP_CONFIG_IBUS_SHIFT) | (100UL<<APP_CONFIG_THROTTLE_SHIFT)) && migrated->gear_ratio_x100==-125);
    assert(AppSettings_GetIbusLimit(migrated->control_config)==2700);
    assert(migrated->axis2_speed_x10==432 && migrated->axis2_position_step_x100==12);
    /* An actual pre-extension v3 record uses only bits[13:8] for Pos Speed. */
    AppSettings legacy3=*migrated;
    legacy3.seq=100; legacy3.flags=APP_FLAG_MOTOR_4015|(120UL<<APP_SPEED_LIMIT_SHIFT)|(60UL<<8);
    legacy3.axis2_flags=APP_FLAG_CURRENT_LOOP|(110UL<<APP_SPEED_LIMIT_SHIFT)|(47UL<<8);
    legacy3.version=3; legacy3.size=offsetof(AppSettings,motor_detect)+4;
    legacy3.motor_detect=crc32(&legacy3,offsetof(AppSettings,motor_detect));
    service(20); memset(flash,255,sizeof(flash)); memcpy(flash,&legacy3,legacy3.size);
    assert(AppStorage_Init());
    assert(AppSettings_GetPositionSpeed(AppStorage_Get()->flags)==60);
    assert(AppSettings_GetPositionSpeed(AppStorage_Get()->axis2_flags)==47);
    assert(AppStorage_Get()->version==5 && AppStorage_Get()->motor_detect==0);
    /* R78 v4 stores its CRC immediately after the packed poles/Rs result. */
    AppSettings legacy4=*AppStorage_Get();
    legacy4.version=4; legacy4.seq=101; legacy4.size=offsetof(AppSettings,motor_detect_l_uh)+4;
    legacy4.motor_detect=(2300U<<8)|249U; /* reversed seven pole pairs */
    legacy4.motor_detect_l_uh=crc32(&legacy4,offsetof(AppSettings,motor_detect_l_uh));
    service(20); memset(flash,255,sizeof(flash)); memcpy(flash,&legacy4,legacy4.size);
    assert(AppStorage_Init() && AppStorage_Get()->version==5);
    assert(AppStorage_Get()->motor_detect==legacy4.motor_detect);
    assert(AppStorage_Get()->motor_detect_l_uh==0 && AppStorage_Get()->motor_detect_flux_uwb==0);
    AppStorage_Get()->motor_detect=(1800U<<8)|11U;
    AppStorage_Get()->motor_detect_l_uh=1326;
    AppStorage_Get()->motor_detect_flux_uwb=7437;
    assert(AppStorage_RequestSave()); service(80); assert(result());
    assert(AppStorage_Init() && AppStorage_Get()->motor_detect==((1800U<<8)|11U));
    assert(AppStorage_Get()->motor_detect_l_uh==1326 && AppStorage_Get()->motor_detect_flux_uwb==7437);
    AppStorage_ResetDefaults(); assert(AppStorage_Get()->motor_detect==0);
    assert(!AppStorage_Get()->motor_detect_l_uh && !AppStorage_Get()->motor_detect_flux_uwb);
    printf("PASS: async save/log, A/B recovery, failure paths, step/speed-limit persistence, versions 1/2 migration and dual-axis v3 persistence\n");
    return 0;
}
