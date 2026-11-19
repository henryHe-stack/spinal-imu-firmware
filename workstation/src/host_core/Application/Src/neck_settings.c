#include "neck_settings.h"
#include "neck_wire.h"
#include <string.h>
#define RECORD_SIZE 64u
#define PAGE_SIZE 1024u
static uint32_t rd(const uint8_t *p)
{ return (uint32_t)p[0]|((uint32_t)p[1]<<8)|((uint32_t)p[2]<<16)|((uint32_t)p[3]<<24); }
static void wr(uint8_t *p,uint32_t v)
{ unsigned i; for(i=0;i<4;++i) p[i]=(uint8_t)(v>>(8*i)); }
void neck_settings_default(NeckSettings *s)
{
    if(!s) return;
    s->version=1; s->cutoff_millihz=500; s->hold_ms=10000;
    s->thresholds_mdeg[0]=15000; s->thresholds_mdeg[1]=30000;
    s->thresholds_mdeg[2]=5000; s->thresholds_mdeg[3]=10000;
}
int neck_settings_valid(const NeckSettings *s)
{
    unsigned i;
    if(!s || s->version!=1 || s->cutoff_millihz<100 || s->cutoff_millihz>20000 ||
        s->hold_ms<100 || s->hold_ms>120000) return 0;
    for(i=0;i<4;++i) if(!s->thresholds_mdeg[i] || s->thresholds_mdeg[i]>90000) return 0;
    return s->thresholds_mdeg[0]<s->thresholds_mdeg[1] && s->thresholds_mdeg[2]<s->thresholds_mdeg[3];
}
int neck_settings_set(NeckSettings *s,uint32_t key,uint32_t value)
{
    NeckSettings next;
    if(!s) return 0;
    next=*s;
    if(key==1) next.cutoff_millihz=value;
    else if(key==2) next.hold_ms=value;
    else if(key>=3 && key<=6) next.thresholds_mdeg[key-3]=value;
    else return 0;
    if(!neck_settings_valid(&next)) return 0;
    *s=next; return 1;
}
static int blank(const uint8_t *p)
{ unsigned i; for(i=0;i<RECORD_SIZE;++i) if(p[i]!=255) return 0; return 1; }
static void decode(const uint8_t *p,NeckSettings *s)
{
    unsigned i; s->version=rd(p+16); s->cutoff_millihz=rd(p+20); s->hold_ms=rd(p+24);
    for(i=0;i<4;++i) s->thresholds_mdeg[i]=rd(p+28+i*4);
}
int neck_settings_open(NeckSettingsStore *s,const NeckSettingsIo *io)
{
    uint32_t offset; uint8_t record[RECORD_SIZE]; NeckSettings candidate;
    unsigned invalid_pages=0; uint32_t erase_intent_offset=UINT32_MAX;
    if(!s) return 0;
    memset(s,0,sizeof(*s));
    if(!io || !io->read || !io->program || !io->erase) return 0;
    s->io=*io; s->latest_offset=UINT32_MAX; s->config=1; neck_settings_default(&s->settings);
    for(offset=0;offset<2*PAGE_SIZE;offset+=RECORD_SIZE) {
        uint16_t crc;
        if(!io->read(io->user,offset,record,sizeof(record))) return 0;
        if(record[62] || record[63]) continue; /* Uncommitted or erased. */
        crc=neck_wire_crc16(record,56);
        decode(record,&candidate);
        if(rd(record)!=0x3256534eu || record[60]!=(uint8_t)crc || record[61]!=(uint8_t)(crc>>8) ||
            !rd(record+4) || !rd(record+8) || !rd(record+12) || !neck_settings_valid(&candidate)) {
            invalid_pages |= 1u<<(offset/PAGE_SIZE); continue;
        }
        /* The last committed record authorizes erasing ONLY the other page.
         * Markers are outside the CRC, programmed and verified before/after erase.
         * No new record is written until the completion marker is durable. */
        if(!record[56] && !record[57] && (record[58] || record[59])) erase_intent_offset=offset;
        if(rd(record+4)==s->sequence) return 0; /* Ambiguous committed generations. */
        if(rd(record+4)>s->sequence) {
            s->sequence=rd(record+4); s->boot=rd(record+8); s->config=rd(record+12);
            s->settings=candidate; s->latest_offset=offset;
        }
    }
    if(invalid_pages) {
        unsigned latest_page=s->latest_offset/PAGE_SIZE;
        if(!s->sequence || latest_page>1 || erase_intent_offset!=s->latest_offset ||
            (invalid_pages&(1u<<latest_page))) return 0;
    }
    s->loaded=1; return 1;
}
static int append(NeckSettingsStore *s,const NeckSettings *settings,uint32_t config,uint32_t boot)
{
    uint8_t record[RECORD_SIZE],check[RECORD_SIZE]; uint32_t page,offset; unsigned i; uint16_t crc;
    if(!s || !s->loaded || !neck_settings_valid(settings) || !config || !boot || s->sequence==UINT32_MAX) return 0;
    page=s->latest_offset==UINT32_MAX?0:s->latest_offset/PAGE_SIZE;
    for(offset=page*PAGE_SIZE;offset<(page+1)*PAGE_SIZE;offset+=RECORD_SIZE) {
        if(!s->io.read(s->io.user,offset,check,sizeof(check))) return 0;
        if(blank(check)) break;
    }
    if(offset==(page+1)*PAGE_SIZE) {
        static const uint8_t zero[2]={0,0};
        uint32_t marker=s->latest_offset+56, probe;
        /* Persist erase intent before touching the obsolete page. */
        if(!s->io.program(s->io.user,marker,zero,2) ||
            !s->io.read(s->io.user,marker,check,2) || check[0] || check[1]) { s->loaded=0; return 0; }
        page^=1u;
        if(!s->io.erase(s->io.user,page)) { s->loaded=0; return 0; }
        for(probe=page*PAGE_SIZE;probe<(page+1)*PAGE_SIZE;probe+=RECORD_SIZE)
            if(!s->io.read(s->io.user,probe,check,64) || !blank(check)) { s->loaded=0; return 0; }
        if(!s->io.program(s->io.user,marker+2,zero,2) ||
            !s->io.read(s->io.user,marker+2,check,2) || check[0] || check[1]) { s->loaded=0; return 0; }
        offset=page*PAGE_SIZE;
    }
    memset(record,255,sizeof(record)); wr(record,0x3256534eu); wr(record+4,s->sequence+1);
    wr(record+8,boot); wr(record+12,config); wr(record+16,settings->version);
    wr(record+20,settings->cutoff_millihz); wr(record+24,settings->hold_ms);
    for(i=0;i<4;++i) wr(record+28+4*i,settings->thresholds_mdeg[i]);
    crc=neck_wire_crc16(record,56); record[60]=(uint8_t)crc; record[61]=(uint8_t)(crc>>8);
    if(!s->io.program(s->io.user,offset,record,62) ||
        !s->io.read(s->io.user,offset,check,sizeof(check)) || memcmp(record,check,64)) return 0;
    record[62]=record[63]=0;
    if(!s->io.program(s->io.user,offset+62,record+62,2) ||
        !s->io.read(s->io.user,offset,check,sizeof(check)) || memcmp(record,check,64)) {
        s->loaded=0; return 0; /* Commit outcome uncertain: reopen before further writes. */
    }
    ++s->sequence; s->boot=boot; s->config=config; s->settings=*settings; s->latest_offset=offset;
    return 1;
}
int neck_settings_save(NeckSettingsStore *s,const NeckSettings *settings,uint32_t config)
{ return s?append(s,settings,config,s->boot):0; }
int neck_settings_reserve_boot(NeckSettingsStore *s)
{ return s && s->boot!=UINT32_MAX ? append(s,&s->settings,s->config,s->boot+1):0; }
