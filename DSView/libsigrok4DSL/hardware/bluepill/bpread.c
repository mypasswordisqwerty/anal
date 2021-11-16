//
//  bpdev.c
//  sigrok4DSL
//
//  Created by john on 14.11.2021.
//

#include "bpread.h"
#include <unistd.h>

#define CHANNELS 9
#define PKSZ 64

struct BPRead bpread;

struct HWEvent {
    uint32_t ms;
    uint16_t tick;
    uint16_t value;
};
#define HWEVENT_SZ sizeof(struct HWEvent)
#define EVBUF_SZ 65
#define BUF_SZ EVBUF_SZ * HWEVENT_SZ

struct RDBuf{
    int sz;
    uint8_t data[BUF_SZ];
};
struct RDBuf buf;
uint64_t spms;
uint64_t adj = 0;


struct Event{
    uint64_t tick;
    uint16_t value;
};

struct Events {
    struct Event ev[EVBUF_SZ];
    int sz;
    struct Event* cur;
    struct Event* last;
};
struct Events events;


struct sr_datafeed_packet packet;
struct sr_datafeed_logic logic = {0};

void* zeroMs = NULL;
void* oneMs = NULL;

int64_t waitingSamples;
int nodataCnt = 0;


static int readEvents(int fd){
    int amount = EVBUF_SZ - events.sz;
    ssize_t sz = read(fd, &buf.data[buf.sz], amount * HWEVENT_SZ - buf.sz);
    if (sz<0){
        sr_err("Error %i from read: %s", errno, strerror(errno));
        return 0;
    }
    buf.sz += sz;
    int rem = buf.sz;
    struct HWEvent* ev = (struct HWEvent*)buf.data;
    struct Event e;
    int ret = 0;
    uint16_t prevValue = events.sz ? events.ev[events.sz-1].value : 0;
    while (rem >= HWEVENT_SZ) {
        e.tick = ev->ms * spms + ev->tick + adj;
        e.value = (ev->value >> 3) & 0x1F;
        if (events.sz && e.value == prevValue) {
            rem -= HWEVENT_SZ;
            break;
        }
        prevValue = e.value;
        if (e.tick < events.ev[0].tick){
            e.tick += 0xFFFFFFFF * spms;
            adj += 0xFFFFFFFF * spms;
        }
        ev++;
        ret++;
        rem -= HWEVENT_SZ;
        events.ev[events.sz++] = e;
        g_assert(events.sz<65);
    }
    buf.sz = rem;
    if (buf.sz > 0) {
        memcpy(buf.data, ev, HWEVENT_SZ);
    }
    if (ret) {
        events.cur = &events.ev[0];
        events.last = &events.ev[events.sz-1];
    }
    return ret;
}

static void sendToChannels(const struct sr_dev_inst *sdi, uint16_t val){
    for (int i = 0; i < CHANNELS; i++) {
        logic.data = val & 1 ? oneMs : zeroMs;
        logic.order = i;
        val >>= 1;
        sr_session_send(sdi, &packet);
    }
}

static void sendSingleValue(const struct sr_dev_inst *sdi, struct Event* value, int64_t samples) {
    if (samples<0){
        sr_err("Samples are negative: %d", samples);
        return;
    }
    logic.length = spms / 8;
    uint64_t rem = samples;
    while(rem > spms) {
        sendToChannels(sdi, value->value);
        rem -= spms;
        waitingSamples -= spms;
    }
    uint64_t smpl = (rem / PKSZ) * PKSZ;
    logic.length = smpl / 8;
    sendToChannels(sdi, value->value);
    rem -= smpl;
    waitingSamples -= smpl;
    value->tick += samples - rem;
}


static void sendEvents(const struct sr_dev_inst *sdi) {
    struct Event* next = events.cur+1;
    if (next->tick - events.cur->tick >= PKSZ) {
        // Send non-switching probes
        sendSingleValue(sdi, events.cur, next->tick - events.cur->tick);
        if (events.cur->tick == next->tick){
            events.cur = next;
        }
        return;
    }
    // Send probes switch
    uint16_t value;
    uint64_t data[CHANNELS] = {0};
    for(int i = 0; i < PKSZ; i++) {
        events.cur->tick++;
        if (next <= events.last) {
            if (events.cur->tick >= next->tick) {
                events.cur++;
                next++;
            }
        }
        value = events.cur->value;
        for (int j = 0; j < CHANNELS; j++) {
            data[j] <<= 1;
            data[j] |= value & 1;
            value >>= 1;
        }
    }
    logic.length = PKSZ/8;
    for (int i=0; i < CHANNELS; i++) {
        logic.data = &data[i];
        logic.order = i;
        sr_session_send(sdi, &packet);
    }
    waitingSamples -= PKSZ;
}


int bpread_receive_data(int fd, int revents, const struct sr_dev_inst *sdi)
{
    if (!revents){
        if (!events.sz){
            return TRUE;
        }
        nodataCnt ++;
        // send remainder on nodata delay
        if (nodataCnt > 10 && nodataCnt * 100 * spms > waitingSamples) {
            nodataCnt = 0;
            sendSingleValue(sdi, &events.ev[0], waitingSamples);
        }
        return waitingSamples>0;
    }
    nodataCnt = 0;
    while (readEvents(fd)) {
        while(events.last->tick - events.cur->tick >= PKSZ) {
            //has data to send
            sendEvents(sdi);
        }
        events.sz = 0;
        for (struct Event* e=events.cur; e<=events.last; e++) {
            events.ev[events.sz++] = *e;
        }
    }
    return waitingSamples>0;
}



static void* msData(uint64_t packetSize, uint8_t value) {
    void* mem = malloc(packetSize);
    memset(mem, value, packetSize);
    return mem;
}

void bpread_start() {
    spms = bpread.samplerate / 1000;
    uint64_t packetSize =  spms / 8;

    packet.status = SR_PKT_OK;
    packet.type = SR_DF_LOGIC;
    packet.payload = &logic;
    logic.format = LA_SPLIT_DATA;
    logic.length = packetSize;

    waitingSamples = bpread.samples;

    zeroMs = msData(packetSize, 0);
    oneMs = msData(packetSize, 0xFF);
    buf.sz = 0;

    events.ev[0].tick = 0;
    events.sz = 0;
}

void bpread_stop() {
    if (zeroMs){
        free(zeroMs);
        free(oneMs);
        zeroMs = NULL;
        oneMs = NULL;
    }
}
