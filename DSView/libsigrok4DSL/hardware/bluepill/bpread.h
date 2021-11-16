//
//  bpdev.h
//  sigrok4DSL
//
//  Created by john on 14.11.2021.
//

#ifndef bpread_h
#define bpread_h

#include "libsigrok.h"
#include "libsigrok-internal.h"

struct BPRead {
    uint64_t samplerate;
    uint64_t samples;
    uint64_t msec;
    int fd;
};

void bpread_start(void);
void bpread_stop(void);
int bpread_receive_data(int fd, int revents, const struct sr_dev_inst *sdi);

extern struct BPRead bpread;
#endif /* bpread_h */
