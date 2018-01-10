#ifndef ADVCAN_H_
#define ADVCAN_H_

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <net/if.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <linux/can/error.h>
#include <fcntl.h>

typedef struct tagCanMsg
{
    int ID;
    int INFO;
    int LEN;
    unsigned char DATA[8];
}CanMsg;

void InitialCAN(const char *device, int baud_rate);
int WriteCAN(CanMsg write_frame);
int ReadCAN(CanMsg *read_frame);


#endif
