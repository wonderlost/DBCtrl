#include "advcan.h"

struct sockaddr_can addr;
struct ifreq ifr;
int canfd = -1;

void InitialCAN(const char *device, int baud_rate)
{
    int setflag, getflag, ret = 0;

    if ((canfd = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0)
    {
        perror("Error while openning socket");
        return;
    }

    strcpy(ifr.ifr_name, device);
    ioctl(canfd, SIOCGIFINDEX, &ifr);

    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(canfd, (struct sockaddr*)&addr, sizeof(addr)) < 0)
    {
        perror("Error in socket bind");
        return;
    }

    setflag = setflag | O_NONBLOCK;
    ret = fcntl(addr, F_SETFL, setflag);
    getflag = fcntl(addr, F_GETFL, 0);

    can_err_mask_t err_mask = CAN_ERR_TX_TIMEOUT | CAN_ERR_BUSOFF;
    ret = setsockop(canfd, SOL_CAN_RAW, CAN_RAW_ERR_FILTER, &err_mask, sizeof(err_mask));
    if (ret != 0)
        perror("set sock opt failed.");

    printf("%f open success.\n", device);
}


int WriteCAN(CanMsg write_frame)
{
    int ret = 0;
    struct can_frame frame;
    memset(&frame, 0, sizeof(can_frame));
    frame.can_dlc = write_frame.LEN;
    frame.can_id = write_frame.ID;
    for (int i = 0; i < frame.can_dlc; i++)
        frame.data[i] = write_frame.DATA[i];

    write(canfd, &frame, sizeof(can_frame));
    return ret;
}


int ReadCAN(CanMsg *read_frame)
{
    int ret = 0;
    struct can_frame frame;
    memset(&frame, 0, sizeof(can_frame));
    ret = read(canfd, &frame, sizeof(can_frame));
    if (ret > 0)
    {
        if (frame.can_id & CAN_ERR_FLAG)
            printf("Error Frame");
        else
        {
            read_frame->ID = frame.can_id;
            read_frame->LEN = frame.can_dlc;
            for (int i = 0; i < read_frame->LEN; i++)
                read_frame->DATA[i] = frame.data[i];
        }
    }
    return ret;
}
