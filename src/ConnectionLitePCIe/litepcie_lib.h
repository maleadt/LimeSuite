/*
 * LitePCIe library
 *
 */
#ifndef LITEPCIE_LIB_H
#define LITEPCIE_LIB_H

#include <linux/types.h>

#define LITEPCIE_FILENAME "/dev/litepcie0"
#define DMA_CHANNEL_TX 1
#define DMA_CHANNEL_RX 2

void litepcie_dma_stop(int fd, uint8_t endpoint_nr, uint8_t channel);

struct litepcie_ioctl_dma_stop {
    __u32 channel;
    __u32 endpoint_nr;
};

struct litepcie_ioctl_reg_rw {
    __u32 adress;
    __u32 valWrite;
    __u32 valRead;
};

#define LITEPCIE_IOCTL 'S'

#define LITEPCIE_IOCTL_DMA_STOP         _IOW(LITEPCIE_IOCTL, 3, struct litepcie_ioctl_dma_stop)

#endif /* LITEPCIE_LIB_H */
