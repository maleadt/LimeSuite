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


void litepcie_dma_start(int fd, int buf_size, uint8_t endpoint_nr, uint8_t channel);
void litepcie_dma_stop(int fd, uint8_t endpoint_nr, uint8_t channel);
int litepcie_writel(int fd, uint32_t addr, uint32_t val);
uint32_t litepcie_readl(int fd, uint32_t addr);


struct litepcie_ioctl_dma_start {
    __u32 buf_size; /* in bytes, must be < dma_buf_pitch. 0 means no TX */
    __u32 channel; /* 1 = TX, 2 = RX, 3 = RX and TX */
    __u32 endpoint_nr; /* must be >=0 and < DMA_ENDPOINT_COUNT */
};

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

#define LITEPCIE_IOCTL_DMA_START        _IOW(LITEPCIE_IOCTL, 1, struct litepcie_ioctl_dma_start)
#define LITEPCIE_IOCTL_DMA_STOP         _IOW(LITEPCIE_IOCTL, 3, struct litepcie_ioctl_dma_stop)
#define LITEPCIE_IOCTL_REG_READ         _IOWR(LITEPCIE_IOCTL, 6, struct litepcie_ioctl_reg_rw)
#define LITEPCIE_IOCTL_REG_WRITE         _IOWR(LITEPCIE_IOCTL, 7, struct litepcie_ioctl_reg_rw)

#endif /* LITEPCIE_LIB_H */
