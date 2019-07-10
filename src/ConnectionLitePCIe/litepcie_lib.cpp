/*
 * LitePCIe library
 */
#include <stdio.h>
#include <sys/ioctl.h>
#include <errno.h>
#include <inttypes.h>
#include "litepcie_lib.h"


void litepcie_dma_start(int fd, int buf_size, uint8_t endpoint_nr, uint8_t channel)
{
    struct litepcie_ioctl_dma_start dma_start;

    dma_start.buf_size = buf_size;
    dma_start.endpoint_nr = endpoint_nr;
    dma_start.channel = channel;

    if (ioctl(fd, LITEPCIE_IOCTL_DMA_START, &dma_start) < 0) {
        perror("LITEPCIE_IOCTL_DMA_START");
    }
}

void litepcie_dma_stop(int fd, uint8_t endpoint_nr, uint8_t channel)
{
    struct litepcie_ioctl_dma_stop dma_stop;
    dma_stop.channel = channel;
    dma_stop.endpoint_nr = endpoint_nr;
    if (ioctl(fd, LITEPCIE_IOCTL_DMA_STOP, &dma_stop) < 0) {
        perror("LITEPCIE_IOCTL_DMA_STOP");
    }
}

int litepcie_writel(int fd, uint32_t addr, uint32_t val)
{
    struct litepcie_ioctl_reg_rw reg_rw;
    reg_rw.adress = addr;
    reg_rw.valWrite = val;
    if (ioctl(fd, LITEPCIE_IOCTL_REG_WRITE, &reg_rw) < 0)
        return -1;
    return 4;
}

uint32_t litepcie_readl(int fd, uint32_t addr)
{
    struct litepcie_ioctl_reg_rw reg_rw;
    reg_rw.adress = addr;
    if (ioctl(fd, LITEPCIE_IOCTL_REG_READ, &reg_rw) < 0)
        return 0;
    return reg_rw.valRead;
}