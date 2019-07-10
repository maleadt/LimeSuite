/*
 * LitePCIe library
 */
#include <stdio.h>
#include <sys/ioctl.h>
#include <errno.h>
#include <inttypes.h>
#include "litepcie_lib.h"


void litepcie_dma_stop(int fd, uint8_t endpoint_nr, uint8_t channel)
{
    struct litepcie_ioctl_dma_stop dma_stop;
    dma_stop.channel = channel;
    dma_stop.endpoint_nr = endpoint_nr;
    if (ioctl(fd, LITEPCIE_IOCTL_DMA_STOP, &dma_stop) < 0) {
        perror("LITEPCIE_IOCTL_DMA_STOP");
    }
}
