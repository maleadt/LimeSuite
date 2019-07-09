/*
 * LitePCIe library
 *
 */
#ifndef LITEPCIE_LIB_H
#define LITEPCIE_LIB_H

#include "litepcie.h"

#define LITEPCIE_FILENAME "/dev/litepcie0"

#define DMA_CHANNEL_TX 1
#define DMA_CHANNEL_RX 2

typedef struct
{
    uint16_t current;
    uint16_t offset;
    uint32_t buf_size;
    uint8_t* dma_buf;
} LitePCIeFIFO;

typedef struct {
    int litepcie_fd;
    LitePCIeFIFO rx_fifo[DMA_ENDPOINT_COUNT];
    LitePCIeFIFO tx_fifo[DMA_ENDPOINT_COUNT];
    struct litepcie_ioctl_mmap_info mmap_info;   
} LitePCIeState;

LitePCIeState *litepcie_open(const char *device_name);
void litepcie_close(LitePCIeState *s);
void litepcie_dma_start(LitePCIeState *s, int buf_size, uint8_t endpoint_nr, uint8_t channel);
void litepcie_dma_stop(LitePCIeState *s, uint8_t endpoint_nr, uint8_t channel);
int litepcie_writel(LitePCIeState *s, uint32_t addr, uint32_t val);
uint32_t litepcie_readl(LitePCIeState *s, uint32_t addr);
int litepcie_fifo_read(LitePCIeState *s, int ep, char* buf, const int count);
int litepcie_fifo_write(LitePCIeState *s, int ep, const char* buf, const int count);
int litepcie_fifo_flush(LitePCIeState *s, int ep);

#endif /* LITEPCIE_LIB_H */
