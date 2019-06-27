/*
 * LitePCIe library
 *
 */
#ifndef LITEPCIE_LIB_H
#define LITEPCIE_LIB_H

#include <stdarg.h>
#include <pthread.h>
#include "litepcie.h"

#define LITEPCIE_FILENAME "/dev/litepcie0"

#define DMA_LOOPBACK_ENABLE 0x1

#define DMA_TABLE_LOOP_INDEX 1 << 0
#define DMA_TABLE_LOOP_COUNT 1 << 16

#define DMA_CHANNEL_TX 1
#define DMA_CHANNEL_RX 2
#define DMA_CHANNEL_TXRX 3

typedef struct
{
    uint16_t front;
    uint16_t back;
    int offset;
    int buf_size;
    int buf_count;
    int dma_buf_size;
    uint8_t* dma_buf;
    int active;
} LitePCIeFIFO;

typedef struct {
    int litepcie_fd;
    struct litepcie_ioctl_mmap_info mmap_info;
    struct litepcie_ioctl_dma_wait tx_dma_wait[DMA_BUFFER_COUNT];
    struct litepcie_ioctl_dma_wait rx_dma_wait[DMA_BUFFER_COUNT];
    uint8_t *reg_buf;
    
    LitePCIeFIFO rx_fifo[DMA_ENDPOINT_COUNT];
    LitePCIeFIFO tx_fifo[DMA_ENDPOINT_COUNT];
} LitePCIeState;

void *litepcie_malloc(int size);
void *litepcie_mallocz(int size);
void litepcie_free(void *ptr);
LitePCIeState *litepcie_open(const char *device_name);
void litepcie_close(LitePCIeState *s);
void litepcie_dma_start(LitePCIeState *s, int buf_size, int buf_count, bool is_loopback, uint8_t endpoint_nr, uint8_t channel);
void litepcie_dma_stop(LitePCIeState *s, uint8_t endpoint_nr, uint8_t channel);
int litepcie_writel(LitePCIeState *s, uint32_t addr, uint32_t val);
uint32_t litepcie_readl(LitePCIeState *s, uint32_t addr);
int litepcie_fifo_read(LitePCIeState *s, int ep, char* buf, const int count);
int litepcie_fifo_write(LitePCIeState *s, int ep, const char* buf, const int count);

#endif /* LITEPCIE_LIB_H */
