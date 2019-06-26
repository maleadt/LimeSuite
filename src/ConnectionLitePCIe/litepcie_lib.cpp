/*
 * LitePCIe library
 *
 */
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <time.h>
#include <errno.h>

#include "litepcie.h"
#include "cutils.h"
#include "config.h"
#include "csr.h"

#include "litepcie_lib.h"
#define DMA_CONTROL_BUF_SIZE 4096

static void* dma_rx_poll_thread(void *args);
int litepcie_fifo_write(LitePCIeState *s, int ep, const char* buf, const int count);
/*
  TODO:
  - DMA overflow/underflow detection
*/

void *litepcie_malloc(int size)
{
    return malloc(size);
}

void *litepcie_mallocz(int size)
{
    void *ptr;
    ptr = litepcie_malloc(size);
    if (!ptr)
        return NULL;
    memset(ptr, 0, size);
    return ptr;
}

void litepcie_free(void *ptr)
{
    free(ptr);
}

static pthread_t t1;

LitePCIeState *litepcie_open(const char *device_name)
{
    LitePCIeState *s;

    s = new LitePCIeState();
    if (!s)
        return NULL;

    s->litepcie_fd = open(device_name, O_RDWR);
    if (s->litepcie_fd < 0) {
        perror(device_name);
        goto fail;
    }

    /* map the DMA buffers */
    if (ioctl(s->litepcie_fd, LITEPCIE_IOCTL_GET_MMAP_INFO, &s->mmap_info) != 0) {
        perror("LITEPCIE_IOCTL_GET_MMAP_INFO");
        exit(1);
    }

    for(int i = 0; i < DMA_ENDPOINT_COUNT; i++){
        s->tx_fifo[i].dma_buf = (uint8_t*)mmap(NULL, s->mmap_info.dma_tx_buf_size *
                            s->mmap_info.dma_tx_buf_count,
                            PROT_READ | PROT_WRITE, MAP_SHARED, s->litepcie_fd,
                            s->mmap_info.dma_tx_buf_offset[i]);
        if (s->tx_fifo[i].dma_buf  == MAP_FAILED) {
            fprintf(stderr, "mmapTX_%d %s\n",i,strerror(errno));
            //perror("mmapTX_%d",i);
            exit(1);
        }

        s->rx_fifo[i].dma_buf = (uint8_t*)mmap(NULL, s->mmap_info.dma_rx_buf_size *
                            s->mmap_info.dma_rx_buf_count,
                            PROT_READ | PROT_WRITE, MAP_SHARED, s->litepcie_fd,
                            s->mmap_info.dma_rx_buf_offset[i]);
        if (s->rx_fifo[i].dma_buf == MAP_FAILED) {
            fprintf(stderr, "mmapRX_%d %s\n",i,strerror(errno));
            //perror("mmap2");
            exit(1);
        }
    }
    /* map the registers */
    s->reg_buf = (uint8_t*)mmap(NULL, s->mmap_info.reg_size,
                      PROT_READ | PROT_WRITE, MAP_SHARED, s->litepcie_fd,
                      s->mmap_info.reg_offset);
    if (s->reg_buf == MAP_FAILED) {
        perror("mmapREG");
        exit(1);
    }

    for(int i = 0; i < DMA_ENDPOINT_COUNT; i++){
        s->rx_fifo[i].dma_buf_size = s->mmap_info.dma_rx_buf_size;
        s->tx_fifo[i].dma_buf_size = s->mmap_info.dma_tx_buf_size;
    }

    for(int i = 0; i < DMA_ENDPOINT_COUNT; i++){
        pthread_mutex_init(&s->rx_fifo[i].lock, NULL);
        pthread_mutex_init(&s->tx_fifo[i].lock, NULL);
        s->rx_fifo[i].active = 0;
        s->tx_fifo[i].active = 0;
        s->rx_fifo[i].front = s->rx_fifo[i].back = 0;
        s->tx_fifo[i].front = s->tx_fifo[i].back = 0;
        s->rx_fifo[i].offset = 0;
        s->tx_fifo[i].offset = 0;
    }
    printf("Mapping was succesful!\n");
    pthread_create(&t1,NULL,dma_rx_poll_thread,s);
    //pthread_create(&t2,NULL,dma_tx_poll_thread,s); // don't need polling now
    return s;
 fail:
    litepcie_close(s);
    return NULL;
}

void litepcie_dma_start(LitePCIeState *s, int buf_size, int buf_count, bool is_loopback, uint8_t endpoint_nr, uint8_t channel)
{
    struct litepcie_ioctl_dma_start dma_start;

    if (buf_count > DMA_BUFFER_COUNT) {
        fprintf(stderr, "DMA start: unsupported buf_count\n");
        exit(1);
    }

    if (channel&DMA_CHANNEL_RX)
    {
        s->rx_fifo[endpoint_nr].buf_count = buf_count;
        s->rx_fifo[endpoint_nr].buf_size = buf_size;
        s->rx_fifo[endpoint_nr].active = 1;
        s->rx_fifo[endpoint_nr].front = s->rx_fifo[endpoint_nr].back = 0;
    }
    if (channel&DMA_CHANNEL_TX)
    {
        s->tx_fifo[endpoint_nr].buf_size = buf_size;
        s->tx_fifo[endpoint_nr].buf_count = buf_count;
        s->tx_fifo[endpoint_nr].active = 1;
        s->tx_fifo[endpoint_nr].front = s->tx_fifo[endpoint_nr].back = 0;
        s->tx_dma_wait[endpoint_nr].buf_num = 0;
        s->tx_dma_wait[endpoint_nr].timeout = 100;
        s->tx_dma_wait[endpoint_nr].tx_wait = true;
        s->tx_dma_wait[endpoint_nr].endpoint_nr = endpoint_nr;
    }

    dma_start.dma_flags = 0;
    if (is_loopback)
        dma_start.dma_flags |= DMA_LOOPBACK_ENABLE;
    dma_start.tx_buf_size = s->tx_fifo[endpoint_nr].buf_size;
    dma_start.tx_buf_count = s->tx_fifo[endpoint_nr].buf_count;
    dma_start.rx_buf_size = s->rx_fifo[endpoint_nr].buf_size;
    dma_start.rx_buf_count = s->rx_fifo[endpoint_nr].buf_count;
    dma_start.endpoint_nr = endpoint_nr;
    dma_start.channel = channel;

    if (ioctl(s->litepcie_fd, LITEPCIE_IOCTL_DMA_START, &dma_start) < 0) {
        perror("LITEPCIE_IOCTL_DMA_START");
    }
}

void litepcie_dma_start_all(LitePCIeState *s, int buf_size, int buf_count, bool is_loopback)
{
    struct litepcie_ioctl_dma_start dma_start;

    if (buf_count > DMA_BUFFER_COUNT) {
        fprintf(stderr, "DMA start: unsupported buf_count\n");
    }

    for(int i = 0; i < DMA_ENDPOINT_COUNT; i++)
    {
        s->tx_fifo[i].buf_size = s->rx_fifo[i].buf_size = buf_size;
        s->tx_fifo[i].buf_count = s->rx_fifo[i].buf_count = buf_count;
    }

    dma_start.dma_flags = 0;
    if (is_loopback)
        dma_start.dma_flags |= DMA_LOOPBACK_ENABLE;

    dma_start.rx_buf_size = s->rx_fifo->buf_size;
    dma_start.rx_buf_count = s->rx_fifo->buf_count;
    dma_start.tx_buf_size = s->tx_fifo->buf_size;
    dma_start.tx_buf_count = s->tx_fifo->buf_count;


    if (ioctl(s->litepcie_fd, LITEPCIE_IOCTL_DMA_START_ALL, &dma_start) < 0) {
        perror("LITEPCIE_IOCTL_DMA_START");
    }
}

void litepcie_dma_stop(LitePCIeState *s, uint8_t endpoint_nr, uint8_t channel)
{
    struct litepcie_ioctl_dma_stop dma_stop;
    dma_stop.channel = channel;
    dma_stop.endpoint_nr = endpoint_nr;
    if (channel&DMA_CHANNEL_RX)
        s->rx_fifo[endpoint_nr].active = 0;
    if (channel&DMA_CHANNEL_TX)
        s->tx_fifo[endpoint_nr].active = 0;
    if (ioctl(s->litepcie_fd, LITEPCIE_IOCTL_DMA_STOP, &dma_stop) < 0) {
        perror("LITEPCIE_IOCTL_DMA_STOP");
    }
}

void litepcie_dma_stop_all(LitePCIeState *s)
{
    if (ioctl(s->litepcie_fd, LITEPCIE_IOCTL_DMA_STOP_ALL, NULL) < 0) {
        perror("LITEPCIE_IOCTL_DMA_STOP_ALL");
    }
}

int litepcie_writel(LitePCIeState *s, uint32_t addr, uint32_t val)
{
    struct litepcie_ioctl_reg_rw reg_rw;
    reg_rw.adress = addr;
    reg_rw.valWrite = val;
    if (ioctl(s->litepcie_fd, LITEPCIE_IOCTL_REG_WRITE, &reg_rw) < 0)
        return -1;
    return 4;
}

uint32_t litepcie_readl(LitePCIeState *s, uint32_t addr)
{
    struct litepcie_ioctl_reg_rw reg_rw;
    reg_rw.adress = addr;
    if (ioctl(s->litepcie_fd, LITEPCIE_IOCTL_REG_READ, &reg_rw) < 0)
        return 0;
    return reg_rw.valRead;
}

void litepcie_close(LitePCIeState *s)
{
    //TODO unmap it Just wrap rx later
    //printf("UNMAPPING endpoints \n");
    for(int i = 0; i < DMA_ENDPOINT_COUNT; i++){
        if (s->tx_fifo[i].dma_buf) {
            munmap(s->tx_fifo[i].dma_buf , s->mmap_info.dma_tx_buf_size *
                s->mmap_info.dma_tx_buf_count);
        }

        if (s->rx_fifo[i].dma_buf) {
            munmap(s->rx_fifo[i].dma_buf, s->mmap_info.dma_rx_buf_size *
                s->mmap_info.dma_rx_buf_count);
        }
    }
    //printf("Unmapping reg buff\n");
    if (s->reg_buf)
        munmap(s->reg_buf, s->mmap_info.reg_size);
   // printf("closing FD\n");
    if (s->litepcie_fd >= 0)
        close(s->litepcie_fd);
    s->litepcie_fd = -1;
    pthread_join(t1, 0);
    delete s;
}


int litepcie_fifo_read(LitePCIeState *s, int ep, char* buf, const int count)
{
    LitePCIeFIFO* fifo = &s->rx_fifo[ep];
    int bytes_read = 0;
    while (count > bytes_read)
    {
        pthread_mutex_lock(&fifo->lock);
        if (fifo->front == fifo->back)
        {
            pthread_mutex_unlock(&fifo->lock);
            return bytes_read;
        }
        const uint8_t *rx_buf = fifo->dma_buf + fifo->front*fifo->dma_buf_size+fifo->offset;
        int n = fifo->buf_size-fifo->offset;
        if (n > count-bytes_read)
        {
            fifo->offset += count-bytes_read;
            pthread_mutex_unlock(&fifo->lock);
            memcpy(buf+bytes_read, rx_buf, count-bytes_read);
            return count;
        }
        fifo->offset = 0;
        fifo->front = add_mod_int(fifo->front, 1, fifo->buf_count);
        pthread_mutex_unlock(&fifo->lock);
        memcpy(buf+bytes_read, rx_buf, n);
        bytes_read += n;
    }
    return bytes_read;
}

int litepcie_fifo_write(LitePCIeState *s, int ep, const char* buf, const int count)
{
    LitePCIeFIFO* fifo = &s->tx_fifo[ep];
    int bytes_written = 0;
    while (count > bytes_written)
    {
        uint8_t *tx_buf = fifo->dma_buf + fifo->back*fifo->dma_buf_size+fifo->offset;
        int n = fifo->buf_size-fifo->offset;
        if (n > count-bytes_written)
        {
            fifo->offset += count-bytes_written;
            memcpy(tx_buf, buf+bytes_written, count-bytes_written);
            return count;
        }
        fifo->offset = 0;
        fifo->back = add_mod_int(fifo->back, 1, fifo->buf_count);
        //printf("front %d back %d\n", fifo->front, fifo->back);
        memcpy(tx_buf, buf+bytes_written, n);
        bytes_written += n;
    }
    return bytes_written;
}

static void* dma_rx_poll_thread(void *args)
{
    LitePCIeState *s = (LitePCIeState *) args;
    struct litepcie_ioctl_dma_wait dma_wait[DMA_ENDPOINT_COUNT];
    for (int i = 0; i < DMA_ENDPOINT_COUNT; i++)
    {
        dma_wait[i].timeout = 100; /* 0.1 second timeout */
        dma_wait[i].tx_wait = false;
        dma_wait[i].endpoint_nr = i;
        dma_wait[i].buf_num = 0;
    }

    while (s->litepcie_fd >= 0)
    {
        for (int i = 0; i < DMA_ENDPOINT_COUNT; i++)
        {
            LitePCIeFIFO* rx_fifo = &s->rx_fifo[i];
            if (!rx_fifo->active)
            {
                dma_wait[i].buf_num = 0;
                usleep(250);
                continue;
            }
            if (ioctl(s->litepcie_fd, LITEPCIE_IOCTL_DMA_WAIT, &dma_wait[i]) < 0)
                continue;
            //printf("rx buf_num: %d\n", dma_wait[i].buf_num);
            pthread_mutex_lock(&rx_fifo->lock);
            rx_fifo->back = add_mod_int(dma_wait[i].buf_num, 1, rx_fifo->buf_count);
            if (sub_mod_int(rx_fifo->back, rx_fifo->front, rx_fifo->buf_count) > rx_fifo->buf_count*2/3) //assume overflow
            {
               // printf("overflow\n");
                rx_fifo->offset = 0;
                rx_fifo->front = add_mod_int(rx_fifo->back, rx_fifo->buf_count*2/3, rx_fifo->buf_count);
            }
            pthread_mutex_unlock(&rx_fifo->lock);
        }
    }
    return 0;
}

int litepcie_send_data(LitePCIeState *s, int ep, const char* buf, const int count, const int timeout)
{

    int bytes_to_write = count - (count % (s->tx_fifo[ep].buf_size)); // can send only in buf_size amounts
    int las_buf_num = s->tx_dma_wait[ep].buf_num;
    int bytes_written = litepcie_fifo_write(s,ep,buf,bytes_to_write);
    s->tx_dma_wait[ep].buf_num += (bytes_written)/(s->tx_fifo[ep].buf_size) - 1;


    struct litepcie_ioctl_reg_rw reg;
    reg.adress = CSR_PCIE_DMA_READER_SCOUNTER_ADDR(ep);
    reg.valWrite = (uint16_t)(s->tx_dma_wait[ep].buf_num + 1);

    s->tx_dma_wait[ep].timeout = timeout;

    if(ioctl(s->litepcie_fd, LITEPCIE_IOCTL_REG_WRITE, &reg) < 0){
        printf("LITEPCIE_IOCTL_REG_WRITE Error\n");
        return 0;
    }

    if(ioctl(s->litepcie_fd,LITEPCIE_IOCTL_DMA_WAIT, &s->tx_dma_wait[ep]) < 0){
        //printf("LITPCIE_IOCTL TX WRITE TIMEOUT\n");
        printf("Last: %d, bytes_written: %d, buf_num: %d, val_write: %d\n",
        las_buf_num,bytes_written,s->tx_dma_wait[ep].buf_num, reg.valWrite);
        if(las_buf_num < s->tx_dma_wait[ep].buf_num)
            return (s->tx_dma_wait[ep].buf_num - las_buf_num)*s->tx_fifo[ep].buf_size;
        else
            return (las_buf_num - s->tx_dma_wait[ep].buf_num)*s->tx_fifo[ep].buf_size;
    }
    static int cnt = 0;
    if (++cnt%16==0)
        printf("dma_buf_num: %d, write %d, %d\n", s->tx_dma_wait[ep].buf_num, reg.valWrite, las_buf_num);
    return bytes_written;
}


