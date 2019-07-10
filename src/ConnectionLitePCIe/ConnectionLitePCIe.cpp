#include "ConnectionLitePCIe.h"
#include <unistd.h>
#include <fcntl.h>
#include <atomic>
#include "Logger.h"
#include "dataTypes.h"
#include <sys/ioctl.h>

using namespace std;

using namespace lime;

static const char* ep_names[] = { "/dev/litepcie1",  "/dev/litepcie2",  "/dev/litepcie3"};


#define DMA_CHANNEL_TX 1
#define DMA_CHANNEL_RX 2

struct litepcie_ioctl_dma_stop {
    uint32_t channel;
    uint32_t endpoint_nr;
};

#define LITEPCIE_IOCTL 'S'
#define LITEPCIE_IOCTL_DMA_STOP         _IOW(LITEPCIE_IOCTL, 3, struct litepcie_ioctl_dma_stop)

ConnectionLitePCIe::ConnectionLitePCIe(const unsigned) :
    isConnected(true)
{
    control_fd = open(LITEPCIE_FILENAME, O_RDWR);
    if (control_fd<0)
    {
        isConnected = false;
        lime::error("Failed to open Lite PCIe");
        return;
    }
    for (int i = 0; i < MAX_EP_CNT; i++)
        ep_fd[i] = open(ep_names[i], O_RDWR);
}

ConnectionLitePCIe::~ConnectionLitePCIe()
{
    if (control_fd >=0)
        close(control_fd);
}

bool ConnectionLitePCIe::IsOpen()
{
    return isConnected;
}

int ConnectionLitePCIe::Write(const unsigned char *buffer, const int length, int /*timeout_ms*/)
{
    return write(control_fd, buffer, length);
}

int ConnectionLitePCIe::Read(unsigned char *buffer, const int length, int timeout_ms)
{
    uint32_t status = 0;
    auto t1 = chrono::high_resolution_clock::now();
    do
    {   //wait for status byte to change
        read(control_fd, &status, sizeof(status));
        if ((status&0xFF00) != 0)
            break;
        std::this_thread::sleep_for(std::chrono::microseconds(250));
    }
    while (std::chrono::duration_cast<std::chrono::milliseconds>(chrono::high_resolution_clock::now() - t1).count() < timeout_ms);

    if ((status&0xFF00)== 0)
        return 0;   //timeout
    return read(control_fd, buffer, length);
}

int ConnectionLitePCIe::GetBuffersCount() const
{
    return 1;
}

int ConnectionLitePCIe::CheckStreamSize(int size) const
{
    return size;
}

int ConnectionLitePCIe::ResetStreamBuffers()
{
    for (int i = 0; i < MAX_EP_CNT; i++)
        if (ep_fd[i] >= 0)
        {
            struct litepcie_ioctl_dma_stop dma_stop;
            dma_stop.channel = DMA_CHANNEL_TX | DMA_CHANNEL_RX;
            dma_stop.endpoint_nr = i;
            if (ioctl(ep_fd[i], LITEPCIE_IOCTL_DMA_STOP, &dma_stop) < 0) {
                lime::error("LITEPCIE_IOCTL_DMA_STOP failed");
            }
        }
    return 0;
}

int ConnectionLitePCIe::ReceiveData(char *buffer, int length, int epIndex, int timeout_ms)
{
    int totalBytesReaded = 0;
    int bytesToRead = length;
    auto t1 = chrono::high_resolution_clock::now();

    do
    {
        int bytesReceived = read(ep_fd[epIndex], buffer+totalBytesReaded, length-totalBytesReaded);
        if (bytesReceived == 0)
        {
            std::this_thread::sleep_for(std::chrono::microseconds(100));
            continue;
        }
        totalBytesReaded += bytesReceived;
        if (totalBytesReaded < length)
            bytesToRead -= bytesReceived;
        else
            break;
    }while (std::chrono::duration_cast<std::chrono::milliseconds>(chrono::high_resolution_clock::now() - t1).count() < timeout_ms);

    return totalBytesReaded;
}

void ConnectionLitePCIe::AbortReading(int epIndex)
{
    if (ep_fd[epIndex] >= 0)
    {
        struct litepcie_ioctl_dma_stop dma_stop;
        dma_stop.channel = DMA_CHANNEL_RX;
        dma_stop.endpoint_nr = epIndex;
        if (ioctl(ep_fd[epIndex], LITEPCIE_IOCTL_DMA_STOP, &dma_stop) < 0) {
            lime::error("LITEPCIE_IOCTL_DMA_STOP failed");
        }
    }
}

int ConnectionLitePCIe::SendData(const char *buffer, int length, int epIndex, int timeout_ms)
{
    int totalBytesSent = 0;
    int bytesToSend = length;
    auto t1 = chrono::high_resolution_clock::now();
    do
    {
        int bytesSent = write(ep_fd[epIndex], buffer+totalBytesSent, length-totalBytesSent);
        if (bytesSent == 0)
        {
            std::this_thread::sleep_for(std::chrono::microseconds(500));
            continue;
        }
        totalBytesSent += bytesSent;
        if (totalBytesSent < length)
            bytesToSend -= bytesSent;
        else
            break;
    }while (std::chrono::duration_cast<std::chrono::milliseconds>(chrono::high_resolution_clock::now() - t1).count() < timeout_ms);
    return totalBytesSent;
}

void ConnectionLitePCIe::AbortSending(int epIndex)
{
    if (ep_fd[epIndex] >= 0)
    {
        struct litepcie_ioctl_dma_stop dma_stop;
        dma_stop.channel = DMA_CHANNEL_TX;
        dma_stop.endpoint_nr = epIndex;
        if (ioctl(ep_fd[epIndex], LITEPCIE_IOCTL_DMA_STOP, &dma_stop) < 0) {
            lime::error("LITEPCIE_IOCTL_DMA_STOP failed");
        }
    }
}

int ConnectionLitePCIe::BeginDataReading(char* buffer, uint32_t length, int ep)
{
    return ep;
}

bool ConnectionLitePCIe::WaitForReading(int contextHandle, unsigned int timeout_ms)
{
    return true;
}

int ConnectionLitePCIe::FinishDataReading(char* buffer, uint32_t length, int contextHandle)
{
    return ReceiveData(buffer, length, contextHandle, 3000);
}

int ConnectionLitePCIe::BeginDataSending(const char* buffer, uint32_t length, int ep)
{
    return SendData(buffer, length,  ep, 3000);
}

bool ConnectionLitePCIe::WaitForSending(int contextHandle, uint32_t timeout_ms)
{
    return true;
}

int ConnectionLitePCIe::FinishDataSending(const char* buffer, uint32_t length, int contextHandle)
{
    return contextHandle;
}
