#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <sys/mman.h>

#include "dma.h"


// DMA address mapping by DMA number index
static const uint32_t dma_offset[] =
{
    DMA0_OFFSET,
    DMA1_OFFSET,
    DMA2_OFFSET,
    DMA3_OFFSET,
    DMA4_OFFSET,
    DMA5_OFFSET,
    DMA6_OFFSET,
    DMA7_OFFSET,
    DMA8_OFFSET,
    DMA9_OFFSET,
    DMA10_OFFSET,
    DMA11_OFFSET,
    DMA12_OFFSET,
    DMA13_OFFSET,
    DMA14_OFFSET,
    DMA15_OFFSET,
};


uint32_t dmanum_to_offset(int dmanum)
{
    int array_size = sizeof(dma_offset) / sizeof(dma_offset[0]);

    if (dmanum >= array_size)
    {
        return 0;
    }

    return dma_offset[dmanum];
}


