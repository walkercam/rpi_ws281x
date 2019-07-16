
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <time.h>

#include "mailbox.h"
#include "clk.h"
#include "gpio.h"
#include "dma.h"
#include "pwm.h"
#include "rpihw.h"

#include "ws2811.h"


#define BUS_TO_PHYS(x)                           ((x)&~0xC0000000)

#define OSC_FREQ                                 19200000   // crystal frequency

/* 4 colors (R, G, B + W), 8 bits per byte, 3 symbols per bit + 55uS low for reset signal */
#define LED_COLOURS                              4
#define LED_RESET_uS                             55
#define LED_BIT_COUNT(leds, freq)                ((leds * LED_COLOURS * 8 * 3) + ((LED_RESET_uS * \
                                                  (freq * 3)) / 1000000))

/* Minimum time to wait for reset to occur in microseconds. */
#define LED_RESET_WAIT_TIME                      300

// Pad out to the nearest uint32 + 32-bits for idle low/high times the number of channels
#define PWM_BYTE_COUNT(leds, freq)               ((((LED_BIT_COUNT(leds, freq) >> 3) & ~0x7) + 4) + 4)

// Symbol definitions
#define SYMBOL_HIGH                              0x6  // 1 1 0
#define SYMBOL_LOW                               0x4  // 1 0 0

// Driver mode definitions
#define PWM	1

// We use the mailbox interface to request memory from the VideoCore.
// This lets us request one physically contiguous chunk, find its
// physical address, and map it 'uncached' so that writes from this
// code are immediately visible to the DMA controller.  This struct
// holds data relevant to the mailbox interface.
typedef struct videocore_mbox {
    int handle;             /* From mbox_open() */
    unsigned mem_ref;       /* From mem_alloc() */
    unsigned bus_addr;      /* From mem_lock() */
    unsigned size;          /* Size of allocation */
    uint8_t *virt_addr;     /* From mapmem() */
} videocore_mbox_t;

typedef struct ws2811_device
{
    volatile uint8_t *pxl_raw;
    volatile dma_t *dma;
    volatile pwm_t *pwm;
    volatile dma_cb_t *dma_cb;
    uint32_t dma_cb_addr;
    volatile gpio_t *gpio;
    volatile cm_clk_t *cm_clk;
    videocore_mbox_t mbox;
} ws2811_device_t;

/**
 * Provides monotonic timestamp in microseconds.
 *
 * @returns  Current timestamp in microseconds or 0 on error.
 */
static uint64_t get_microsecond_timestamp()
{
    struct timespec t;

    if (clock_gettime(CLOCK_MONOTONIC_RAW, &t) != 0) {
        return 0;
    }

    return (uint64_t) t.tv_sec * 1000000 + t.tv_nsec / 1000;
}

/**
 * Map all devices into userspace memory.
 *
 * @param    ws2811  ws2811 instance pointer.
 *
 * @returns  0 on success, -1 otherwise.
 */
static int map_registers(ws2811_t *ws2811)
{
    ws2811_device_t *device = ws2811->device;
    const rpi_hw_t *rpi_hw = ws2811->rpi_hw;
    uint32_t base = ws2811->rpi_hw->periph_base;
    uint32_t dma_addr;
    uint32_t offset = 0;

    dma_addr = dmanum_to_offset(ws2811->dmanum);
    if (!dma_addr)
    {
        return -1;
    }
    dma_addr += rpi_hw->periph_base;

    device->dma = mapmem(dma_addr, sizeof(dma_t), DEV_MEM);
    if (!device->dma)
    {
        return -1;
    }

    device->pwm = mapmem(PWM_OFFSET + base, sizeof(pwm_t), DEV_MEM);
    if (!device->pwm)
    {
        return -1;
    }

    device->gpio = mapmem(GPIO_OFFSET + base, sizeof(gpio_t), DEV_MEM);
    if (!device->gpio)
    {
        return -1;
    }

    offset = (0x001010a0);

    device->cm_clk = mapmem(offset + base, sizeof(cm_clk_t), DEV_MEM);
    if (!device->cm_clk)
    {
        return -1;
    }

    return 0;
}

/**
 * Unmap all devices from virtual memory.
 *
 * @param    ws2811  ws2811 instance pointer.
 *
 * @returns  None
 */
static void unmap_registers(ws2811_t *ws2811)
{
    ws2811_device_t *device = ws2811->device;

    if (device->dma)
    {
        unmapmem((void *)device->dma, sizeof(dma_t));
    }

    if (device->pwm)
    {
        unmapmem((void *)device->pwm, sizeof(pwm_t));
    }

    if (device->cm_clk)
    {
        unmapmem((void *)device->cm_clk, sizeof(cm_clk_t));
    }

    if (device->gpio)
    {
        unmapmem((void *)device->gpio, sizeof(gpio_t));
    }
}

/**
 * Given a userspace address pointer, return the matching bus address used by DMA.
 *     Note: The bus address is not the same as the CPU physical address.
 *
 * @param    addr   Userspace virtual address pointer.
 *
 * @returns  Bus address for use by DMA.
 */
static uint32_t addr_to_bus(ws2811_device_t *device, const volatile void *virt)
{
    videocore_mbox_t *mbox = &device->mbox;

    uint32_t offset = (uint8_t *)virt - mbox->virt_addr;

    return mbox->bus_addr + offset;
}

/**
 * Stop the PWM controller.
 *
 * @param    ws2811  ws2811 instance pointer.
 *
 * @returns  None
 */
static void stop_pwm(ws2811_t *ws2811)
{
    ws2811_device_t *device = ws2811->device;
    volatile pwm_t *pwm = device->pwm;
    volatile cm_clk_t *cm_clk = device->cm_clk;

    // Turn off the PWM in case already running
    pwm->ctl = 0;
    usleep(10);

    // Kill the clock if it was already running
    cm_clk->ctl = CM_CLK_CTL_PASSWD | CM_CLK_CTL_KILL;
    usleep(10);
    while (cm_clk->ctl & CM_CLK_CTL_BUSY)
        ;
}


/**
 * Setup the PWM controller in serial mode on both channels using DMA to feed the PWM FIFO.
 *
 * @param    ws2811  ws2811 instance pointer.
 *
 * @returns  None
 */
static int setup_pwm(ws2811_t *ws2811)
{
    ws2811_device_t *device = ws2811->device;
    volatile dma_t *dma = device->dma;
    volatile dma_cb_t *dma_cb = device->dma_cb;
    volatile pwm_t *pwm = device->pwm;
    volatile cm_clk_t *cm_clk = device->cm_clk;
    int maxcount = ws2811->count;
    uint32_t freq = ws2811->freq;
    int32_t byte_count;
    uint32_t osc_freq = OSC_FREQ;

    stop_pwm(ws2811);

    // Setup the Clock - Use OSC @ 19.2Mhz w/ 3 clocks/tick
    cm_clk->div = CM_CLK_DIV_PASSWD | CM_CLK_DIV_DIVI(osc_freq / (3 * freq));
    cm_clk->ctl = CM_CLK_CTL_PASSWD | CM_CLK_CTL_SRC_OSC;
    cm_clk->ctl = CM_CLK_CTL_PASSWD | CM_CLK_CTL_SRC_OSC | CM_CLK_CTL_ENAB;
    usleep(10);
    while (!(cm_clk->ctl & CM_CLK_CTL_BUSY))
        ;

    // Setup the PWM, use delays as the block is rumored to lock up without them.  Make
    // sure to use a high enough priority to avoid any FIFO underruns, especially if
    // the CPU is busy doing lots of memory accesses, or another DMA controller is
    // busy.  The FIFO will clock out data at a much slower rate (2.6Mhz max), so
    // the odds of a DMA priority boost are extremely low.

    pwm->rng1 = 32;  // 32-bits per word to serialize
    usleep(10);
    pwm->ctl = RPI_PWM_CTL_CLRF1;
    usleep(10);
    pwm->dmac = RPI_PWM_DMAC_ENAB | RPI_PWM_DMAC_PANIC(7) | RPI_PWM_DMAC_DREQ(3);
    usleep(10);
    pwm->ctl = RPI_PWM_CTL_USEF1 | RPI_PWM_CTL_MODE1 |
               RPI_PWM_CTL_USEF2 | RPI_PWM_CTL_MODE2;
    if (ws2811->invert)
    {
        pwm->ctl |= RPI_PWM_CTL_POLA1;
    }
    usleep(10);
    pwm->ctl |= RPI_PWM_CTL_PWEN1 | RPI_PWM_CTL_PWEN2;

    // Initialize the DMA control block
    byte_count = PWM_BYTE_COUNT(maxcount, freq);
    dma_cb->ti = RPI_DMA_TI_NO_WIDE_BURSTS |  // 32-bit transfers
                 RPI_DMA_TI_WAIT_RESP |       // wait for write complete
                 RPI_DMA_TI_DEST_DREQ |       // user peripheral flow control
                 RPI_DMA_TI_PERMAP(5) |       // PWM peripheral
                 RPI_DMA_TI_SRC_INC;          // Increment src addr

    dma_cb->source_ad = addr_to_bus(device, device->pxl_raw);

    dma_cb->dest_ad = (uintptr_t)&((pwm_t *)PWM_PERIPH_PHYS)->fif1;
    dma_cb->txfr_len = byte_count;
    dma_cb->stride = 0;
    dma_cb->nextconbk = 0;

    dma->cs = 0;
    dma->txfr_len = 0;

    return 0;
}

/**
 * Start the DMA feeding the PWM FIFO.  This will stream the entire DMA buffer out of both
 * PWM channels.
 *
 * @param    ws2811  ws2811 instance pointer.
 *
 * @returns  None
 */
static void dma_start(ws2811_t *ws2811)
{
    ws2811_device_t *device = ws2811->device;
    volatile dma_t *dma = device->dma;
    uint32_t dma_cb_addr = device->dma_cb_addr;

    dma->cs = RPI_DMA_CS_RESET;
    usleep(10);

    dma->cs = RPI_DMA_CS_INT | RPI_DMA_CS_END;
    usleep(10);

    dma->conblk_ad = dma_cb_addr;
    dma->debug = 7; // clear debug error flags
    dma->cs = RPI_DMA_CS_WAIT_OUTSTANDING_WRITES |
              RPI_DMA_CS_PANIC_PRIORITY(15) |
              RPI_DMA_CS_PRIORITY(15) |
              RPI_DMA_CS_ACTIVE;
}

/**
 * Initialize the application selected GPIO pins for PWM operation.
 *
 * @param    ws2811  ws2811 instance pointer.
 *
 * @returns  0 on success, -1 on unsupported pin
 */
static int gpio_init(ws2811_t *ws2811)
{
    volatile gpio_t *gpio = ws2811->device->gpio;
    int altnum;

    altnum = pwm_pin_alt(ws2811->gpionum);

    gpio_function_set(gpio, ws2811->gpionum, altnum);

    return 0;
}

/**
 * Initialize the PWM DMA buffer with all zeros, inverted operation will be
 * handled by hardware.  The DMA buffer length is assumed to be a word
 * multiple.
 *
 * @param    ws2811  ws2811 instance pointer.
 *
 * @returns  None
 */
void pwm_raw_init(ws2811_t *ws2811)
{
    volatile uint32_t *pxl_raw = (uint32_t *)ws2811->device->pxl_raw;
    int wordcount = (PWM_BYTE_COUNT(ws2811->count, ws2811->freq) / sizeof(uint32_t));
    int wordpos = 0;

    for (int i = 0; i < wordcount; i++)
    {
        pxl_raw[wordpos] = 0x0;
        wordpos += 2;   //jump up in twos as even words are channel 0 and odd words are chan 1 (unused)
    }
}

/**
 * Cleanup previously allocated device memory and buffers.
 *
 * @param    ws2811  ws2811 instance pointer.
 *
 * @returns  None
 */
void ws2811_cleanup(ws2811_t *ws2811)
{
    ws2811_device_t *device = ws2811->device;

    if (ws2811->leds)
    {
        free(ws2811->leds);
    }
    ws2811->leds = NULL;

    if (device->mbox.handle != -1)
    {
        videocore_mbox_t *mbox = &device->mbox;

        unmapmem(mbox->virt_addr, mbox->size);
        mem_unlock(mbox->handle, mbox->mem_ref);
        mem_free(mbox->handle, mbox->mem_ref);
        mbox_close(mbox->handle);

        mbox->handle = -1;
    }

    if (device) {
        free(device);
    }
    ws2811->device = NULL;
}


/*
 *
 * Application API Functions
 *
 */


/**
 * Allocate and initialize memory, buffers, pages, PWM, DMA, and GPIO.
 *
 * @param    ws2811  ws2811 instance pointer.
 *
 * @returns  0 on success, -1 otherwise.
 */
ws2811_return_t ws2811_init(ws2811_t *ws2811)
{
    ws2811_device_t *device;
    const rpi_hw_t *rpi_hw;

    ws2811->rpi_hw = rpi_hw_detect();
    rpi_hw = ws2811->rpi_hw;

    ws2811->device = malloc(sizeof(*ws2811->device));
    memset(ws2811->device, 0, sizeof(*ws2811->device));
    device = ws2811->device;

    //only compatible with 3B and other similar boards
    //assumes gpionum is 18 and not set to a bad value
    //PWM MODE ONLY

    // Determine how much physical memory we need for DMA
    device->mbox.size = PWM_BYTE_COUNT(ws2811->count, ws2811->freq) + sizeof(dma_cb_t);
    // Round up to page size multiple
    device->mbox.size = (device->mbox.size + (PAGE_SIZE - 1)) & ~(PAGE_SIZE - 1);
    device->mbox.handle = mbox_open();
    device->mbox.mem_ref = mem_alloc(device->mbox.handle, device->mbox.size, PAGE_SIZE,
                                     rpi_hw->videocore_base == 0x40000000 ? 0xC : 0x4);
    device->mbox.bus_addr = mem_lock(device->mbox.handle, device->mbox.mem_ref);
    device->mbox.virt_addr = mapmem(BUS_TO_PHYS(device->mbox.bus_addr), device->mbox.size, DEV_MEM);

    // Initialize all pointers to NULL.  Any non-NULL pointers will be freed on cleanup.
    device->pxl_raw = NULL;
    device->dma_cb = NULL;
    ws2811->leds = NULL;

    // Allocate the LED buffers and set to zero
    ws2811->leds = malloc(sizeof(ws2811_led_t) * ws2811->count);
    memset(ws2811->leds, 0, sizeof(ws2811_led_t) * ws2811->count);

    if (!ws2811->strip_type)
    {
      ws2811->strip_type=WS2811_STRIP_RGB;
    }

    ws2811->wshift = (ws2811->strip_type >> 24) & 0xff;
    ws2811->rshift = (ws2811->strip_type >> 16) & 0xff;
    ws2811->gshift = (ws2811->strip_type >> 8)  & 0xff;
    ws2811->bshift = (ws2811->strip_type >> 0)  & 0xff;

    device->dma_cb = (dma_cb_t *)device->mbox.virt_addr;
    device->pxl_raw = (uint8_t *)device->mbox.virt_addr + sizeof(dma_cb_t);

    pwm_raw_init(ws2811);

    memset((dma_cb_t *)device->dma_cb, 0, sizeof(dma_cb_t));

    // Cache the DMA control block bus address
    device->dma_cb_addr = addr_to_bus(device, device->dma_cb);

    // Map the physical registers into userspace
    map_registers(ws2811);

    // Initialize the GPIO pins
    gpio_init(ws2811);

    // Setup the PWM, clocks, and DMA
    setup_pwm(ws2811);

    return WS2811_SUCCESS;
}

/**
 * Shut down DMA, PWM, and cleanup memory.
 *
 * @param    ws2811  ws2811 instance pointer.
 *
 * @returns  None
 */
void ws2811_fini(ws2811_t *ws2811)
{
    ws2811_wait(ws2811);
    stop_pwm(ws2811);
    unmap_registers(ws2811);
    ws2811_cleanup(ws2811);
}

/**
 * Wait for any executing DMA operation to complete before returning.
 *
 * @param    ws2811  ws2811 instance pointer.
 *
 * @returns  0 on success, -1 on DMA competion error
 */
ws2811_return_t ws2811_wait(ws2811_t *ws2811)
{
    volatile dma_t *dma = ws2811->device->dma;

    while ((dma->cs & RPI_DMA_CS_ACTIVE) &&
           !(dma->cs & RPI_DMA_CS_ERROR))
    {
        usleep(10);
    }

    if (dma->cs & RPI_DMA_CS_ERROR)
    {
        fprintf(stderr, "DMA Error: %08x\n", dma->debug);
        return WS2811_ERROR_DMA;
    }

    return WS2811_SUCCESS;
}

/**
 * Render the DMA buffer from the user supplied LED arrays and start the DMA
 * controller.  This will update all LEDs on both PWM channels.
 *
 * @param    ws2811  ws2811 instance pointer.
 *
 * @returns  None
 */
ws2811_return_t  ws2811_render(ws2811_t *ws2811)
{
    volatile uint8_t *pxl_raw = ws2811->device->pxl_raw;
    int bitpos;
    int i, k, l;
    unsigned j;
    ws2811_return_t ret = WS2811_SUCCESS;
    uint32_t protocol_time = 0;
    static uint64_t previous_timestamp = 0;

    bitpos = 31;

    int wordpos = 0; // PWM0
    const int scale = (ws2811->brightness & 0xff) + 1;
    
    uint8_t array_size = 3; // Assume 3 color LEDs, RGB
    // If our shift mask includes the highest nibble, then we have 4 LEDs, RBGW.
    if (ws2811->strip_type & SK6812_SHIFT_WMASK)
    {
        array_size = 4;
    }

    // 1.25µs per bit
    protocol_time = ws2811->count * array_size * 8 * 1.25;

    for (i = 0; i < ws2811->count; i++)                // Led
    {
        uint8_t color[] =
        {
            (((ws2811->leds[i] >> ws2811->rshift) & 0xff) * scale) >> 8, // red
            (((ws2811->leds[i] >> ws2811->gshift) & 0xff) * scale) >> 8, // green
            (((ws2811->leds[i] >> ws2811->bshift) & 0xff) * scale) >> 8, // blue
            (((ws2811->leds[i] >> ws2811->wshift) & 0xff) * scale) >> 8, // white
        };

        for (j = 0; j < array_size; j++)               // Color
        {
            for (k = 7; k >= 0; k--)                   // Bit
            {
                // Inversion is handled by hardware for PWM, otherwise by software here
                uint8_t symbol = SYMBOL_LOW;

                if (color[j] & (1 << k))
                {
                    symbol = SYMBOL_HIGH;
                }

                for (l = 2; l >= 0; l--)               // Symbol
                {
                    uint32_t *wordptr = &((uint32_t *)pxl_raw)[wordpos];   // PWM

                    *wordptr &= ~(1 << bitpos);
                    if (symbol & (1 << l))
                    {
                        *wordptr |= (1 << bitpos);
                    }

                    bitpos--;
                    if (bitpos < 0)
                    {
                        // Every other word is on the same channel for PWM
                        wordpos += 2;
                        bitpos = 31;
                    }
                }
            }
        }
    }

    // Wait for any previous DMA operation to complete.
    if ((ret = ws2811_wait(ws2811)) != WS2811_SUCCESS)
    {
        return ret;
    }
    if (ws2811->render_wait_time != 0) {
        const uint64_t current_timestamp = get_microsecond_timestamp();
        uint64_t time_diff = current_timestamp - previous_timestamp;

        if (ws2811->render_wait_time > time_diff) {
            usleep(ws2811->render_wait_time - time_diff);
        }
    }

    dma_start(ws2811);

    // LED_RESET_WAIT_TIME is added to allow enough time for the reset to occur.
    previous_timestamp = get_microsecond_timestamp();
    ws2811->render_wait_time = protocol_time + LED_RESET_WAIT_TIME;

    return ret;
}

const char * ws2811_get_return_t_str(const ws2811_return_t state)
{
    const int index = -state;
    static const char * const ret_state_str[] = { WS2811_RETURN_STATES(WS2811_RETURN_STATES_STRING) };

    if (index < (int)(sizeof(ret_state_str) / sizeof(ret_state_str[0])))
    {
        return ret_state_str[index];
    }

    return "";
}
