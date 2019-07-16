#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <signal.h>
#include <stdarg.h>
#include <getopt.h>


#include "clk.h"
#include "gpio.h"
#include "dma.h"
#include "pwm.h"

#include "ws2811.h"

// defaults for cmdline options
#define TARGET_FREQ             WS2811_TARGET_FREQ
#define GPIO_PIN                18
#define DMA                     10
#define STRIP_TYPE            	WS2811_STRIP_GRB		// WS2812/SK6812RGB integrated chip+leds

#define LED_COUNT               16

int led_count = LED_COUNT;

ws2811_t ledstring =
{
    .freq = TARGET_FREQ,
    .dmanum = DMA,
    .gpionum = GPIO_PIN,
    .count = LED_COUNT,
    .invert = 0,
    .brightness = 255,
    .strip_type = STRIP_TYPE,
};

static uint8_t running = 1;

/*
    0x00200000,  // red
    0x00201000,  // orange
    0x00202000,  // yellow
    0x00002000,  // green
    0x00002020,  // lightblue
    0x00000020,  // blue
    0x00100010,  // purple
    0x00200010,  // pink
*/

static void ctrl_c_handler(int signum)
{
	(void)(signum);
	running = 0;
}

static void setup_handlers(void)
{
    struct sigaction sa =
    {
        .sa_handler = ctrl_c_handler,
    };

    sigaction(SIGINT, &sa, NULL);
    sigaction(SIGTERM, &sa, NULL);
}

int main()
{
    ws2811_return_t ret;

    setup_handlers();

    if ((ret = ws2811_init(&ledstring)) != WS2811_SUCCESS)
    {
        fprintf(stderr, "ws2811_init failed: %s\n", ws2811_get_return_t_str(ret));
        return ret;
    }

    while (running)
    {
	static int colour = 0x00100010;
	static int direction = 0;	//0 for down, 1 for up
	
	if(!direction)
	{
	    colour = colour - 0x00010001;
	}
	else
	{
	    colour = colour + 0x00010001;
	}
    
	if(!colour)
	{
	    direction = 1;
	}
	if(colour >= 0x00100010)
	{
	    direction = 0;
	}
	 
	
	
	for (int x = 0; x < led_count; x++)
	{
	    ledstring.leds[x] = colour;
	}

        if ((ret = ws2811_render(&ledstring)) != WS2811_SUCCESS)
        {
            fprintf(stderr, "ws2811_render failed: %s\n", ws2811_get_return_t_str(ret));
            break;
        }

        // 20 frames /sec
        usleep(1000000 / 20);
    }

    for (int x = 0; x < led_count; x++)
    {
        ledstring.leds[x] = 0;
    }

    ws2811_render(&ledstring);
    ws2811_fini(&ledstring);

    printf ("\n");
    return ret;
}
