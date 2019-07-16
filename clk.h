#ifndef __CLK_H__
#define __CLK_H__


typedef struct {
    uint32_t ctl;
#define CM_CLK_CTL_PASSWD                        (0x5a << 24)
#define CM_CLK_CTL_MASH(val)                     ((val & 0x3) << 9)
#define CM_CLK_CTL_FLIP                          (1 << 8)
#define CM_CLK_CTL_BUSY                          (1 << 7)
#define CM_CLK_CTL_KILL                          (1 << 5)
#define CM_CLK_CTL_ENAB                          (1 << 4)
#define CM_CLK_CTL_SRC_GND                       (0 << 0)
#define CM_CLK_CTL_SRC_OSC                       (1 << 0)
#define CM_CLK_CTL_SRC_TSTDBG0                   (2 << 0)
#define CM_CLK_CTL_SRC_TSTDBG1                   (3 << 0)
#define CM_CLK_CTL_SRC_PLLA                      (4 << 0)
#define CM_CLK_CTL_SRC_PLLC                      (5 << 0)
#define CM_CLK_CTL_SRC_PLLD                      (6 << 0)
#define CM_CLK_CTL_SRC_HDMIAUX                   (7 << 0)
    uint32_t div;
#define CM_CLK_DIV_PASSWD                        (0x5a << 24)
#define CM_CLK_DIV_DIVI(val)                     ((val & 0xfff) << 12)
#define CM_CLK_DIV_DIVF(val)                     ((val & 0xfff) << 0)
} __attribute__((packed, aligned(4))) cm_clk_t;



#endif /* __CLK_H__ */
