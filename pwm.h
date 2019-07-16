
#ifndef __PWM_H__
#define __PWM_H__


/*
 *
 * Pin mappint of alternate pin configuration for PWM
 *
 * GPIO    ALT PWM0   ALT PWM1
 *
 *  12        0
 *  13                   0
 *  18        5
 *  19                   5
 *  40        0
 *  41                   0
 *  45                   0
 *  52        1
 *  53                   1
 *
 */


typedef struct
{
    uint32_t ctl;
#define RPI_PWM_CTL_MSEN2                        (1 << 15)
#define RPI_PWM_CTL_USEF2                        (1 << 13)
#define RPI_PWM_CTL_POLA2                        (1 << 12)
#define RPI_PWM_CTL_SBIT2                        (1 << 11)
#define RPI_PWM_CTL_RPTL2                        (1 << 10)
#define RPI_PWM_CTL_MODE2                        (1 << 9)
#define RPI_PWM_CTL_PWEN2                        (1 << 8)
#define RPI_PWM_CTL_MSEN1                        (1 << 7)
#define RPI_PWM_CTL_CLRF1                        (1 << 6)
#define RPI_PWM_CTL_USEF1                        (1 << 5)
#define RPI_PWM_CTL_POLA1                        (1 << 4)
#define RPI_PWM_CTL_SBIT1                        (1 << 3)
#define RPI_PWM_CTL_RPTL1                        (1 << 2)
#define RPI_PWM_CTL_MODE1                        (1 << 1)
#define RPI_PWM_CTL_PWEN1                        (1 << 0)
    uint32_t sta;
#define RPI_PWM_STA_STA4                         (1 << 12)
#define RPI_PWM_STA_STA3                         (1 << 11)
#define RPI_PWM_STA_STA2                         (1 << 10)
#define RPI_PWM_STA_STA1                         (1 << 9)
#define RPI_PWM_STA_BERR                         (1 << 8)
#define RPI_PWM_STA_GAP04                        (1 << 7)
#define RPI_PWM_STA_GAP03                        (1 << 6)
#define RPI_PWM_STA_GAP02                        (1 << 5)
#define RPI_PWM_STA_GAP01                        (1 << 4)
#define RPI_PWM_STA_RERR1                        (1 << 3)
#define RPI_PWM_STA_WERR1                        (1 << 2)
#define RPI_PWM_STA_EMPT1                        (1 << 1)
#define RPI_PWM_STA_FULL1                        (1 << 0)
    uint32_t dmac;
#define RPI_PWM_DMAC_ENAB                        (1 << 31)
#define RPI_PWM_DMAC_PANIC(val)                  ((val & 0xff) << 8)
#define RPI_PWM_DMAC_DREQ(val)                   ((val & 0xff) << 0)
    uint32_t resvd_0x0c;
    uint32_t rng1;
    uint32_t dat1;
    uint32_t fif1;
    uint32_t resvd_0x1c;
    uint32_t rng2;
    uint32_t dat2;
} __attribute__((packed, aligned(4))) pwm_t;


#define PWM_OFFSET                               (0x0020c000)
#define PWM_PERIPH_PHYS                          (0x7e20c000)


typedef struct
{
    int pinnum;
    int altnum;
} pwm_pin_table_t;

typedef struct
{
    const int count;
    const pwm_pin_table_t *pins;
} pwm_pin_tables_t;


int pwm_pin_alt(int pinnum);


#endif /* __PWM_H__ */
