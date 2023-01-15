#ifndef RTC_CTL_H
#define RTC_CTL_H

/* Copyright (c) 2015 Timothe Litt litt at acm ddot org
 * All rights reserved.
 *
 * This software is provided under GPL V2, including its disclaimer of
 * warranty.  Licensing under other terms may be available from the author.
 *
 * See the COPYING file for the well-known text of GPL V2.
 *
 * Bug reports, fixes, suggestions and improvements are welcome.
 *
 */

#ifdef __TIMESTAMP__
static const char rtc_ctl_h_version[] = __TIMESTAMP__;
#endif

/* Calibration data file
 */

#ifndef CALIB_FILENAME
#  define CALIB_FILENAME "/etc/rtc-ctl.dat"
#endif

/* DMB (data memory barrier) is available in ARMv7+.
 * RPi 2 has it, RPi 1 (A, B, B+) do not.
 */

#if !( defined(__ARM_ARCH_6__) && !defined( __ARM_ARCH_7__ ) )
#  define HAVE_DMB
#endif

/* Note: This header must be included after the library headers.
 *       __GLIBC__ is defined by including any library header.
 */

#if !defined( HAVE_TM_GMTOFF ) && defined( __GLIBC__ ) && defined( _BSD_SOURCE )
#  define HAVE_TM_GMTOFF 1
#endif

/* Check that clock is synchronized before setting TOY to system time.
 */

#ifndef CHECK_NTP
#  define CHECK_NTP 1
#endif

/* Check for NTP using the adjtimex() syscall rather than running ntpq.
 *
 *At the moment, 'TIME_OK' is returned even with NTP stoppped.
 * Need to investigate before turning this on.
 */

#ifndef HAVE_SYS_TIMEX
#define HAVE_SYS_TIMEX 0
#endif

#include <stdint.h>

#ifndef __GNUC__
    /* Probably pointless, given the __asm__ calls */
#  define __inline__ inline
#  define __attribute__(x)
#endif


#define TIMER_BASE_OFFSET  0x00003000
#define GPIO_BASE_OFFSET   0x00200000

typedef volatile struct _TIMER_REGS {
    volatile uint32_t cs;                       /* Control/status */
    volatile uint32_t clo;                      /* Counter low */
    volatile uint32_t chi;                      /* Counter high */
    volatile uint32_t compare[4];               /* Compare */
} TIMER_REGS;

typedef volatile struct _GPIO_REGS {
    volatile uint32_t fsel[6];                  /* Function select */
    volatile uint32_t rsvd_0;
    volatile uint32_t set[2];                   /* Set pin true */
    volatile uint32_t rsvd_1;
    volatile uint32_t clr[2];                   /* Set pin false */
    volatile uint32_t rsvd_2;
    volatile uint32_t level[2];                 /* Pin input */
    volatile uint32_t rsvd_3;
    volatile uint32_t evt[2];                   /* Event detect */
    volatile uint32_t rsvd_4;
    volatile uint32_t ren[2];                   /* Rising edge enable */
    volatile uint32_t rsvd_5;
    volatile uint32_t fen[2];                   /* Falling edge enable */
    volatile uint32_t rsvd_6;
    volatile uint32_t hen[2];                   /* High Detect enable */
    volatile uint32_t rsvd_7;
    volatile uint32_t len[2];                   /* Low detect enable */
    volatile uint32_t rsvd_8;
    volatile uint32_t aren[2];                  /* Async rising enable */
    volatile uint32_t rsvd_9;
    volatile uint32_t afen[2];                  /* Async falling enable */
    volatile uint32_t rsvd_10;
    volatile uint32_t pud;                      /* Pullup/down enable : N.B. Write-only */
    volatile uint32_t pudclk[2];                /* Pullup/down enable clock */
    volatile uint32_t rsvd_11;
} GPIO_REGS;

typedef enum { /* 3-bit fields, packed 10/uint32_t register */
    GPIO_FSEL_INPUT   = 0x0,    /* Input */
    GPIO_FSEL_OUTPUT  = 0x1,    /* Output */
    GPIO_FSEL_ALTFN0  = 0x4,    /* Alternate function 0 */
    GPIO_FSEL_ALTFN1  = 0x5,    /* Alternate function 1 */
    GPIO_FSEL_ALTFN2  = 0x6,    /* Alternate function 2 */
    GPIO_FSEL_ALTFN3  = 0x7,    /* Alternate function 3 */
    GPIO_FSEL_ALTFN4  = 0x3,    /* Alternate function 4 */
    GPIO_FSEL_ALTFN5  = 0x2,    /* Alternate function 5 */
    GPIO_FSEL_FNMASK  = 0x7     /* Function select bit mask */
} GPIO_FSEL;

typedef enum { /* PUD values. *ONE* 32-bit register, pudclk selects pins that get value */
    GPIO_PUD_DISABLE  = 0x00,   /* Disable pull-up/down */
    GPIO_PUD_DOWN     = 0x01,   /* Enable Pull Down control */
    GPIO_PUD_UP       = 0x02    /* Enable Pull Up control  */
} GPIO_PUDCTL;


/* GPIO Pin Numbers as connector pins (from bcm2835.h)
 *
 * RPi version 2 has some slightly different pinouts, and these are values V2_*.
 * RPi B+ has yet differnet pinouts and these are defined in BPLUS_*.
 * At bootup, pins 8 and 10 are set to UART0_TXD, UART0_RXD (ie the alt0 function) respectively
 *
 * If you are using the RPi Compute Module, use the GPIO number: these don't apply.
 */
typedef enum {
    GPIO_P1_03        =  0,  /* Version 1, Pin P1-03 */
    GPIO_P1_05        =  1,  /* Version 1, Pin P1-05 */
    GPIO_P1_07        =  4,  /* Version 1, Pin P1-07 */
    GPIO_P1_08        = 14,  /* Version 1, Pin P1-08, defaults to alt function 0 UART0_TXD */
    GPIO_P1_10        = 15,  /* Version 1, Pin P1-10, defaults to alt function 0 UART0_RXD */
    GPIO_P1_11        = 17,  /* Version 1, Pin P1-11 */
    GPIO_P1_12        = 18,  /* Version 1, Pin P1-12, can be PWM channel 0 in ALT FUN 5 */
    GPIO_P1_13        = 21,  /* Version 1, Pin P1-13 */
    GPIO_P1_15        = 22,  /* Version 1, Pin P1-15 */
    GPIO_P1_16        = 23,  /* Version 1, Pin P1-16 */
    GPIO_P1_18        = 24,  /* Version 1, Pin P1-18 */
    GPIO_P1_19        = 10,  /* Version 1, Pin P1-19, MOSI when SPI0 in use */
    GPIO_P1_21        =  9,  /* Version 1, Pin P1-21, MISO when SPI0 in use */
    GPIO_P1_22        = 25,  /* Version 1, Pin P1-22 */
    GPIO_P1_23        = 11,  /* Version 1, Pin P1-23, CLK when SPI0 in use */
    GPIO_P1_24        =  8,  /* Version 1, Pin P1-24, CE0 when SPI0 in use */
    GPIO_P1_26        =  7,  /* Version 1, Pin P1-26, CE1 when SPI0 in use */

    /* RPi Version 2 */
    V2_GPIO_P1_03     =  2,  /* Version 2, Pin P1-03 */
    V2_GPIO_P1_05     =  3,  /* Version 2, Pin P1-05 */
    V2_GPIO_P1_07     =  4,  /* Version 2, Pin P1-07 */
    V2_GPIO_P1_08     = 14,  /* Version 2, Pin P1-08, defaults to alt function 0 UART0_TXD */
    V2_GPIO_P1_10     = 15,  /* Version 2, Pin P1-10, defaults to alt function 0 UART0_RXD */
    V2_GPIO_P1_11     = 17,  /* Version 2, Pin P1-11 */
    V2_GPIO_P1_12     = 18,  /* Version 2, Pin P1-12, can be PWM channel 0 in ALT FUN 5 */
    V2_GPIO_P1_13     = 27,  /* Version 2, Pin P1-13 */
    V2_GPIO_P1_15     = 22,  /* Version 2, Pin P1-15 */
    V2_GPIO_P1_16     = 23,  /* Version 2, Pin P1-16 */
    V2_GPIO_P1_18     = 24,  /* Version 2, Pin P1-18 */
    V2_GPIO_P1_19     = 10,  /* Version 2, Pin P1-19, MOSI when SPI0 in use */
    V2_GPIO_P1_21     =  9,  /* Version 2, Pin P1-21, MISO when SPI0 in use */
    V2_GPIO_P1_22     = 25,  /* Version 2, Pin P1-22 */
    V2_GPIO_P1_23     = 11,  /* Version 2, Pin P1-23, CLK when SPI0 in use */
    V2_GPIO_P1_24     =  8,  /* Version 2, Pin P1-24, CE0 when SPI0 in use */
    V2_GPIO_P1_26     =  7,  /* Version 2, Pin P1-26, CE1 when SPI0 in use */

    /* RPi Version 2, new plug P5 */
    V2_GPIO_P5_03     = 28,  /* Version 2, Pin P5-03 */
    V2_GPIO_P5_04     = 29,  /* Version 2, Pin P5-04 */
    V2_GPIO_P5_05     = 30,  /* Version 2, Pin P5-05 */
    V2_GPIO_P5_06     = 31,  /* Version 2, Pin P5-06 */

    /* RPi B+ J8 header */
    BPLUS_GPIO_J8_03  =  2,  /* B+, Pin J8-03 */
    BPLUS_GPIO_J8_05  =  3,  /* B+, Pin J8-05 */
    BPLUS_GPIO_J8_07  =  4,  /* B+, Pin J8-07 */
    BPLUS_GPIO_J8_08  = 14,  /* B+, Pin J8-08, defaults to alt function 0 UART0_TXD */
    BPLUS_GPIO_J8_10  = 15,  /* B+, Pin J8-10, defaults to alt function 0 UART0_RXD */
    BPLUS_GPIO_J8_11  = 17,  /* B+, Pin J8-11 */
    BPLUS_GPIO_J8_12  = 18,  /* B+, Pin J8-12, can be PWM channel 0 in ALT FUN 5 */
    BPLUS_GPIO_J8_13  = 27,  /* B+, Pin J8-13 */
    BPLUS_GPIO_J8_15  = 22,  /* B+, Pin J8-15 */
    BPLUS_GPIO_J8_16  = 23,  /* B+, Pin J8-16 */
    BPLUS_GPIO_J8_18  = 24,  /* B+, Pin J8-18 */
    BPLUS_GPIO_J8_19  = 10,  /* B+, Pin J8-19, MOSI when SPI0 in use */
    BPLUS_GPIO_J8_21  =  9,  /* B+, Pin J8-21, MISO when SPI0 in use */
    BPLUS_GPIO_J8_22  = 25,  /* B+, Pin J8-22 */
    BPLUS_GPIO_J8_23  = 11,  /* B+, Pin J8-23, CLK when SPI0 in use */
    BPLUS_GPIO_J8_24  =  8,  /* B+, Pin J8-24, CE0 when SPI0 in use */
    BPLUS_GPIO_J8_26  =  7,  /* B+, Pin J8-26, CE1 when SPI0 in use */
    BPLUS_GPIO_J8_29  =  5,  /* B+, Pin J8-29,  */
    BPLUS_GPIO_J8_31  =  6,  /* B+, Pin J8-31,  */
    BPLUS_GPIO_J8_32  = 12,  /* B+, Pin J8-32,  */
    BPLUS_GPIO_J8_33  = 13,  /* B+, Pin J8-33,  */
    BPLUS_GPIO_J8_35  = 19,  /* B+, Pin J8-35,  */
    BPLUS_GPIO_J8_36  = 16,  /* B+, Pin J8-36,  */
    BPLUS_GPIO_J8_37  = 26,  /* B+, Pin J8-37,  */
    BPLUS_GPIO_J8_38  = 20,  /* B+, Pin J8-38,  */
    BPLUS_GPIO_J8_40  = 21   /* B+, Pin J8-40,  */
} RPiGPIOPin;

/* Memory barrier. */

extern __inline__  __attribute__((always_inline)) void MB( void ) {
#ifdef HAVE_DMB
	__asm__( "dmb" : : : "memory" );
#else
	__asm__(              "\
  mov r10,#0                 \n\
  mcr p15,0,r10, c7, c10, 5  \n\
  " : : : "r10", "memory" );
#endif
	return;
}

/* Read register (with memory barriers)
 */

extern __inline__  __attribute__((always_inline)) uint32_t RDREG( volatile uint32_t *paddr ) {
    uint32_t ret;

#ifdef HAVE_DMB
	__asm__(        "\
  dmb                    \
  ldr %[ret], [%[paddr]] \
  dmb                    \
" : [ret] "=r" (ret) : [paddr] "r" (paddr) : "memory" );
#else
	__asm__(            "\
  mov r10,#0               \n\
  mcr p15,0,r10, c7, c10, 5\n\
  ldr %[ret], [%[paddr]]   \n\
  mcr p15,0,r10, c7, c10, 5\n\
" : [ret] "=r" (ret) : [paddr] "r" (paddr) : "r10", "memory" );
#endif
	return ret;
}

/* Write register (with memory barriers) */

extern __inline__  __attribute__((always_inline)) void WRREG( volatile uint32_t *paddr, uint32_t value ) {
#ifdef HAVE_DMB
	__asm__(          "\
  dmb                      \
  str %[value], [%[paddr]] \
  dmb                      \
" : : [paddr] "r" (paddr), [value] "r" (value) : "memory" );
#else
	__asm__(            "\
  mov r10,#0               \n\
  mcr p15,0,r10, c7, c10, 5\n\
  str %[value], [%[paddr]] \n\
  mcr p15,0,r10, c7, c10, 5\n\
" : : [paddr] "r" (paddr), [value] "r" (value) : "r10", "memory" );
#endif
	return;
}

/* GPIO data manipulation.
 *
 * SET - asserts pin
 * CLR - deasserts pin
 * SET_TO - set to a computed value
 * IS_SET - true if pin is asserted
 *
 * These macros all have implicit MBs, are atomic, and should inline..
 */

#define GPIO_SET( pin ) WRREG( &gpio->set[((pin) & 0x20)? 1: 0], (1 << ((pin) & 0x1F)) )
#define GPIO_CLR( pin ) WRREG( &gpio->clr[((pin) & 0x20)? 1: 0], (1 << ((pin) & 0x1F)) )
#define GPIO_SET_TO( pin, value ) do { \
	if( value )                    \
	    GPIO_SET( pin );	       \
	else                           \
	    GPIO_CLR( pin ); } while( 0 )
#define GPIO_IS_SET( pin ) (RDREG( &gpio->level[((pin) & 0x20)? 1: 0] ) & (1 << ((pin) & 0x1F)))

/* Macros where user is responsible for barriers.
 * There are a bunch of rules with varying degrees of subtlety.
 * Basically, the IO Peripherals will return reads to different peripherals out of order, 
 * and the CPU doesn't handle it.  Same for write acks.  So barriers are required whenever
 * switching from one peripheral to another.  This includes interrupts/exceptions.
 * The builtins conservatively put a barrier before each load/store, assuming that they
 * are interrupting/switching from some other stream, and one after to protect the
 * interrupted/next stream.  Direct access doesn't provide any barriers.  Note that the
 * system timer and the GPIO pins are two different peripherals.
 */

#define NB_GPIO_SET( pin ) do { \
	gpio->set[((pin) & 0x20)? 1: 0] = 1 << ((pin) & 0x1F);	\
    } while( 0 )
#define NB_GPIO_CLR( pin ) do { \
	gpio->clr[((pin) & 0x20)? 1: 0] = 1 << ((pin) & 0x1F);	\
    } while( 0 )
#define NB_GPIO_SET_TO( pin, value ) do { \
	if( value )                       \
	    NB_GPIO_SET( pin );           \
	else                              \
	    NB_GPIO_CLR( pin );           \
    } while( 0 )
#define NB_GPIO_IS_SET( pin ) (gpio->level[((pin) & 0x20)? 1: 0] & (1 << ((pin) & 0x1F)))

#endif
