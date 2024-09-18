#include "sdkconfig.h"
#include "soc/rtc_cntl_reg.h"
#include "soc/rtc_io_reg.h"
#include "soc/soc_ulp.h"
#include "soc/sens_reg.h"
/*
 *  Enable GPIO_NUM16
 *  Read RTC GPIO 0-15. If one register has changed, wake up the main program
 */ 
#define SENSOR_POWER_PIN 16
/* Lower 16 IOs are for reading pulses ,
 * The higher IOs are for writing (enable power for sensor).
 */
/* set SENSOR_POWER_PIN  */

#define WRITE_GPIO_16(v) WRITE_RTC_REG(RTC_GPIO_IN_REG, RTC_GPIO_IN_NEXT_S + SENSOR_POWER_PIN,1, v)
/* Read the value of lower 16 RTC IOs into R0 */
#define READ_GPIO_0_15 READ_RTC_REG(RTC_GPIO_IN_REG, RTC_GPIO_IN_NEXT_S, 16)\



	/* Define variables, which go into .bss section (zero-initialized data) */
	.bss
	.global pin_mask
pin_mask:
	.long 0
	.global previous_low_registers
previous_low_registers:
	.long 0
	.global current_low_registers
current_low_registers:
	.long 0
	.global debug1
debug1:
	.long 0
	.global debug0
debug0:
	.long 0
	.global debug2
debug2:
	.long 0
	.global debug3
debug3:
	.long 0
    .global pulse_counters
pulse_counters:  
		/* 0-3 */
		.long 0
        .long 0
        .long 0
     	.long 0
		/* 4-7 */
        .long 0
		.long 0
        .long 0
     	.long 0
		/* 8-11 */
        .long 0
		.long 0
        .long 0
     	.long 0
		/* 12-15 */
        .long 0
		.long 0
        .long 0
     	.long 0
last_pulse_counter_cycle_count:  
		/* 0-3 */
		.long 0
        .long 0
        .long 0
     	.long 0
		/* 4-7 */
        .long 0
		.long 0
        .long 0
     	.long 0
		/* 8-11 */
        .long 0
		.long 0
        .long 0
     	.long 0
		/* 12-15 */
        .long 0
		.long 0
        .long 0
     	.long 0

	/* 125ns /cycle  8 cycles = 1ms debounce wait time: 80ms = 640 cycles */
	.set debounce_wait_cycles, 640
	/* 125ns /cycle  8 cycles = 1ms poll0 interval  : 1s = 8000 cycles */
	.set poll_cycles, 8000

	/* Code goes into .text section */
	.text
	.global entry
entry:
#if CONFIG_IDF_TARGET_ESP32S2
    /* ESP32S2 powers down RTC periph when entering deep sleep and thus by association SENS_SAR_IO_MUX_CONF_REG */
	WRITE_RTC_FIELD(SENS_SAR_IO_MUX_CONF_REG, SENS_IOMUX_CLK_GATE_EN, 1)
#elif CONFIG_IDF_TARGET_ESP32S3
    /* ESP32S3 powers down RTC periph when entering deep sleep and thus by association SENS_SAR_PERI_CLK_GATE_CONF_REG */
    WRITE_RTC_FIELD(SENS_SAR_PERI_CLK_GATE_CONF_REG, SENS_IOMUX_CLK_EN, 1);
#endif

loop:
	/* previous register content to r1 */
	WRITE_GPIO_16(1)
	WAIT 8000
	READ_GPIO_0_15
	move	r2, r0
	WRITE_GPIO_16(0)

	/* Previous result - current results == 0 > unchanged */
	move r3, pin_mask
	ld r0, r3,0
	move r3, previous_low_registers
	ld r1, r3, 0

	and		r2,r2,r0
	and		r1,r1,r0
	move r3, debug0
	st r2, r3, 0
	move r3, debug1
	st r1, r3, 0
	sub 	r0, r2, r1
	jump nothing_changed, EQ
	move r3, debug2
	st r0, r3, 0

	/* Some bit have changed */
	/* Wait for debouncing */
	WAIT debounce_wait_cycles
	WRITE_GPIO_16(1)
	WAIT 8000
	READ_GPIO_0_15
	move	r2, r0
	WRITE_GPIO_16(0)

	/* check again for changes after debouncing */
	move r3, pin_mask
	ld r0, r3,0
	and		r2,r2,r0
	and		r1,r1,r0
	sub 	r0, r2, r1
	jump nothing_changed, EQ
	/* store new register bitmap */
	move r3, current_low_registers	
	st r2, r3, 0
	move r3, debug3
	move r2, 0xFFFF
	st r2, r3, 0

	jump wake_up
nothing_changed:
	move r3, debug3
	move r2, 0x8888
	st r2, r3, 0

	halt:


