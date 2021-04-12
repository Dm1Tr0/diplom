#ifndef TIMERS
#define TIMERS

#include "macro.h"
#include <libopencm3/cm3/cortex.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/cm3/sync.h>
#include <libopencm3/cm3/systick.h>
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>

enum t7_mode {
    shadow_reg = 1,
    one_shot = 2,
    upd_on_ovf = 4,
};


inline sk_attr_inline_only uint32_t get_sys_ticks(void);

inline uint32_t sk_attr_inline_only get_t7_ticks(void);

weak_attr void tim7_func(void);

weak_attr void sys_tick_func(void);

inline sk_attr_inline_only void __WFI(void);

void timer_7_init(enum t7_mode mode, uint32_t clockF, uint32_t period, uint32_t presk);

void timer_sysT_init(uint32_t needfreq,uint32_t clk_sourse);

void sys_delay_ms(uint32_t ms);

void sys_delay_sec(uint32_t sec);

void t7_delay_ms(uint32_t ms);

void t7_delay_sec(uint32_t sec);

#endif
