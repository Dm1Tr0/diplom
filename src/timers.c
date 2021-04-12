#include "timers.h"

volatile uint32_t t7_mask = 0;

volatile uint32_t __sys_timer_t = 0;
volatile uint32_t __t7_ticks = 0;

volatile uint32_t __sys_fhz = 0;
volatile uint32_t  __t7_fhz = 0;
// these two wariables are this is used to indicate the interrupt of the timers
volatile uint8_t sys_clk_flag = 0;
volatile uint8_t t7_clk_flag = 0;


inline sk_attr_inline_only uint32_t  get_sys_ticks(void)
{
    return __sys_timer_t;
}

sk_attr_inline_only uint32_t  get_t7_ticks(void)
{
    return __t7_ticks;
}

// user function for tim7
weak_attr void tim7_func(void)
{
    if(t7_mask & one_shot)
        timer_enable_counter(TIM7);
}
// user function for systick
weak_attr void sys_tick_func(void)
{

}

void tim7_isr(void)
{
    __t7_ticks++;
    t7_clk_flag = 1ul;
    tim7_func();
    timer_clear_flag(TIM7, TIM_SR_UIF);
}

void sys_tick_handler(void)
{
    __sys_timer_t++;
    sys_clk_flag = 1ul;
    sys_tick_func();

}

inline sk_attr_inline_only void __WFI(void)
{
	__asm__ volatile ("wfi");
}

void timer_7_init(enum t7_mode mode, uint32_t clockF, uint32_t period, uint32_t presk)
{
        timer_disable_counter(TIM7);
	// assume timer block is clocked @ 16 MHz
	// and we want 1 us tick time as delay function has 1 us resolution
        timer_set_prescaler(TIM7, presk - 1);
        timer_set_period(TIM7, period );

        __t7_fhz = (clockF / presk)/period;


        // enble preload if requierd
        if( mode & shadow_reg)
        timer_disable_preload(TIM7);
                // enavle
        if( mode & one_shot)
	        timer_one_shot_mode(TIM7);

        if( mode & upd_on_ovf) {
                timer_update_on_overflow(TIM7);
    }

	timer_enable_irq(TIM7, TIM_DIER_UIE);
	timer_clear_flag(TIM7, TIM_SR_UIF);		// just in case

	nvic_set_priority(NVIC_TIM7_IRQ, 2);
    nvic_enable_irq(NVIC_TIM7_IRQ);

    timer_enable_counter(TIM7);
}

void timer_sysT_init(uint32_t needfreq,uint32_t clk_sourse)
{
    systick_counter_disable();

    uint32_t F;

    if(clk_sourse == STK_CSR_CLKSOURCE_AHB)
        F = 16000000ul;
    else if(clk_sourse == STK_CSR_CLKSOURCE_AHB_DIV8)
        F = 16000000ul / 8ul;
    else
    {
        clk_sourse = STK_CSR_CLKSOURCE_AHB;
        F = 16000000ul;
    }

    systick_set_clocksource(clk_sourse);

    uint32_t period = F / needfreq;

	systick_set_reload(period);
    __sys_fhz = needfreq;
    // reload systick cur val
    STK_CVR = period;

    nvic_set_priority(NVIC_SYSTICK_IRQ, 1);
	nvic_enable_irq(NVIC_SYSTICK_IRQ);
    systick_interrupt_enable();
    systick_counter_enable();
}

void sys_delay_ms(uint32_t ms)
{
    //current walue of __sys_timer_t
    uint32_t temp = get_sys_ticks();
    //time to wait
    uint32_t time_to_count = (__sys_fhz / 1000) * ms;
    // current + time to count
    uint32_t time = time_to_count + temp;

    while(time >=  get_sys_ticks()) {
        sys_clk_flag = 0;
        while (sys_clk_flag == 0) {
        }
    }
}

void sys_delay_sec(uint32_t sec)
{
    uint32_t temp = get_sys_ticks();
    uint32_t time_to_count = __sys_fhz * sec;
    uint32_t time = time_to_count + temp;

    while(time >= get_sys_ticks()) {
        sys_clk_flag = 0;
        while (sys_clk_flag == 0) {
        }
    }
}

void t7_delay_ms(uint32_t ms)
{
    uint32_t temp = get_t7_ticks();
    uint32_t time_to_count = (__t7_fhz/ 1000) * ms;
    uint32_t time = time_to_count + temp;

    while(time >= get_t7_ticks()) {
        t7_clk_flag = 0;
        while (t7_clk_flag == 0) {
        }
    }
}

void t7_delay_sec(uint32_t sec)
{
    uint32_t temp = get_t7_ticks();
    uint32_t time_to_count = __t7_fhz * sec;
    uint32_t time = time_to_count + temp;

    while(time >= get_t7_ticks()) {
        t7_clk_flag = 0;
        while (t7_clk_flag == 0) {
        }
    }
}
