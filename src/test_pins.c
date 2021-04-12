#include "sk_lib.h"
#include <libopencm3/stm32/rcc.h>

sk_pin TFT_DC     = { .port=sk_port_A, .pin=3, .inv=false, .pull=GPIO_PUPD_PULLUP, .mode=GPIO_MODE_OUTPUT};
sk_pin TFT_RESET  = { .port=sk_port_A, .pin=2, .inv=false, .pull=GPIO_PUPD_NONE, .mode=GPIO_MODE_OUTPUT};
// manualy initialized in spi_init
sk_pin TFT_CS     = { .port=sk_port_A, .pin=4, .inv=false, .pull=GPIO_PUPD_PULLUP, .mode=GPIO_MODE_OUTPUT};

int main (void)
{
	rcc_periph_clock_enable(RCC_GPIOA);
	sk_pin_mode(TFT_DC);
	sk_pin_set(TFT_DC, false);
	return 0;
}