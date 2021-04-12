#include "sk_lib.h"

#if defined(SK_USE_GLSK_DEFINITIONS) && SK_USE_GLSK_DEFINITIONS
// some STM32F4DISCOVERY pins
const sk_pin sk_io_led_orange = { .port=sk_port_D, .pin=13, .inv=false, .pull=GPIO_PUPD_NONE, .mode=GPIO_MODE_OUTPUT};
const sk_pin sk_io_led_red    = { .port=sk_port_D, .pin=14, .inv=false, .pull=GPIO_PUPD_NONE, .mode=GPIO_MODE_OUTPUT};
const sk_pin sk_io_led_green  = { .port=sk_port_D, .pin=12, .inv=false, .pull=GPIO_PUPD_NONE, .mode=GPIO_MODE_OUTPUT};
const sk_pin sk_io_led_blue   = { .port=sk_port_D, .pin=15, .inv=false, .pull=GPIO_PUPD_NONE, .mode=GPIO_MODE_OUTPUT};
const sk_pin sk_io_btn_usr    = { .port=sk_port_A, .pin=0,  .inv=false, .pull=GPIO_PUPD_NONE, .mode=GPIO_MODE_INPUT};
// some GL-SK pins yet to set
const sk_pin sk_io_btn_right  = { .port=sk_port_C, .pin=11, .inv=true, .pull=GPIO_PUPD_PULLUP, .mode=GPIO_MODE_INPUT};
const sk_pin sk_io_btn_mid    = { .port=sk_port_A, .pin=15, .inv=true, .pull=GPIO_PUPD_PULLUP, .mode=GPIO_MODE_INPUT};
const sk_pin sk_io_btn_left   = { .port=sk_port_C, .pin=9,  .inv=true, .pull=GPIO_PUPD_PULLUP, .mode=GPIO_MODE_INPUT};
const sk_pin sk_io_btn_up     = { .port=sk_port_C, .pin=6,  .inv=true, .pull=GPIO_PUPD_PULLUP, .mode=GPIO_MODE_INPUT};
const sk_pin sk_io_btn_down   = { .port=sk_port_C, .pin=8,  .inv=true, .pull=GPIO_PUPD_PULLUP, .mode=GPIO_MODE_INPUT};
#endif

void sk_pin_set(sk_pin pin, bool value)
{
    if(value ^ pin.inv) {
        gpio_set(sk_port_num_to_gpio(pin.port),(1 << pin.pin));
    } else {
        gpio_clear(sk_port_num_to_gpio(pin.port),(1 << pin.pin));
    }
}

bool sk_pin_read(sk_pin pin)
{
    return (gpio_port_read(sk_port_num_to_gpio(pin.port) & (1 << pin.pin))) ^ pin.inv;
}

void sk_pin_mode(sk_pin pin)
{
    gpio_mode_setup(sk_port_num_to_gpio(pin.port), pin.mode, pin.pull,(1 << pin.pin));
}

void sk_pin_toggle(sk_pin pin)
{
	gpio_toggle(sk_port_num_to_gpio(pin.port),(1 << pin.pin));
}

void glsk_pins_init(uint8_t mask)
{
    if (mask & DISCOVER) {
        sk_pin pinsD [] = {sk_io_led_red, sk_io_led_green, sk_io_led_orange, sk_io_led_blue, sk_io_btn_usr};
        sk_arr_foreach(a_pin, pinsD) {
            sk_pin_mode(a_pin);
        }
    }
    if (mask & GL_BORD) {
        sk_pin pinsG [] = {sk_io_btn_up, sk_io_btn_mid, sk_io_btn_down, sk_io_btn_left, sk_io_btn_left};
        sk_arr_foreach(a_pin, pinsG) {
            sk_pin_mode(a_pin);
        }
    }

}

void sk_group_set(sk_pin * pins,uint8_t mask) {
    int shif = 0;
    sk_arr_foreach(a_pin, pins) {
        sk_pin_set(a_pin, (mask >> shif) & 1);
        shif++;
    }
}
