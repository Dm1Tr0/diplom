#include "sk_lib.h"
#include "stdbool.h"
#include "timers.h"
#include "gfx.h"
#include "printf.h"
#include <libopencmsis/core_cm3.h>
#include <libopencm3/cm3/cortex.h>
// #include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/stm32/pwr.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/spi.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "usb.h"
//#include <stdlib.h>



// header file
#define swap(a,b) {int16_t t=a;a=b;b=t;}

#define TFT9341_MADCTL_MY  0x80
#define TFT9341_MADCTL_MX  0x40
#define TFT9341_MADCTL_MV  0x20
#define TFT9341_MADCTL_ML  0x10
#define TFT9341_MADCTL_RGB 0x00
#define TFT9341_MADCTL_BGR 0x08
#define TFT9341_MADCTL_MH  0x04
#define TFT9341_ROTATION (TFT9341_MADCTL_MX | TFT9341_MADCTL_BGR)
#define	TFT9341_BLACK   0x0000
#define	TFT9341_BLUE    0x001F
#define	TFT9341_RED     0xF800
#define	TFT9341_GREEN   0x07E0
#define TFT9341_CYAN    0x07FF
#define TFT9341_MAGENTA 0xF81F
#define TFT9341_YELLOW  0xFFE0
#define TFT9341_WHITE   0xFFFF

uint16_t display_width; // this thing could be static
uint16_t display_hight; // this thing could be static

//header file

sk_pin TFT_DC     = { .port=sk_port_A, .pin=3, .inv=false, .pull=GPIO_PUPD_PULLUP, .mode=GPIO_MODE_OUTPUT};
sk_pin TFT_RESET  = { .port=sk_port_A, .pin=2, .inv=false, .pull=GPIO_PUPD_NONE, .mode=GPIO_MODE_OUTPUT};
// manualy initialized in spi_init
sk_pin TFT_CS     = { .port=sk_port_A, .pin=4, .inv=false, .pull=GPIO_PUPD_NONE, .mode=GPIO_MODE_OUTPUT};

static const char* parametrs[] = {
        "T = %s C\n",
        "overal load %s\n",
        "core0 speed %s\n",
        "core1 speed %s\n",
        "core2 speed %s\n",
        "core3 speed %s\n"
};

static uint8_t param_sz = sizeof(parametrs) / sizeof(char*);

static void clock_init(void)
{
	// * Until now we have been working with internal 16 MHz RC oscillator using default config
	//   Let's switch to higher 168 MHz frequency and use more precise external crystal oscillator
	// * On Discovery board we have two crystal oscillators: X1 and X2.
	//   X1 is used to clock on-board ST-Link, which in default configuration outputs clock to MCU
	//   X2 can clock MCU directly, but this requires desoldering (removing) R68 resistor
	//      to free the line. Obviously we're not interested in this option.
	//   So, we will use ST-Link MCO (Master Clock Output) from X1 to provide 8 MHz external
	//   clock for our MCU.
	// * MCO is derived from X1 is 8 MHz, 30 ppm (+- 240 Hz) external crystal oscillator

	// * We will get this 8 MHz clock and multipy it on internal MCU PLL by factor of x21
	//   to get 168 MHz and use this resulting frequency to clock AHB bus

	// After Reset, we are running from internal 16 MHz clock

	// Set HCE to external clock source (HSE bypass), not crystal. See page 218 RM
	rcc_osc_bypass_enable(RCC_HSE);		// bypass load capacitors used for crystals
	rcc_osc_on(RCC_HSE);				// enable High-Speed External clock (HSE)
	// trap until external clock is detected as stable
	while (!rcc_is_osc_ready(RCC_HSE));


	// as page 79 DS tells to operate at high frequency, we need to supply more power
	// by setting VOS=1 bit in power register
	rcc_periph_clock_enable(RCC_PWR);
	pwr_set_vos_scale(PWR_SCALE1);
	rcc_periph_clock_disable(RCC_PWR);	// no need to configure or use features requiring clock

	// Configure PLL
	rcc_osc_off(RCC_PLL);		// Disable PLL before configuring it

	// * Set PLL multiplication factor as specified at page 226 RM
	//   F<main> = Fin * PLLN / PLLM		-- main PLL clock (intermediate)
	//   F<genout> = F<main> / PLLP			-- AHB clock output
	//   F<Qdomain> = F<main> / PLLQ		-- Q clock domain
	//										(must be <= 48 MHz or exactly 48 MHz if HS-USB is used)
	// * rcc_set_main_pll_hse(PLLM, PLLN, PLLP, PLLQ, PLLR)
	// 		PLLM		Divider for the main PLL input clock
	// 		PLLN		Main PLL multiplication factor for VCO
	// 		PLLP		Main PLL divider for main system clock
	// 		PLLQ		Main PLL divider for USB OTG FS, SDMMC & RNG
	// 		0 PLLR		Main PLL divider for DSI (for parts without DSI, provide 0 here)
	// * PLL has limitations:
	//   PLLM:	2 <= PLLM <= 63. After PLLM division frequency must be from 1 MHz to 2 MHz
	//					         Whereas 2 MHz is recommended
	//	 	So, set PLLM = 8 MHz / 2 MHz = /4 division
	//   PLLN:  50 <= PLLN <= 432. After PLLN multiplication frequency must be from 100 to 432 MHz
	//   PLLP:  PLLP is /2, /4, /6 or /8. After PLLP division, frequency goes to AHB and sysclk
	//							  And should not exceed 168 MHz
	//   PLLQ:  2 <= PLLQ <= 15. After PLLQ division frequency must be below 48 MHz
	//						     and exactly 48 MHz if SDIO peripheral is used
	//	    So, select PLLQ to achieve 48 MHz
	// The resulting solution is:
	// PLLM = 4		-- 8/4 = 2 MHz input to PLL mul stage
	// PLLN = 168   -- F<main> = 2 * 168 = 336 MHz
	// PLLP = 2		-- F<genout> = 336 / 2 = 168 MHz for our CPU and AHB
	// PLLQ = 7		-- F<Qdomain> = 336 / 7 = 48 MHz exactly
	rcc_set_main_pll_hse(4, 168, 2, 7, 0);		// with input from HSE to PLL
	rcc_css_disable();		// Disable clock security system
	rcc_osc_on(RCC_PLL);				// Enable PLL
	while (!rcc_is_osc_ready(RCC_PLL)); // Wait for PLL out clock to stabilize


	// Set all prescalers.
	// (!) Important. Different domains have different maximum allowed clock frequencies
	//     So we need to set corresponding prescalers before switching AHB to PLL
	// AHB should not exceed 168 MHZ		-- divide 168 MHz by /1
	// APB1 should not exceed 42 MHz		-- divide AHB by /4 = 168 / 4 = 42 MHz
	// APB2 should not exceed 84 MHz		-- divide AHB by /2 = 168 / 2 = 84 MHz
	rcc_set_hpre(RCC_CFGR_HPRE_DIV_NONE);
    rcc_set_ppre1(RCC_CFGR_PPRE_DIV_4);
    rcc_set_ppre2(RCC_CFGR_PPRE_DIV_2);

	// Enable caches. Flash is slow (around 30 MHz) and CPU is fast (168 MHz)
	flash_dcache_enable();
	flash_icache_enable();

	// IMPORTANT! We must increase flash wait states (latency)
	// otherwise fetches from flash will ultimately fail
	flash_set_ws(FLASH_ACR_LATENCY_7WS);

	// Select PLL as AHB bus (and CPU clock) source
    rcc_set_sysclk_source(RCC_CFGR_SW_PLL);
	// Wait for clock domain to be changed to PLL input
	rcc_wait_for_sysclk_status(RCC_PLL);

	// set by hand since we've not used rcc_clock_setup_pll
	rcc_ahb_frequency = 168000000ul;
	rcc_apb1_frequency = rcc_ahb_frequency / 4;
	rcc_apb2_frequency = rcc_ahb_frequency / 2;

	// Disable internal 16 MHz RC oscillator (HSI)
	rcc_osc_off(RCC_HSI);
}





static inline void dc_data(void)
{
        sk_pin_set(TFT_DC, true);
}

static inline void dc_command(void)
{
        sk_pin_set(TFT_DC, false);
}

static inline void cs_active(void)
{
        sk_pin_set(TFT_CS, false);
}

static inline void cs_off(void)
{
        sk_pin_set(TFT_CS, true);
}

void send_command_spi_tft(uint8_t command)
{

        dc_command();
        spi_send(SPI1, command);
        //spi_read(SPI1); // in order to provide delay

}

void send_data_spi_tft(uint8_t data)
{
        dc_data();
        spi_send(SPI1, data);
        //spi_read(SPI1); // in order to provide delay

}

void write_data_spi_tft(void * buff, uint8_t len)
{
        uint8_t * d = buff;
	if ((!len) || (NULL == d))
		return;

        for(int i = 0; i < len; i++){
                send_data_spi_tft(d[i]);
	}

}


void mode_TFT_pins(void)
{
        gpio_set_output_options(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_100MHZ, (1 << 3) | (1 << 2) | (1 << 4));
	sk_pin_mode(TFT_RESET);
	sk_pin_mode(TFT_DC);
        sk_pin_mode(TFT_CS);
}


void reset_TFT(void)
{
        sk_pin_set(TFT_RESET, false);
        t7_delay_ms(5);
        sk_pin_set(TFT_RESET, true);
}

void spi_tft_init (void)
{
        // don`t forger about clocking of GPIO_A

        mode_TFT_pins(); // configure some pins as a part of spi interface
	gpio_set_output_options(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_100MHZ, (1 << 5) | (1 << 7) | (1 << 6));
	gpio_set_af(GPIOA, GPIO_AF5, (1 << 7) | (1 << 5));
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, (1 << 5) | (1 << 7) | (1 << 6));
	// manual cs

	rcc_periph_clock_enable(RCC_SPI1);

	spi_disable(SPI1);

	// in case of problems with data transfer
	spi_set_baudrate_prescaler(SPI1, SPI_CR1_BR_FPCLK_DIV_2);

	spi_set_master_mode(SPI1);

	spi_set_full_duplex_mode(SPI1);

	spi_set_dff_8bit(SPI1);

	spi_disable_crc(SPI1);

	spi_send_msb_first(SPI1);

        // SET APB2 presc to 2


        spi_enable_ss_output(SPI1);
	// as a default
	//cpha = 0
	//cpol = 0

	spi_enable(SPI1);
}


void set_addr_window(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
        uint8_t data[20];

        send_command_spi_tft(0x2A); // CASET

        data[0] = (x0 >> 8) & 0xFF;
        data[1] = x0 & 0xFF;
        data[2] = (x1 >> 8) & 0xFF;
        data[3] = x1 & 0xFF; // different from original version
        write_data_spi_tft(data, 4);

        send_command_spi_tft(0x2B);

        data[0] = (y0 >> 8) & 0xFF;
        data[1] = y0 & 0xFF;
        data[2] = (y1 >> 8) & 0xFF;
        data[3] = y1 & 0xFF; // different from original version
        write_data_spi_tft(data,4);

        send_command_spi_tft(0x2C);
}

void tft_fill_rect(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color)
{
        cs_active();
        if((x1 >= display_width) || (y1 >= display_hight) || (x2 >= display_width) || (y2 >= display_hight))
        return;

	if(x1>x2) swap(x1,x2);
	if(y1>y2) swap(y1,y2);

        set_addr_window(x1, y1, x2, y2);
        uint8_t data[] = { color >> 8 & 0xFF, color & 0xFF}; // different from original version
        for(uint32_t i = 0; i < (x2-x1+1)*(y2-y1+1); i++)
        {
                write_data_spi_tft(data, 2);
        }
        cs_off();
}

void draw_pix(int x, int y, uint16_t collor)
{
        tft_fill_rect(x,y,x,y,collor);
}

void fill_screen(uint16_t collor)
{
        cs_active();
        tft_fill_rect(0, 0, display_width-1, display_hight-1, collor);
        cs_off();
}

void tft_set_rotation(uint8_t r)
{
        char boof[10];
        sprintf(boof, " r vallue %d \n", r);
        cs_active( );

        send_command_spi_tft(0x36);
        switch(r)
        {
                case 0:
                send_data_spi_tft(0x48);
                gfx_setRotation(r);
                break;
                case 1:
                send_data_spi_tft(0x28);
                gfx_setRotation(r);
                break;
                case 2:
                send_data_spi_tft(0x88);
                gfx_setRotation(r);
                break;
                case 3:
                send_data_spi_tft(0xE8);
                gfx_setRotation(r);
                break;
        }
        //gfx_puts(boof);
        cs_off();

}

void display_init(uint16_t display_w, uint16_t display_h)
{
        uint8_t data[20];
        reset_TFT();
        cs_active();
        send_command_spi_tft(0x01);
        t7_delay_sec(1);
        //Power Control A
        data[0] = 0x39;
        data[1] = 0x2C;
        data[2] = 0x00;
        data[3] = 0x34;
        data[4] = 0x02;
        send_command_spi_tft(0xCB);
        write_data_spi_tft(data, 5);
        //Power Control B
        data[0] = 0x00;
        data[1] = 0xC1;
        data[2] = 0x30;
        send_command_spi_tft(0xCF);
        write_data_spi_tft(data, 3);
        //Driver timing control A
        data[0] = 0x85;
        data[1] = 0x00;
        data[2] = 0x78;
        send_command_spi_tft(0xE8);
        write_data_spi_tft(data, 3);
        //Driver timing control B
        data[0] = 0x00;
        data[1] = 0x00;
        send_command_spi_tft(0xEA);
        write_data_spi_tft(data, 2);
        //Power on Sequence control
        data[0] = 0x64;
        data[1] = 0x03;
        data[2] = 0x12;
        data[3] = 0x81;
        send_command_spi_tft(0xED);
        write_data_spi_tft(data, 4);
        //Pump ratio control
        data[0] = 0x20;
        send_command_spi_tft(0xF7);
        write_data_spi_tft(data, 1);
        //Power Control,VRH[5:0]
        data[0] = 0x10;
        send_command_spi_tft(0xC0);
        write_data_spi_tft(data, 1);
        //Power Control,SAP[2:0];BT[3:0]
        data[0] = 0x10;
        send_command_spi_tft(0xC1);
        write_data_spi_tft(data, 1);
        //VCOM Control 1
        data[0] = 0x3E;
        data[1] = 0x28;
        send_command_spi_tft(0xC5);
        write_data_spi_tft(data, 2);
        //VCOM Control 2
        data[0] = 0x86;
        send_command_spi_tft(0xC7);
        write_data_spi_tft(data, 1);
        //Memory Acsess Control
        data[0] = 0x48;
        send_command_spi_tft(0x36);
        write_data_spi_tft(data, 1);
        //Pixel Format Set
        data[0] = 0x55;//16bit
        send_command_spi_tft(0x3A);
        write_data_spi_tft(data, 1);
        //Frame Rratio Control, Standard RGB Color
        data[0] = 0x00;
        data[1] = 0x18;
        send_command_spi_tft(0xB1);
        write_data_spi_tft(data, 2);
        //Display Function Control
        data[0] = 0x08;
        data[1] = 0x82;
        data[2] = 0x27;//320 строк
        send_command_spi_tft(0xB6);
        write_data_spi_tft(data, 3);
        //Enable 3G (пока не знаю что это за режим)
        data[0] = 0x00;//не включаем
        send_command_spi_tft(0xF2);
        write_data_spi_tft(data, 1);
        //Gamma set
        data[0] = 0x01;//Gamma Curve (G2.2) (Кривая цветовой гаммы)
        send_command_spi_tft(0x26);
        write_data_spi_tft(data, 1);
        //Positive Gamma  Correction
        data[0] = 0x0F;
        data[1] = 0x31;
        data[2] = 0x2B;
        data[3] = 0x0C;
        data[4] = 0x0E;
        data[5] = 0x08;
        data[6] = 0x4E;
        data[7] = 0xF1;
        data[8] = 0x37;
        data[9] = 0x07;
        data[10] = 0x10;
        data[11] = 0x03;
        data[12] = 0x0E;
        data[13] = 0x09;
        data[14] = 0x00;
        send_command_spi_tft(0xE0);
        write_data_spi_tft(data, 15);
        //Negative Gamma  Correction
        data[0] = 0x00;
        data[1] = 0x0E;
        data[2] = 0x14;
        data[3] = 0x03;
        data[4] = 0x11;
        data[5] = 0x07;
        data[6] = 0x31;
        data[7] = 0xC1;
        data[8] = 0x48;
        data[9] = 0x08;
        data[10] = 0x0F;
        data[11] = 0x0C;
        data[12] = 0x31;
        data[13] = 0x36;
        data[14] = 0x0F;
        send_command_spi_tft(0xE1);
        write_data_spi_tft(data, 15);
        send_command_spi_tft(0x11);//Выйдем из спящего режима
        t7_delay_ms(120);
        //display on
        data[0] = TFT9341_ROTATION;
        send_command_spi_tft(0x29);
        write_data_spi_tft(data, 1);
        cs_off();
        display_hight = display_h;
        display_width = display_w;

        void print_state(char * string, int vall){
                char boof[100];

        }

}

void parce_print(char *string, size_t s_sz) {
        int i;
        int j;
        int param_i = 0;
        char boof[10];
        char print_boof[50];
        int last_p = 0;
        for(i=0; i < s_sz; i++) {
                if (string[i] == '-') {
                        for(j=last_p; j < i; j++) {
                                boof[j - last_p] = string[j];
                                // sprintf(print_boof,"%c", string[j]);
                        	// gfx_puts(print_boof);
                                // sprintf(print_boof,"-/%d/ \n", i);
                        	// gfx_puts(print_boof);
                        }

                        last_p = i + 1;
                        // sprintf(print_boof,"\n");
                        // gfx_puts(print_boof);
                        sprintf(print_boof, parametrs[param_i++], boof);
                        gfx_puts(print_boof);
                        memset(boof, 0, 10);
                        memset(print_boof, 0, 50);
                }
        }
}

int main (void)
{
        // initialization
        //overal clocking
        rcc_periph_clock_enable(RCC_GPIOA);
        rcc_periph_clock_enable(RCC_GPIOD);
        rcc_periph_clock_enable(RCC_TIM7);
        glsk_pins_init(DISCOVER);
        //timer init
        sk_pin_set(sk_io_led_red, true);
        clock_init();
        timer_7_init(upd_on_ovf, 168000000ul, 1000ul , 16ul);
        sk_pin_toggle(sk_io_led_red);
        //spi initialize

        mode_TFT_pins();
        cs_off();
        spi_tft_init();
        usb_vcp_init();

        display_init(240, 320);
        gfx_init(&draw_pix, GFX_HEIGHT, GFX_WIDTH);
        fill_screen(TFT9341_BLACK);

        // cs_active();
        // tft_fill_rect(100,100,200,200,TFT9341_BLACK);
        // cs_off();

        // cs_active();
        // gfx_drawPixel(100,100, GFX_COLOR_WHITE);
        // gfx_drawPixel(100,101, GFX_COLOR_WHITE);
        // gfx_drawPixel(101,100, GFX_COLOR_WHITE);
        // gfx_drawPixel(101,101, GFX_COLOR_WHITE);
        // cs_off();

        tft_set_rotation(3);
        gfx_setCursor(0,40);
        gfx_setTextColor(GFX_COLOR_GREEN, TFT9341_BLACK);
        gfx_setTextSize(2);

        cs_active();

        char p_buff[256];

        bool isWriten = false;
        bool isclean = true;
        char buf[128];

        size_t len = 0;
        //size_t l = 1;

        sprintf(p_buff,"wow");
        gfx_puts(p_buff);

        // while (1) {
        //
	//         if (len >= sizeof(buf)) {
	// 		break;
	// 	}
        //
	// 	if (usb_vcp_avail()) {
        //                 if(!isclean) {
        //                         fill_screen(TFT9341_BLACK);
        //                         gfx_setCursor(0,40);
        //                         isclean =true;
        //                 }
	// 		char ch = usb_vcp_recv_byte();
	// 		buf[len++] = ch;
        //                 // sprintf(p_buff,"%c", ch); //reading all special simbols
        //         	// gfx_puts(p_buff);
        //                 //
        //                 // sprintf(p_buff,"-%ds\n", len); //reading all special simbols
        //         	// gfx_puts(p_buff);
        //                 // memset(p_buff,0,256);
        //
        //                 isWriten = true;
	// 	}
        //
        //         if (!usb_vcp_avail() && isWriten) {
        //                 sprintf(p_buff, "%s", buf); //reading all special simbols
        //         	gfx_puts(p_buff);
        //                 parce_print(p_buff, strlen(p_buff));
        //                 memset(p_buff,0,256);
        //                 memset(buf,0,128);
        //                 isWriten = false;
        //                 isclean = false;
        //                 len = 0;
        //         }
        //
        //         nvic_disable_irq(NVIC_TIM7_IRQ);
        //         __WFI(); // waiting for usb otg fs interrupt
        //         nvic_disable_irq(NVIC_TIM7_IRQ);
        // }




        // char boof[100];
        // char *str;
        // while(1) { int rot = 0;
        //
        // fill_screen(TFT9341_BLACK);
        //
        // for(int i = 0; i<10; i++) {
        //         sprintf(boof, "%s ; %d \n", "hi maaan", i);
        //         cs_active( );
        //         gfx_puts(boof);
        //         cs_off();
        // }
        //
        // str = "--------------------";
        // cs_active();
        // gfx_puts(str);
        // cs_off();
        //
        // gfx_setCursor(0, 0);
        //
        // t7_delay_sec(5);
        // rot++;
        // }


        return 0;
}
