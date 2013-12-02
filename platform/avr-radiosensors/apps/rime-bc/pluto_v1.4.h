
/* 
   Pin assigments for sensors board revision 1.4 
   using MCU AtMega128rfa1 
*/

#define LED_YELLOW   (1<<PE3) /* PE3 1k pullup to Vcc */  
#define LED_RED      (1<<PE4) /* PE4 1k pullup to Vcc */  
#define USB_PWR      PB5 /* High if FTDI TTL-USB cable sources */ 
#define P0           PE5 /* Pulse count input. Pullup via jumper */
#define P1           PE6 /* Pulse count input via optional Comparator */
#define PWR_1        PE7 /* Programmable power pin Vcc via P-FET */
#define AV_IN        PF0 /* V_IN ADC input. 100k/1M volt. divider */
#define A1           PF1 /* A1 ADC input. 100k/300k volt. divider */
#define A2           PF2 /* A2 ADC input  100k/300k volt. divider */
#define RX0          PE0 /* RX UART0 -- FTDI TTL-USB cable */
#define TX0          PE1 /* TX UART0 -- FTDI TTL-USB cable */

#define OW_BUS_0     PD7 /* One-Wire bus w temp/ID Pulled Up. Separate Vcc */
#define OW_BUS_1     PD6 /* One-Wire bus extra Pulled Up. Separate Vcc */

/* AVR ISP standard connected */
/* 16 MHz  xtal */
/* RTC 32.768 Hz  xtal */

/*Optional HIH610 connected on SPI */
#define HUM_PWR      PE2 /* Power pin for HIH6130 also to ena. HIH 6130 Alarm mode */

/*Optional comparator on reverse side. Photo transistor 0805 or 1206 connected
  to comparator */



