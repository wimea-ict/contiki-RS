/*
  Contiki Rime BRD demo.
  Broadcast Temp, Bat Voltage, RSSI, LQI DEMO. Robert Olsson <robert@herjulf.se>

  Almost all code comes from:

  * Copyright (c) 2007, Swedish Institute of Computer Science.
 * All rights reserved.
 *

  See Contiki for full copyright.

*/

#include <avr/sleep.h>
#include "contiki.h"
#include "contiki-net.h"
#include "lib/list.h"
#include "lib/memb.h"
#include "lib/random.h"
#include "net/rime.h"
#include <stdio.h>
#include "pluto_v1.4.h"

#define MAX_NEIGHBORS 64
#define SIZE          40

unsigned char charbuf[SIZE];

uint16_t h,m;
double v_avr;
double temp;

struct broadcast_message {
  uint8_t seqno;
  uint8_t buf[40];
};

struct neighbor {
  struct neighbor *next;
  rimeaddr_t addr;

  /* The ->last_rssi and ->last_lqi fields hold the Received Signal
     Strength Indicator (RSSI) and CC2420 Link Quality Indicator (LQI)
     values that are received for the incoming broadcast packets. */
  uint16_t last_rssi, last_lqi;
  uint8_t last_seqno;

  /* The ->avg_gap contains the average seqno gap that we have seen
     from this neighbor. */
  uint32_t avg_seqno_gap;
};

MEMB(neighbors_memb, struct neighbor, MAX_NEIGHBORS);
LIST(neighbors_list);

static struct broadcast_conn broadcast;

#define SEQNO_EWMA_UNITY 0x100 /* Moving average */
#define SEQNO_EWMA_ALPHA 0x040

PROCESS(init_process, "Init process");
PROCESS(broadcast_process, "Broadcast process");

AUTOSTART_PROCESSES(&init_process, &broadcast_process);


int radio_sleep = 0;

#define RTC_SCALE 16

static void rtcc_init(void)
{
  TIMSK2 &=~((1<<TOIE2)|(1<<OCIE2A));  /* Disable TC0 interrupt */
  ASSR |= (1<<AS2); /* set Timer/Counter0 to be asynchronous */
  TCNT2 = 0x00;
  //TCCR2B = 0x05;  /* prescale the timer to / 128 to --> 1s */
  TCCR2B = 0x02;  /* prescale the timer to / 8   16 Hz*/
  while(ASSR&0x07);
  TIMSK2 |= (1<<TOIE2);  
}

ISR(TIMER2_OVF_vect)
{
  static int rtc = RTC_SCALE;
  
  if (--rtc == 0 ) {
    static int i;
    rtc = RTC_SCALE;
    if(!(i++ % 10)) 
      process_post(&broadcast_process, 0x12, NULL);
  }
}

void read_sensors(void)
{
  uint16_t s;
  uint8_t p1;
  
  BATMON = 16; //Stabilize at highest range and lowest voltage

  ADCSRB |= (1<<MUX5);   //this bit buffered till ADMUX written to!
  ADMUX = 0xc9;         // Select internal 1.6 volt ref, temperature sensor ADC channel
  ADCSRA = 0x85;        //Enable ADC, not free running, interrupt disabled, clock divider 32 (250 KHz@ 8 MHz)
  ADCSRA |= (1<<ADSC);         //Throwaway conversion
  while 
    (ADCSRA &( 1<<ADSC) ); //Wait
  ADCSRA |= (1<<ADSC);          //Start conversion
  while 
    (ADCSRA & (1<<ADSC) ); //Wait
  h = ADC;
  temp = (double) h * 1.13 - 272.8;
  h = 11*h-2728+(h>>2);       //Convert to celcius*10 (should be 11.3*h, approximate with 11.25*h)
  ADCSRA = 0;               //disable ADC
  ADMUX = 0;                //turn off internal vref      
  m = h/10; s=h-10*m;

  for ( p1=16; p1<31; p1++) { 
    BATMON = p1;
    /*  delay_us(100); */
    if ((BATMON & (1<<BATMON_OK)) == 0 ) 
      break;
  }
  v_avr = (double) 2550-75*16-75 + 75 * p1; //-75 to take the floor of the 75 mv transition window
  v_avr = v_avr/1000;
}

static void broadcast_recv(struct broadcast_conn *c, const rimeaddr_t *from)
{
  struct neighbor *n;
  struct broadcast_message *msg;
  uint8_t seqno_gap;
  static uint16_t cnt;

  PORTE &= ~LED_RED;  /* ON */
  msg = packetbuf_dataptr();

  /* Check if we already know this neighbor. */
  for(n = list_head(neighbors_list); n != NULL; n = list_item_next(n)) {

    if(rimeaddr_cmp(&n->addr, from)) 
      break;
  }

  if(n == NULL) {
    n = memb_alloc(&neighbors_memb); /* New neighbor */

    if(! n ) 
      goto out;
    
    rimeaddr_copy(&n->addr, from);
    n->last_seqno = msg->seqno - 1;
    n->avg_seqno_gap = SEQNO_EWMA_UNITY;
    list_add(neighbors_list, n);
  }

  n->last_rssi = packetbuf_attr(PACKETBUF_ATTR_RSSI);
  n->last_lqi = packetbuf_attr(PACKETBUF_ATTR_LINK_QUALITY);

  /* Compute the average sequence number gap from this neighbor. */
  seqno_gap = msg->seqno - n->last_seqno;

  
  n->avg_seqno_gap = (((uint32_t)seqno_gap * SEQNO_EWMA_UNITY) *
                      SEQNO_EWMA_ALPHA) / SEQNO_EWMA_UNITY +
                      ((uint32_t)n->avg_seqno_gap * (SEQNO_EWMA_UNITY -
                                                     SEQNO_EWMA_ALPHA)) / SEQNO_EWMA_UNITY;

  n->last_seqno = msg->seqno;

  printf("&: %s [ADDR=%-d.%-d SEQ=%-d RSSI=%-u LQI=%-u DRP=%-d.%02d]\n",
	 msg->buf,
         from->u8[0], from->u8[1], msg->seqno,
         n->last_rssi,
         n->last_lqi, 
         (int)(n->avg_seqno_gap / SEQNO_EWMA_UNITY),
         (int)(((100UL * n->avg_seqno_gap) / SEQNO_EWMA_UNITY) % 100));

out:
  PORTE |= LED_RED;  /* OFF */
}


static const struct broadcast_callbacks broadcast_call = {broadcast_recv};

/*
  Channel 11 - 26 

  MAX TX_PWR_3DBM       ( 0 )
  MIN TX_PWR_17_2DBM    ( 15 )

  Channels (11, 15, 20, 25 are WiFi-free).
  RSSI is high while LQI is low -> Strong interference, 
  Both are low -> Try different locations.
*/

PROCESS_THREAD(broadcast_process, ev, data)
{
  static struct etimer et;
  static uint8_t seqno;
  struct broadcast_message msg;
  uint8_t ch, txpwr;

  PROCESS_EXITHANDLER(broadcast_close(&broadcast);)
  PROCESS_BEGIN();
  broadcast_open(&broadcast, 129, &broadcast_call);

  //rf230_set_txpower(15);
  txpwr = rf230_get_txpower();
  ch = rf230_get_channel();
  // rf230_set_channel(ch);
  printf("Ch=%-d TxPwr=%-d\n", ch, txpwr);

  while(1) {
    int len;

    //etimer_set(&et, CLOCK_SECOND * 2);
    //PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
    PROCESS_YIELD_UNTIL(ev == 0x12);

    PORTE &= ~LED_YELLOW; /* On */

    read_sensors();

    len = snprintf((char *) msg.buf, sizeof(msg.buf), 
		  "T_MCU=%-5.1f V_MCU=%-4.2f ", temp, v_avr);
    msg.seqno = seqno;
    packetbuf_copyfrom(&msg, sizeof(struct broadcast_message));
    
    broadcast_send(&broadcast);

    seqno++;

    printf("&: %s\n", msg.buf);

    PORTE |= LED_YELLOW; 
  }
  PROCESS_END();
}

PROCESS_THREAD(init_process, ev, data)
{
  PROCESS_BEGIN();

  printf("Init\n");

  DDRE |= LED_YELLOW;
  DDRE |= LED_RED;

  PORTE |= LED_RED; 
  PORTE |= LED_YELLOW; 

  DDRD |= (1<<4);

  rtcc_init();
  
  PROCESS_END();
}

static struct pt send_thread;

PT_THREAD(send_message(char *txt, rimeaddr_t b, int *retval))
{
  PT_BEGIN(&send_thread);

    printf("%s \n", txt);

  *retval = 0;
  PT_END(&send_thread);
}
