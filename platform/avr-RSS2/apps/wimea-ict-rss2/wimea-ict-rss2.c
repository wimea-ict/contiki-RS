/*
 * Copyright (c) 2012, STMicroelectronics.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 * Some code adopted from Robert Olsson <robert@herjulf.se> and Manee
 *  @Author: Nsabagwa Mary, Flavia Nshemerirwe, Osbert Mugabe, Nahabwe Brian
 * The application reads sensor data, transmits it via broadbast, unicast .. using RIME
 */
/*---------------------------------------------------------------------------*/
#include "contiki.h"
#include "wimea-ict-rss2.h"
#include <avr/eeprom.h>
#include "dev/temp_mcu-sensor.h"
#include "dev/temp-sensor.h"
#include "dev/light-sensor.h"
#include "dev/adc.h"
#include "dev/battery-sensor.h"
#include <stdio.h> /* For printf() */
#include "serial-shell.h"
#include "dev/serial-line.h"
#include <string.h>
#include "params.h"
#include "dev/i2c.h"
#include "dev/ds1307.h"
#include "sys/etimer.h"
#include "net/rime.h"
#include "random.h"


#define NAME_LENGTH 10
/* This #define defines the maximum amount of neighbors we can remember. */
#define MAX_NEIGHBORS 16
/* These two defines are used for computing the moving average for the
   broadcast sequence number gaps. */
#define SEQNO_EWMA_UNITY 0x100
#define SEQNO_EWMA_ALPHA 0x040


//Configuration Parameters
uint16_t EEMEM eemem_transmission_interval;
uint8_t EEMEM eemem_node_name;

uint16_t saved_interval,h,m;
char buff[6];
char * value;
double v_avr, temp;
//unsigned char eui64_addr[8] ;

/* The neighbors_list is a Contiki list that holds the neighbors we
   have seen thus far. */
/* This structure holds information about neighbors. */
struct neighbor {
	/* The ->next pointer is needed since we are placing these on a Contiki list. */
	struct neighbor *next;

	/* The ->addr field holds the Rime address of the neighbor. */
	rimeaddr_t addr;

	/* The ->last_rssi and ->last_lqi fields hold the Received Signal
     Strength Indicator (RSSI) and CC2420 Link Quality Indicator (LQI)
     values that are received for the incoming broadcast packets. */
	uint16_t last_rssi, last_lqi;

	/* Each broadcast packet contains a sequence number (seqno). The
     ->last_seqno field holds the last sequenuce number we saw from
     this neighbor. */
	uint8_t last_seqno;

	/* The ->avg_gap contains the average seqno gap that we have seen
    from this neighbor. */
	uint32_t avg_seqno_gap;
};

/*---------------------------------------------------------------------------*/
/* This is the structure of broadcast messages. */
struct broadcast_message {
  uint8_t seqno;
};

/*---------------------------------------------------------------------------*/
/* This MEMB() definition defines a memory pool from which we allocate
   neighbor entries. */
MEMB(neighbors_memb, struct neighbor, MAX_NEIGHBORS);
LIST(neighbors_list);

/*---------------------------------------------------------------------------*/
PROCESS(temp_read_process, "Temperature Readings");
PROCESS(serial_input, "Serial line input commands");
PROCESS(save_config_process, "Save configurations");
//PROCESS(light_sensor_process, "Light sensor readings");
PROCESS(broadcast_process, "Broadcast example");

//PROCESS(temp_surounding_read_process, "Temperature Readings");
AUTOSTART_PROCESSES(&serial_input, &save_config_process);

PROCESS_THREAD(save_config_process, ev, data)
{
	uint8_t eeprom_interval;
	PROCESS_BEGIN();
	DS1307_init(); //initialise the DS1307 RTC
	cli();
	eeprom_interval = eeprom_read_word(&eemem_transmission_interval);
	saved_interval = (eeprom_interval == 0xFF) ? 60 : eeprom_interval;
	sei(); 
	 //get_eui64_addr(eui64_addr);
	process_start(&temp_read_process, NULL);
	process_start(&broadcast_process, NULL);
	PROCESS_END();  
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(temp_read_process, ev, data)
{
	static struct etimer etimer;
	static int temp_m=0;
	static int temp=0;
	static int light =0;
	static int adc =0;
	PROCESS_BEGIN();
  
	SENSORS_ACTIVATE(temp_mcu_sensor);
	SENSORS_ACTIVATE(temp_sensor);
	SENSORS_ACTIVATE(light_sensor);
	SENSORS_ACTIVATE(adc_sensor);//new driver
	SENSORS_ACTIVATE(battery_sensor);
	//cli();
	//saved_interval = eeprom_read_word(&eemem_transmission_interval);
	//sei();
  
	while(1) {
		etimer_set(&etimer, CLOCK_SECOND * eeprom_read_word(&eemem_transmission_interval));//PRINT_INTERVAL
		temp_m = temp_mcu_sensor.value(0);
		temp = temp_sensor.value(0);
		light=light_sensor.value(0);
		adc = adc_sensor.value(VIN_VALUE);
		PROCESS_WAIT_EVENT();
		printf("%s %s ", DS1307_get_date(), DS1307_get_time());
		printf("MCU Temperature = %d, ", temp_m);
		printf("Temperature = %d, ", temp);   	
		printf("Light = %d, ", light);
		printf("V_MCU = %.1f, ", (double) battery_sensor.value(0)/1000);
		printf("V_IN = %d, ",  adc);
		printf("V_IN = %.1f, ", (double) adc_read_v_in());
		printf("A1 = %.1f, ", (double) adc_read_a1());
		printf("A2 = %.1f\n", (double)adc_read_a2());
 }
  
	SENSORS_DEACTIVATE(temp_mcu_sensor);
	SENSORS_DEACTIVATE(temp_sensor);
	SENSORS_DEACTIVATE(light_sensor);
	SENSORS_DEACTIVATE(adc_sensor);//new driver
	SENSORS_DEACTIVATE(battery_sensor);
  
  PROCESS_END();
}

/*---------------------------------------------------------------------------*/
/*PROCESS_THREAD(light_sensor_process, ev, data)
{
	static int light =0;
	PROCESS_BEGIN();
	
	SENSORS_ACTIVATE(light_sensor);
	
	while(1) {
		light=light_sensor.value(0);   	
		printf(" Light = %d\n", light);
	}
	SENSORS_DEACTIVATE(light_sensor);
  
  PROCESS_END();
}*/

PROCESS_THREAD(serial_input, ev, data)
{
	PROCESS_BEGIN();

	char delimiter[] = " ";
	char *command = NULL ;
	uint8_t flag;
      
	for(;;) {
		PROCESS_YIELD_UNTIL(ev == serial_line_event_message);
     
		/* if(ev == serial_line_event_message) {
			printf("received line: %s\n", (char *)data);
		} */
	 
		command = (char*)strtok((char*)data, (const char*)delimiter) ;		
		//printf("the command is %s",command);
		if(!strncmp(command, "h", 1)){
			print_help_command_menu() ;
		} else if(!strncmp(command,"ri",2)) {
			value=(char*) malloc(6);		
			strncpy(value, command+3, 6);//(strlen(command)-3)
			if(strlen(command)>2)
				change_reporting_interval(value);
			else 
				display_reporting_interval();
			free(value);
		}else if(!strncmp(command,"ss",2)) {//system summary
				display_system_information();
		}
		else if(!strncmp(command, "name", 4)) {
			value=(char*) malloc(12);		
			strncpy(value, command+5, 12);//(strlen(command)-3)
			if(strlen(command)>4)
				change_node_name(value);
			else 
				display_node_name();
			free(value);
		}
		else if(!strncmp(command,"u", 1)) {//system summary
			display_system_uptime();
		}
		else if(!strncmp(command,"date", 4)) {//date setup
			value = (char*) malloc(8);
			flag = 0;
			strncpy(value, command+5, 8);
			if (strlen(command)>4){
				set_datetime(value, flag);
				printf("Date set: %s\n", DS1307_get_date());
			}else{
				printf("Date: %s\n", DS1307_get_date());
			}
		}
		else if(!strncmp(command,"time", 4)) {//time setup
			value = (char*) malloc(8);
			flag = 1;
			strncpy(value, command+5, 8);
			if (strlen(command)>4){
				set_datetime(value, flag);
				printf("Time set: %s\n", DS1307_get_time());
			}else{
				printf("Time: %s\n", DS1307_get_time());
			}
		}
		else {
			printf("Invalid command %s. Try h for a list of commands\n", command) ;
		}	// more commands can be added here
	}   	
	PROCESS_END();
}

//change node name
void change_node_name(char *value){
	cli();
	char new_name[NAME_LENGTH];
	strncpy(new_name, value, NAME_LENGTH);
    eeprom_update_block((const void *)&new_name, (void *)&eemem_node_name, NAME_LENGTH);
	sei();
	printf("Node Name changed to %s\n", new_name);
	
}

void display_node_name(){
	cli();
	uint8_t node_name[NAME_LENGTH];
	eeprom_read_block((void*)&node_name, (const void*)&eemem_node_name, NAME_LENGTH);
	sei();
	printf("Node Name = %s\n", (char *)node_name);
}

//changing reporting interval
void 
change_reporting_interval(char* value)
{
    int interval = atoi(value);
	cli();
	eeprom_update_word(&eemem_transmission_interval, interval);  
	sei();
	printf("Interval changed to %d\n",interval);
	//process snch, that sends event data
}

void
display_system_uptime()
{
    printf("System has been up for %ld seconds \n", clock_seconds());// clock_time()
}

//Display Mote information
void
display_system_information()
{
	printf("System Information");
}

//Displaying reporting interval 
void
display_reporting_interval()
{
	cli();
	saved_interval = eeprom_read_word(&eemem_transmission_interval);
	sei();
	printf("Current Interval is %d\n",saved_interval);
}

//set date or time
void set_datetime(char *value, uint8_t choice){
	int8_t i=0, datetime[3];
	char new_datetime[9];
	strncpy(new_datetime, value, 9);
	char *split_datetime=NULL;
	if (choice == 0){
		split_datetime = strtok (new_datetime, "/");
		while (split_datetime != NULL && i < 3 )
		{
			datetime[i] = atoi(split_datetime);
			split_datetime = strtok (NULL, "/");
			i++;
		}
		DS1307_set_date(datetime[0], datetime[1], datetime[2]);
	} else {
		split_datetime = strtok (new_datetime, ":");
		while (split_datetime != NULL && i < 3 )
		{
			datetime[i] = atoi(split_datetime);
			split_datetime = strtok (NULL, ":");
			i++;
		}
		DS1307_set_time(datetime[0], datetime[1], datetime[2]);
	}
}

//menu 
void
print_help_command_menu()
{
	printf("\n------------------ Menu--------------------------------\n") ;
	printf("\n Prints a list of supported commands \t Usage: h") ;
	printf("\n Display System Summary \t Usage: ss");
	printf("\n Display System Uptime \t Usage: u");
	printf("\n Set/Display Node Name \t Usage: name <node name>");
	printf("\n Set/Display reporting interval \t  Usage: ri <period in seconds>");
	printf("\n Set the report tag mask \t Usage: tagmask <var1,var2>"); 
	printf("\n Set Time\t time h:m:s");
	printf("\n Set Date\t date d/m/yy");
	printf("\n Display Time\t time");
	printf("\n Display Date\t date\n");
	printf("---------------------------------------------------------------\n\n");
}
/*---------------------------------------------------------------------------*/

/* This function is called whenever a broadcast message is received. */
static void
broadcast_recv(struct broadcast_conn *c, const rimeaddr_t *from)
{
	struct neighbor *n;
	struct broadcast_message *m;
	uint8_t seqno_gap;

	/* The packetbuf_dataptr() returns a pointer to the first data byte
     in the received packet. */
	m = packetbuf_dataptr();

	/* Check if we already know this neighbor. */
	for(n = list_head(neighbors_list); n != NULL; n = list_item_next(n)) {

		/* We break out of the loop if the address of the neighbor matches
		the address of the neighbor from which we received this
		broadcast message. */
		if(rimeaddr_cmp(&n->addr, from)) {
			break;
		}
	}

	/* If n is NULL, this neighbor was not found in our list, and we
    allocate a new struct neighbor from the neighbors_memb memory
    pool. */
	if(n == NULL) {
		n = memb_alloc(&neighbors_memb);

		/* If we could not allocate a new neighbor entry, we give up. We
		could have reused an old neighbor entry, but we do not do this
		for now. */
		if(n == NULL) {
			return;
		}

		/* Initialize the fields. */
		rimeaddr_copy(&n->addr, from);
		n->last_seqno = m->seqno - 1;
		n->avg_seqno_gap = SEQNO_EWMA_UNITY;

		/* Place the neighbor on the neighbor list. */
		list_add(neighbors_list, n);
	}

	/* We can now fill in the fields in our neighbor entry. */
	n->last_rssi = packetbuf_attr(PACKETBUF_ATTR_RSSI);
	n->last_lqi = packetbuf_attr(PACKETBUF_ATTR_LINK_QUALITY);

	/* Compute the average sequence number gap we have seen from this neighbor. */
	seqno_gap = m->seqno - n->last_seqno;
	n->avg_seqno_gap = (((uint32_t)seqno_gap * SEQNO_EWMA_UNITY) *
                      SEQNO_EWMA_ALPHA) / SEQNO_EWMA_UNITY +
                      ((uint32_t)n->avg_seqno_gap * (SEQNO_EWMA_UNITY -
                                                     SEQNO_EWMA_ALPHA)) /
    SEQNO_EWMA_UNITY;

	/* Remember last seqno we heard. */
	n->last_seqno = m->seqno;

	/* Print out a message. */
	printf("broadcast message received from %d.%d with seqno %d, RSSI %u, LQI %u, avg seqno gap %d.%02d\n",
         from->u8[0], from->u8[1],
         m->seqno,
         packetbuf_attr(PACKETBUF_ATTR_RSSI),
         packetbuf_attr(PACKETBUF_ATTR_LINK_QUALITY),
         (int)(n->avg_seqno_gap / SEQNO_EWMA_UNITY),
         (int)(((100UL * n->avg_seqno_gap) / SEQNO_EWMA_UNITY) % 100));
	printf("broadcast message received from %d.%d: '%s'\n",
         from->u8[0], from->u8[1], (char *)packetbuf_dataptr());
}
	
/* This is where we define what function to be called when a broadcast
   is received. We pass a pointer to this structure in the
   broadcast_open() call below. */
static const struct broadcast_callbacks broadcast_call = {broadcast_recv};
static struct broadcast_conn broadcast;

/*---------------------------------------------------------------------------*/
PROCESS_THREAD(broadcast_process, ev, data)
{
	static struct etimer et;
	PROCESS_EXITHANDLER(broadcast_close(&broadcast);)
	PROCESS_BEGIN();
	broadcast_open(&broadcast, 129, &broadcast_call);

	while(1) {
		/* Delay 2-4 seconds */
		etimer_set(&et, CLOCK_SECOND * 4 + random_rand() % (CLOCK_SECOND * 4));

		PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));

		packetbuf_copyfrom("Hello", 6);
		broadcast_send(&broadcast);
		printf("broadcast message sent\n");
	}
	PROCESS_END();
}

