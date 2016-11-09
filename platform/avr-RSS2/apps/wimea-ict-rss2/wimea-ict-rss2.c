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
 *  @Author: Nsabagwa Mary
 * The application reads sensor data, transmits it via broadbast, unicast .. using RIME
 */
/*---------------------------------------------------------------------------*/
#include "contiki.h"
#include "wimea-ict-rss2.h"
#include <avr/eeprom.h>
#include "dev/temp_mcu-sensor.h"
#include "dev/temp-sensor.h"
#include <stdio.h> /* For printf() */
#include "serial-shell.h"
#include "dev/serial-line.h"
#include <string.h>
#include "params.h"

#define PRINT_INTERVAL 5 * CLOCK_SECOND

//Configuration Parameters
#ifdef TRANSMISSION_INTERVAL
uint16_t eemem_transmission_interval EEMEM = TRANSMISSION_INTERVAL;
#else
uint16_t eemem_transmission_interval EEMEM = 60;
#endif  
uint16_t eemem_node_name EEMEM;

uint16_t saved_interval,node_name,h,m;
char buff[6];
char * value;
double v_avr,temp;
//unsigned char eui64_addr[8] ;

/*---------------------------------------------------------------------------*/
PROCESS(temp_read_process, "Temperature Readings");
PROCESS(serial_input, "Serial line input commands");
PROCESS(save_config_process, "Save configurations");
//PROCESS(temp_surounding_read_process, "Temperature Readings");
AUTOSTART_PROCESSES(&temp_read_process,&serial_input,&save_config_process);

PROCESS_THREAD(save_config_process, ev, data){
 PROCESS_BEGIN();
 cli(); 
 saved_interval = eeprom_read_word(&eemem_transmission_interval);
 sei(); 
if(itoa(saved_interval,buff,6)==NULL){
  cli();
  eeprom_write_word(&eemem_transmission_interval, 60);
  sei();} 
  //get_eui64_addr(eui64_addr);
  PROCESS_END();  
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(temp_read_process, ev, data)
{
  static struct etimer etimer;
  unsigned int temp_m=0;
  unsigned int temp=0;
  PROCESS_BEGIN();
  
  SENSORS_ACTIVATE(temp_mcu_sensor);
  SENSORS_ACTIVATE(temp_sensor);
  cli();
	saved_interval = eeprom_read_word(&eemem_transmission_interval);
  sei();
  
  while(1) {
    etimer_set(&etimer, saved_interval);//PRINT_INTERVAL
	temp_m = temp_mcu_sensor.value(TEMP_MCU_SENSOR);
	temp = temp_sensor.value(TEMP_SENSOR);
    PROCESS_WAIT_EVENT();
    printf("MCU Temperature:%d ,",(int)temp_m);//, (int)temp_m //temp_mcu_sensor.value(TEMP_MCU_SENSOR)
   	printf("Temperature:%d\n", (int)temp );//, (int)temp //temp_sensor.value(TEMP_SENSOR)   	
  }
  
  PROCESS_END();
}

PROCESS_THREAD(serial_input, ev, data)
{
   PROCESS_BEGIN();

   char delimiter[] = " ";
   char *command = NULL ;
      
   for(;;) {
     PROCESS_YIELD_UNTIL(ev == serial_line_event_message);
     
    /* if(ev == serial_line_event_message) {
       printf("received line: %s\n", (char *)data);
     } */
	 
     command = (char*)strtok((char*)data, (const char*)delimiter) ;		
    //printf("the command is %s",command);
     if(!strncmp(command, "h",1)){
	print_help_command_menu() ;
     } else if(!strncmp(command,"ri",2)) {
	    value=(char*) malloc(3);		
		strncpy(value,command+3,6);//(strlen(command)-3)
		if(strlen(command)>2)
			change_reporting_interval(value);
		else 
			display_reporting_interval();
     }else if(!strncmp(command,"ss",2)) {//system summary
	        display_system_information();
     }
	 else if(!strncmp(command,"name",4)) {
	    value=(char*) malloc(12);		
		strncpy(value,command+5,12);//(strlen(command)-3)
		if(strlen(command)>4)
			change_node_name(value);
		else 
			display_node_name();
     }
	 else if(!strncmp(command,"u",1)) {//system summary
	        display_system_uptime();
     }
	 else {
	printf("Invalid command %s. Try h for a list of commands\n", command) ;
     }	// more commands can be added here
   }   	
   PROCESS_END();
}
//change node name 
void change_node_name(char* value){
   // int n = atoi(value);
	cli();
	eeprom_write_word(&eemem_node_name, (uint16_t)value);  
	sei();
	printf("Node Name changed to %s\n",value);
}

void display_node_name(){
cli();
node_name = eeprom_read_word(&eemem_node_name);
sei();
  printf("Node Name= %s\n",(char *) node_name);
}

//changing reporting interval
void change_reporting_interval(char* value){

    int interval = atoi(value);
	cli();
	eeprom_write_word(&eemem_transmission_interval, interval);  
	sei();
	printf("Interval changed to %d\n",interval);
}
void display_system_uptime(){
    printf("System has been up for %d Ticks \n", CLOCK_SECOND);// clock_time()
}
//Display Mote information
void display_system_information(){
 printf("System Information");
}

//Displaying reporting interval 
void display_reporting_interval(){
cli();
saved_interval = eeprom_read_word(&eemem_transmission_interval);
sei();
  printf("Current Interval is %d\n",saved_interval);
}



void read_sensors(void)
{
  //uint16_t s;
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
  m = h/10; 
 // s=h-10*m;

  for ( p1=16; p1<31; p1++) { 
    BATMON = p1;
    /*  delay_us(100); */
    if ((BATMON & (1<<BATMON_OK)) == 0 ) 
      break;
  }
  v_avr = (double) 2550-75*16-75 + 75 * p1; //-75 to take the floor of the 75 mv transition window
  v_avr = v_avr/1000;
}


//menu 
void print_help_command_menu(){
 printf("\n------------------ Menu--------------------------------\n") ;
 printf("\n Prints a list of supported commands \t Usage: h") ;
 printf("\n Display System Summary \t Usage: ss");
 printf("\n Display System Uptime \t Usage: u");
 printf("\n Set/Display Node Name \t Usage: name <node name>");
 printf("\n Set/Display reporting interval \t  Usage: ri <period in seconds>");
 printf("\n Set the report tag mask \t Usage: tagmask <var1,var2>\n"); 
 printf("---------------------------------------------------------------\n\n");
}
/*---------------------------------------------------------------------------*/
