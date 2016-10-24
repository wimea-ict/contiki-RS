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
 *
 *  @Author: Nsabagwa Mary
 * The application reads sensor data, transmits it via broadbast, unicast .. using RIME
 */
/*---------------------------------------------------------------------------*/
#include "contiki.h"
#include "wimea-ict-rss2.h"
#include "dev/temp_mcu-sensor.h"
#include "dev/temp-sensor.h"
#include <stdio.h> /* For printf() */
#include "serial-shell.h"
#include "dev/serial-line.h"
#include <string.h>
#include "params.h"

#define PRINT_INTERVAL 5 * CLOCK_SECOND

/*---------------------------------------------------------------------------*/
PROCESS(temp_read_process, "Temperature Readings");
PROCESS(serial_input, "Serial line input commands");
//PROCESS(temp_surounding_read_process, "Temperature Readings");
AUTOSTART_PROCESSES(&temp_read_process,&serial_input);
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(temp_read_process, ev, data)
{
  static struct etimer etimer;
  unsigned int temp_m=0;
  unsigned int temp=0;
  PROCESS_BEGIN();
  
  SENSORS_ACTIVATE(temp_mcu_sensor);
  SENSORS_ACTIVATE(temp_sensor);
  while(1) {
    etimer_set(&etimer, PRINT_INTERVAL);
	temp_m = temp_mcu_sensor.value(TEMP_MCU_SENSOR);
	temp = temp_sensor.value(TEMP_SENSOR);
    PROCESS_WAIT_EVENT();
    printf("MCU Temperature:%d ,", (int)temp_m);//temp_mcu_sensor.value(TEMP_MCU_SENSOR)
   	printf("Surrounding Temperature:%d \n", (int)temp );//temp_sensor.value(TEMP_SENSOR)
   	
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
     
     if(ev == serial_line_event_message) {
       printf("received line: %s\n", (char *)data);
     } 
	 
     command = (char*)strtok((char*)data, (const char*)delimiter) ;		

     if(!strncmp(command, "h",1)){
	print_help_command_menu() ;
     } else if(!strncmp(command,"ri",2)) {
		display_default_interval();
     } else {
	printf("Invalid command %s. Try h for a list of commands\n", command) ;
     }	// more commands can be added here

   }
   	
   PROCESS_END();
}

void display_default_interval(){
printf("Current Interval is %d\n",params_get_transmission_interval());

}

void print_help_command_menu(){
 printf("\n------------------ Menu--------------------------------\n") ;
 printf("\n Prints a list of supported commands \t Usage: h") ;
 printf("\n Set the report interval \t  Usage: ri <period in seconds>");
 printf("\n Set the report tag mask \t Usage: tagmask <var1,var2>\n");
 printf("---------------------------------------------------------------\n\n");

}
/*---------------------------------------------------------------------------*/
