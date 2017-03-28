/*
 * Copyright (c) 2015, Copyright Markus Hidell, Robert Olsson
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
 * This file is part of the Contiki operating system.
 *
 *
 * Authors  : Maximus Byamukama, Flavia Nshemerirwe
 * Created : 2017-03-27
 */
 

#define GET_TIME	0
#define GET_DATE	1
#define SET_SEC   	2
#define SET_MIN		3
#define SET_HRS		4
#define SET_DAY		5
#define SET_MONTH	6
#define SET_YEAR	7

/*DS1307 RTC ROUTINES*/
#define REG_SECS 0x00
#define REG_MINS 0x01
#define REG_HRS 0x02
#define DAYOFWK_REGISTER 0x03
#define REG_DAYS 0x04
#define REG_MONTH 0x05
#define REG_YRS 0x06
#define CONTROL_REGISTER 0x07
#define RAM_BEGIN 0x08
#define RAM_END 0x3F

uint8_t bcd_to_dec(uint8_t val);
uint8_t dec_to_bcd(uint8_t val);
void DS1307_init(void);
const char * DS1307_get_time();
void DS1307_set_time(uint8_t h, uint8_t m, uint8_t s);
const char * DS1307_get_date();
void DS1307_set_date(uint8_t d, uint8_t m, uint8_t y);

