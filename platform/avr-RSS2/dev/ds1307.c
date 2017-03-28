/*
 * Copyright (c) 2005-2010, Swedish Institute of Computer Science
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
 * Authors  : Maximus Byamukama, Flavia Nshemerirwe
 * Created : 2017-03-27
 */

#include "contiki.h"
#include <compat/twi.h>
#include <stdio.h>
#include "dev/ds1307.h"
#include "dev/i2c.h"

/*---------------------------------------------------------------------------*/
/*Convert bcd value to decimal value*/
uint8_t
bcd_to_dec(uint8_t val)
{
	return( (val/16*10) + (val%16) );
}

/*Convert decimal value to bcd value */
uint8_t
dec_to_bcd(uint8_t val)
{
	return( (val/10*16) + (val%10) );
}
/*---------------------------------------------------------------------------*/
/*initialize DS1307 RTC*/
void
DS1307_init(void)
{
	i2c_init(F_SCL);
	i2c_start(DS1307);
	i2c_write(DS1307+TW_WRITE);
	i2c_write(CONTROL_REGISTER);
	i2c_write(0x00);
	i2c_stop(); 
}
/*---------------------------------------------------------------------------*/
/*set rtc time*/
void
DS1307_set_time(uint8_t h, uint8_t m, uint8_t s)
{
	uint8_t adj = 0;
    if (h>11) 
    {
        h -= 12;  
        adj = 0x40;                    // set 12-hr mode
    }                 
	i2c_write_mem(DS1307, REG_SECS, dec_to_bcd(s));
	i2c_write_mem(DS1307, REG_MINS, dec_to_bcd(m));
	i2c_write_mem(DS1307, REG_HRS, dec_to_bcd(h)+adj);
}
/*---------------------------------------------------------------------------*/
/*set rtc date*/
void
DS1307_set_date(uint8_t d, uint8_t m, uint8_t y)
{
	i2c_write_mem(DS1307, REG_DAYS, dec_to_bcd(d));
	i2c_write_mem(DS1307, REG_MONTH, dec_to_bcd(m));
	i2c_write_mem(DS1307, REG_YRS, dec_to_bcd(y));
}

/*---------------------------------------------------------------------------*/
/*read rtc time**/
const char * DS1307_get_time()
{
	uint8_t h[1], m[1], s[1];
	static char time[10];
	i2c_read_mem(DS1307,REG_HRS, h, 1);
	i2c_read_mem(DS1307,REG_MINS, m, 1);
	i2c_read_mem(DS1307,REG_SECS, s, 1);
	if (h[0] & 0x40)                  // 12hr mode:
		h[0] &= 0x1F;                     // use bottom 5 bits (pm bit = temp & 0x20)
    else h[0] &= 0x3F;                // 24hr mode: use bottom 6 bits
	
	snprintf(time, 10, "%02d:%02d:%02d" ,bcd_to_dec(h[0]), bcd_to_dec(m[0]), bcd_to_dec(s[0]));
    return time;
}
/*---------------------------------------------------------------------------*/
/*read rtc date*/
const char * DS1307_get_date()
{
	uint8_t d[1], m[1], y[1];
	static char date[10];
	i2c_read_mem(DS1307, REG_DAYS, d, 1);
	i2c_read_mem(DS1307, REG_MONTH, m, 1);
	i2c_read_mem(DS1307, REG_YRS, y, 1);
	
	snprintf(date, 10, "%02d/%02d/%02d" ,bcd_to_dec(d[0]), bcd_to_dec(m[0]), bcd_to_dec(y[0]));
    return date;
}
 
