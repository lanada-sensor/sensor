/*
 * Copyright (c) 2007, Swedish Institute of Computer Science.
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
 */

/**
 * \file
 *         app-sink to collect data from nodes
 * \author
 *         Jinhwan, Jung <jhjung@lanada.kaist.ac.kr>
 */

#include "contiki.h"
#include "net/rime.h"
#include "net/netstack.h" // for using netstack

#include "dev/button-sensor.h"

#include "dev/leds.h"

#include "sys/etimer.h" //KJY
#include "ntp.h" //KJY
#include "sclock.h" //KJY

#include <stdio.h>


#define DATA 				0x10	// '00010000'
#define	SYNC_START			0x20	// '00100000'
#define	SYNC_ACK			0x42	// '01000010'
#define	SYNC_END			0x80	// '10000000'

/*---------------------------------------------------------------------------*/
PROCESS(app_sink_process, "Sensor network App sink for test start");
AUTOSTART_PROCESSES(&app_sink_process);
/*---------------------------------------------------------------------------*/
//static variables
typedef enum {DATA_state, DATA_flag, SYNC_state, END_flag, ERROR} input_flag;
static input_flag flag;

static uint8_t data_backup[PACKETBUF_SIZE];
static uint8_t length_backup;

//time sync
static struct ntp *ntp; //KJY
static struct sclock *sc; //KJY
/*---------------------------------------------------------------------------*/
//functions declaration
static void Data_print();
/*---------------------------------------------------------------------------*/

static void
recv_uc(struct unicast_conn *c, const rimeaddr_t *from)
{
	uint8_t* packet_temp;
	uint8_t check_bit;
	packet_temp=(uint8_t*)packetbuf_dataptr();

	check_bit=packet_temp[0];

	if(check_bit==0x10)// check_bit == '00'
	{
		flag = DATA_flag;
	}
	else
	{
		printf("Error occur\n");
	}
	length_backup=packetbuf_datalen();
	packetbuf_copyto(data_backup);

	return;

/*	uint8_t* packet_temp;
	uint8_t check_bit;
	packet_temp=(uint8_t*)packetbuf_dataptr();
//	check_bit=0;

	check_bit=packet_temp[0];

	if(check_bit==0x10)// check_bit == '00'
	{
		Data_print();
	}
	else
	{
		printf("Error occur\n");
	}*/

}
static const struct unicast_callbacks unicast_callbacks = {recv_uc};
static struct unicast_conn uc;

static void
Data_print()
{
	uint16_t length=packetbuf_datalen(); //this length means bit level, ex) length=3 -> dataptr[0]=000xxxxx
	uint8_t index;
	int i; //loop variable
	uint8_t *dataptr_temp;

	dataptr_temp=(uint8_t *)packetbuf_dataptr();

	printf("App-sink Received DATA : ");
	for(i=0;i<length*8;i++)
	{
		index=i/8;
		if(dataptr_temp[index]>127) //if the value is greater than 127, the MSB of byte is '1'
		{
			printf("1");
		}
		else
		{
			printf("0");
		}
		dataptr_temp[index]=dataptr_temp[index]<<1;
		if(i%8==7)
		{
			printf(" ");
		}
	}
	printf("\n");

}
static uint8_t
Send(uint8_t type)
{

	if (type == SYNC_START)//if type is SYNC
	{
		printf("[APP] Send sync start!\n");
		packetbuf_set_attr(PACKETBUF_ATTR_PACKET_TYPE,SYNC_START);
		//send SYNC_start to PREV node
		return unicast_send(&uc, packetbuf_addr(PACKETBUF_ADDR_PREVIOUS));//TR result return
	}
	else
	{
		printf("[APP SINK] error: wrong type\n");
	}

	return 0;
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(app_sink_process, ev, data)
{
	static struct etimer et;
	PROCESS_EXITHANDLER(unicast_close(&uc);)

	PROCESS_BEGIN();

	//sync init : KJY
	sclock_create(&sc, TYPE_CLOCK);
	ntp_create(&ntp);

	unicast_open(&uc, 146, &unicast_callbacks);
	printf("Sink channel open\n waiting for data\n");
	NETSTACK_RDC.on();
	while(1) {
		etimer_set(&et, CLOCK_SECOND/1000);
		PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
		  if(flag)
		  {

			  switch(flag)
			  {
			  case DATA_flag:
				  flag = SYNC_state;
				  packetbuf_copyfrom(data_backup,length_backup);
				  Data_print();
				  packetbuf_clear();
				  Send(SYNC_START);
				  break;
			  case SYNC_state:
				  break;
			  case END_flag:
				  flag=DATA_state;
				  break;
			  }
		  }
	}

	PROCESS_END();
}
/*---------------------------------------------------------------------------*/
