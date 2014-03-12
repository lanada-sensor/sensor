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
 *         APP layer for sensor network
 * \author
 *         Jinhwan, Jung <jhjun@lanada.kaist.ac.kr>
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
#include <stdlib.h>
#define DEBUGPRINT 1

#define DATA 				0x10	// '00010000'
#define	SYNC_START			0x20	// '00100000'
#define	SYNC_ACK			0x42	// '01000010'
#define	SYNC_END			0x80	// '10000000'

/*---------------------------------------------------------------------------*/
PROCESS(app_layer_process, "Sensor network App layer start");
AUTOSTART_PROCESSES(&app_layer_process);
/*---------------------------------------------------------------------------*/
//static variables
typedef enum {DATA_flag=1, SYNC_flag, END_flag} input_flag;

static uint8_t result_data;

static uint8_t is_data_or_sync;
static input_flag flag;

static int clock_drift;
static uint8_t is_sleep_mode;

static uint8_t data_backup[PACKETBUF_SIZE];
static uint8_t length_backup;

//time sync
static struct etimer et;
static struct ntp *ntp; //KJY
static struct sclock *sc; //KJY
static struct timestamp sleeptime;

/*---------------------------------------------------------------------------*/
//functions declaration
static void Address_setup();
static int Sensor_start();
static uint8_t Sensor_calc(int);
static uint8_t Plb_on();
static uint8_t Send(uint8_t);
static void Data_aggregation();
static int Sync_calc();
static void Sync_modifying(int);

/*---------------------------------------------------------------------------*/
//callback functions
static void
recv_uc(struct unicast_conn *c, const rimeaddr_t *from)
{
	printf("[app] recv callback\n");
	uint8_t* packet_temp;
	uint8_t check_bit;
	packet_temp=(uint8_t*)packetbuf_dataptr();
//	printf("datalen : %d, %x %x\n",packetbuf_datalen(),packet_temp[0],packet_temp[1]);

	check_bit=packet_temp[0]; //6 right shifts
	if(check_bit==DATA) // check_bit == '0x10'
	{
		is_data_or_sync=DATA;
		flag=DATA_flag;
	}
	else if(check_bit==SYNC_ACK) // check_bit == '0x42'
	{
		is_data_or_sync=SYNC_ACK;
		flag=SYNC_flag;
	}
	else if(check_bit==SYNC_END) // check_bit == '0x80'
	{
		is_data_or_sync=SYNC_END;
		flag=END_flag;
	}
	else
	{
		is_data_or_sync=0;
	}
	length_backup=packetbuf_datalen();
	packetbuf_copyto(data_backup);
//	printf("data_backup %x %x length : %d\n",data_backup[0],data_backup[1],length_backup);
	return;
/*	switch(is_data_or_sync)
	{
	case DATA: Data_aggregation();
	Send(DATA);
#if DEBUGPRINT
	printf("Receiving DATA and Sending aggregation data\n");
#endif
	break;
	case SYNC_ACK: clock_drift=Sync_calc();
	Sync_modifying(clock_drift);
	Send(SYNC_ACK);
#if DEBUGPRINT
	printf("Receiving SYNC and Sending a SYNC start signal\n");
#endif
	break;
	case SYNC_END: is_sleep_mode=1;
#if DEBUGPRINT
	printf("Receiving END\n");
#endif
	break;
	default:printf("exception case, ERROR occur\n");
	break;
	}*/
}
static void
sent_uc(struct unicast_conn *c, int status, int num_tx)
{
	printf("sent callback [status : %d]\n",status);
	flag=END_flag;
	//printf("unicast message sent status : %d, number of tx : %d\n message is %s\n",status,num_tx,(char*)packetbuf_dataptr());
}
 //it is no longer useful
static const struct unicast_callbacks unicast_callbacks = {recv_uc,sent_uc};//sent_uc is not used at the present stage
static struct unicast_conn uc;
/*---------------------------------------------------------------------------*/
//Address function
static void
Address_setup()
{
	rimeaddr_t next_node_addr;
	rimeaddr_t prev_node_addr;
	if(rimeaddr_node_addr.u8[0]==1)
	{
		packetbuf_set_datalen(0);

	}
	next_node_addr.u8[0]=rimeaddr_node_addr.u8[0]+1;
	next_node_addr.u8[1]=rimeaddr_node_addr.u8[1];
	prev_node_addr.u8[0]=rimeaddr_node_addr.u8[0]-1;
	prev_node_addr.u8[1]=rimeaddr_node_addr.u8[1]; //it only can handle the case of MAX_NODE <256

	packetbuf_set_addr(PACKETBUF_ADDR_NOW,&rimeaddr_node_addr);
	packetbuf_set_addr(PACKETBUF_ADDR_NEXT,&next_node_addr);
	packetbuf_set_addr(PACKETBUF_ADDR_PREVIOUS,&prev_node_addr);

#if DEBUGPRINT
	printf("[app] Setting addresses\n");
#endif
	//address setting using packetbuf_set_addr
}
/*---------------------------------------------------------------------------*/
//Sensor functions
static int //return type check
Sensor_start()
{
	//sending a signal to Sensor to operate
#if DEBUGPRINT
	printf("[app] Sensing complete\n");
#endif
	return 1; //return sensing result
}
static uint8_t
Sensor_calc(int data)
{
	//calculate sensing data and return result value 0 or 1
#if DEBUGPRINT
	printf("[app] Sensing calculation complete\n");
#endif
	return 1;
}
/*---------------------------------------------------------------------------*/
//plb signals
static uint8_t
Plb_on()
{

	//sending a signal to plb layer
#if DEBUGPRINT
	printf("[app] plb on request\n");
#endif
	NETSTACK_RDC.on();
	return 1;
}

static uint8_t
Send(uint8_t type)
{

	if(type==DATA) //if type is DATA
	{
#if DEBUGPRINT
	printf("[app] send data (%d)\n",result_data);
#endif
		packetbuf_set_attr(PACKETBUF_ATTR_PACKET_TYPE,DATA);
		//send DATA to NEXT node
		return unicast_send(&uc, packetbuf_addr(PACKETBUF_ADDR_NEXT));//TR result return
	}
	else//if type is SYNC
	{
		packetbuf_set_attr(PACKETBUF_ATTR_PACKET_TYPE,SYNC_START);
		//send SYNC_start to PREV node
		return unicast_send(&uc, packetbuf_addr(PACKETBUF_ADDR_PREVIOUS));//TR result return
	}
}
/*---------------------------------------------------------------------------*/
//data function
static void
Data_aggregation()
{
	uint16_t length=packetbuf_datalen();
	uint16_t node_num=(uint16_t)rimeaddr_node_addr.u8[0]; //this length means bit level, the bit position that is result data locates
	uint8_t shift_amt,index; //ptr index value, the number of shift operations to apply
	uint8_t *dataptr_temp;
	dataptr_temp=(uint8_t *)packetbuf_dataptr();
//	printf("Data_aggregation %d %x %x, total length %d\n",length,dataptr_temp[0],dataptr_temp[1],packetbuf_totlen());
	if(node_num==1) //it means START node case, set DATA bits at first byte
	{
		dataptr_temp[0]=DATA;// first byte = 0x10
		length=1;
	}
	if(node_num%8)
	{
		shift_amt=8-node_num%8;
	}
	else //node_num%8 == 0
	{
		shift_amt=0;
	}

	if(node_num%8==1)
	{
			length++;
	}
	index=length-1;
	dataptr_temp[index]|=result_data<<shift_amt;

	packetbuf_set_datalen(length);
	//data aggregation using dataptr
#if DEBUGPRINT
	printf("[app] received data aggregation\n");
#endif
}
/*---------------------------------------------------------------------------*/

PROCESS_THREAD(app_layer_process, ev, data)
{
	int sensor_value; // type check
	static struct etimer et;
	//data_backup=(uint8_t*)malloc(PACKETBUF_SIZE*sizeof(uint8_t));
	PROCESS_EXITHANDLER(unicast_close(&uc);)

	PROCESS_BEGIN();

	//sync init : KJY
	sclock_create(&sc, TYPE_CLOCK);
	ntp_create(&ntp);

#if DEBUGPRINT
	printf("PROCESS BEGINNING\n");
#endif
	SENSORS_ACTIVATE(button_sensor);
	unicast_open(&uc, 146, &unicast_callbacks);
#if DEBUGPRINT
	printf("[app] Setting a unicast channel\n");
#endif
	Address_setup();

	

	while(1) {
		is_sleep_mode=0;
		sensor_value=Sensor_start();
		result_data=Sensor_calc(sensor_value);

		PROCESS_WAIT_EVENT_UNTIL((ev==sensors_event) && (data == &button_sensor));

		Plb_on();
		if(rimeaddr_node_addr.u8[0]==1 && rimeaddr_node_addr.u8[1]==0)
		{
			packetbuf_clear();
			Data_aggregation();
			Send(DATA);
		}
#if DEBUGPRINT
		printf("[app] waiting until intterupt\n");
#endif
		//PROCESS_WAIT_EVENT_UNTIL(is_sleep_mode);
		while(!is_sleep_mode) // can i replace it with PROCESS_WAIT_UNTIL or some PROCESS function?
		{
//			printf("app layer! %d\n",flag);
			  etimer_set(&et, CLOCK_SECOND/1000);
			  PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
			  if(flag)
			  {
				  printf("[app] flag value : %d\n",flag);
				  switch(flag)
				  {
				  case DATA_flag:
					  flag=0;
					  packetbuf_copyfrom(data_backup,length_backup);
//					  printf("APP DATA_flag %x %x\n",data_backup[0],data_backup[1]);
//					  printf("APP DATAPTR %x %x\n",((uint8_t*)packetbuf_dataptr())[0],((uint8_t*)packetbuf_dataptr())[1]);
					  Data_aggregation();
					  Send(DATA);
					  break;
				  case SYNC_flag:
					  flag=0;
					  //sync calc
					  packetbuf_clear();
					  packetbuf_copyfrom(data_backup,length_backup);
//					  printf("[app] len : %d //kdw\n",packetbuf_datalen());
//					  print_packet(packetbuf_dataptr(),packetbuf_datalen());
					  ntp_handle_ack(ntp, packetbuf_dataptr());//KJY
					  if (rimeaddr_node_addr.u8[0]==1 && rimeaddr_node_addr.u8[1]==0){
						  is_sleep_mode=1;
						  NETSTACK_RDC.off(0);
						  break;
					  }
					  packetbuf_clear();
					  printf("[app] Synchronized!!");
					  print_ntp(ntp);
					  Send(SYNC_START);
					  break;
				  case END_flag:
					  flag=0;
					  is_sleep_mode=1;
					  NETSTACK_RDC.off(0);
					  break;
				  }
			  }
			//waiting
		} //JJH55
#if DEBUGPRINT
		printf("[app] goto sleep mode, wake up after 21days\n");
#endif
		NETSTACK_RDC.off(0);

	    leds_on(LEDS_RED);
	    etimer_set(&et, CLOCK_SECOND/2);
	    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
	    leds_off(LEDS_RED);

	    timestamp_init(&sleeptime);
		sleeptime.sec = 10;
	    while(!timestamp_is_empty(&sleeptime)){
	      sclock_etimer_set_long(sc, &et, &sleeptime);
	      PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));

	    }
	    printf("[app] sleep end\n");

	    leds_on(LEDS_YELLOW);
	    etimer_set(&et, CLOCK_SECOND/2);
	    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
	    leds_off(LEDS_YELLOW);
	}



	PROCESS_END();
}
/*---------------------------------------------------------------------------*/
