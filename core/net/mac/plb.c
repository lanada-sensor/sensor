#include "net/mac/plb.h"
#include "net/packetbuf.h"
#include "net/queuebuf.h"
#include "net/netstack.h"
#include "sys/pt.h"
#include "sys/rtimer.h"
#include "net/rime.h"
#include "ntp.h" //KJY
#include "sclock.h" //KJY
#include <string.h>/*need?*/
#include <stdio.h>

#define DEBUG 1
#if DEBUG
#define PRINTF(...) printf("[PLB] ");printf(__VA_ARGS__)
#define PRINTFF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#define PRINTFF(...)
#endif

#define DEBUG_PACKET 0

#define DEBUG_POWER_CYCLE 0
#if DEBUG_POWER_CYCLE
#define PRINT_P(...) printf("[PLB] ");printf(__VA_ARGS__)
#else
#define PRINT_P(...)
#endif

/*---------------------------------------------------------------------------*/
/* Constans */
#define RTIMER_ARCH_MSECOND RTIMER_ARCH_SECOND/1000
#define PC_ON_TIME (RTIMER_ARCH_SECOND / 160)
#define PC_OFF_TIME (RTIMER_ARCH_SECOND / NETSTACK_RDC_CHANNEL_CHECK_RATE - PC_ON_TIME)

#define MAX_STROBE_SIZE 100
#define MAX_ACK_SIZE 100
#define MAX_SYNC_SIZE (100+1+(TIMESTAMP_BYTE*3))

#define NETWORK_HDR_SIZE 6
#define FRAMER_HDR_SIZE 9

#define INTER_PACKET_INTERVAL              RTIMER_ARCH_SECOND / 5000
#define ACK_LEN 100
#define DEBUG
#define AFTER_ACK_DETECTECT_WAIT_TIME      RTIMER_ARCH_SECOND / 1000

#define TIMEOUT_COUNT 100
//////////////////////////

#define DEFAULT_STROBE_WAIT_TIME (5 * PC_ON_TIME / 8)

struct plb_config plb_config = { PC_ON_TIME, PC_OFF_TIME, 4 * PC_ON_TIME
		+ PC_OFF_TIME, DEFAULT_STROBE_WAIT_TIME };
/*---------------------------------------------------------------------------*/
// Static variables : PACKET_TYPE
#define BEACON_SD 			0x02	// '00000010'
#define BEACON_SD_ACK 		0x03	// '00000011'
#define BEACON_DS 			0x04	// '00000100'
#define BEACON_DS_ACK		0x05	// '00000101'
#define PREAMBLE	 		0x08	// '00001000'
#define PREAMBLE_ACK 		0x09	// '00001001'
#define PREAMBLE_ACK_DATA 	0x19	// '00011001'
#define DATA 				0x10	// '00010000'
#define DATA_ACK			0x11	// '00010001'
#define	SYNC_START			0x20	// '00100000'
#define SYNC_REQ			0x21	// '00100001'
#define	SYNC_ACK			0x42	// '01000010'
#define	SYNC_END			0x80	// '10000000'
/*---------------------------------------------------------------------------*/
static struct rtimer rt;
static struct pt pt;

static int is_init;
static int is_plb_on;
static int is_radio_on;

static int has_data;
static int send_req;
static int wait_packet;

static int c_wait;
static int sync_state;	//kdw2
static int send_fail; //JJH 0220
static int send_fail_sync; //JJH 0221
static int data_ack_check; //JJH 0224 to check data_ack for timeout_nonfail function


static uint16_t timeout_count;
static uint16_t timeout_nonfail_count; //JJH 0224

static rimeaddr_t addr_next;
static rimeaddr_t addr_prev;
static rimeaddr_t addr_ack;

static uint8_t *global_dataptr_temp;
static int temp_len;

static uint8_t *global_dataptr_temp_recv;
static int temp_recv_len;

//time sync
static struct ntp *ntp; //KJY
static struct sclock *sc; //KJY

// send
static mac_callback_t sent_callback;
static void* sent_ptr;

/*---------------------------------------------------------------------------*/
static void plb_init(void);
static void plb_send(mac_callback_t sent, void *ptr);
static void plb_send_list(mac_callback_t sent, void *ptr,
struct rdc_buf_list *buf_list);
static void plb_input(void);
static int plb_on(void);
static int plb_off(int keep_radio_on);
static unsigned short plb_channel_check_interval(void);
/*---------------------------------------------------------------------------*/
static int plb_beacon_sd(void);
static int plb_beacon_ds(void);
static char plb_powercycle(void); // return type is char?
static char plb_send_strobe(rimeaddr_t *dst, int *acked, uint8_t type);
static void radio_on();
static void radio_off();
static int plb_create_header(rimeaddr_t *dst, uint8_t type);
static int plb_create_header_data(rimeaddr_t *dst, uint8_t type);
static int plb_wait_ack(uint8_t sending_type);
static int plb_wait_data_ack(uint8_t sending_type);
static int plb_send_data(mac_callback_t sent, void *ptr);
static int plb_send_sync_start(void);
static int plb_send_sync(uint8_t type);
static int plb_send_sync_end(uint8_t type);
static int plb_send_sync_ack(uint8_t type);
static int plb_send_sync_req(void);
static void plb_timeout(void);
static void plb_nonfail_timeout(void);
static void print_packet(uint8_t *packet, int len);
static void hold_time(rtimer_clock_t interval);
/*---------------------------------------------------------------------------*/
static int plb_beacon_sd(void) {
	int acked = 0;
	PRINTF("plb_beacon_sd;  to %u.%u \n", addr_next.u8[0], addr_next.u8[1]);

	// send strobe
	if (plb_send_strobe(&addr_next, &acked, BEACON_SD) < 0) {
		return MAC_TX_ERR_FATAL;
	}

	// check ack
	if (acked == 1) {
		PRINTF("plb_beacon_sd : acked -> set c_wait\n");
		c_wait = 1;
	}
	return 0;
}
/*---------------------------------------------------------------------------*/
static int plb_beacon_ds(void) {
	int acked = 0;
	PRINTF("plb_beacon_ds;  to %u.%u \n", addr_prev.u8[0], addr_prev.u8[1]);

	if (addr_prev.u8[0] == 0 && addr_prev.u8[1] == 0) {
		PRINTF("plb_beacon_ds : no beacon (first node)\n");
		return -1;
	}

	/* send beacon */
	if (plb_send_strobe(&addr_prev, &acked, BEACON_DS) < 0) {
		return MAC_TX_ERR_FATAL;
	}

	/* wait for data */
	if (acked == 1) {
		PRINTF("plb_beacon_ds : acked\n");
	}
	return 0;
}
/*---------------------------------------------------------------------------*/
static char plb_powercycle(void) {

	PRINT_P("plb_powercycle [sr:%d cw:%d]\n",send_req,c_wait);

	PT_BEGIN(&pt);
	while (1) {
		if (is_plb_on == 0){
			break;
		}
		// check on/send state
		if (send_req == 1 && c_wait == 1&& !send_fail) { //add !send_fail JJH6
			PRINTF("plb_powercycle send DATA <start>\n");
			send_req = 0; //avoid repeat sending
			plb_send_data(sent_callback, sent_ptr);
			PRINTF("plb_powercycle send DATA <end>\n");
			radio_off();
		}
		if(send_fail || !c_wait || send_fail_sync)//1. fail to send preamble or data, 2. dest node doesn't wake up, 3. fail to send SYNC_ACK JJH 0222
		{
			timeout_count++;
			if(timeout_count==100)//after 100 powercycle, call timeout plb_function
			{
				PRINTF("powercycle TIMEOUT! restart beacon or sync sending!\n");
				plb_timeout();

			}
		}
		if(!send_fail && !send_fail_sync)
		{
			timeout_nonfail_count++;
			if(timeout_nonfail_count==200)//after 200 powercycle, call timeout_nonfail function JJH 0224
			{
				PRINTF("powercycle TIMEOUT! not by sending fail\n");
				plb_nonfail_timeout();
			}
		}
		/* on */
		radio_on();
		rtimer_set(&rt, RTIMER_NOW() + PC_ON_TIME, 1,
				(void (*)(struct rtimer *, void *)) plb_powercycle, NULL);
		PT_YIELD(&pt);

		/* off */
		if (sync_state == 1)
		{
			radio_on();
		}
		else
		{
			if (wait_packet == 0) {
				radio_off();
			} else if (wait_packet > 0) {
				wait_packet = 0;
			}
		}
		rtimer_set(&rt, RTIMER_NOW() + PC_OFF_TIME, 1,
				(void (*)(struct rtimer *, void *)) plb_powercycle, NULL);
		PT_YIELD(&pt);
	}
	PT_END(&pt);
}
/*---------------------------------------------------------------------------*/
static char plb_send_strobe(rimeaddr_t *dst, int *acked, uint8_t type) {
	PRINTF(
			"plb_send_strobe  [dst: %u.%u] [type: %x] ", dst->u8[0], dst->u8[1], type);

	uint8_t strobe[MAX_STROBE_SIZE];
	int strobe_len = 0;
	int strobe_num = 0;
	rtimer_clock_t t0;
	rtimer_clock_t t;

	// Make PLB header
	packetbuf_clear();
	if ((strobe_len = plb_create_header(dst, type)) < 0) {
		return -1;
	}

	// Make packet -> strobe
	strobe_len = strobe_len + 1; // assign space for packet type
	if (strobe_len > (int) sizeof(strobe)) {
		/* Failed to send */
		PRINTF("plb: send failed, too large header\n");
		return -1;
	}
	memcpy(strobe, packetbuf_hdrptr(), strobe_len);

	/* Send beacon and wait ack : STROBE_NUM_MAX times */
	radio_on();

	t0 = RTIMER_NOW();
	t = RTIMER_NOW();
	for (strobe_num = 0;
			((*acked) == 0)
						&& RTIMER_CLOCK_LT(RTIMER_NOW(), t0 + (plb_config.strobe_time));
			strobe_num++) {

		while (((*acked) == 0)
				&& RTIMER_CLOCK_LT(RTIMER_NOW(), t + plb_config.strobe_wait_time)) {

		}
		t = RTIMER_NOW();

#if DEBUG_PACKET
		print_packet(strobe, strobe_len);
#endif

		if (NETSTACK_RADIO.send(strobe, strobe_len) != RADIO_TX_OK) {
			PRINTFF("E");
		}
		(*acked) = plb_wait_ack(type);
//		if (*acked) {
//			PRINTF("ack! return: %d\n", *acked);
//		}
	}

	if (acked == 0) {
		radio_off();
	}

	PRINTFF(" strobe done [try: %d]\n", strobe_num);
	return 0;
}
/*---------------------------------------------------------------------------*/
/* Put common frame header into packetbuf */
static int plb_create_header(rimeaddr_t *dst, uint8_t type) {
	int i = 0;

	/* Set address */
	packetbuf_set_addr(PACKETBUF_ADDR_SENDER, &rimeaddr_node_addr);
	packetbuf_set_addr(PACKETBUF_ADDR_RECEIVER, dst);

	// setting a type of packet at the dataptr's first byte(8bit).
	int length;
	length = packetbuf_datalen();
	uint8_t *dataptr_temp;
	dataptr_temp = (uint8_t *) packetbuf_dataptr();
	for (i = length; i > 0; i--) {
		dataptr_temp[i] = dataptr_temp[i - 1]; //dataptr data shift to insert type at first byte
	}

	dataptr_temp[0] = type;
	packetbuf_set_datalen(++length);

	/* Create frame */
	int len = 0;
	len = NETSTACK_FRAMER.create();

	return len;
}
/*---------------------------------------------------------------------------*/
static int plb_create_header_data(rimeaddr_t *dst, uint8_t type) {

	/* Set address */
	packetbuf_set_addr(PACKETBUF_ADDR_SENDER, &rimeaddr_node_addr);
	packetbuf_set_addr(PACKETBUF_ADDR_RECEIVER, dst);

	/* Create frame */
	int len = 0;
	len = NETSTACK_FRAMER.create();

	return len;
}
/*---------------------------------------------------------------------------*/
static int plb_wait_ack(uint8_t sending_type) {

	uint8_t ackbuf[ACK_LEN + 2];
	uint8_t type;
	int len;

	wait_packet = 1;

	int ack_received = 0;
	hold_time(INTER_PACKET_INTERVAL);

	/* Check for incoming ACK. */
	if ((NETSTACK_RADIO.receiving_packet() || NETSTACK_RADIO.pending_packet()
			|| NETSTACK_RADIO.channel_clear() == 0)) {

		hold_time(AFTER_ACK_DETECTECT_WAIT_TIME);

		while (1) {
			len = NETSTACK_RADIO.read(ackbuf, ACK_LEN);
			if (len == 0)
				break;

			type = ackbuf[FRAMER_HDR_SIZE];
			if (((sending_type + 1) & type) == type) {
				ack_received = 1;
				if (type == PREAMBLE_ACK_DATA) {
					ack_received = 2; // preamble_ack_data JJH
				}
				else if (type == SYNC_REQ){
					temp_recv_len = 5;
					memcpy(global_dataptr_temp_recv, ackbuf+FRAMER_HDR_SIZE+1, temp_recv_len);
#if DEBUG_PACKET
	printf("plb_wait_ack :");print_packet(global_dataptr_temp_recv, temp_recv_len);
#endif
				}
				break;
			}

		}

	}
	wait_packet = 0;
	return ack_received;
}
/*---------------------------------------------------------------------------*/
static int plb_wait_data_ack(uint8_t sending_type) {

	uint8_t ackbuf[ACK_LEN + 2];
	uint8_t type;
	int len;

	wait_packet = 1;

	int ack_received = 0;
	hold_time(INTER_PACKET_INTERVAL * 100); // ?????

	/* Check for incoming ACK. */
	if ((NETSTACK_RADIO.receiving_packet() || NETSTACK_RADIO.pending_packet()
			|| NETSTACK_RADIO.channel_clear() == 0)) {

		hold_time(AFTER_ACK_DETECTECT_WAIT_TIME);
		while (1) {
			len = NETSTACK_RADIO.read(ackbuf, ACK_LEN);
			if (len == 0)
				break;

#if DEBUG_PACKET
			print_packet(ackbuf, len);
#endif

			type = ackbuf[len - 1];
			if (((sending_type + 1) & type) == type) {
				ack_received = 1;
				break;
			}

		}
	}
	wait_packet = 0;
	return ack_received;
}
/*---------------------------------------------------------------------------*/
static void plb_send_ack(uint8_t type) {

	uint8_t ack[MAX_ACK_SIZE];
	int ack_len = 0;

	//set address from input packet
	addr_ack.u8[0] = packetbuf_addr(PACKETBUF_ADDR_SENDER)->u8[0];
	addr_ack.u8[1] = packetbuf_addr(PACKETBUF_ADDR_SENDER)->u8[1];

	PRINTF("plb_send_ack [dst: %u.%u] [type: %x]\n", addr_ack.u8[0], addr_ack.u8[1], type);

	packetbuf_clear();
	if ((ack_len = plb_create_header(&addr_ack, type)) < 0) {
		PRINTF("ERROR: plb_create_header ");
		return;
	}
	ack_len++;

	//Make ack frame//
	if (ack_len > (int) sizeof(ack)) {
		// Failed to send //
		PRINTF("plb: send failed, too large header\n");
		return;
	}

	memcpy(ack, packetbuf_hdrptr(), ack_len);

	// Send beacon and wait ack : STROBE_NUM_MAX times //
	radio_on();

#if DEBUG_PACKET
	print_packet(ack, ack_len);
#endif

	if (NETSTACK_RADIO.send(ack, ack_len) != RADIO_TX_OK) {
		PRINTF("ERROR: plb ack send\n");
		return;
	}

	radio_off();
	return;
}
/*---------------------------------------------------------------------------*/
static int plb_send_data(mac_callback_t sent, void *ptr) {

	PRINTF("plb_send_data\n");

	int ret;
	int last_sent_ok = 0;
	int acked;
	int temp = 0;
	int preamble_count=0; //PREAMBLE retransmission JJH6
	int data_count=0; //PREAMBLE retransmission JJH6
	acked = 0;

	while(preamble_count<3&&data_count<3){
		plb_send_strobe(&addr_next, &acked, PREAMBLE);
		if (acked == 1) {
			acked=0; //initialize acked value=0 JJH6
			PRINTF("plb_send_data DATA_PREAMBLE_ACKED!\n");
			packetbuf_clear();
			packetbuf_copyfrom(global_dataptr_temp, temp_len);
			if (plb_create_header_data(&addr_next, DATA) < 0) {
				PRINTF("ERROR: plb_create_header ");
				send_fail=1;//JJH 0221
				return -1; //ERROR case : -1
			}
			radio_on();
			while(data_count<3 && !acked)
			{
				PRINTF("plb_send_data send DATA packet\n");

#if DEBUG_PACKET
				print_packet(packetbuf_hdrptr(), packetbuf_totlen());
#endif

				if (NETSTACK_RADIO.send(packetbuf_hdrptr(), packetbuf_totlen())
						!= RADIO_TX_OK) {
					PRINTF("plb_send_data DATA error!\n");
					send_fail=1; //JJH 0221
					return -1; //ERROR case
					ret = MAC_TX_ERR;
				}
				acked = plb_wait_data_ack(DATA); //just once?

				if (acked == 1) //data ack received
				{
					PRINTF("plb_send_data DATA_ACKED!\n");
					last_sent_ok = 1;
					data_ack_check=1; //JJH 0224
					ret = MAC_TX_OK;
				} else if (!acked) //do not receive data ack, increase data_count JJH6
				{
					PRINTF("*DO NOT RECEIVED DATA ACK %d TRIAL!\n",++data_count);
					ret = MAC_TX_ERR;
				}
			}
			radio_off();
			return last_sent_ok;
		} else if (acked == 2) //if receive preamble ack data
		{
			PRINTF("PREAMBLE ACK DATA RECEIVED!\n");
			break;
		} else //do not receive preamble ack, increase preamble_count JJH6
		{
			PRINTF("DO NOT RECEIVED PREAMBLE ACK %d TRIAL!\n",++preamble_count);
		}
	}
	if(preamble_count==3||data_count==3) //it is case for send fail
	{
		send_fail=1;
	}

	return 0;
}
/*---------------------------------------------------------------------------*/
static int plb_send_sync_start(void) //kdw sync
{
	PRINTF("(sync) plb_send_sync_start\n");
	int acked = 0;
	int sync_start_count=0;
	sync_state = 1;

	while(sync_start_count<3 && !acked)
	{
		if (plb_send_strobe(&addr_prev, &acked, SYNC_START) < 0) {
			return MAC_TX_ERR_FATAL;
		}

		if (acked == 1) // check ack
		{
			plb_send_sync_ack(SYNC_ACK); // send sync_ack !!! // copy data at in function
		}
		else
		{
			PRINTF("(sync) plb_send_sync_start ERROR TRIAL %d\n",++sync_start_count); // try again //JJH 0221 make it try again
		}
	}
	if(sync_start_count==3)
	{
		send_fail_sync=1;
	}
	return 0;
}
/*---------------------------------------------------------------------------*/
static int plb_send_sync_req() //kdw sync
{
	sync_state = 1;

	// make packet //
	uint8_t sync[MAX_SYNC_SIZE];
	int sync_len = 0;
	int sync_type_len = 1; //1 byte for app check
	int sync_hdr_len = 0;
	rimeaddr_t * temp_addr;

	packetbuf_clear();

	temp_addr = &addr_next;

	if ((sync_hdr_len = plb_create_header(temp_addr, SYNC_REQ)) < 0) {
		PRINTF("ERROR: plb_create_header (plb_send_sync_req) ");
		return -1;
	}

	PRINTF("(sync) plb_send_sync [dst: %d.%d] [type: %x]\n", temp_addr->u8[0], temp_addr->u8[1], SYNC_REQ);
	sync_len = sync_hdr_len + sync_type_len;
	if (sync_len > (int) sizeof(sync)) {
		// Failed to send //
		PRINTF("plb: send failed, too large header\n");
		return -1;
	}

	memcpy(sync, packetbuf_hdrptr(), sync_len);

	// put timestamp : KJY
	ntp_make_request(ntp, sync+sync_len);
	sync_len += NTP_REQUEST_LEN;

#if DEBUG_PACKET
	printf("[PLB] send_req :");print_packet(sync, sync_len);
#endif

	// Send sync//
	radio_on();

	if (NETSTACK_RADIO.send(sync, sync_len) != RADIO_TX_OK)
	{
		PRINTF("ERROR: plb ack send\n");
		return -1;
	}
	return 0;
}
/*---------------------------------------------------------------------------*/
static int plb_send_sync_ack(uint8_t type) //kdw sync
{
	PRINTF("plb_send_sync\n");

	sync_state = 1;

	// make packet //
	uint8_t sync[MAX_SYNC_SIZE];
	int sync_len = 0;
	int sync_data_len = 1; //1 byte for app check
	int sync_hdr_len = 0;
	int sync_count=0; //JJH 0221
	int tx_result=1; //JJH 0221
	rimeaddr_t * temp_addr;

	packetbuf_clear();

	if (type == SYNC_ACK)
	{
		temp_addr = &addr_prev;

		packetbuf_clear();
		packetbuf_copyfrom(global_dataptr_temp, temp_len);
		((uint8_t*)packetbuf_dataptr())[temp_len]=type;
		ntp_make_ack(ntp, packetbuf_dataptr()+1+NETWORK_HDR_SIZE, global_dataptr_temp_recv);

		packetbuf_set_datalen(temp_len+1+NTP_ACK_LEN);

		sync_data_len = packetbuf_datalen();
		if ((sync_hdr_len = plb_create_header_data(temp_addr, type)) < 0) {
			PRINTF("ERROR: plb_create_header ");
			return -1;
		}

	}
	else if (type == SYNC_END)
	{
		temp_addr = &addr_next;

		if ((sync_hdr_len = plb_create_header(temp_addr, type)) < 0) {
			PRINTF("ERROR: plb_create_header ");
			return -1;
		}
	}
	else //error
	{
		PRINTF("(sync) plb_send_sync ERROR: wrong type");
		return -1;
	}


	PRINTF("(sync) plb_send_sync [dst:error: %d.%d] [type: %x]\n", temp_addr->u8[0], temp_addr->u8[1], type);
	sync_len = sync_hdr_len + sync_data_len;
	if (sync_len > (int) sizeof(sync)) {
		// Failed to send //
		PRINTF("plb: send failed, too large header\n");
		return -1;
	}

	memcpy(sync, packetbuf_hdrptr(), sync_len);
#if DEBUG_PACKET
	printf("[PLB] send_sync :");print_packet(sync, sync_len);
#endif

	// Send sync//
	radio_on();
	while(sync_count<3 && tx_result!=RADIO_TX_OK)
		{
			if ((tx_result=NETSTACK_RADIO.send(sync, sync_len)) != RADIO_TX_OK)
			{
				PRINTF("ERROR: plb sync type %x send TRIAL %d",type,++sync_count);
			}
			if(type==SYNC_END) // if SYNC_END Sending fails, retransmit until Sending successes JJH 0222
			{
				sync_count=0;
			}
		}
	if(sync_count==3)
	{
		send_fail_sync=2; //SYNC ACK fail case JJH 0222
	}
	return type;
}
/*---------------------------------------------------------------------------*/
static int plb_send_sync_end(uint8_t type) //kdw sync
{
	PRINTF("plb_send_sync_end\n ");

	sync_state = 1;

	// make packet //
	uint8_t sync[MAX_SYNC_SIZE];
	int sync_len = 0;
	int sync_data_len = 1; //1 byte for app check
	int sync_hdr_len = 0;
	int sync_count=0; //JJH 0221
	int tx_result=1; //JJH 0221
	rimeaddr_t * temp_addr;

	packetbuf_clear();

	if (type == SYNC_ACK)
	{
		temp_addr = &addr_prev;
		packetbuf_clear();
		packetbuf_copyfrom(global_dataptr_temp, temp_len);
		((uint8_t*)packetbuf_dataptr())[temp_len]=type;
		packetbuf_set_datalen(temp_len+1);
		sync_data_len = packetbuf_datalen();
		//		put data
		//		sync_data_len += len_clock * 3;
		if ((sync_hdr_len = plb_create_header_data(temp_addr, type)) < 0) {
			PRINTF("ERROR: plb_create_header ");
			return -1;
		}
		// put timestamp : KJY
		ntp_make_ack(ntp, sync+(sync_hdr_len+sync_data_len), global_dataptr_temp_recv);
		sync_data_len += NTP_ACK_LEN;
	}
	else if (type == SYNC_END)
	{
		temp_addr = &addr_next;

		if ((sync_hdr_len = plb_create_header(temp_addr, type)) < 0) {
			PRINTF("ERROR: plb_create_header ");
			return -1;
		}
	}
	else //error
	{
		PRINTF("(sync) plb_send_sync ERROR: wrong type");
		return -1;
	}


	PRINTF("(sync) plb_send_sync [dst: %d.%d] [type: %x]\n", temp_addr->u8[0], temp_addr->u8[1], type);
	/*
	if ((sync_hdr_len = plb_create_header(temp_addr, type)) < 0) {
		PRINTF("ERROR: plb_create_header ");
		return -1;
	}*/

	sync_len = sync_hdr_len + sync_data_len;
	if (sync_len > (int) sizeof(sync)) {
		// Failed to send //
		PRINTF("plb: send failed, too large header\n");
		return -1;
	}

	memcpy(sync, packetbuf_hdrptr(), sync_len);

	// Send sync//
	radio_on();
//	PRINTF("[sync] send sync \n");
#if 1//DEBUG_PACKET
	printf("[PLB] send_sync :");print_packet(sync, sync_len);
#endif
	while(sync_count<3 && tx_result!=RADIO_TX_OK)
		{
			if ((tx_result=NETSTACK_RADIO.send(sync, sync_len)) != RADIO_TX_OK)
			{
				PRINTF("ERROR: plb sync type %x send TRIAL %d",type,++sync_count);
				//		return -1; JJH 0221
			}
			if(type==SYNC_END) // if SYNC_END Sending fails, retransmit until Sending successes JJH 0222
			{
				sync_count=0;
			}
		}

	if(sync_count==3)
	{
		send_fail_sync=2; //SYNC ACK fail case JJH 0222
	}
	//	radio_off();

	return type;
}
/*---------------------------------------------------------------------------*/
static void print_packet(uint8_t *packet, int len) {
	int i, j;
	uint8_t num, num2;
	uint8_t jisu;

	for (i = 0; i < len; i++) {
		num = packet[i];
		for (j = 7; j > -1; j--) {
			jisu = 1 << j;
			num2 = num & jisu;
			num2 = num2 >> j;
			if (num & jisu) {
				printf("1");
			} else {
				printf("0");
			}
		}
		printf(" ");
	}
	printf("\n");
}
/*---------------------------------------------------------------------------*/
static void hold_time(rtimer_clock_t interval) {
	rtimer_clock_t rct;
	rct = RTIMER_NOW();
	while (RTIMER_CLOCK_LT(RTIMER_NOW(), rct+interval)) {
	}
}
/*---------------------------------------------------------------------------*/
static void radio_on() {
	PRINT_P("radio_on\n");
	if (is_radio_on == 0) {
		NETSTACK_RADIO.on();
		is_radio_on = 1;
	}
}
static void radio_off() {
	PRINT_P("radio_off\n");
	if (is_radio_on == 1) {

		NETSTACK_RADIO.off();
		is_radio_on = 0;
	}
}
/*---------------------------------------------------------------------------*/
static void plb_init(void) {
	PRINTF("plb_init\n");

	is_init = 1;
	PT_INIT(&pt);

	// init value
	is_plb_on = 0;
	is_radio_on = 0;
	has_data = 0;
	send_req = 0;
	wait_packet = 0;
	c_wait = 0;
	sync_state =0;
	send_fail=0;//JJH 0220
	send_fail_sync=0;//JJH 0221
	timeout_count=0;//JJH 0220
	timeout_nonfail_count=0;//JJH 0224
	data_ack_check=0;//JJH 0224

	temp_len = PACKETBUF_SIZE; //JJH4
	global_dataptr_temp = (uint8_t*) malloc(sizeof(uint8_t) * temp_len); // need to be free JJH3,JJH4
	global_dataptr_temp_recv = (uint8_t*) malloc(sizeof(uint8_t) * temp_len); // kdw

	//time sync init : KJY
	sclock_create(&sc, TYPE_CLOCK);
	sclock_init(sc, TYPE_CLOCK);
	ntp_create(&ntp);
	ntp_init(ntp, sc);

	// set address
	addr_next.u8[0] = rimeaddr_node_addr.u8[0] + 1;
	if (rimeaddr_node_addr.u8[0] > 0) {
		addr_prev.u8[0] = rimeaddr_node_addr.u8[0] - 1;
	} else {
		addr_prev.u8[0] = 0;
	}
	return;
}
/*---------------------------------------------------------------------------*/
static void plb_send(mac_callback_t sent, void *ptr) {
	if (packetbuf_attr(PACKETBUF_ATTR_PACKET_TYPE) == DATA) //data
	{
		PRINTF("plb_send : DATA\n");
		send_req = 1;
		sent_callback = sent;
		sent_ptr = ptr;
		temp_len=packetbuf_totlen();//JJH4 TEST
		packetbuf_copyto(global_dataptr_temp);
#if DEBUG_PACKET
		PRINTF("(DATA) plb_send packetbuf copyto test : ");//JJH4 TEST
		print_packet(global_dataptr_temp,packetbuf_totlen());//JJH3
#endif
	}
	//kdw sync
	else if (packetbuf_attr(PACKETBUF_ATTR_PACKET_TYPE) == SYNC_START) //sync
	{
		PRINTF("plb_send : SYNC_START\n");
		sent_callback = sent;
		sent_ptr = ptr;
		temp_len=packetbuf_totlen();//JJH4 TEST
		packetbuf_copyto(global_dataptr_temp);

#if DEBUG_PACKET
		PRINTF("(sync) plb_send packetbuf copyto test : ");//JJH4 TEST
		print_packet(global_dataptr_temp,packetbuf_totlen());//JJH3
#endif
		plb_send_sync_start();
	} else // error
	{
		PRINTF("plb_send : ERROR\n");
		mac_call_sent_callback(sent, ptr, MAC_TX_ERR, 1); //error   fill this
	}

	return;
}
/*---------------------------------------------------------------------------*/
/*
 * made by nullrdc
 */
static void plb_send_list(mac_callback_t sent, void *ptr,
		struct rdc_buf_list *buf_list) {
	PRINTF("plb_send_list\n");
	while (buf_list != NULL) {
		/* We backup the next pointer, as it may be nullified by
		 * mac_call_sent_callback() */
		struct rdc_buf_list *next = buf_list->next;
		int last_sent_ok;

		queuebuf_to_packetbuf(buf_list->buf);
		//    last_sent_ok = plb_send_data(sent, ptr); //removed by kdw
		last_sent_ok = 0; //kdw
		/* If packet transmission was not successful, we should back off and let
		 * upper layers retransmit, rather than potentially sending out-of-order
		 * packet fragments. */
		if (!last_sent_ok) {
			return;
		}
		buf_list = next;
	}
}
/*---------------------------------------------------------------------------*/
/*
 * BEACON_SD,		BEACON_SD_ACK 보내줌,
 * BEACON_SD_ACK, 	무시
 * BEACON_DS,		BEACON_DS_ACK 보내줌, c_wait set
 * BEACON_DS_ACK, 	무시
 * PREAMBLE,		power cycle data wait 모드
 * PREAMBLE_ACK, 	무시
 * PREAMBLE_ACK_DATA 	무시
 * DATA,			app 으로 올림,
 * DATA_ACK,		끝 아무것도 딱히 안해도됨
 * SYNC_START,		SYNC_REQ 전송, time stamp 찍어서
 * SYNC_REQ,		SYNC_ACK 전송, time stamp 찍어서
 * SYNC_ACK,		SYNC_END 전송, time stamp,  app 으로 올려서 계산
 * SYNC_END			app 으로 올림
 */
static void plb_input(void) {

	int length; //for copy packetbuf to temp pointer JJH5
//	length = packetbuf_copyto(global_dataptr_temp);
	if (NETSTACK_FRAMER.parse() >= 0) {
		if (rimeaddr_cmp(packetbuf_addr(PACKETBUF_ADDR_RECEIVER),
				&rimeaddr_node_addr)) {

#if DEBUG_PACKET
			print_packet(packetbuf_dataptr(),packetbuf_datalen());
#endif
			uint8_t type;
			if(packetbuf_datalen()==1)
			{
				type=((uint8_t*) packetbuf_dataptr())[0];
			}
			else
			{
				type= ((uint8_t*) packetbuf_dataptr())[NETWORK_HDR_SIZE];
			}
			PRINTF(
					"plb_input [src: %d.%d] [type: %x]\n", packetbuf_addr(PACKETBUF_ADDR_SENDER)->u8[0], packetbuf_addr(PACKETBUF_ADDR_SENDER)->u8[1], type);

			switch (type) {
			case BEACON_SD:
				//		if( a_wait == 0 ){
				plb_send_ack(BEACON_SD_ACK);
				//			a_wait = 1;
				//		}
				break;
			case BEACON_DS:
				//		if( c_wait == 0 ){
				plb_send_ack(BEACON_DS_ACK);
				c_wait = 1;
				//		}0x80
				break;
			case PREAMBLE:
				//		if (preamble_got == 0)	{
				if (has_data == 0) {
					plb_send_ack(PREAMBLE_ACK);
					radio_on();
					wait_packet = 1;
				} else if (has_data == 1) {
					plb_send_ack(PREAMBLE_ACK_DATA);
				}
				//			preamble_got = 1;
				//		}
				break;
			case DATA:
				NETSTACK_MAC.input();
				plb_send_ack(DATA_ACK); //Sending DATA_ACK before MAC.input
				break;
			case SYNC_START:
				plb_send_sync_req();
				break;
			case SYNC_ACK:
				NETSTACK_MAC.input();
				plb_send_sync_ack(SYNC_END);
				break;
			case SYNC_END:
				mac_call_sent_callback(sent_callback, sent_ptr, 0, 1);

				break;
			}
		} else {
			//		PRINTF("THIS INPUT IS NOT FOR US\n");
		}
	}
}
/*---------------------------------------------------------------------------*/
static int plb_on(void) {
	PRINTF("plb_on\n");
	if (!is_plb_on) {
		is_plb_on = 1;

		// init value
		is_radio_on = 0;
		has_data = 0;
		send_req = 0;
		wait_packet = 0;

		c_wait = 0;
		sync_state = 0;
		send_fail=0;//JJJH 0220
		send_fail_sync=0;//JJH 0221
		timeout_count=0;//JJH 0220
		timeout_nonfail_count=0;
		data_ack_check=0;

		// run procedure
		plb_beacon_sd();
		plb_beacon_ds();
		plb_powercycle();

	} else {
		PRINTF("already on\n");
		return -1;
	}
	return 0;
}
/*-----------------------------------plb_input----------------------------------------*/
static int plb_off(int keep_radio_on) {
	PRINTF("plb_off\n");
	if (wait_packet) {
		return NETSTACK_RADIO.on();
	} else {
		is_plb_on = 0;
		return NETSTACK_RADIO.off();
	}
}
/*---------------------------------------------------------------------------*/
static void plb_timeout(void)
{
	if(send_fail==1)
	{
	send_fail=0; //JJH6
	send_req=1; //to restart preamble and data sending
	c_wait=0; // to retransmit beacon_sd;
	timeout_count=0;
	plb_beacon_sd();
	}
	else if(send_fail_sync==1)
	{
		send_fail_sync=0;
		timeout_count=0;
		plb_send_sync_start();
	}
	else if(send_fail_sync==2)
	{
		send_fail_sync=0;
		timeout_count=0;
		plb_send_sync_ack(SYNC_ACK);
	}
	/*else if(send_fail_sync==3)
	{
		send_fail_sync=0;
		timeout_count=0;
		plb_send_sync(SYNC_END);
	}JJH 0222*/

	return;
}
/*---------------------------------------------------------------------------*/
static void plb_nonfail_timeout(void)
{
	if(!sync_state && !data_ack_check) //it's not a sync state and not receive data ack JJH 0224
	{
		send_req=1;
		c_wait=0;
		timeout_nonfail_count=0;
		plb_beacon_sd();
	}
	else if(sync_state)//for sync state
	{
		timeout_nonfail_count=0;
		plb_send_sync_start();
	}
	return;

}
/*---------------------------------------------------------------------------*/
static unsigned short plb_channel_check_interval(void) {
	return 0;
}
/*---------------------------------------------------------------------------*/
const struct rdc_driver plb_driver = { "PLB", plb_init, plb_send, plb_send_list,
		plb_input, plb_on, plb_off, plb_channel_check_interval, };
/*---------------------------------------------------------------------------*/

