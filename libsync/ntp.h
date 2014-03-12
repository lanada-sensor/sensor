#ifndef NTP_H
#define NTP_H
#include "sclock.h"

#define DEBUG_NTP 0;

/* Protocol */
/* <Request>
 * |t1(TIMESTAMP_BYTE)|
 *
 * <ACK>
 * |preamble:2(1bytes)|t1(TIMESTAMP_BYTE)|t2(TIMESTAMP_BYTE)|t3(TIMESTAMP_BYTE)|
 */
#define NTP_REQUEST_LEN TIMESTAMP_BYTE
#define NTP_ACK_LEN (TIMESTAMP_BYTE*3)

struct ntp{
  struct sclock *sc;
  struct timestamp t1, t2, t3, t4;
};

//#define DEBUG_DRIFT
#ifdef DEBUG_DRIFT
static int total_sec;
static int total_drift;
#endif

/* ntp functions */
static struct ntp ntp_static;
static struct ntp* ntp_ptr;
void ntp_create(struct ntp **ntp);
void ntp_init(struct ntp *ntp, struct sclock *clock); /* read README */
int ntp_put_t1(struct ntp *ntp, uint8_t *send); /* update sclock, t1 = now, put t1 into packet */
int ntp_put_t2(struct ntp *ntp, uint8_t *send); /* update sclock, t2 = now, put t1 into packet */
int ntp_put_t3(struct ntp *ntp, uint8_t *send); /* update sclock, t3 = now, put t1 into packet */
int ntp_make_request(struct ntp *ntp, uint8_t *send); /* update sclock, put t1 */
int ntp_make_ack(struct ntp *ntp, uint8_t *send, uint8_t *recv); /* update sclock, put t1, t2, t3 */
void ntp_handle_ack(struct ntp *ntp, uint8_t *recv); /* check t1, t2, t3. update clock, t4 = now. calc time sync */
void ntp_calc(struct ntp *ntp); /* calc time sync. only offset used */
void print_ntp(struct ntp* ntp);

/* NTP README
 * <struct ntp initialization>
 * struct sclock sc;
 * sclock_init(&sc);
 * struct ntp ntp;
 * ntp_init(&ntp,&sc);
 * 
 * <ntp packet length>
 * each timestamp packet : TIMESTAMP_BYTE
 * sync request : TIMESTAMP_BYTE
 * sync ack : 1 + TIMESTAMP_BYTE*3
 * sync end : 1
 */ 


#endif /* NTP_H */
