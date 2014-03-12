#ifndef SCLOCK_H
#define SCLOCK_H

#include "timestamp.h"
#include "contiki.h"
#include "sys/rtimer.h"
#include "sys/etimer.h"
#include "sys/pt.h"

/* #define SLOPE_INIT CLOCK_SECOND */
#define SCLOCK_UNIT_MSEC_RTIMER 10
#define SCLOCK_UNIT_MSEC_CLOCK 100
#define SCLOCK_SLOPE_INIT_RTIMER (RTIMER_ARCH_SECOND*SCLOCK_UNIT_MSEC_RTIMER/(MSEC_MAX+1))
#define SCLOCK_SLOPE_INIT_CLOCK (CLOCK_SECOND*SCLOCK_UNIT_MSEC_CLOCK/(MSEC_MAX+1))
#define UINT16_T_MAX 65535
#define RTIMER_CLOCK_T_MAX UINT16_MAX
#define CLOCK_TIME_T_MAX UINT16_MAX

/* sclock */
struct sclock{
#define TYPE_RTIMER 0
#define TYPE_CLOCK 1
  struct timestamp now; /* Time of now */
  int type; /* timer type : RTIMER or CLOCK */
  uint16_t slope; /* ticks per unit time. */
  uint16_t last_tick;
};

static struct sclock sclock_rtimer;
static struct sclock sclock_clock;
void sclock_create(struct sclock **sc, int type);

void sclock_init(struct sclock *sc, int type); /* type : TYPE_RITMER or TYPE_CLOCK */
void sclock_update(struct sclock *sc); /***** Calculate timestamp now. Please use this before use other functions *****/
void sclock_now(struct sclock *sc, struct timestamp *ts); /* ts = updated now */

/* TYPE_RTIMER functions */
void sclock_rtimer_set(struct sclock *sc, struct rtimer* rt, struct timestamp *interval, rtimer_callback_t func, void *ptr);
int sclock_rtimer_set_long(struct sclock *sc, struct rtimer* rt, struct timestamp *interval, rtimer_callback_t func, void *ptr);// not implemented
void sclock_hold_time(struct sclock *sc, struct timestamp *interval);

/* TYPE_CLOCK functions */
void sclock_etimer_set(struct sclock *sc, struct etimer *et, struct timestamp *interval);
int sclock_etimer_set_long(struct sclock *sc, struct etimer* et, struct timestamp *interval);
uint16_t sclock_tick_to_timestamp(struct sclock *sc, struct timestamp *ts, uint16_t tick);

static struct sclock sclock_rtimer;
static struct sclock sclock_clock;




/* sclock README
 * 
 * sclock_rtimer_set, sclock_hold_time : sec, msec ( <2sec )
 * sclock_etimer_set : min, sec, msec ( <1min ) 
 * sclock_etimer_set_long : all time ( <10year ), interval changed.
 * 
 * < sclock_rtimer_set example >
 * PT_BEGIN(&pt);
 * while(1){
 *  sclock_etimer_set_long(sc, rt, interval,  (void (*)(struct rtimer *, void *))func, ptr);
 *  PT_YIELD(&pt);
 * }
 * PT_END(&pt);
 * 
 * < sclock_etimer_set_long example >
 * PT_BEGIN(&pt);
 * while(1){
 *  sclock_etimer_set_long(sc, et, interval);
 *  PROCESS_WAIT_EVENT_UNITL(etimer_expired(&et);
 * }
 * PT_END(&pt);
 *
 * < sclock_etimer_set_long example >
 * while(1){
 *  //set interval in here
 *  while(!timestamp_is_empty(&interval)){
 *   sclock_etimer_set_long(sc, et, interval);
 *   PROCESS_WAIT_EVENT_UNITL(etimer_expired(&et));
 *  }
 * }
 *                            OR
 * while(1){
 *  //set interval in here
 *  while(sclock_etimer_set_long(sc, et, interval) != 0){
 *   PROCESS_WAIT_EVENT_UNITL(etimer_expired(&et));
 *  }
 * }


 */
#endif /* SCLOCK_H */
