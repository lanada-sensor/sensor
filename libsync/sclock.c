#include "sclock.h"
#include "contiki.h"

void sclock_create(struct sclock **sc, int type)
{
  if(type == TYPE_RTIMER){
    *sc = &sclock_rtimer;
    }
  if(type == TYPE_CLOCK){
    *sc = &sclock_clock;
  }
}

void
sclock_init(struct sclock *sc, int type)
{
  timestamp_init(&sc->now);
  if(type == TYPE_RTIMER){
    sc->slope = SCLOCK_SLOPE_INIT_RTIMER;
    sc->last_tick = RTIMER_NOW();
    sc->type = TYPE_RTIMER;
  }
 
 else{
    sc->slope = SCLOCK_SLOPE_INIT_CLOCK;
    sc->last_tick = clock_time();
    sc->type = TYPE_CLOCK;
  }
}

void
sclock_update(struct sclock *sc)
{
  if( sc->type == TYPE_RTIMER ){
    rtimer_clock_t now_tick = RTIMER_NOW();
    rtimer_clock_t interval;
    if(now_tick >= (sc->last_tick)){
      interval = now_tick-(sc->last_tick);
    }
    else{
      interval = (RTIMER_CLOCK_T_MAX - sc->last_tick) + now_tick;
    }

    rtimer_clock_t remain = interval%(sc->slope);
    sc->last_tick = now_tick-remain;

    struct timestamp ts_calc;
    timestamp_init(&ts_calc);
    ts_calc.msec = (interval/(sc->slope))*SCLOCK_UNIT_MSEC_RTIMER;
    timestamp_add(&(sc->now),&ts_calc);
  }
  else{
    clock_time_t now_tick = clock_time();
    clock_time_t interval;
    if(now_tick >= (sc->last_tick)){
      interval = now_tick-(sc->last_tick);
    }
    else{
      interval = (CLOCK_TIME_T_MAX - sc->last_tick) + now_tick;
    }

    clock_time_t remain = interval%(sc->slope);
    sc->last_tick = now_tick-remain;

    struct timestamp ts_calc;
    timestamp_init(&ts_calc);
    ts_calc.msec = (interval/(sc->slope))*SCLOCK_UNIT_MSEC_CLOCK;
    timestamp_add(&(sc->now),&ts_calc);
  }
}

void
sclock_now(struct sclock *sc, struct timestamp *ts)
{
  sclock_update(sc);
  timestamp_cpy(ts, &(sc->now));
}

uint16_t sclock_tick_to_timestamp(struct sclock *sc, struct timestamp *ts, uint16_t tick)
{
  uint16_t unit_msec=0;
  uint16_t msec=0;
  uint16_t sec=0;
  uint16_t min=0;
  uint16_t hour=0;
  uint16_t rest_tick=0;
  timestamp_init(ts);

  if( sc->type == TYPE_CLOCK ){
    unit_msec = tick / (sc->slope);
    rest_tick = tick % (sc->slope);
    if(unit_msec > (UINT16_T_MAX/SCLOCK_UNIT_MSEC_CLOCK)){
      sec = unit_msec / ((MSEC_MAX+1)/SCLOCK_UNIT_MSEC_CLOCK);
      unit_msec = unit_msec % ((MSEC_MAX+1)/SCLOCK_UNIT_MSEC_CLOCK);
      msec = unit_msec*SCLOCK_UNIT_MSEC_CLOCK;
      msec += (unit_msec%((MSEC_MAX+1)/SCLOCK_UNIT_MSEC_CLOCK))*SCLOCK_UNIT_MSEC_CLOCK;
      if(sec > SEC_MAX){
	min = sec / (SEC_MAX+1);
	sec = sec % (SEC_MAX+1);
	if(min > MIN_MAX){
	  hour = min / (MIN_MAX+1);
	  min = min % (MIN_MAX+1);
	}      
      }      
    }
    else{ /* unit_msec <= (UINT16_T_MAX/SCLOCK_UNIT_MSEC_CLOCK) */
      msec = unit_msec*SCLOCK_UNIT_MSEC_CLOCK;
    }
  }

  if( sc->type == TYPE_RTIMER ){
    unit_msec = tick / (sc->slope);
    rest_tick = tick % (sc->slope);
    if(unit_msec > (UINT16_T_MAX/SCLOCK_UNIT_MSEC_RTIMER)){
      sec = unit_msec / ((MSEC_MAX+1)/SCLOCK_UNIT_MSEC_RTIMER);
      unit_msec = unit_msec % ((MSEC_MAX+1)/SCLOCK_UNIT_MSEC_RTIMER);
      msec = unit_msec*SCLOCK_UNIT_MSEC_RTIMER;
      msec += (unit_msec%((MSEC_MAX+1)/SCLOCK_UNIT_MSEC_RTIMER))*SCLOCK_UNIT_MSEC_RTIMER;
      if(sec > SEC_MAX){
	min = sec / (SEC_MAX+1);
	sec = sec % (SEC_MAX+1);
	if(min > MIN_MAX){
	  hour = min / (MIN_MAX+1);
	  min = min % (MIN_MAX+1);
	}      
      }      
    }
    else{ /* unit_msec <= (UINT16_T_MAX/SCLOCK_UNIT_MSEC_RTIMER) */
      msec = unit_msec*SCLOCK_UNIT_MSEC_RTIMER;
    }
  }
  ts->msec = msec;
  ts->sec = sec;
  ts->min = min;
  ts->hour = hour;
  timestamp_arrange(ts);

  return rest_tick;
}

void sclock_rtimer_set(struct sclock *sc, struct rtimer* rt, struct timestamp *interval, rtimer_callback_t func, void *ptr)
{
  if(sc->type == TYPE_CLOCK){
    return;
  }

  clock_time_t tick=0;
  tick += ((MSEC_MAX+1)/SCLOCK_UNIT_MSEC_RTIMER*(sc->slope))*interval->sec;
  tick += (SCLOCK_UNIT_MSEC_RTIMER*(sc->slope))*interval->msec;
  
  /* set rtimer */
  rtimer_set(rt, RTIMER_NOW()+tick, 1,
	     (void (*)(struct rtimer *, void *))func, ptr);
}

void sclock_hold_time(struct sclock *sc, struct timestamp *interval)
{
  if(sc->type == TYPE_CLOCK){
    return;
  }

  clock_time_t tick=0;
  tick += (((MSEC_MAX+1)/SCLOCK_UNIT_MSEC_RTIMER)*(sc->slope)*(SEC_MAX+1))*interval->min;
  tick += ((MSEC_MAX+1)/SCLOCK_UNIT_MSEC_RTIMER*(sc->slope))*interval->sec;
  tick += (SCLOCK_UNIT_MSEC_RTIMER*(sc->slope))*interval->msec;

  /* while loop */
  rtimer_clock_t now = RTIMER_NOW();
  while(RTIMER_CLOCK_LT(RTIMER_NOW(), now+tick)){}
}

void sclock_etimer_set(struct sclock *sc, struct etimer *et, struct timestamp *interval)
{
  if(sc->type == TYPE_RTIMER){
    return;
  }

  clock_time_t tick=0;
  tick += (((MSEC_MAX+1)/SCLOCK_UNIT_MSEC_CLOCK)*(sc->slope)*(SEC_MAX+1))*interval->min;
  tick += ((MSEC_MAX+1)/SCLOCK_UNIT_MSEC_CLOCK*(sc->slope))*interval->sec;
  tick += (SCLOCK_UNIT_MSEC_CLOCK*(sc->slope))*interval->msec;
  
  /* etimer_set */
  etimer_set(et, tick);
}

int sclock_etimer_set_long(struct sclock *sc, struct etimer* et, struct timestamp *interval)
{
  if(sc->type == TYPE_RTIMER){
    return -1;
  }

  struct timestamp limit;
  sclock_tick_to_timestamp(sc, &limit, CLOCK_TIME_T_MAX-10000);
  if( timestamp_cmp(&limit, interval) <= 0 ){  /* interval >= limit */
    timestamp_subtract(interval,&limit);
    etimer_set(et, CLOCK_TIME_T_MAX-10000);
    return 1;
  }
  else{
    sclock_etimer_set(sc, et, interval);
    timestamp_init(interval);
    return 0;
  }

}

