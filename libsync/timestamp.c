#include "timestamp.h"

int timestamp_arrange(struct timestamp *ts)
{
  if(ts->msec > MSEC_MAX){
    ts->sec += ts->msec / (MSEC_MAX+1);
    ts->msec = ts->msec % (MSEC_MAX+1);
  }
  if(ts->sec > SEC_MAX){
    ts->min += ts->sec / (SEC_MAX+1);
    ts->sec = ts->sec % (SEC_MAX+1);
  }
  if(ts->min > MIN_MAX){
    ts->hour += ts->min / (MIN_MAX+1);
    ts->min = ts->min % (MIN_MAX+1);
  }
  if(ts->hour > HOUR_MAX){
    ts->day += ts->hour / (HOUR_MAX+1);
    ts->hour = ts->hour % (HOUR_MAX+1);
  }
  if(ts->day > DAY_MAX){
    ts->month += ts->day / (DAY_MAX+1);
    ts->day = ts->day % (DAY_MAX+1);
  }
  if(ts->month > MONTH_MAX){
    ts->year += ts->month / (MONTH_MAX+1);
    ts->month = ts->month % (MONTH_MAX+1);
  }
  if(ts->year > YEAR_MAX){
    return -1;
  }
  return 0;
}

int timestamp_is_overflow(struct timestamp *ts)
{
  if(ts->msec > MSEC_MAX){
    return -1;
  }
  if(ts->sec > SEC_MAX){
    return -1;
  }
  if(ts->min > MIN_MAX){
    return -1;
  }
  if(ts->hour > HOUR_MAX){
    return -1;
  }
  if(ts->day > DAY_MAX){
    return -1;
  }
  if(ts->month > MONTH_MAX){
    return -1;
  }
  if(ts->year > YEAR_MAX){
    return -1;
  }
  return 0;

}

int timestamp_is_empty(struct timestamp *ts)
{
  if(ts->msec == 0 && ts->sec == 0 && ts->min == 0 && ts->hour == 0 
     && ts->day == 0 && ts->month == 0 && ts->year == 0){
    return 1;
  }
  return 0;
}

void timestamp_init(struct timestamp *ts)
{
  ts->year = 0;
  ts->month = 0;
  ts->day = 0;
  ts->hour = 0;
  ts->min = 0;
  ts->sec = 0;
  ts->msec = 0;
}

int timestamp_add(struct timestamp *dst, struct timestamp *src)
{
  dst->year += src->year;
  dst->month += src->month;
  dst->hour += src->hour;
  dst->day += src->day;
  dst->min += src->min;
  dst->sec += src->sec;
  dst->msec += src->msec;
  
  return timestamp_arrange(dst);
}

int timestamp_plus(struct timestamp *dst, struct timestamp *right, struct timestamp *left)
{
  dst->year = right->year + left->year;
  dst->month = right->month + left->month;
  dst->day = right->day + left->day;
  dst->hour = right->hour + left->hour;
  dst->min = right->min + left->min;
  dst->sec = right->sec + left->sec;
  dst->msec = right->msec + left->msec;
  
  return timestamp_arrange(dst);
}
int timestamp_subtract(struct timestamp *dst, struct timestamp *src)
{
  if(dst->msec < src->msec){
    src->sec++;
    dst->msec = (dst->msec + (MSEC_MAX+1)) - src->msec;
  }
  else{
    dst->msec = dst->msec - src->msec;
  }

  if(dst->sec < src->sec){
    src->min++;
    dst->sec = (dst->sec + (SEC_MAX+1)) - src->sec;
  }
  else{
    dst->sec = dst->sec - src->sec;
  }

  if(dst->min < src->min){
    src->hour++;
    dst->min = (dst->min + (MIN_MAX+1)) - src->min;
  }
  else{
    dst->min = dst->min - src->min;
  }

  if(dst->hour < src->hour){
    src->day++;
    dst->hour = (dst->hour + (HOUR_MAX+1)) - src->hour;
  }
  else{
    dst->hour = dst->hour - src->hour;
  }

  if(dst->day < src->day){
    src->month++;
    dst->day = (dst->day + (DAY_MAX+1)) - src->day;
  }
  else{
    dst->day = dst->day - src->day;
  }

  if(dst->month < src->month){
    src->year++;
    dst->month = (dst->month + (MONTH_MAX+1)) - src->month;
  }
  else{
    dst->month = dst->month - src->month;
  }

  if(dst->year < src->year){
    return -1;
  }
  else{
    dst->year = dst->year - src->year;
  }

  return 0;
}
int timestamp_minus(struct timestamp *dst, struct timestamp *right, struct timestamp *left)
{
  if(right->msec < left->msec){
    left->sec++;
    dst->msec = (right->msec + (MSEC_MAX+1)) - left->msec;
  }
  else{
    dst->msec = right->msec - left->msec;
  }

  if(right->sec < left->sec){
    left->min++;
    dst->sec = (right->sec + (SEC_MAX+1)) - left->sec;
  }
  else{
    dst->sec = right->sec - left->sec;
  }

  if(right->min < left->min){
    left->hour++;
    dst->min = (right->min + (MIN_MAX+1)) - left->min;
  }
  else{
    dst->min = right->min - left->min;
  }

  if(right->hour < left->hour){
    left->day++;
    dst->hour = (right->hour + (HOUR_MAX+1)) - left->hour;
  }
  else{
    dst->hour = right->hour - left->hour;
  }

  if(right->day < left->day){
    left->month++;
    dst->day = (right->day + (DAY_MAX+1)) - left->day;
  }
  else{
    dst->day = right->day - left->day;
  }

  if(right->month < left->month){
    left->year++;
    dst->month = (right->month + (MONTH_MAX+1)) - left->month;
  }
  else{
    dst->month = right->month - left->month;
  }

  if(right->year < left->year){
    return -1;
  }
  else{
    dst->year = right->year - left->year;
  }

  return 0;
}

void timestamp_divide(struct timestamp *ts, unsigned int div)
{
  uint8_t rest=0;

  rest = (ts->year)%div;
  ts->year = (ts->year)/div;

  ts->month += rest*MONTH_MAX;
  rest = (ts->month)%div;
  ts->month = (ts->month)/div;

  ts->day += rest*DAY_MAX;
  rest = (ts->day)%div;
  ts->day = (ts->day)/div;

  ts->hour += rest*HOUR_MAX;
  rest = (ts->hour)%div;
  ts->hour = (ts->hour)/div;

  ts->min += rest*MIN_MAX;
  rest = (ts->min)%div;
  ts->min = (ts->min)/div;

  ts->sec += rest*SEC_MAX;
  rest = (ts->sec)%div;
  ts->sec = (ts->sec)/div;

  ts->msec += rest*MSEC_MAX;
  rest = (ts->msec)%div;
  ts->msec = (ts->msec)/div;
}

int packet_get_index(int bit, int* inner_i)
{
  int result = bit/8;
  *inner_i = ((result+1)*8) - bit -1;
  return bit/8;
}

int timestamp_to_packet(struct timestamp *ts, uint8_t *packet)
{
  memset(packet, 0, TIMESTAMP_BYTE);
  int start_bit;
  int bit=0;
  uint16_t tmp=0;
  int packet_i;
  int inner_i;
  
  tmp = ts->year;
  start_bit = 0;
  for(bit = (start_bit+YEAR_BIT-1); bit >= start_bit; bit--){
    packet_i  = packet_get_index(bit, &inner_i);
    packet[packet_i] = packet[packet_i] | ((tmp & 0x1) << (inner_i));
    tmp = tmp >> 1;
  }

  tmp = ts->month;
  start_bit += YEAR_BIT;
  for(bit = (start_bit+MONTH_BIT-1); bit >= start_bit; bit--){
    packet_i  = packet_get_index(bit, &inner_i);
    packet[packet_i] = packet[packet_i] | ((tmp & 0x1) << (inner_i));
    tmp = tmp >> 1;
  }

  tmp = ts->day;
  start_bit += MONTH_BIT;
  for(bit = (start_bit+DAY_BIT-1); bit >= start_bit; bit--){
    packet_i  = packet_get_index(bit, &inner_i);
    packet[packet_i] = packet[packet_i] | ((tmp & 0x1) << (inner_i));
    tmp = tmp >> 1;
  }

  tmp = ts->hour;
  start_bit += DAY_BIT;
  for(bit = (start_bit+HOUR_BIT-1); bit >= start_bit; bit--){
    packet_i  = packet_get_index(bit, &inner_i);
    packet[packet_i] = packet[packet_i] | ((tmp & 0x1) << (inner_i));
    tmp = tmp >> 1;
  }

  tmp = ts->min;
  start_bit += HOUR_BIT;
  for(bit = (start_bit+MIN_BIT-1); bit >= start_bit; bit--){
    packet_i  = packet_get_index(bit, &inner_i);
    packet[packet_i] = packet[packet_i] | ((tmp & 0x1) << (inner_i));
    tmp = tmp >> 1;
  }

  tmp = ts->sec;
  start_bit += MIN_BIT;
  for(bit = (start_bit+SEC_BIT-1); bit >= start_bit; bit--){
    packet_i  = packet_get_index(bit, &inner_i);
    packet[packet_i] = packet[packet_i] | ((tmp & 0x1) << (inner_i));
    tmp = tmp >> 1;
  }

  tmp = ts->msec;
  start_bit += SEC_BIT;
  for(bit = (start_bit+MSEC_BIT-1); bit >= start_bit; bit--){
    packet_i  = packet_get_index(bit, &inner_i);
    packet[packet_i] = packet[packet_i] | ((tmp & 0x1) << (inner_i));
    tmp = tmp >> 1;
  }

  return 0;
}

void timestamp_from_packet(struct timestamp *ts, uint8_t *packet_in)
{
  uint8_t packet[TIMESTAMP_BYTE];
  memcpy(packet, packet_in, TIMESTAMP_BYTE);

  int start_bit;
  int bit=0;
  int packet_i, inner_i;
  uint16_t tmp;
  
  tmp = 0;
  start_bit = 0;
  for(bit = start_bit; bit < (start_bit+YEAR_BIT); bit++){
    packet_i  = packet_get_index(bit, &inner_i);
    tmp = tmp | ((packet[packet_i] & (0x1 << inner_i)) >> inner_i);
    
    if( bit != start_bit+YEAR_BIT-1 )
      tmp = tmp << 1;
  }
  ts->year = tmp;

  tmp = 0;
  start_bit += YEAR_BIT;
  for(bit = start_bit; bit < (start_bit+MONTH_BIT); bit++){
    packet_i  = packet_get_index(bit, &inner_i);
    tmp = tmp | ((packet[packet_i] & (0x1 << inner_i)) >> inner_i);
    
    if( bit != start_bit+MONTH_BIT-1 )
      tmp = tmp << 1;
  }
  ts->month = tmp;

  tmp = 0;
  start_bit += MONTH_BIT;
  for(bit = start_bit; bit < (start_bit+DAY_BIT); bit++){
    packet_i  = packet_get_index(bit, &inner_i);
    tmp = tmp | ((packet[packet_i] & (0x1 << inner_i)) >> inner_i);
    
    if( bit != start_bit+DAY_BIT-1 )
      tmp = tmp << 1;
  }
  ts->day = tmp;

  tmp = 0;
  start_bit += DAY_BIT;
  for(bit = start_bit; bit < (start_bit+HOUR_BIT); bit++){
    packet_i  = packet_get_index(bit, &inner_i);
    tmp = tmp | ((packet[packet_i] & (0x1 << inner_i)) >> inner_i);
    
    if( bit != start_bit+HOUR_BIT-1 )
      tmp = tmp << 1;
  }
  ts->hour = tmp;

  tmp = 0;
  start_bit += HOUR_BIT;
  for(bit = start_bit; bit < (start_bit+MIN_BIT); bit++){
    packet_i  = packet_get_index(bit, &inner_i);
    tmp = tmp | ((packet[packet_i] & (0x1 << inner_i)) >> inner_i);
    
    if( bit != start_bit+MIN_BIT-1 )
      tmp = tmp << 1;
  }
  ts->min = tmp;

  tmp = 0;
  start_bit += MIN_BIT;
  for(bit = start_bit; bit < (start_bit+SEC_BIT); bit++){
    packet_i  = packet_get_index(bit, &inner_i);
    tmp = tmp | ((packet[packet_i] & (0x1 << inner_i)) >> inner_i);
    
    if( bit != start_bit+SEC_BIT-1 )
      tmp = tmp << 1;
  }
  ts->sec = tmp;

  tmp = 0;
  start_bit += SEC_BIT;
  for(bit = start_bit; bit < (start_bit+MSEC_BIT); bit++){
    packet_i  = packet_get_index(bit, &inner_i);
    tmp = tmp | ((packet[packet_i] & (0x1 << inner_i)) >> inner_i);
    
    if( bit != start_bit+MSEC_BIT-1 )
      tmp = tmp << 1;
  }
  ts->msec = tmp;


}

int timestamp_cmp(struct timestamp *rt, struct timestamp *lt)
{
  if(rt->year < lt->year){
    return -1;
  }
  else if(rt->year > lt->year){
    return 1;
  }

  if(rt->month < lt->month){
    return -1;
  }
  else if(rt->month > lt->month){
    return 1;
  }

  if(rt->day < lt->day){
    return -1;
  }
  else if(rt->day > lt->day){
    return 1;
  }

  if(rt->hour < lt->hour){
    return -1;
  }
  else if(rt->hour > lt->hour){
    return 1;
  }

  if(rt->min < lt->min){
    return -1;
  }
  else if(rt->min > lt->min){
    return 1;
  }

  if(rt->sec < lt->sec){
    return -1;
  }
  else if(rt->sec > lt->sec){
    return 1;
  }

  if(rt->msec < lt->msec){
    return -1;
  }
  else if(rt->msec > lt->msec){
    return 1;
  }

  return 0;
}

void timestamp_cpy(struct timestamp *dst, struct timestamp *src)
{
  dst->year = src->year;
  dst->month = src->month;
  dst->day = src->day;
  dst->hour = src->hour;
  dst->min = src->min;
  dst->sec = src->sec;
  dst->msec = src->msec;
}


void print_timestamp(struct timestamp *ts)
{
  printf("Year|Month|Day|Hour|Min|Sec|Msec\n");
  printf("%4u|%5u|%3u|%4u|%3u|%3u|%4u\n",ts->year,ts->month,ts->day,
	 ts->hour,ts->min,ts->sec,ts->msec);
}

