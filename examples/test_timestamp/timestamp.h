#ifndef TIMESTAMP_H
#define TIMESTAMP_H
#include <stdio.h>
#include <stdint.h>

struct timestamp
{
#define YEAR_MAX 15
#define MONTH_MAX 12
#define DAY_MAX 31
#define HOUR_MAX 23
#define MIN_MAX 59
#define SEC_MAX 59
#define MSEC_MAX 999

#define YEAR_BIT 4
#define MONTH_BIT 4
#define DAY_BIT 5
#define HOUR_BIT 5
#define MIN_BIT 6
#define SEC_BIT 6
#define MSEC_BIT 10
#define TIMESTAMP_BIT (YEAR_BIT+MONTH_BIT+DAY_BIT+HOUR_BIT+MIN_BIT+SEC_BIT+MSEC_BIT)

  uint8_t year;
  uint8_t month;
  uint8_t day;
  uint8_t hour;
  uint8_t min;
  uint8_t sec;
  uint16_t msec;
};

enum timestamp_type{
  YEAR, MONTH, DAY, HOUR, MIN, SEC, MSEC, TOTAL  
};

void timestamp_init(struct timestamp *ts);
int timestamp_to_packet(struct timestamp *ts, uint8_t *packet, int start_index);
void timestamp_from_packet(struct timestamp *ts, uint8_t *packet, int start_index);

void print_timestamp(struct timestamp *ts);

#endif /* TIMESTAMP_H */
