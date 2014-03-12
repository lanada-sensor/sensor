#ifndef TIMESTAMP_H
#define TIMESTAMP_H
#include <stdio.h>
#include <stdint.h>
#include <string.h>

struct timestamp
{
  /* Maximum of member variables */
#define YEAR_MAX 15
#define MONTH_MAX 12
#define DAY_MAX 31
#define HOUR_MAX 23
#define MIN_MAX 59
#define SEC_MAX 59
#define MSEC_MAX 999

  /* Bits for packet */
#define YEAR_BIT 4
#define MONTH_BIT 4
#define DAY_BIT 5
#define HOUR_BIT 5
#define MIN_BIT 6
#define SEC_BIT 6
#define MSEC_BIT 10
#define TIMESTAMP_BIT (YEAR_BIT+MONTH_BIT+DAY_BIT+HOUR_BIT+MIN_BIT+SEC_BIT+MSEC_BIT)
  /* Total bytes of packet */
#define TIMESTAMP_BYTE (TIMESTAMP_BIT/8)

  /* Member variables */
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

void timestamp_init(struct timestamp *ts); /* Make time = 0 */
int timestamp_to_packet(struct timestamp *ts, uint8_t *packet); /* timestamp => packet */
void timestamp_from_packet(struct timestamp *ts, uint8_t *packet); /* packet => timestamp */
int timestamp_add(struct timestamp *dst, struct timestamp *src); /* dst += src . If overflow, return -1 */
int timestamp_plus(struct timestamp *dst, struct timestamp *right, struct timestamp *left); /* dst = right+left. If overflow, return -1 */
int timestamp_subtract(struct timestamp *dst, struct timestamp *src); /* dst -= src. If dst < src, return -1 */
int timestamp_minus(struct timestamp *dst, struct timestamp *right, struct timestamp *left); /* dst = right - left. If right < left, return -1 */
void timestamp_divide(struct timestamp *ts, unsigned int div); /* ts /= div. please use div as small number*/
int timestamp_arrange(struct timestamp *ts); /* arrange timestamp with its MAX. If overflow, retunr -1 */
int timestamp_cmp(struct timestamp *rt, struct timestamp *lt);  /* If rt > lt, return > 0. If rt == lt return 0. If rt < lt, return < 0 */
void timestamp_cpy(struct timestamp *dst, struct timestamp *src); /* src = dst. Copy with value */
int timestamp_is_empty(struct timestamp *ts); /* Return 1 when ts == 0. Else, return 0 */

void print_timestamp(struct timestamp *ts);

#endif /* TIMESTAMP_H */
