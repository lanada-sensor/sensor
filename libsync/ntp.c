#include "ntp.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

void print_packet(uint8_t *packet, int len)
{
  int i,j;
  uint8_t num, num2;
  uint8_t jisu;

  for(i=0; i<len; i++){
    num = packet[i];
    for(j=7; j>-1; j--){
      jisu = 1 << j;
      num2 = num&jisu;
      num2 = num2 >> j;
      if(num&jisu){
    	  printf("1");
      }
      else{
    	  printf("0");
      }
    } 
    printf(" ");
  }
  printf("\n");
}



void ntp_create(struct ntp **ntp)
{
    *ntp = &ntp_static;
}


void
ntp_init(struct ntp *ntp, struct sclock *clock)
{
  ntp->sc = clock;
  timestamp_init(&(ntp->t1));
  timestamp_init(&(ntp->t2));
  timestamp_init(&(ntp->t3));
  timestamp_init(&(ntp->t4));

#ifdef DEBUG_DRIFT
  total_sec = 0;
  total_drift = 0;
#endif
}

int
ntp_put_t1(struct ntp *ntp, uint8_t *send)
{
  sclock_update(ntp->sc);
  timestamp_cpy(&(ntp->t1), &(ntp->sc->now));
  timestamp_to_packet(&(ntp->t1), send);
  return TIMESTAMP_BYTE;
}
int
ntp_put_t2(struct ntp *ntp, uint8_t *send)
{
  sclock_update(ntp->sc);
  timestamp_cpy(&(ntp->t2), &(ntp->sc->now));
  timestamp_to_packet(&(ntp->t2), send);
  return TIMESTAMP_BYTE;
}
int
ntp_put_t3(struct ntp *ntp, uint8_t *send){
  sclock_update(ntp->sc);
  timestamp_cpy(&(ntp->t3), &(ntp->sc->now));
  timestamp_to_packet(&(ntp->t3), send);
  return TIMESTAMP_BYTE;
}

int
ntp_make_request(struct ntp *ntp, uint8_t *send)
{
  return ntp_put_t1(ntp, send);
}
int
ntp_make_ack(struct ntp *ntp, uint8_t *send, uint8_t *recv)
{
  timestamp_from_packet(&(ntp->t1),recv);
//  send[0] = 2; /* ack preamble |10| */
  timestamp_to_packet(&(ntp->t1), send);
  ntp_put_t2(ntp, send+TIMESTAMP_BYTE);
//  int i;
//  for(i=0; i<100; i++)
//    {
//    }
  ntp_put_t3(ntp, send+TIMESTAMP_BYTE*2);
  return TIMESTAMP_BYTE*3;
}
void
ntp_handle_ack(struct ntp *ntp, uint8_t *recv)
{
  struct timestamp t1_recv;
  if(recv[0] == 0x42)
  {
    timestamp_from_packet(&t1_recv, recv+1);
    /* due to duplicate strobe input, it does not work now. user later */
//    if(timestamp_cmp(&(ntp->t1), &t1_recv) == 0){ /* ntp->t1 == t1_recv */

    timestamp_from_packet(&(ntp->t1), recv+1);
    timestamp_from_packet(&(ntp->t2), recv+(1+TIMESTAMP_BYTE));
    timestamp_from_packet(&(ntp->t3), recv+(1+TIMESTAMP_BYTE*2));
    sclock_now(ntp->sc, &(ntp->t4));
    ntp_calc(ntp);
//    }
//    else{
//      printf("[NTP] t1 mismatch error\n");
//    }
  }
  else{
    printf("[NTP] ack preamble error\n");
  }
}
void
ntp_calc(struct ntp *ntp)
{
  struct timestamp t1;
  timestamp_cpy(&t1,&(ntp->t1));
  struct timestamp t2;
  timestamp_cpy(&t2,&(ntp->t2));
  struct timestamp t3;
  timestamp_cpy(&t3,&(ntp->t3));
  struct timestamp t4;
  timestamp_cpy(&t4,&(ntp->t4));

  struct timestamp t21; /* t2-t1 */
  int neg_21=0;
  struct timestamp t34; /* t3-t4 */
  int neg_34=0;
  struct timestamp offset; /* ((t2-t1)+(t3-t4))/2 */
  timestamp_init(&offset);
  int neg_offset=0;
  struct timestamp rtt; /* (t2-t1)-(t3-t4) */
  timestamp_init(&rtt);
  int neg_rtt=0;

  /* t2-t1 */
  if( timestamp_cmp(&t2, &t1) < 0 ){ /* t2 < t1 */
    timestamp_minus(&t21, &t1, &t2);
    neg_21 = 1;
  }
  else{ /* t2 >= t1 */
    timestamp_minus(&t21, &t2, &t1);
    neg_21 = 0;
  }

  /* t3-t4 */
  if( timestamp_cmp(&t3, &t4) < 0 ){ /* t3 < t4 */
    timestamp_minus(&t34, &t4, &t3);
    neg_34 = 1;
  }
  else{ /* t3 >= t4 */
    timestamp_minus(&t34, &t3, &t4);
    neg_34 = 0;
  }

  /* offset */
  if( neg_21 && neg_34 ){ /* (-)+(-) */
    timestamp_plus(&offset, &t21, &t34);
    neg_offset = 1;
  }
  else if( neg_21 && !neg_34 ){ /* (-)+(+) */
    if( timestamp_cmp(&t21, &t34) > 0 ){ /* t21 > t34 */
      timestamp_minus(&offset, &t21, &t34);
      neg_offset = 1;
    }
    else if( timestamp_cmp(&t21, &t34) < 0 ){ /* t21 < t34 */
      timestamp_minus(&offset, &t34, &t21);
      neg_offset = 0;
    }
  }
  else if( !neg_21 && neg_34 ){ /* (+)+(-) */
    if( timestamp_cmp(&t21, &t34) > 0 ){ /* t21 > t34 */
      timestamp_minus(&offset, &t21, &t34);
      neg_offset = 0;
    }
    else if( timestamp_cmp(&t21, &t34) < 0 ){ /* t21 < t34 */
      timestamp_minus(&offset, &t34, &t21);
      neg_offset = 1;
    }
  }
  else if( !neg_21 && !neg_34 ){ /* (+)+(+) */
    timestamp_plus(&offset, &t21, &t34);
    neg_offset = 0;
  }
  timestamp_divide(&offset, 2);

  /* rtt */
  /* not implemented */

  /* apply offset and rtt */
#ifdef DEBUG_DRIFT
  if( total_sec > 4){
    if(neg_offset){
      total_drift -= offset.msec;
    }
    else{
      total_drift += offset.msec;
    }
  }
  total_sec++;
  printf("%d\t%d\n", total_sec, total_drift);
#endif
  
  struct timestamp threshold;
  timestamp_init(&threshold);
  if( ntp->sc->type == TYPE_RTIMER ){
    threshold.msec = SCLOCK_UNIT_MSEC_RTIMER;
  }
  else if( ntp->sc->type == TYPE_CLOCK ){
    threshold.msec = SCLOCK_UNIT_MSEC_CLOCK;
  }

  if(neg_offset){ /* offset < 0 */
    timestamp_subtract(&(ntp->sc->now), &offset);
    if( timestamp_cmp(&threshold, &offset) < 0 ){ /* threshold < offset */
      //(ntp->sc->slope) ++;
    }
  }
  else if(!neg_offset){ /* offset > 0 */
    timestamp_add(&(ntp->sc->now), &offset);
    if( timestamp_cmp(&threshold, &offset) < 0 ){ /* threshold < offset */
      //(ntp->sc->slope) --;
    }
  }
  
}

void
print_ntp(struct ntp* ntp){
  printf("<t1>\n");
  print_timestamp(&(ntp->t1));
  printf("<t2>\n");
  print_timestamp(&(ntp->t2));
  printf("<t3>\n");
  print_timestamp(&(ntp->t3));
  printf("<t4>\n");
  print_timestamp(&(ntp->t4));
}

