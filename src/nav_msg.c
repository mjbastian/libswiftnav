/*
 * Copyright (C) 2010 Swift Navigation Inc.
 * Contact: Henry Hallam <henry@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <stdio.h>
#include <string.h>
#include <math.h>

#include "constants.h"
#include "nav_msg.h"

#define NAV_MSG_BIT_PHASE_THRES 5

void nav_msg_init(nav_msg_t *n)
{
  /* Initialize the necessary parts of the nav message state structure. */
  n->subframe_bit_index = 0;
  n->bit_phase = 0;
  n->bit_phase_ref = 0;
  n->bit_phase_count = 0;
  n->nav_bit_integrate = 0;
  n->subframe_start_index = 0;
  memset(n->subframe_bits, 0, sizeof(n->subframe_bits));
  n->next_subframe_id = 1;
}

u32 extract_word(nav_msg_t *n, u16 bit_index, u8 n_bits, u8 invert)
{
  /* Extract a word of n_bits length (n_bits <= 32) at position bit_index into
   * the subframe. Takes account of the offset stored in n, and the circular
   * nature of the n->subframe_bits buffer. */

  /* Offset for the start of the subframe in the buffer. */
  if (n->subframe_start_index) {
    if (n->subframe_start_index > 0)
      bit_index += n->subframe_start_index; /* Standard. */
    else {
      bit_index -= n->subframe_start_index; /* Bits are inverse! */
      invert = !invert;
    }

    bit_index--;
  }

  /* Wrap if necessary. */
  if (bit_index > NAV_MSG_SUBFRAME_BITS_LEN*32)
    bit_index -= NAV_MSG_SUBFRAME_BITS_LEN*32;

  u8 bix_hi = bit_index >> 5;
  u8 bix_lo = bit_index & 0x1F;
  u32 word = n->subframe_bits[bix_hi] << bix_lo;

  if (bix_lo) {
    bix_hi++;
    if (bix_hi == NAV_MSG_SUBFRAME_BITS_LEN)
      bix_hi = 0;
    word |=  n->subframe_bits[bix_hi] >> (32 - bix_lo);
  }

  if (invert)
    word = ~word;

  return word >> (32 - n_bits);
}


s32 nav_msg_update(nav_msg_t *n, s32 corr_prompt_real)
{
  /* Called once per tracking loop update (atm fixed at 1 PRN [1 ms]). Performs
   * the necessary steps to recover the nav bit clock, store the nav bits and
   * decode them. */

  s32 TOW_ms = -1;

  /* Do we have bit phase lock yet? (Do we know which of the 20 possible PRN
   * offsets corresponds to the nav bit edges?) */
  n->bit_phase++;
  n->bit_phase %= 20;

  if (n->bit_phase_count < NAV_MSG_BIT_PHASE_THRES) {

    /* No bit phase lock yet. */
    if ((n->nav_bit_integrate > 0) != (corr_prompt_real > 0)) {
      /* Edge detected. */
      if (n->bit_phase == n->bit_phase_ref)
        /* This edge came N*20 ms after the last one. */
        n->bit_phase_count++;
      else {
        /* Store the bit phase hypothesis. */
        n->bit_phase_ref = n->bit_phase;
        n->bit_phase_count = 1;
      }
    }
    /* Store the correlation for next time. */
    n->nav_bit_integrate = corr_prompt_real;

  } else {

    /* We have bit phase lock. */
    if (n->bit_phase != n->bit_phase_ref) {
      /* Sum the correlations over the 20 ms bit period. */
      n->nav_bit_integrate += corr_prompt_real;
    } else {
      /* Dump the nav bit, i.e. determine the sign of the correlation over the
       * nav bit period. */

      /* Is bit 1? */
      if (n->nav_bit_integrate > 0) {
        n->subframe_bits[n->subframe_bit_index >> 5] |= \
          1 << (31 - (n->subframe_bit_index & 0x1F));
      } else {
        /* Integrated correlation is negative, so bit is 0. */
        n->subframe_bits[n->subframe_bit_index >> 5] &= \
          ~(1 << (31 - (n->subframe_bit_index & 0x1F)));
      }

      /* Zero the integrator for the next nav bit. */
      n->nav_bit_integrate = 0;

      n->subframe_bit_index++;
      if (n->subframe_bit_index == NAV_MSG_SUBFRAME_BITS_LEN*32)
        n->subframe_bit_index = 0;

      /* Yo dawg, are we still looking for the preamble? */
      if (!n->subframe_start_index) {
        /* We're going to look for the preamble at a time 360 nav bits ago,
         * then again 60 nav bits ago. */
        #define SUBFRAME_START_BUFFER_OFFSET (NAV_MSG_SUBFRAME_BITS_LEN*32 - 360)

        /* Check whether there's a preamble at the start of the circular
         * subframe_bits buffer. */
        u8 preamble_candidate = extract_word(n, n->subframe_bit_index + SUBFRAME_START_BUFFER_OFFSET, 8, 0);

        if (preamble_candidate == 0x8B) {
           n->subframe_start_index = n->subframe_bit_index + SUBFRAME_START_BUFFER_OFFSET + 1;
        }
        else if (preamble_candidate == 0x74) {
           n->subframe_start_index = -(n->subframe_bit_index + SUBFRAME_START_BUFFER_OFFSET + 1);
        }

        if (n->subframe_start_index) {
          // Looks like we found a preamble, but let's confirm.
          if (extract_word(n, 300, 8, 0) == 0x8B) {
            // There's another preamble in the following subframe.  Looks good so far.
            // Extract the TOW:

            unsigned int TOW_trunc = extract_word(n,30,17,extract_word(n,29,1,0)); // bit 29 is D30* for the second word, where the TOW resides.
            TOW_trunc++;  // Increment it, to see what we expect at the start of the next subframe
            if (TOW_trunc >= 7*24*60*10)  // Handle end of week rollover
              TOW_trunc = 0;

            if (TOW_trunc == extract_word(n,330,17,extract_word(n,329,1,0))) {
              // We got two appropriately spaced preambles, and two matching TOW counts.  Pretty certain now.

              // The TOW in the message is for the start of the NEXT subframe.
              // That is, 240 nav bits' time from now, since we are 60 nav bits into the second subframe that we recorded.
              if (TOW_trunc)
                TOW_ms = TOW_trunc * 6000 - (300-60)*20;
              else  // end of week special case
                TOW_ms = 7*24*60*60*1000 - (300-60)*20;
              //printf("TOW = hh:%02d:%02d.%03d\n", (int) (TOW_ms / 60000 % 60), (int)(TOW_ms / 1000 % 60), (int)(TOW_ms % 1000));
			  n->tow = TOW_ms / 1000;
            } else
              n->subframe_start_index = 0;  // the TOW counts didn't match - disregard.
          } else
            n->subframe_start_index = 0;    // didn't find a second preamble in the right spot - disregard.
        }
      }
    }
  }
  return TOW_ms;
}


int parity(u32 x)
{
  /* Returns 1 if there are an odd number of bits set. */
  x ^= x >> 1;
  x ^= x >> 2;
  x ^= x >> 4;
  x ^= x >> 8;
  x ^= x >> 16;
  return (x & 1);
}

int nav_parity(u32 *word) {
// expects a word where MSB = D29*, bit 30 = D30*, bit 29 = D1, ... LSB = D30 as described in IS-GPS-200E Table 20-XIV
// Inverts the bits if necessary, and checks the parity.
// Returns 0 for success, 1 for fail.

  if (*word & 1<<30)     // inspect D30*
    *word ^= 0x3FFFFFC0; // invert all the data bits!

 // printf("w=%08X  ",(unsigned int )word);

  if (parity(*word & 0xBB1F34A0 /* 0b10111011000111110011010010100000 */)) // check d25 (see IS-GPS-200E Table 20-XIV)
    return 25;

  if (parity(*word & 0x5D8F9A50 /* 0b01011101100011111001101001010000 */)) // check d26
    return 26;

  if (parity(*word & 0xAEC7CD08 /* 0b10101110110001111100110100001000 */)) // check d27
    return 27;

  if (parity(*word & 0x5763E684 /* 0b01010111011000111110011010000100 */)) // check d28
    return 28;

  if (parity(*word & 0x6BB1F342 /* 0b01101011101100011111001101000010 */)) // check d29
    return 29;

  if (parity(*word & 0x8B7A89C1 /* 0b10001011011110101000100111000001 */)) // check d30
    return 30;

  return 0;
}

bool subframe_ready(nav_msg_t *n) {
  return (n->subframe_start_index != 0);
}

s32 sign_extend(u32 arg, u32 bits)
{
	union {
		int s32;
		unsigned u32;
	} fourbyte;
	fourbyte.u32 = arg;
	fourbyte.u32 <<= (32-bits);
	fourbyte.s32 >>= (32-bits); // sign-extend it
	return fourbyte.s32;
}

/* TODO: move to broadcast_almanac.c/h */
#include "almanac.h"

sat_datastore_t	sat_datastore[MAXSTORE];

void pagestore_init(void)
{
	for (int i=0; i<MAXSTORE; i++) {
		memset(&sat_datastore[i], 0, sizeof(sat_datastore_t) );
		sat_datastore[i].prn = i;
		sat_datastore[i].toa_completed = 0;
		sat_datastore[i].toa_mark = 0;
	}
}

void almanac_words_to_alm(const u32 words[], almanac_t *alm, u8 prn, u16 weekno)
{
	alm->prn = prn;
	
	/**< Eccentricity in radians. */
	u32 ecc		= (words[3-3] >> (30 - 24) & 0xffff);		  /* 16 bits, apply scale 2^-21 */
	alm->ecc	= ecc * pow(2, -21);

	/**< Time of Applicability in seconds since Sunday. */
	u32 toa		= ((words[4-3] >> (30 - 8)  &   0xff) << 12); /* 8 bits,  scale 2^12 */
	alm->toa	= toa; 				 	

	/**< Inclination in radians. */
	// TODO: check PI accuracy
	alm->inc	= (0.3f + (sign_extend((words[4-3] >> (30 - 24) & 0xffff) , 16)) * pow(2, -19)) * M_PI;	
	
	/**< Rate of Right Ascension in radians/sec. */ 
	s32 rora	= sign_extend((words[5-3] >> (30 - 16) & 0xffff), 16);
	alm->rora	= rora * pow(2, -38) * M_PI;	
	u32 health	= ((words[5-3] >> (30 - 24) & 0xff));
	alm->healthy = health ? 0 : 1; /* TODO: decode bits */
	
	/**< Semi-major axis in meters. */
	u32 sqrta	= ((words[6-3] >> (30 - 24) & 0xffffff));
	alm->a  	= sqrta * pow(2, -11);	
	alm->a		= alm->a * alm->a; // remove sqrt
	
	/**< Right Ascension at Week in radians. */
	s32 raaw	= sign_extend((words[7-3] >> (30 - 24) & 0xffffff), 24);
	alm->raaw   = raaw * pow(2, -23) * M_PI;	
	
	/**< Argument of Perigee in radians. */
	s32 argp    = sign_extend((words[8-3] >> (30 - 24) & 0xffffff), 24);
	alm->argp	= argp * pow(2, -23) * M_PI;

 	/**< Mean Anomaly at Time of Applicability in radians. */
	s32 m0		= sign_extend((words[9-3] >> (30 - 24) & 0xffffff), 24);
	alm->ma		= m0 * pow(2, -23) * M_PI;

	/**< 0-order clock correction in seconds. */
	/**< 1-order clock correction in seconds/second. */
	u32 afw     = (words[10-3] >> (30 - 24) & 0xffffff) >> 2; /* remove t bits next to parity */
	// t = b0 b 1
	u32 af0 = (afw & 0x7) | (((afw >> (3 + 11)) & 0xff) << 3);
	u32 af1 = (afw >> 3) & 0x7ff;
	alm->af0    = sign_extend(af0, 11) * pow(2, -20);	
	alm->af1	= sign_extend(af1, 11) * pow(2, -38);   
	
	/* copy weekno */
	alm->week = weekno;
}

void almanac_page17words_sf4to_struct(u32 words[], page17_t *p17)
{
	/* special message */
	u32 w3		= ((words[3-3] >> (30 - 24) & 0xffffff));
	p17->svid       = (w3 >> 16) & 0xff;
	p17->message[0] = (w3 >> 8) & 0xff;
	p17->message[1] = (w3 >> 0) & 0xff;
	int j = 2;
	for (int i=4; i<10; i++) {
		u32 w4		= ((words[i-3] >> (30 - 24) & 0xffffff));
		p17->message[j++] = (w4 >> 16) & 0xff;
		p17->message[j++] = (w4 >> 8) & 0xff;
		p17->message[j++] = (w4 >> 0) & 0xff;
	}
	u32 w10		= ((words[10-3] >> (30 - 24) & 0xffffff));
	p17->message[j++] = (w10 >> 16) & 0xff;
	p17->message[j++] = (w10 >> 8) & 0xff;
	p17->message[j] =0;
}

void almanac_page18words_sf4to_struct(u32 words[], page18_t *p18)
{
	/* UTC and ionospheric data */
	u32 w3		= (words[3-3] >> (30 - 24) & 0xffffff);
	p18->svid   = (w3 >> 16) & 0xff;
	p18->a[0]	= (w3 >> 8)  & 0xff;
	p18->a[1]	= (w3 >> 0)  & 0xff;
	u32 w4		= (words[4-3] >> (30 - 24) & 0xffffff);
	p18->a[2]	= (w4 >> 8)  & 0xff;
	p18->a[3]	= (w4 >> 8)  & 0xff;
	p18->b[0]	= (w4 >> 8)  & 0xff;
	u32 w5		= (words[5-3] >> (30 - 24) & 0xffffff);
	p18->b[1]	= (w5 >> 8)  & 0xff;
	p18->b[2]	= (w5 >> 8)  & 0xff;
	p18->b[3]	= (w5 >> 0)  & 0xff;
	u32 w6		= (words[6-3] >> (30 - 24) & 0xffffff);
	p18->a1		= w6;
	u32 w7		= (words[7-3] >> (30 - 24) & 0xffffff);
	u32 w8		= (words[8-3] >> (30 - 24) & 0xffffff);
	p18->a0		= (w7 << 8) | ((w8 >> 16) & 0xff);
	p18->t_ot	= (w8 >> 8)  & 0xff;
	p18->wn_t	= (w8 >> 0)  & 0xff;
	u32 w9		= (words[9-3] >> (30 - 24) & 0xffffff);
	p18->dt_ls	= (w9 >> 16) & 0xff;
	p18->wn_lsf	= (w9 >> 8)  & 0xff;
	p18->dn     = (w9 >> 0)  & 0xff;
	u32 w10		= (words[10-3] >> (30 - 24) & 0xffffff);
	p18->d_t_lsf= (w10 >> 16) & 0xff;
	p18->res    = (w10 >> 2) & 0x3fff;
}

void almanac_page25words_sf4to_struct(u32 words[], page25_t *p25)
{
	p25->svid_4 = 0; /* TODO decode */
	u32 w3		= ((words[3-3] >> (30 - 24) & 0xffffff));
	p25->aspoof[0]  = (w3 >> 12) & 0x0f;
	p25->aspoof[1]  = (w3 >> 8 ) & 0x0f;
	p25->aspoof[2]  = (w3 >> 4 ) & 0x0f;
	p25->aspoof[3]  = (w3 >> 0 ) & 0x0f;
	u32 w4		= ((words[4-3] >> (30 - 24) & 0xffffff));
	p25->aspoof[4]  = (w4 >> 20) & 0x0f;
	p25->aspoof[5]  = (w4 >> 16) & 0x0f;
	p25->aspoof[6]  = (w4 >> 12) & 0x0f;
	p25->aspoof[7]  = (w4 >> 8 ) & 0x0f;
	p25->aspoof[8]  = (w4 >> 4 ) & 0x0f;
	p25->aspoof[9]  = (w4 >> 0 ) & 0x0f;
	u32 w5		= ((words[5-3] >> (30 - 24) & 0xffffff));
	p25->aspoof[10] = (w5 >> 20) & 0x0f;
	p25->aspoof[11] = (w5 >> 16) & 0x0f;
	p25->aspoof[12] = (w5 >> 12) & 0x0f;
	p25->aspoof[13] = (w5 >> 8 ) & 0x0f;
	p25->aspoof[14] = (w5 >> 4 ) & 0x0f;
	p25->aspoof[15] = (w5 >> 0 ) & 0x0f;
	u32 w6		= ((words[6-3] >> (30 - 24) & 0xffffff));
	p25->aspoof[16] = (w6 >> 20) & 0x0f;
	p25->aspoof[17] = (w6 >> 16) & 0x0f;
	p25->aspoof[18] = (w6 >> 12) & 0x0f;
	p25->aspoof[19] = (w6 >> 8 ) & 0x0f;
	p25->aspoof[20] = (w6 >> 4 ) & 0x0f;
	p25->aspoof[21] = (w6 >> 0 ) & 0x0f;
	u32 w7		= ((words[7-3] >> (30 - 24) & 0xffffff));
	p25->aspoof[22] = (w7 >> 20) & 0x0f;
	p25->aspoof[23] = (w7 >> 16) & 0x0f;
	p25->aspoof[24] = (w7 >> 12) & 0x0f;
	p25->aspoof[25] = (w7 >> 8 ) & 0x0f;
	p25->aspoof[26] = (w7 >> 4 ) & 0x0f;
	p25->aspoof[27] = (w7 >> 0 ) & 0x0f;
	u32 w8		= ((words[8-3] >> (30 - 24) & 0xffffff));
	p25->aspoof[28] = (w8 >> 20) & 0x0f;
	p25->aspoof[29] = (w8 >> 16) & 0x0f;
	p25->aspoof[30] = (w8 >> 12) & 0x0f;
	p25->aspoof[31] = (w8 >> 8 ) & 0x0f;
	/* health data for sv 25-32 */
	// TODO: 2 reserved bits
	p25->svhealth[24] = (w8 & 0x3f);
	u32 w9		= ((words[9-3] >> (30 - 24) & 0xffffff));
	p25->svhealth[25]  = (w9 >> 18) & 0x3f;
	p25->svhealth[26]  = (w9 >> 12) & 0x3f;
	p25->svhealth[27]  = (w9 >> 6 ) & 0x3f;
	p25->svhealth[28]  = (w9 >> 0 ) & 0x3f;
	u32 w10		= ((words[10-3] >> (30 - 24) & 0xffffff));
	p25->svhealth[29]  = (w10 >> 18) & 0x3f;
	p25->svhealth[30]  = (w10 >> 12) & 0x3f;
	p25->svhealth[31]  = (w10 >> 6 ) & 0x3f;
	/* TODO: 4 reserved bits */
}

void almanac_page25words_sf5to_struct(u32 words[], page25_t *p25, u8 prn)
{
	p25->prn = prn;
	p25->svid_5 = 0; /* TODO decode */
	
	/**< TOA and WNA */
	u32 w3			= (words[3-3] >> (30 - 24) & 0xffff);		/* 16 bits, apply scale 2^-21 */
	p25->toa		= ((w3 >> 8) & 0xff) << 12;					/* apply scale 2^12 */
	p25->wna_raw	= (w3 & 0xff);								/* needs to be merged with full 10-bit weekno of sf1 */

	u32 w4		= ((words[4-3] >> (30 - 24) & 0xffffff));
	p25->svhealth[0]  = (w4 >> 18) & 0x3f;
	p25->svhealth[1]  = (w4 >> 12) & 0x3f;
	p25->svhealth[2]  = (w4 >> 6 ) & 0x3f;
	p25->svhealth[3]  = (w4 >> 0 ) & 0x3f;
	u32 w5		= ((words[5-3] >> (30 - 24) & 0xffffff));
	p25->svhealth[4]  = (w5 >> 18) & 0x3f;
	p25->svhealth[5]  = (w5 >> 12) & 0x3f;
	p25->svhealth[6]  = (w5 >> 6 ) & 0x3f;
	p25->svhealth[7]  = (w5 >> 0 ) & 0x3f;
	u32 w6		= ((words[6-3] >> (30 - 24) & 0xffffff));
	p25->svhealth[8]  = (w6 >> 18) & 0x3f;
	p25->svhealth[9]  = (w6 >> 12) & 0x3f;
	p25->svhealth[10] = (w6 >> 6 ) & 0x3f;
	p25->svhealth[11] = (w6 >> 0 ) & 0x3f;
	u32 w7		= ((words[7-3] >> (30 - 24) & 0xffffff));
	p25->svhealth[12] = (w7 >> 18) & 0x3f;
	p25->svhealth[13] = (w7 >> 12) & 0x3f;
	p25->svhealth[14] = (w7 >> 6 ) & 0x3f;
	p25->svhealth[15] = (w7 >> 0 ) & 0x3f;
	u32 w8		= ((words[8-3] >> (30 - 24) & 0xffffff));
	p25->svhealth[16] = (w8 >> 18) & 0x3f;
	p25->svhealth[17] = (w8 >> 12) & 0x3f;
	p25->svhealth[18] = (w8 >> 6 ) & 0x3f;
	p25->svhealth[19] = (w8 >> 0 ) & 0x3f;
	u32 w9		= ((words[9-3] >> (30 - 24) & 0xffffff));
	p25->svhealth[20] = (w9 >> 18) & 0x3f;
	p25->svhealth[21] = (w9 >> 12) & 0x3f;
	p25->svhealth[22] = (w9 >> 6 ) & 0x3f;
	p25->svhealth[23] = (w9 >> 0 ) & 0x3f;
	u32 w10		= ((words[10-3] >> (30 - 24) & 0xffffff));
	p25->res_6b   = (w10 >> 18) & 0x3f;
	p25->res_16b  = (w10 >> 2) & 0xffff;
}

void almanac_alm_to_yuma(almanac_t *alm)
{
	printf("******** Week %d almanac for PRN-%02d ********\n", alm->week, alm->prn);
	printf("ID:                         %02d\n", alm->prn);
	printf("Health:                     %03o\n", alm->healthy ? 0 : 1);
	printf("Eccentricity:               %0.10E\n", alm->ecc);
	printf("Time of Applicability(s):   %6.4f\n", alm->toa);
	printf("Orbital Inclination(rad):   %0.10f\n", alm->inc);
	printf("Rate of Right Ascen(r/s):   %0.10E\n", alm->rora);
	printf("SQRT(A)  (m 1/2):           %4.6g\n", sqrt(alm->a));
	printf("Right Ascen at Week(rad):   %0.10E\n", alm->raaw);
	printf("Argument of Perigee(rad):   %0.10f\n", alm->argp);
	printf("Mean Anom(rad):             %1.10E\n", alm->ma);
	printf("Af0(s):                     %0.10E\n", alm->af0);
	printf("Af1(s/s):                   %0.10E\n", alm->af1);
	printf("week:                       %4d\n\n", alm->week);
}
extern almanac_t almanac[32];

void almanac_page17_dump(page17_t *p17)
{
	printf("Message: '");
	for (int i=0; i<22; i++) {
		printf("%c", p17->message[i]);
	}
	printf("'\n");
}

void almanac_page18_dump(page18_t *p18)
{
	printf("page 18 data:\n");
	printf("svid %d\n", p18->svid);
	printf("a[]  %d %d %d %d\n", p18->a[0], p18->a[1], p18->a[2], p18->a[3]);
	printf("b[]  %d %d %d %d\n", p18->b[0], p18->b[1], p18->b[2], p18->b[3]);
	printf("a1[]  %lu\n", p18->a1);
	printf("a0[]  %lu\n", p18->a0);
	printf("t_ot     0x%02x\n", p18->t_ot);
	printf("wn_t     0x%02x\n", p18->wn_t);
	printf("dt_ls    0x%02x\n", p18->dt_ls);
	printf("wn_lsf   0x%02x\n", p18->wn_lsf);
	printf("dn       0x%02x\n", p18->dn);
	printf("d_t_lsf  0x%02x\n", p18->d_t_lsf);
	printf("res      0x%04x\n", p18->res);
	
	printf("\n");
}

void almanac_page25_dump(page25_t *p25, u8 prn)
{
	printf("Page25 of PRN %d TOA %lu WNAraw %02x \n ", prn+1, p25->toa, p25->wna_raw);
	printf("as: [");
	for (int i=0; i<32; i++) {
		printf("%02x ", ((p25->aspoof[i] & 0x08) << 1) | (p25->aspoof[i] & 0x7));
	}
	printf("]\n");
	printf("health: [");
	for (int i=0; i<32; i++) {
		printf("%02x ", p25->svhealth[i]);
	}
	printf("]\n");
}

const u32 *almanac_bcwords_get(const sat_datastore_t* sd, u8 prn)
{
	const u32 *words;
	u8 i = prn;
	if (i < 25) {
		// only 25 entries long
		words = &sd->sf45_words[i][1][0];
	} else {
		int page = 0;
		switch (i) {
			case 25: case 26: case 27: case 28:
				page = 2 + (i - 25);
				words = &sd->sf45_words[page-1][0][8];
				break;
			case 29: case 30: case 31: case 32:
				page = 7 + (i - 25);
				words = &sd->sf45_words[page-1][0][8];
				break;
		}
	}
	return words;
}

void almanac_bcwords_dump(const sat_datastore_t* sd, u8 prn)
{
	const u32 *words;
	words = almanac_bcwords_get(sd, prn);
	for (int i=0; i<8; i++) {
		u32 word = (words[i] >> 1);
		printf(" %08lx %2lx", word & 0x3fffffc0, word & 0x3f);
	}
	printf("\n");
}

void almanac_bcwords_to_alm(almanac_t *dstalm, const sat_datastore_t* sd, u8 prn)
{
	const u32 *words;
	u16 weekno = sd->weekno;
	memset(&dstalm, 0, sizeof(almanac_t));
	words = almanac_bcwords_get(sd, prn);
	almanac_words_to_alm(words, dstalm, prn + 1, weekno); // TODO: check +1
}

const u32 rom_almanac[32][8] = {
	/* words with parity to be converted to almanac_t */
	{ 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000 } , // PRN 01
	{ 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000 } , // PRN 02
	{ 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000 } , // PRN 03
	{ 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000 } , // PRN 04
	{ 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000 } , // PRN 05
	{ 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000 } , // PRN 06
	{ 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000 } , // PRN 07
	{ 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000 } , // PRN 08
	{ 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000 } , // PRN 09
	{ 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000 } , // PRN 10
	{ 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000 } , // PRN 11
	{ 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000 } , // PRN 12
	{ 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000 } , // PRN 13
	{ 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000 } , // PRN 14
	{ 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000 } , // PRN 15
	{ 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000 } , // PRN 16
	{ 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000 } , // PRN 17
	{ 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000 } , // PRN 18
	{ 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000 } , // PRN 19
	{ 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000 } , // PRN 20
	{ 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000 } , // PRN 21
	{ 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000 } , // PRN 22
	{ 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000 } , // PRN 23
	{ 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000 } , // PRN 24
	{ 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000 } , // PRN 25
	{ 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000 } , // PRN 26
	{ 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000 } , // PRN 27
	{ 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000 } , // PRN 28
	{ 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000 } , // PRN 29
	{ 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000 } , // PRN 30
	{ 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000 } , // PRN 31
	{ 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000 }   // PRN 32
};
const u16 rom_weekno = 748;

void almanac_getrom(almanac_t *dstalm, u8 prn)
{
	// rom_almanac
	const u32 *words;
	u16 weekno = rom_weekno;
	memset(&dstalm, 0, sizeof(almanac_t));
	words = &rom_almanac[prn][0];
	almanac_words_to_alm(words, dstalm, prn, weekno);
}

void process_almanac_dump(sat_datastore_t* sd)
{
	printf("Almanac as produced by PRN %d:\n", sd->prn + 1);
	for (int i=0; i<32; i++) {
		/* convert broadcast words to almanac */
		almanac_t alm;
		almanac_bcwords_dump(sd, i);
		almanac_bcwords_to_alm(&alm, sd, i);
		// dump in yuma
		almanac_alm_to_yuma(&alm);
	}
	// dump p17
	almanac_page17_dump(&sd->p17);
	// dump p18 
	almanac_page18_dump(&sd->p18);
	// dump p25 
	almanac_page25_dump(&sd->p25, sd->prn);
}

typedef struct {
//	u8 srcprn;	/* prn supplying the version of the almanac */
//	u8 prn;     /* target prn describted by the almanac entry */
	u16 weekno;	/* gpsweekno mod 1024 */
	u8 	toa;	/* toa div 2^12 */
	u8  wtcount;	/* count of same weekno/toa combinations of targetprn's */
} almanac_score_t;

/* 1st idx is srnprn, 2nd idx dstprn */
almanac_score_t almanac_score[32][32];

void alamanc_scores_init(void)
{
	/* location is implicit */
	for (int i=0; i<32; i++)
		for (int j=0; j<32; j++) {
			almanac_score[i][j].weekno  = 0xffff; /* invalid entry */
			almanac_score[i][j].toa     = 0;
			almanac_score[i][j].wtcount = 0;
		}
}

/* updates almanac scores for prn */
void update_alamanc_scores(u8 dstprn)
{
	// dstprn 0..31
	/* loop over all srcprn's providing almanac data for this prn */
	for (int i=0; i<32; i++) {
		almanac_score[i][dstprn].wtcount = 0;
	}
	/* count via group by toa/weekno combinations */
	for (int i=0; i<32; i++) {
		u16 weekno = almanac_score[i][dstprn].weekno;
		if (weekno != 0xffff) {
			/* valid weekno, toa must be valid too */
			u8 toa = almanac_score[i][dstprn].toa;
			/* now look for entries with same weekno/toa combination */
			for (int j=0; j<32; j++) {
				if (j == i) continue; /* skip own entry */
				if ((almanac_score[j][dstprn].toa == toa) &&
					(almanac_score[j][dstprn].weekno == weekno)) {
					almanac_score[j][dstprn].wtcount++;
				}
			}
		}
	}
}

u8 find_best_bc_almanac_for_prn(u8 dstprn, u16 weekno, u8 toa)
{
	/* returns 1st srcprn with best score */
	int score = 0;
	u8 srcprn = 0xff; // invalid
	for (int i=0; i<32; i++) {
		/* check if weekno matches too */
		if (almanac_score[i][dstprn].weekno != 0xffff) {
			/* valid entry */
			if (almanac_score[i][dstprn].weekno > weekno) {
				printf("almanac weekno too new\n");
			} else {
				if (almanac_score[i][dstprn].toa > toa) {
					printf("almanac toa too new\n");
				} else {
					if (almanac_score[i][dstprn].wtcount > score) {
						score = almanac_score[i][dstprn].wtcount;
						srcprn = i;
						printf("found srcprn %d score %d\n", srcprn, score);
					} else {
						printf("almanac weekno %u toa %u score %u too low\n", weekno, toa, almanac_score[i][dstprn].wtcount);
					}
				}
			}
		}
	}
	return srcprn; /* use alm[ret] */
}

/* find any almanac for prn */
void find_any_almanac_for_prn(almanac_t *dstalmanac, u8 prn, u16 weekno, u8 toa)
{
	/* guess weekno and toa */
	u8 srcprn = find_best_bc_almanac_for_prn(prn, weekno, toa);
	if (srcprn == 0xff) {
		/* no bc almanac available, use CG almanac */
		
		/* no CG almanac ? use rom-based one that was valid on production date */
		almanac_getrom(dstalmanac, prn);
	} else {
		 /* build almanac_t from found datastore */
		 almanac_bcwords_to_alm(dstalmanac, &sat_datastore[srcprn], prn);
	}
}

/* 
 *	decode almanac data for a single sat
 */
s8 process_almanac_checkcomplete(sat_datastore_t* sd, u32 weekno)
{
	u8 percent = 0;
	/* check if all sf4 and sf5 bits are set */
	for (int i=0; i<25; i++) {
		percent += (sd->pagestatus[i] >> 3) & 0x01;
		percent += (sd->pagestatus[i] >> 4) & 0x01;
	}
//	printf("checkcomplete: %d of 50 weekno %u\n", percent, weekno);
	if (percent < 50) 
		return 0;
	/* check if ephemeris is also picked up */
	if ((sd->pagestatus[0] & 0x07) == 0x07) {
		/* set is complete */
		sd->toa_mark 		= sd->p25.toa; /* already scaled to seconds */
		sd->toa_completed	= 1;
		/* now calc weekno for almanac */
		sd->weekno			= (weekno & 0xffffff00) | (sd->p25.wna_raw & 0xff);
		printf("prn %u COMPLETE TOA %lu Weekno %lu merged weekno %lu\n", sd->prn, sd->toa_mark, weekno, sd->weekno);
		
		/* dump in YUMA format to debug*/
		// process_almanac_dump(sd); // disabled  DEBUG too big for outputbuffer 
	} else {
		// almanac complete, but not ephermeris ???
		printf("ODD almanac complete, but still no ephemeris %02x\n", sd->pagestatus[0]);
		return 0;
	}
	// complete!
	return 1;
}

void process_almanac_markincomplete(sat_datastore_t *sd, u8 sf_id, u8 page)
{
	if (sf_id < 4) return;
	if (sf_id > 5) return;
	sd->pagestatus[page] &= ~(1 << sf_id);
}
 
void process_almanac_words(u32 words[], u8 prn, u16 weekno)
{
	almanac_t alm;
	memset(&alm, 0, sizeof(almanac_t));
	almanac_words_to_alm(words, &alm, prn + 1, weekno);
#if 0
	almanac_alm_to_yuma(&alm);
	/* dump flash-based almanac too */
	almanac_alm_to_yuma(&almanac[alm.prn-1]); /* 1st entry is dummy entry ? */
#endif
}

void process_page17_sf4_words(page17_t *p17, u32 words[])
{
	almanac_page17words_sf4to_struct(words, p17);
	almanac_page17_dump(p17);
}

void process_page18_sf4_words(page18_t *p18, u32 words[])
{
	almanac_page18words_sf4to_struct(words, p18);
	almanac_page18_dump(p18);
}

void process_page25_sf4_words(page25_t *p25, u32 words[])
{
	almanac_page25words_sf4to_struct(words, p25);
	// will be printed along with sf5 
}

void process_page25_sf5_words(page25_t *p25, u32 words[], u8 prn)
{
	almanac_page25words_sf5to_struct(words, p25, prn);
	almanac_page25_dump(p25, prn);
}

void almanacbuilder_add(u8 srcprn, u8 page, u8 sf_id, u32 words[], u16 weekno)
{
	page17_t *p17 = &sat_datastore[srcprn].p17; /* special message */
	page18_t *p18 = &sat_datastore[srcprn].p18; /* utc & ionospheric data */
	page25_t *p25 = &sat_datastore[srcprn].p25; /* almanacset meta data */
	/* decode page, sf_id, check validity mask */
	if (sf_id == 5) {
		if (page < 24) {
			// almanac_t *alm = &almanacsets[ai].alm[page];
			/* almanac data for PRN1-24 */
			u8 prn = page;
			printf("srcprn %d sfid 5 BC Almanac for SV %d weekno %d ", srcprn+1, prn+1, weekno);
			process_almanac_words(words, prn, weekno);
		} else {
			printf("page 25: applicability data \n");
			process_page25_sf5_words(p25, words, srcprn);
		}
	} else if (sf_id == 4) {
		u8 prn;
		/* parse SV 25-32 data */
		switch (page) {
			case 2: case 3: case 4: case 5:
				/* pages 2,3,4,5 almanac data of SV 25,26,27,28 */
				prn =  25 + (page - 2);
				printf("srcprn %d sfid 4 BC Almanac data for SV %d\n", srcprn+1, prn + 1);
				process_almanac_words(words, prn, weekno);
				break;
			// 6 reserved above
			case 7: case 8: case 9: case 10:
				/* pages 7,8,9,10 almanac data of SV 29,30,31,32 */
				prn = 29 + (page - 7);
				printf("srcprn %d sfid 4 BC Almanac data for SV %d\n", srcprn+1,prn + 1);
				process_almanac_words(words, prn, weekno);
				break;
			case 16: /* mod 25, 16 = page 17 */
				/* page 17: special messages */
				printf("special message page %d\n", page);
				process_page17_sf4_words(p17, words);
				break;
			case 17: /* mod 24, 17 = page 18 */
				/* page 18 ionospheric and UTC data */
				printf("ionospheric and UTC data\n");
				process_page18_sf4_words(p18, words);
				break;			
			case 24:
				process_page25_sf4_words(p25, words); // no PRN yet, will be filled once sf5 sweeps by
				break;
		}
	}
}

void pagestore_process(nav_msg_t *n, u8 page, u8 sf_id)
{
	u8 slot = n->src_prn;
		
	/* store and compare data in n->frame_words[5][8]; */
	if (sf_id >= 4) {
		/* sf_id 4..5  store in almanac set section */
		if ((sat_datastore[slot].pagestatus[page] & (1 << (sf_id-1))) == 0) {
			/* store in alamanac section (repeated every 12.5 mins) */
			printf("PRN %2d page %2d sf %2d STOR ", n->src_prn+1, page+1, sf_id);
			for (int i=0; i<8; i++) {
				u32 word = (n->frame_words[sf_id-1][i] >> 1);
				printf(" %08lx %2lx", word & 0x3fffffc0, word & 0x3f);
			}
			printf("\n");
			// store
			u32 *dst = &sat_datastore[slot].sf45_words[page][sf_id-4][0];
			memcpy(dst, &n->frame_words[sf_id-1][0], 8*sizeof(u32));
			// flag storage is used
			sat_datastore[slot].pagestatus[page] |= (1 << (sf_id-1));
			// post-process sf 4/5 into one or more almanacversions
			almanacbuilder_add(n->src_prn, page, sf_id, dst, n->weekno);
			
			/* sf4 or sf5 decoded, check if complete / trigger save almanac of a specific SRCPRN to flash */
			process_almanac_checkcomplete(&sat_datastore[n->src_prn], n->weekno);
		} else {
			/* compare with existing alamanac data, repeated every 12.5 mins */
			if (memcmp(&n->frame_words[sf_id-1][0], &sat_datastore[slot].sf45_words[page][sf_id-4][0], 8*sizeof(u32)) == 0) {
				printf("PRN %2d page %2d sf %2d same ", n->src_prn+1, page+1, sf_id);
				for (int i=0; i<8; i++) {
					u32 word = (n->frame_words[sf_id-1][i] >> 1);
					printf(" %08lx %2lx", word & 0x3fffffc0, word & 0x3f);
				}
				printf("\n");
				process_almanac_checkcomplete(&sat_datastore[n->src_prn], n->weekno);
			} else {
				printf("PRN %2d page %2d sf %2d CHGD \n", n->src_prn+1, page+1, sf_id);
				process_almanac_markincomplete(&sat_datastore[slot], sf_id, page);
				printf("FR ");
				for (int i=0; i<8; i++) {
					u32 word = sat_datastore[slot].sf45_words[page][sf_id-4][i] >> 1;
					printf(" %08lx %2lx", word & 0x3fffffc0, word & 0x3f);
				}
				printf("\nTO ");
				for (int i=0; i<8; i++) {
					u32 word = (n->frame_words[sf_id-1][i] >> 1);
					printf(" %08lx %2lx", word & 0x3fffffc0, word & 0x3f);
				}
				printf("\n");
				// store new value
				memcpy(&sat_datastore[slot].sf45_words[page][sf_id-4][0], &n->frame_words[sf_id-1][0], 8*sizeof(u32));

				

				// post-process into one or more new almanac versions
				//almanacbuilder_change(n->src_prn, page, sf_id, &sat_datastore[slot].sf45_words[page][sf_id-4][0]);
				process_almanac_checkcomplete(&sat_datastore[n->src_prn], n->weekno);
			}
		}		
	} else {
		/* sf_id 1..3  store in ephemeris section (repeated every 30 secs) */
		if ((sat_datastore[slot].pagestatus[0] & (1 << (sf_id-1))) == 0) {
			/* new epheremis data received */
#if 0			
			printf("PRN %2d EPHpg %2d sf %2d STOR ", n->src_prn+1, page+1, sf_id);
			for (int i=0; i<8; i++) {
				u32 word = (n->frame_words[sf_id-1][i] >> 1);
				printf(" %08lx %2lx", word & 0x3fffffc0, word & 0x3f);
			}
			printf("\n");
#endif			
			// store
			memcpy(&sat_datastore[slot].sf123_words[sf_id-1][0], &n->frame_words[sf_id-1][0], 8*sizeof(u32));
			// flag storage is used in page 0
			sat_datastore[slot].pagestatus[0] |= (1 << (sf_id-1));
			// TODO: ephemerisbuilder_add(n->src_prn, sf_id, sat_datastore[slot].sf123_words[sf_id-1]);
		} else {
			/* sf_id 1..3 ephemeris data, repeated every 30 secs */
			/* compare with existing ephemeris data received */
			if (memcmp(&n->frame_words[sf_id-1][0], &sat_datastore[slot].sf123_words[sf_id-1][0], 8*sizeof(u32)) == 0) {
#if 0	
				printf("PRN %2d EPHpg %2d sf %2d same ", n->src_prn+1, page+1, sf_id);
				for (int i=0; i<8; i++) {
					u32 word = (n->frame_words[sf_id-1][i] >> 1);
					printf(" %08lx %2lx", word & 0x3fffffc0, word & 0x3f);
				}
				printf("\n");
#endif				
			} else {
				printf("PRN %2d EPHpg %2d sf %2d CHGD \n", n->src_prn+1, page+1, sf_id);
				printf("FR ");
				for (int i=0; i<8; i++) {
					u32 word = sat_datastore[slot].sf123_words[sf_id-1][i] >> 1;
					printf(" %08lx %2lx", word & 0x3fffffc0, word & 0x3f);
				}
				printf("\nTO ");
				for (int i=0; i<8; i++) {
					u32 word = (n->frame_words[sf_id-1][i] >> 1);
					printf(" %08lx %2lx", word & 0x3fffffc0, word & 0x3f);
				}
				printf("\n");
				// store new ephermeris
				memcpy(&sat_datastore[slot].sf123_words[sf_id-1][0], &n->frame_words[sf_id-1][0], 8*sizeof(u32));
				// TODO: ephemerisbuilder_change
			}
		}
	}
}

void pagestore_error(nav_msg_t *n, u8 page, u8 sf_id, u8 word)
{
	u8 slot = 0xff;
	printf("parity PRN %d page %2d sf_id %d word %d\n", n->src_prn+1, page + 1, sf_id, word);
	for (int i=0; i<MAXSTORE; i++) {
		if (sat_datastore[i].prn == n->src_prn) {
			slot = i; 
			break;
		}
	}
	if (slot == 0xff)
		return;
	/* log bit/parity error */
	sat_datastore[slot].errcnt++;
}

s8 process_subframe(nav_msg_t *n, ephemeris_t *e) {
  // Check parity and parse out the ephemeris from the most recently received subframe

  // First things first - check the parity, and invert bits if necessary.
  // process the data, skipping the first word, TLM, and starting with HOW

  // printf("  %d  ", (n->subframe_start_index > 0));

  /* TODO: Check if inverted has changed and detect half cycle slip. */
  if (n->inverted != (n->subframe_start_index < 0))
    printf("Nav phase flip\n");
  n->inverted = (n->subframe_start_index < 0);

  if (!e) {
    printf(" process_subframe: CALLED WITH e = NULL!\n");
    n->subframe_start_index = 0;  // Mark the subframe as processed
    n->next_subframe_id = 1;      // Make sure we start again next time
    return -1;
  }
  u32 sf_word2 = extract_word(n, 28, 32, 0);
  if (nav_parity(&sf_word2)) {
      printf("SUBFRAME PARITY ERROR (word 2)\n");
      n->subframe_start_index = 0;  // Mark the subframe as processed
      n->next_subframe_id = 1;      // Make sure we start again next time
      return -2;
  }

  u8 sf_id = sf_word2 >> 8 & 0x07;    // Which of 5 possible subframes is it?
  u8 page = ((n->tow -6)/ 30) % 25;

  /*printf("sf_id = %d, nsf = %d\n",sf_id, n->next_subframe_id);*/

  if (sf_id <= 3 && sf_id == n->next_subframe_id) {  // Is it the one that we want next?

    for (int w = 0; w < 8; w++) {   // For words 3..10
      n->frame_words[sf_id-1][w] = extract_word(n, 30*(w+2) - 2, 32, 0);    // Get the bits
      // MSBs are D29* and D30*.  LSBs are D1...D30
      if (nav_parity(&n->frame_words[sf_id-1][w])) {  // Check parity and invert bits if D30*
        //printf("SUBFRAME PARITY ERROR (word %d)\n", w+3);
		pagestore_error(n, page, sf_id, w+3);
		
        n->next_subframe_id = 1;      // Make sure we start again next time
        n->subframe_start_index = 0;  // Mark the subframe as processed
        return -3;
      }
    }
    n->subframe_start_index = 0;  // Mark the subframe as processed
    n->next_subframe_id++;

	// TODO: onl store
	pagestore_process(n, page, sf_id);

    if (sf_id == 3) {
      // Got all of subframes 1 to 3
      n->next_subframe_id = 1;      // Make sure we start again next time

      // Now let's actually go through the parameters...

      // These unions facilitate signed/unsigned conversion and sign extension
      // TODO: Use types from common.h here
      union {
        char s8;
        unsigned char u8;
      } onebyte;

      union
      {
        short s16;
        unsigned short u16;
      } twobyte;

      union
      {
        int s32;
        unsigned u32;
      } fourbyte;

      // Subframe 1: SV health, T_GD, t_oc, a_f2, a_f1, a_f0

      e->toe.wn = (n->frame_words[0][3-3] >> (30-10) & 0x3FF);       // GPS week number (mod 1024): Word 3, bits 20-30
	  /* store for processing for sf4/5 */
	  n->weekno = e->toe.wn;
	  
      e->toe.wn += GPS_WEEK_CYCLE*1024;
      e->toc.wn = e->toe.wn;

      e->healthy = !(n->frame_words[0][3-3] >> (30-17) & 1);     // Health flag: Word 3, bit 17
      if (!e->healthy)
        printf("UNHEALTHY\n");

      onebyte.u8 = n->frame_words[0][7-3] >> (30-24) & 0xFF;  // t_gd: Word 7, bits 17-24
      e->tgd = onebyte.s8 * pow(2,-31);

      e->toc.tow = (n->frame_words[0][8-3] >> (30-24) & 0xFFFF) * 16;   // t_oc: Word 8, bits 8-24

      onebyte.u8 = n->frame_words[0][9-3] >> (30-8) & 0xFF;         // a_f2: Word 9, bits 1-8
      e->af2 = onebyte.s8 * pow(2,-55);

      twobyte.u16 = n->frame_words[0][9-3] >> (30-24) & 0xFFFF;     // a_f1: Word 9, bits 9-24
      e->af1 = twobyte.s16 * pow(2,-43);

      fourbyte.u32 = n->frame_words[0][10-3] >> (30-22) & 0x3FFFFF; // a_f0: Word 10, bits 1-22
      fourbyte.u32 <<= 10; // Shift to the left for sign extension
      fourbyte.s32 >>= 10; // Carry the sign bit back down and reduce to signed 22 bit value
      e->af0 = fourbyte.s32 * pow(2,-31);


      // Subframe 2: crs, dn, m0, cuc, ecc, cus, sqrta, toe

      twobyte.u16 = n->frame_words[1][3-3] >> (30-24) & 0xFFFF;     // crs: Word 3, bits 9-24
      e->crs = twobyte.s16 * pow(2,-5);

      twobyte.u16 = n->frame_words[1][4-3] >> (30-16) & 0xFFFF;     // dn: Word 4, bits 1-16
      e->dn = twobyte.s16 * pow(2,-43) * GPS_PI;

      fourbyte.u32 = ((n->frame_words[1][4-3] >> (30-24) & 0xFF) << 24) // m0: Word 4, bits 17-24
                  | (n->frame_words[1][5-3] >> (30-24) & 0xFFFFFF);     // and word 5, bits 1-24
      e->m0 = fourbyte.s32 * pow(2,-31) * GPS_PI;

      twobyte.u16 = n->frame_words[1][6-3] >> (30-16) & 0xFFFF;    // cuc: Word 6, bits 1-16
      e->cuc = twobyte.s16 * pow(2,-29);

      fourbyte.u32 = ((n->frame_words[1][6-3] >> (30-24) & 0xFF) << 24) // ecc: Word 6, bits 17-24
                  | (n->frame_words[1][7-3] >> (30-24) & 0xFFFFFF);     // and word 7, bits 1-24
      e->ecc = fourbyte.u32 * pow(2,-33);


      twobyte.u16 = n->frame_words[1][8-3] >> (30-16) & 0xFFFF;   // cus: Word 8, bits 1-16
      e->cus = twobyte.s16 * pow(2,-29);


      fourbyte.u32 = ((n->frame_words[1][8-3] >> (30-24) & 0xFF) << 24) // sqrta: Word 8, bits 17-24
                  | (n->frame_words[1][9-3] >> (30-24) & 0xFFFFFF);     // and word 9, bits 1-24
      e->sqrta = fourbyte.u32 * pow(2,-19);

      e->toe.tow = (n->frame_words[1][10-3] >> (30-16) & 0xFFFF) * 16;   // t_oe: Word 10, bits 1-16


      // Subframe 3: cic, omega0, cis, inc, crc, w, omegadot, inc_dot

      twobyte.u16 = n->frame_words[2][3-3] >> (30-16) & 0xFFFF;   // cic: Word 3, bits 1-16
      e->cic = twobyte.s16 * pow(2,-29);

      fourbyte.u32 = ((n->frame_words[2][3-3] >> (30-24) & 0xFF) << 24) // omega0: Word 3, bits 17-24
                  | (n->frame_words[2][4-3] >> (30-24) & 0xFFFFFF);     // and word 4, bits 1-24
      e->omega0 = fourbyte.s32 * pow(2,-31) * GPS_PI;

      twobyte.u16 = n->frame_words[2][5-3] >> (30-16) & 0xFFFF; // cis: Word 5, bits 1-16
      e->cis = twobyte.s16 * pow(2,-29);

      fourbyte.u32 = ((n->frame_words[2][5-3] >> (30-24) & 0xFF) << 24) // inc (i0): Word 5, bits 17-24
                  | (n->frame_words[2][6-3] >> (30-24) & 0xFFFFFF);     // and word 6, bits 1-24
      e->inc = fourbyte.s32 * pow(2,-31) * GPS_PI;

      twobyte.u16 = n->frame_words[2][7-3] >> (30-16) & 0xFFFF; // crc: Word 7, bits 1-16
      e->crc = twobyte.s16 * pow(2,-5);

      fourbyte.u32 = ((n->frame_words[2][7-3] >> (30-24) & 0xFF) << 24) // w (omega): Word 7, bits 17-24
                  | (n->frame_words[2][8-3] >> (30-24) & 0xFFFFFF);     // and word 8, bits 1-24
      e->w = fourbyte.s32 * pow(2,-31) * GPS_PI;

      fourbyte.u32 = n->frame_words[2][9-3] >> (30-24) & 0xFFFFFF;     // Omega_dot: Word 9, bits 1-24
      fourbyte.u32 <<= 8; // shift left for sign extension
      fourbyte.s32 >>= 8; // sign-extend it
      e->omegadot = fourbyte.s32 * pow(2,-43) * GPS_PI;


      twobyte.u16 = n->frame_words[2][10-3] >> (30-22) & 0x3FFF;  // inc_dot (IDOT): Word 10, bits 9-22
      twobyte.u16 <<= 2;
      twobyte.s16 >>= 2;  // sign-extend
      e->inc_dot = twobyte.s16 * pow(2,-43) * GPS_PI;


      e->valid = 1;

      /*printf("Health %d\n", e->healthy);*/
      /*printf("TGD %16g\n", e->tgd);*/
      /*printf("TOC %16u\n", (unsigned int)e->toc);*/
      /*printf("af2 %16g\n", e->af2);*/
      /*printf("af1 %16g\n", e->af1);*/
      /*printf("af0 %16g\n", e->af0);*/
      /*printf("CRS %16g\n", e->crs);*/
      /*printf("DN %16g\n", e->dn);*/
      /*printf("M0 %16g\n", e->m0);*/
      /*printf("CUC %16g\n", e->cuc);*/
      /*printf("Ecc %16g\n", e->ecc);*/
      /*printf("CUS %16g\n", e->cus);*/
      /*printf("SQRT A %16g\n", e->sqrta);*/
      /*printf("TOE %16u\n", (unsigned int)e->toe);*/
      /*printf("CIC %16g\n", e->cic);*/
      /*printf("omega0 %16g\n", e->omega0);*/
      /*printf("CIS %16g\n", e->cis);*/
      /*printf("Inc %16g\n", e->inc);*/
      /*printf("CRC %16g\n", e->crc);*/
      /*printf("W %16g\n", e->w);*/
      /*printf("omegadot %16g\n", e->omegadot);*/
      /*printf("inc_dot %16g\n", e->inc_dot);*/
      return 1;

    }
  } else {  // didn't get the subframe that we want next
		if (sf_id >= 4) {
			for (int w = 0; w < 8; w++) {   // For words 3..10
			  n->frame_words[sf_id-1][w] = extract_word(n, 30*(w+2) - 2, 32, 0);    // Get the bits
			  // MSBs are D29* and D30*.  LSBs are D1...D30
			  if (nav_parity(&n->frame_words[sf_id-1][w])) {  // Check parity and invert bits if D30*
				//printf("SFDATA PARITY ERROR (word %d)\n", w+3);
				pagestore_error(n, page, sf_id, w+3);
				n->next_subframe_id = 1;      // Make sure we start again next time
				n->subframe_start_index = 0;  // Mark the subframe as processed
				return -3;
			  }
			}
			pagestore_process(n, page, sf_id);

			/* TODO: build broadcasr almanac and compare it */
        }
		
		n->next_subframe_id = 1;      // Make sure we start again next time
		n->subframe_start_index = 0;  // Mark the subframe as processed
  }

  return 0;


}

