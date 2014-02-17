/*
 * Copyright (C) 2010 Swift Navigation Inc.
 * Contact: Fergus Noble <fergus@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef LIBSWIFTNAV_ALMANAC_H
#define LIBSWIFTNAV_ALMANAC_H

#include "common.h"
#include "almanac.h"

/** \addtogroup almanac
 * \{ */

/** Structure containing the GPS almanac for one satellite. */
typedef struct {
  double ecc;   /**< Eccentricity in radians. */
  double toa;   /**< Time of Applicability in seconds since Sunday. */
  double inc;   /**< Inclination in radians. */
  double rora;  /**< Rate of Right Ascension in radians/sec. */
  double a;     /**< Semi-major axis in meters. */
  double raaw;  /**< Right Ascension at Week in radians. */
  double argp;  /**< Argument of Perigee in radians. */
  double ma;    /**< Mean Anomaly at Time of Applicability in radians. */
  double af0;   /**< 0-order clock correction in seconds. */
  double af1;   /**< 1-order clock correction in seconds/second. */
  u16 week;     /**< GPS week number, modulo 1024. */
  u8 prn;       /**< PRN number of the satellite. */
  u8 healthy;   /**< Satellite health status. */
  u8 valid;     /**< Almanac is valid. */
} almanac_t;

/** \} */

/* broadcast almanac structures */

typedef struct {
	/* special message */
	u8	svid;
	u8	message[23];
} page17_t;

typedef struct {
	u8	svid;
	/* ionospheric data and UTC, leap second */
	u8 a[4];
	u8 b[4];
	u32 a1;
	u32 a0;
	u8 t_ot;
	u8 wn_t;
	u8 dt_ls;
	u8 wn_lsf;
	u8 dn;
	u8 d_t_lsf;
	u16 res;
} page18_t;

typedef struct {
	u8 		prn;
		
	u32 	toa;
	u8		wna_raw;
	u8  	aspoof[32];
	u8		svhealth[32];
	
	/* other data */
	u8 		svid_4;
	u8 		svid_5;
	u8 		res_6b;
	u16 	res_16b;
} page25_t;

#define MAXSTORE 32

/* prndatastore for navdata checks */
typedef struct {
	u8  prn;
	u8	pagestatus[25]; /* b0=sf1 ... b4=sf5 stored in that page */
	/* ephemeris data of this svid. repeated every 30 secs */
	u32	sf123_words[3][8];    /* 3 kb total for 32 sats */
	/* broadcast alamanac data of this svid, repeated every 30 secs */
	u32	sf45_words[25][2][8]; /* 50 kb total for 32 sats */
	
	/* special message */
	page17_t	p17;
	/* utc & ionospheric data */
	page18_t	p18;
	/* integer-level decoded page 25 sf4/5 data */
	page25_t	p25;

	/* maintain weekno and toa for almanac */
	u8	toa_completed;	/* set once all pages retrieved, reset if toa changed (!= toa_mark) */
	u32	toa_mark;
	u32 weekno;		/* weekno expanded to 10 bits using sf1 weekno */

	// error stats
	u32	errcnt;
} sat_datastore_t;

void calc_sat_state_almanac(almanac_t* alm, double t, s16 week,
                            double pos[3], double vel[3]);
void calc_sat_az_el_almanac(almanac_t* alm, double t, s16 week,
                            double ref[3], double* az, double* el);
double calc_sat_doppler_almanac(almanac_t* alm, double t, s16 week,
                                double ref[3]);

#endif /* LIBSWIFTNAV_ALMANAC_H */
