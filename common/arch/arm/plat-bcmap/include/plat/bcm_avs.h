/*******************************************************************************
* Copyright 2010 Broadcom Corporation.  All rights reserved.
*
* 	@file	arch/arm/plat-bcmap/include/plat/bcm_avs.h
*
* Unless you and Broadcom execute a separate written software license agreement
* governing use of this software, this software is licensed to you under the
* terms of the GNU General Public License version 2, available at
* http://www.gnu.org/copyleft/gpl.html (the "GPL").
*
* Notwithstanding the above, under no circumstances may you combine this
* software in any way with any other Broadcom software provided under a license
* other than the GPL, without Broadcom's express prior written consent.
*******************************************************************************/

/*
*
*****************************************************************************
*
*  bcm_lcdc.h
*
*  PURPOSE:
*
*     This file contains platform data structures for Broadcom AVS driver
*
*  NOTES:
*
*****************************************************************************/

#ifndef __PLAT_BCM_AVS_H
#define __PLAT_BCM_AVS_H

enum
{
	SILICON_TYPE_SLOW = 0,
	SILICON_TYPE_TYPICAL,
	SILICON_TYPE_FAST,
	SILICON_TYPE_UNKNOWN
};

struct silicon_type_info
{
	/* LPM voltage. Pass -1 if no update needed */
	int lpm_voltage;
	/* NM voltage. Pass -1 if no update needed */
	int nm_voltage;
	/* NM2 makwuz mode voltage. Pass -1 if no update needed */
	int nm2_makwuz_voltage; //add eve oc
	/* NM2 ngenos mode voltage. Pass -1 if no update needed */
	int nm2_ngenos_voltage; //add eve oc
	/* NM2 turbo mode voltage. Pass -1 if no update needed */
	int nm2_turbo_voltage;
	/* NM2 kenceng mode voltage. Pass -1 if no update needed */
	int nm2_kenceng_voltage; //add eve oc
	/* NM2 kebut mode voltage. Pass -1 if no update needed */
	int nm2_kebut_voltage; //add eve oc
	/* NM2 normal mode voltage. Pass -1 if no update needed */
	int nm2_normal_voltage; //add eve oc
	/* NM2 lelet mode voltage. Pass -1 if no update needed */
	int nm2_lelet_voltage; //add eve oc
	/* NM2 lemot mode voltage. Pass -1 if no update needed */
	int nm2_lemot_voltage; //add eve oc
};

struct bcm_avs_platform_data_t
{
	/* Name of the regulator to control core LPM voltage
	 * Pass NULL if not supported/need not update
	 */
	char *core_lpm_regl;

	/* Name of the regulator to control core normal mode voltage
	 * Pass NULL if not supported/need not update
	 */
	char *core_nml_regl;

	/* Name of the regulator to control core lemot mode voltage
	 * Pass NULL if not supported/need not update
	 */
	char *core_lemot_regl; //add eve oc

	/* Name of the regulator to control core lelet mode voltage
	 * Pass NULL if not supported/need not update
	 */
	char *core_lelet_regl; //add eve oc

	/* Name of the regulator to control core normal mode voltage
	 * Pass NULL if not supported/need not update
	 */
	char *core_normal_regl; //add eve oc

	/* Name of the regulator to control core kebut mode voltage
	 * Pass NULL if not supported/need not update
	 */
	char *core_kebut_regl; //add eve oc

	/* Name of the regulator to control core kenceng mode voltage
	 * Pass NULL if not supported/need not update
	 */
	char *core_kenceng_regl; //add eve oc

	/* Name of the regulator to control core turbo mode voltage
	 * Pass NULL if not supported/need not update
	 */
	char *core_turbo_regl; //add eve oc

	/* Name of the regulator to control core ngenos mode voltage
	 * Pass NULL if not supported/need not update
	 */
	char *core_ngenos_regl; //add eve oc

	/* Name of the regulator to control core makwuz mode voltage
	 * Pass NULL if not supported/need not update
	 */
	char *core_makwuz_regl; //add eve oc

	/* OTP params to idenltify silicon type.
	 * Assumption : total number of bits <=32
	 */
	int otp_bit_lsb;
	int otp_bit_msb;

	/* OTP value ranges:
	 * > fast_silicon_thold => fast silicon
	 * < slow_silicon_thold => slow silicon
	 * (> slow_silicon_thold && <= fast_silicon_thold) ==> typical silicon
	 */
	u32 fast_silicon_thold;
	u32 slow_silicon_thold;

	/* Pass NULL if not supported */
	struct silicon_type_info *fast_silicon;
	/* Pass NULL if not supported */
	struct silicon_type_info *typical_silicon;
	/* Pass NULL if not supported */
	struct silicon_type_info *slow_silicon;

	/* Silicon type notification callback - can be NULL */
	void (*notify_silicon_type) (int type);
};

#endif	/* __PLAT_BCM_AVS_H */
