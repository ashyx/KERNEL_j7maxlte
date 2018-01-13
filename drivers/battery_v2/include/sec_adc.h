/*
 * sec_adc.h
 * Samsung Mobile Charger Header
 *
 * Copyright (C) 2017 Samsung Electronics, Inc.
 *
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef __SEC_ADC_H
#define __SEC_ADC_H __FILE__

#include <linux/iio/consumer.h>
#include "sec_battery.h"
#include "sec_charging_common.h"

#define VENDOR_UNKNOWN 0
#define VENDOR_LSI 1
#define VENDOR_QCOM 2

enum adc_aux_channels_mtk {
	MT6757_AUXIN0 = 0,
	MT6757_AUXIN1,
	MT6757_AUXIN2,
	MT6757_AUXIN3,
	MT6757_AUXIN4,
};

#endif /* __SEC_ADC_H */

