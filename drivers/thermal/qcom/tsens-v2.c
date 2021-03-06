// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2015, The Linux Foundation. All rights reserved.
 * Copyright (c) 2018, Linaro Limited
 */

#include <linux/regmap.h>
#include "tsens.h"

#define STATUS_OFFSET		0xa0
#define LAST_TEMP_MASK		0xfff
#define STATUS_VALID_BIT	BIT(21)
#define CODE_SIGN_BIT		BIT(11)

static int get_temp_tsens_v2(struct tsens_device *tmdev, int id, int *temp)
{
	struct tsens_sensor *s = &tmdev->sensor[id];
	u32 code;
	unsigned int sensor_addr;
	int last_temp = 0, last_temp2 = 0, last_temp3 = 0, ret;

	sensor_addr = STATUS_OFFSET + s->hw_id * 4;
	ret = regmap_read(tmdev->map, sensor_addr, &code);
	if (ret)
		return ret;
	last_temp = code & LAST_TEMP_MASK;
	if (code & STATUS_VALID_BIT)
		goto done;

	/* Try a second time */
	ret = regmap_read(tmdev->map, sensor_addr, &code);
	if (ret)
		return ret;
	if (code & STATUS_VALID_BIT) {
		last_temp = code & LAST_TEMP_MASK;
		goto done;
	} else {
		last_temp2 = code & LAST_TEMP_MASK;
	}

	/* Try a third/last time */
	ret = regmap_read(tmdev->map, sensor_addr, &code);
	if (ret)
		return ret;
	if (code & STATUS_VALID_BIT) {
		last_temp = code & LAST_TEMP_MASK;
		goto done;
	} else {
		last_temp3 = code & LAST_TEMP_MASK;
	}

	if (last_temp == last_temp2)
		last_temp = last_temp2;
	else if (last_temp2 == last_temp3)
		last_temp = last_temp3;
done:
	/* Code sign bit is the sign extension for a negative value */
	if (last_temp & CODE_SIGN_BIT)
		last_temp |= ~CODE_SIGN_BIT;

	/* Temperatures are in deciCelicius */
	*temp = last_temp * 100;

	return 0;
}

static const struct tsens_ops ops_generic_v2 = {
	.init		= init_common,
	.get_temp	= get_temp_tsens_v2,
};

const struct tsens_data data_tsens_v2 = {
	.ops            = &ops_generic_v2,
};

/* Kept around for backward compatibility with old msm8996.dtsi */
const struct tsens_data data_8996 = {
	.num_sensors	= 13,
	.ops		= &ops_generic_v2,
};
