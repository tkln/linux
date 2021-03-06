/*
 * Copyright (C) 2017 Free Electrons
 * Copyright (C) 2017 NextThing Co
 *
 * Author: Boris Brezillon <boris.brezillon@free-electrons.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/mtd/rawnand.h>

/*
 * Special Micron status bit 3 indicates that the block has been
 * corrected by on-die ECC and should be rewritten.
 */
#define NAND_ECC_STATUS_WRITE_RECOMMENDED	BIT(3)

/*
 * On chips with 8-bit ECC and additional bit can be used to distinguish
 * cases where a errors were corrected without needing a rewrite
 *
 * Bit 4 Bit 3 Bit 0 Description
 * ----- ----- ----- -----------
 * 0     0     0     No Errors
 * 0     0     1     Multiple uncorrected errors
 * 0     1     0     4 - 6 errors corrected, recommend rewrite
 * 0     1     1     Reserved
 * 1     0     0     1 - 3 errors corrected
 * 1     0     1     Reserved
 * 1     1     0     7 - 8 errors corrected, recommend rewrite
 */
#define NAND_ECC_STATUS_MASK		(BIT(4) | BIT(3) | BIT(0))
#define NAND_ECC_STATUS_UNCORRECTABLE	BIT(0)
#define NAND_ECC_STATUS_4_6_CORRECTED	BIT(3)
#define NAND_ECC_STATUS_1_3_CORRECTED	BIT(4)
#define NAND_ECC_STATUS_7_8_CORRECTED	(BIT(4) | BIT(3))

struct nand_onfi_vendor_micron {
	u8 two_plane_read;
	u8 read_cache;
	u8 read_unique_id;
	u8 dq_imped;
	u8 dq_imped_num_settings;
	u8 dq_imped_feat_addr;
	u8 rb_pulldown_strength;
	u8 rb_pulldown_strength_feat_addr;
	u8 rb_pulldown_strength_num_settings;
	u8 otp_mode;
	u8 otp_page_start;
	u8 otp_data_prot_addr;
	u8 otp_num_pages;
	u8 otp_feat_addr;
	u8 read_retry_options;
	u8 reserved[72];
	u8 param_revision;
} __packed;

static int micron_nand_setup_read_retry(struct mtd_info *mtd, int retry_mode)
{
	struct nand_chip *chip = mtd_to_nand(mtd);
	u8 feature[ONFI_SUBFEATURE_PARAM_LEN] = {retry_mode};

	return nand_set_features(chip, ONFI_FEATURE_ADDR_READ_RETRY, feature);
}

/*
 * Configure chip properties from Micron vendor-specific ONFI table
 */
static int micron_nand_onfi_init(struct nand_chip *chip)
{
	struct nand_parameters *p = &chip->parameters;
	struct nand_onfi_vendor_micron *micron = (void *)p->onfi.vendor;

	if (chip->parameters.onfi.version && p->onfi.vendor_revision) {
		chip->read_retries = micron->read_retry_options;
		chip->setup_read_retry = micron_nand_setup_read_retry;
	}

	if (p->supports_set_get_features) {
		set_bit(ONFI_FEATURE_ADDR_READ_RETRY, p->set_feature_list);
		set_bit(ONFI_FEATURE_ON_DIE_ECC, p->set_feature_list);
		set_bit(ONFI_FEATURE_ADDR_READ_RETRY, p->get_feature_list);
		set_bit(ONFI_FEATURE_ON_DIE_ECC, p->get_feature_list);
	}

	return 0;
}

static int micron_nand_on_die_ooblayout_ecc(struct mtd_info *mtd, int section,
					    struct mtd_oob_region *oobregion)
{
	if (section >= 4)
		return -ERANGE;

	oobregion->offset = (section * 16) + 8;
	oobregion->length = 8;

	return 0;
}

static int micron_nand_on_die_ooblayout_free(struct mtd_info *mtd, int section,
					     struct mtd_oob_region *oobregion)
{
	if (section >= 4)
		return -ERANGE;

	oobregion->offset = (section * 16) + 2;
	oobregion->length = 6;

	return 0;
}

static const struct mtd_ooblayout_ops micron_nand_on_die_ooblayout_ops = {
	.ecc = micron_nand_on_die_ooblayout_ecc,
	.free = micron_nand_on_die_ooblayout_free,
};

static int micron_nand_on_die_ecc_setup(struct nand_chip *chip, bool enable)
{
	u8 feature[ONFI_SUBFEATURE_PARAM_LEN] = { 0, };

	if (enable)
		feature[0] |= ONFI_FEATURE_ON_DIE_ECC_EN;

	return nand_set_features(chip, ONFI_FEATURE_ON_DIE_ECC, feature);
}


static int micron_nand_on_die_ecc_status_4(struct mtd_info *mtd,
					   struct nand_chip *chip, u8 status)
{
	/*
	 * The internal ECC doesn't tell us the number of bitflips
	 * that have been corrected, but tells us if it recommends to
	 * rewrite the block. If it's the case, then we pretend we had
	 * a number of bitflips equal to the ECC strength, which will
	 * hint the NAND core to rewrite the block.
	 */
	if (status & NAND_STATUS_FAIL) {
		mtd->ecc_stats.failed++;
	} else if (status & NAND_ECC_STATUS_WRITE_RECOMMENDED) {
		mtd->ecc_stats.corrected += chip->ecc.strength;
		return chip->ecc.strength;
	}

	return 0;
}

static int micron_nand_on_die_ecc_status_8(struct mtd_info *mtd,
					   struct nand_chip *chip, u8 status)
{
	/*
	 * With 8/512 we have more information but still don't know precisely
	 * how many bit-flips were seen.
	 */
	switch (status & NAND_ECC_STATUS_MASK) {
	case NAND_ECC_STATUS_UNCORRECTABLE:
		mtd->ecc_stats.failed++;
		return 0;
	case NAND_ECC_STATUS_1_3_CORRECTED:
		mtd->ecc_stats.corrected += 3;
		return 3;
	case NAND_ECC_STATUS_4_6_CORRECTED:
		mtd->ecc_stats.corrected += 6;
		/* rewrite recommended */
		return 6;
	case NAND_ECC_STATUS_7_8_CORRECTED:
		mtd->ecc_stats.corrected += 8;
		/* rewrite recommended */
		return 8;
	default:
		return 0;
	}
}

static int
micron_nand_read_page_on_die_ecc(struct mtd_info *mtd, struct nand_chip *chip,
				 uint8_t *buf, int oob_required,
				 int page)
{
	u8 status;
	int ret, max_bitflips = 0;

	ret = micron_nand_on_die_ecc_setup(chip, true);
	if (ret)
		return ret;

	ret = nand_read_page_op(chip, page, 0, NULL, 0);
	if (ret)
		goto out;

	ret = nand_status_op(chip, &status);
	if (ret)
		goto out;

	ret = nand_exit_status_op(chip);
	if (ret)
		goto out;

	if (chip->ecc.strength == 4)
		max_bitflips = micron_nand_on_die_ecc_status_4(mtd, chip, status);
	else
		max_bitflips = micron_nand_on_die_ecc_status_8(mtd, chip, status);

	ret = nand_read_data_op(chip, buf, mtd->writesize, false);
	if (!ret && oob_required)
		ret = nand_read_data_op(chip, chip->oob_poi, mtd->oobsize,
					false);

out:
	micron_nand_on_die_ecc_setup(chip, false);

	return ret ? ret : max_bitflips;
}

static int
micron_nand_write_page_on_die_ecc(struct mtd_info *mtd, struct nand_chip *chip,
				  const uint8_t *buf, int oob_required,
				  int page)
{
	int ret;

	ret = micron_nand_on_die_ecc_setup(chip, true);
	if (ret)
		return ret;

	ret = nand_write_page_raw(mtd, chip, buf, oob_required, page);
	micron_nand_on_die_ecc_setup(chip, false);

	return ret;
}

enum {
	/* The NAND flash doesn't support on-die ECC */
	MICRON_ON_DIE_UNSUPPORTED,

	/*
	 * The NAND flash supports on-die ECC and it can be
	 * enabled/disabled by a set features command.
	 */
	MICRON_ON_DIE_SUPPORTED,

	/*
	 * The NAND flash supports on-die ECC, and it cannot be
	 * disabled.
	 */
	MICRON_ON_DIE_MANDATORY,
};

/*
 * These parts are known to have on-die ECC forceably enabled
 */
static u8 micron_on_die_ecc[] = {
	0xd1, /* MT29F1G08ABAFA */
	0xa1, /* MT29F1G08ABBFA */
};

/*
 * Try to detect if the NAND support on-die ECC. To do this, we enable
 * the feature, and read back if it has been enabled as expected. We
 * also check if it can be disabled, because some Micron NANDs do not
 * allow disabling the on-die ECC and we don't support such NANDs for
 * now.
 *
 * This function also has the side effect of disabling on-die ECC if
 * it had been left enabled by the firmware/bootloader.
 */
static int micron_supports_on_die_ecc(struct nand_chip *chip)
{
	u8 feature[ONFI_SUBFEATURE_PARAM_LEN] = { 0, };
	int ret;
	int i;

	for (i = 0; i < ARRAY_SIZE(micron_on_die_ecc); i++)
		if (chip->id.data[1] == micron_on_die_ecc[i])
			return MICRON_ON_DIE_MANDATORY;

	if (!chip->parameters.onfi.version)
		return MICRON_ON_DIE_UNSUPPORTED;

	if (chip->bits_per_cell != 1)
		return MICRON_ON_DIE_UNSUPPORTED;

	ret = micron_nand_on_die_ecc_setup(chip, true);
	if (ret)
		return MICRON_ON_DIE_UNSUPPORTED;

	ret = nand_get_features(chip, ONFI_FEATURE_ON_DIE_ECC, feature);
	if (ret < 0)
		return ret;

	if ((feature[0] & ONFI_FEATURE_ON_DIE_ECC_EN) == 0)
		return MICRON_ON_DIE_UNSUPPORTED;

	ret = micron_nand_on_die_ecc_setup(chip, false);
	if (ret)
		return MICRON_ON_DIE_UNSUPPORTED;

	ret = nand_get_features(chip, ONFI_FEATURE_ON_DIE_ECC, feature);
	if (ret < 0)
		return ret;

	if (feature[0] & ONFI_FEATURE_ON_DIE_ECC_EN)
		return MICRON_ON_DIE_MANDATORY;

	/*
	 * We only support on-die ECC of 4/512 or 8/512
	 */
	if  (chip->ecc_strength_ds != 4 && chip->ecc_strength_ds != 8)
		return MICRON_ON_DIE_UNSUPPORTED;

	return MICRON_ON_DIE_SUPPORTED;
}

static int micron_nand_init(struct nand_chip *chip)
{
	struct mtd_info *mtd = nand_to_mtd(chip);
	int ondie;
	int ret;

	ret = micron_nand_onfi_init(chip);
	if (ret)
		return ret;

	if (mtd->writesize == 2048)
		chip->bbt_options |= NAND_BBT_SCAN2NDPAGE;

	ondie = micron_supports_on_die_ecc(chip);

	if (ondie == MICRON_ON_DIE_MANDATORY &&
	    chip->ecc.mode != NAND_ECC_ON_DIE) {
		pr_err("On-die ECC forcefully enabled, not supported\n");
		return -EINVAL;
	}

	if (chip->ecc.mode == NAND_ECC_ON_DIE) {
		if (ondie == MICRON_ON_DIE_UNSUPPORTED) {
			pr_err("On-die ECC selected but not supported\n");
			return -EINVAL;
		}

		chip->ecc.bytes = chip->ecc_strength_ds * 2;
		chip->ecc.size = 512;
		chip->ecc.strength = chip->ecc_strength_ds;
		chip->ecc.algo = NAND_ECC_BCH;
		chip->ecc.read_page = micron_nand_read_page_on_die_ecc;
		chip->ecc.write_page = micron_nand_write_page_on_die_ecc;
		chip->ecc.read_page_raw = nand_read_page_raw;
		chip->ecc.write_page_raw = nand_write_page_raw;

		mtd_set_ooblayout(mtd, &micron_nand_on_die_ooblayout_ops);
	}

	return 0;
}

static void micron_fixup_onfi_param_page(struct nand_chip *chip,
					 struct nand_onfi_params *p)
{
	/*
	 * MT29F1G08ABAFAWP-ITE:F and possibly others report 00 00 for the
	 * revision number field of the ONFI parameter page. Assume ONFI
	 * version 1.0 if the revision number is 00 00.
	 */
	if (le16_to_cpu(p->revision) == 0)
		p->revision = cpu_to_le16(ONFI_VERSION_1_0);
}

const struct nand_manufacturer_ops micron_nand_manuf_ops = {
	.init = micron_nand_init,
	.fixup_onfi_param_page = micron_fixup_onfi_param_page,
};
