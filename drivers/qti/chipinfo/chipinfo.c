/*
 * Copyright (c) 2026, Qualcomm Technologies, Inc. and/or its subsidiaries.
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <drivers/qti/chipinfo/chipinfo.h>
#include <drivers/qti/platforminfo/platforminfodefs.h>
#include <drivers/qti/smem/smem.h>

#include "chipinfo_internal.h"

#define SMEM_HW_SW_BUILD_ID 0x89

static struct chipinfo_ctxt chipinfo_ctxt;

uint32_t chipinfo_get_chip_version(void)
{
	if (!chipinfo_ctxt.initialized) {
		return CHIPINFO_VERSION_UNKNOWN;
	}

	return chipinfo_ctxt.version;
}

enum chipinfo_id chipinfo_get_chip_id(void)
{
	if (!chipinfo_ctxt.initialized) {
		return CHIPINFO_ID_UNKNOWN;
	}

	return chipinfo_ctxt.chipinfo_id;
}

enum chipinfo_family chipinfo_get_chip_family(void)
{
	if (!chipinfo_ctxt.initialized) {
		return CHIPINFO_FAMILY_UNKNOWN;
	}

	return chipinfo_ctxt.family_id;
}

bool chipinfo_is_part_disabled(enum chipinfo_part part, uint32_t part_idx)
{
	uint32_t i;

	if (!chipinfo_ctxt.initialized) {
		return false;
	}

	if ((part == CHIPINFO_PART_UNKNOWN) ||
	    ((uint32_t)part >= CHIPINFO_NUM_PARTS)) {
		return false;
	}

	if (part_idx == 0U) {
		/* Flat array reflects the overall part fuse state reported by XBL. */
		return chipinfo_ctxt.disabled_features[part] != 0U;
	}

	/* Search the per-instance table; assume present if absent or not found. */
	for (i = 0U; i < chipinfo_ctxt.num_part_info; i++) {
		const struct platforminfo_part_info *entry =
			&chipinfo_ctxt.part_info[i];

		if (((uint32_t)entry->part == (uint32_t)part) &&
		    ((uint32_t)entry->instance == part_idx)) {
			return entry->disabled != 0U;
		}
	}

	return false;
}

enum chipinfo_result qti_chipinfo_init(void)
{
	struct platforminfo_smem *smem;
	size_t size;
	uint32_t fmt;
	uint32_t chip_id;
	uint32_t chip_family;
	uint32_t i;
	int ret;

	/* Access the socinfo SMEM region populated by the boot firmware. */
	ret = qti_smem_lookup(QTI_SMEM_HOST_COMMON, SMEM_HW_SW_BUILD_ID,
			      QTI_SMEM_FLAG_NONE, (void **)&smem, &size);
	if (ret != 0 || smem == NULL || size < sizeof(uint32_t)) {
		return CHIPINFO_ERROR_NOT_FOUND;
	}

	/*
	 * XBL only populates fields up to the format version it reports, so
	 * gate every field read on fmt. chip_id/version exist from v1;
	 * chip_family from v12.
	 */
	fmt = smem->format;

	if (fmt < 1U) {
		return CHIPINFO_ERROR_INVALID_DATA;
	}

	/*
	 * The format version says which fields the layout defines, not that the
	 * item is large enough to hold them; validate size before dereferencing.
	 */
	if (size < PLATFORMINFO_SMEM_SIZE_V1) {
		return CHIPINFO_ERROR_INVALID_DATA;
	}

	/* Bound the raw values before casting into the driver enums. */
	chip_id = smem->chip_id;
	if (chip_id >= CHIPINFO_NUM_IDS) {
		chip_id = CHIPINFO_ID_UNKNOWN;
	}
	chipinfo_ctxt.chipinfo_id = (enum chipinfo_id)chip_id;
	chipinfo_ctxt.version = smem->chip_version;

	if (fmt >= PLATFORMINFO_FORMAT_VER_12 &&
	    size >= PLATFORMINFO_SMEM_SIZE_V12) {
		chip_family = smem->chip_family;
		if (chip_family >= CHIPINFO_NUM_FAMILIES) {
			chip_family = CHIPINFO_FAMILY_UNKNOWN;
		}
		chipinfo_ctxt.family_id = (enum chipinfo_family)chip_family;
	}

	/*
	 * Disabled-features array: offset from the SMEM base, one uint32_t per
	 * part, non-zero means disabled. Entries past num_parts stay present.
	 */
	if ((fmt >= PLATFORMINFO_FORMAT_VER_14) &&
	    (size >= PLATFORMINFO_SMEM_SIZE_V14)) {
		uint32_t offset = smem->disabled_features_array_offset;
		uint32_t num = smem->num_parts;
		const uint32_t *features;

		if (num > CHIPINFO_NUM_PARTS) {
			num = CHIPINFO_NUM_PARTS;
		}

		/* offset <= size first, so (size - offset) below cannot underflow. */
		if ((offset != 0U) && (num != 0U) && (offset <= size) &&
		    (num <= (size - offset) / sizeof(uint32_t))) {
			features = (const uint32_t *)((uintptr_t)smem + offset);
			for (i = 0U; i < num; i++) {
				chipinfo_ctxt.disabled_features[i] =
					features[i];
			}
		}
	}

	/* Per-instance Qultivate table (format >= 23): point into SMEM, no copy. */
	if ((fmt >= PLATFORMINFO_FORMAT_VER_23) &&
	    (size >= PLATFORMINFO_SMEM_SIZE_V23)) {
		uint32_t offset = smem->anPartInstancesOffset;
		uint32_t num = smem->nNumPartInstances;

		/* offset <= size first, so (size - offset) below cannot underflow. */
		if ((offset != 0U) && (num != 0U) && (offset <= size) &&
		    (num <=
		     (size - offset) / sizeof(struct platforminfo_part_info))) {
			chipinfo_ctxt.part_info =
				(const struct platforminfo_part_info
					 *)((uintptr_t)smem + offset);
			chipinfo_ctxt.num_part_info = num;
		}
	}

	chipinfo_ctxt.initialized = true;

	return CHIPINFO_SUCCESS;
}
