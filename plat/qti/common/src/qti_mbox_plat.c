// SPDX-License-Identifier: BSD-3-Clause
/*
 * Copyright (c) 2026, Qualcomm Technologies, Inc. and/or its subsidiaries.
 */

/*
 * Hawi platform mailbox channel configuration.
 *
 * The TME QMP mailbox shared memory region is allocated in SMEM.
 * The physical address is discovered at runtime via qti_smem_lookup()
 * using SMEM item SMEM_TME_MAILBOX (629) in the TZ-TME edge partition.
 *
 * Channel: "tme-qmp"
 *   Transport:   QMP (Qualcomm Message Protocol)
 *   Local EP:    SCORE (EL3 firmware, this file)
 *   Remote EP:   MCORE (TME)
 *   Shared mem:  SMEM item SMEM_TME_MAILBOX (629), TZ-TME edge partition
 *   Signal reg:  HAWI_IPCC_TZ_TME_MBOX_REG (IPCC)
 */

#include <stddef.h>

#include <common/debug.h>
#include <drivers/qti/mbox/qti_mbox_plat.h>
#include <drivers/qti/mbox/qti_mbox_qmp.h>
#include <drivers/qti/smem/smem.h>
#include <lib/utils_def.h>

/*
 * IPCC register used to signal the TME on Hawi.
 * Writing HAWI_IPCC_TZ_TME_MBOX_VAL to HAWI_IPCC_TZ_TME_MBOX_REG
 * triggers an interrupt to the TME processor.
 */
#define HAWI_IPCC_TZ_TME_MBOX_REG 0x0110100CUL
#define HAWI_IPCC_TZ_TME_MBOX_VAL 0x00100000U

/*
 * SMEM processor ID for the TME processor on Hawi.
 * Used with qti_smem_host_id() to construct the SMEM host identifier
 * for the TZ-TME edge partition lookup.
 */
#define QTI_SMEM_PROC_TME ((uint16_t)14U)

/*
 * SMEM item identifier for the TME QMP mailbox shared memory region.
 * This item is allocated in the TZ-TME edge partition.
 */
#define SMEM_TME_MAILBOX ((uint16_t)629U)

/*
 * QMP transport configuration for the TME channel.
 *
 * desc_base and shared_size are populated at runtime by
 * qti_mbox_plat_init() via SMEM lookup.  The config is declared
 * non-const to allow runtime population.
 */
static struct qti_mbox_qmp_config qti_mbox_qmp_tme_cfg = {
	.desc_base   = 0U,  /* populated by qti_mbox_plat_init() */
	.shared_size = 0U,  /* populated by qti_mbox_plat_init() */
	.remote_signal = {
		.reg   = HAWI_IPCC_TZ_TME_MBOX_REG,
		.value = HAWI_IPCC_TZ_TME_MBOX_VAL,
	},
};

static struct qti_mbox_qmp_priv qti_mbox_qmp_tme_priv;

static const struct qti_mbox_chan_config qti_mbox_channels[] = {
	{
		.name = "tme-qmp",
		.ops = &qti_mbox_qmp_ops,
		.transport_cfg = &qti_mbox_qmp_tme_cfg,
		.transport_priv = &qti_mbox_qmp_tme_priv,
	},
};

static struct qti_mbox_chan_slot
	qti_mbox_chan_slots[ARRAY_SIZE(qti_mbox_channels)];

static const struct qti_mbox_plat_data qti_mbox_plat_data = {
	.configs = qti_mbox_channels,
	.slots = qti_mbox_chan_slots,
	.num_channels = ARRAY_SIZE(qti_mbox_channels),
};

/*
 * qti_mbox_plat_init() - discover the TME mailbox address via SMEM.
 *
 * Constructs the TME SMEM host identifier via qti_smem_host_id(), then
 * calls qti_smem_lookup(SMEM_TME_MAILBOX) in the TZ-TME edge partition
 * to obtain the physical base address and size of the shared QMP
 * descriptor region.  On success, writes the platform data pointer into
 * @plat_data.
 *
 * @plat_data: [out] set to &qti_mbox_plat_data on success.
 *
 * Return: 0 on success, -1 on failure.
 */
int qti_mbox_plat_init(const struct qti_mbox_plat_data **plat_data)
{
	void *smem_buf = NULL;
	size_t smem_size = 0U;
	uint16_t tme_host;
	int ret;

	if (!plat_data)
		return -1;

	ret = qti_smem_host_id(QTI_SMEM_PROC_TME, 0U, 0U, 0U, &tme_host);
	if (ret != 0) {
		ERROR("qti_mbox_plat: failed to build TME host ID\n");
		return -1;
	}

	ret = qti_smem_lookup(tme_host, SMEM_TME_MAILBOX, QTI_SMEM_FLAG_NONE,
			      &smem_buf, &smem_size);
	if (ret != 0 || !smem_buf) {
		ERROR("qti_mbox_plat: SMEM_TME_MAILBOX not found\n");
		return -1;
	}
	if (smem_size == 0U) {
		ERROR("qti_mbox_plat: SMEM_TME_MAILBOX size is zero\n");
		return -1;
	}

	qti_mbox_qmp_tme_cfg.desc_base = (uintptr_t)smem_buf;
	qti_mbox_qmp_tme_cfg.shared_size = (uint32_t)smem_size;

	INFO("qti_mbox_plat: TME mailbox at 0x%lx size 0x%x\n",
	     (unsigned long)qti_mbox_qmp_tme_cfg.desc_base,
	     qti_mbox_qmp_tme_cfg.shared_size);

	*plat_data = &qti_mbox_plat_data;
	return 0;
}
