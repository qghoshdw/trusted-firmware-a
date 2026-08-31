#
# Copyright (c) 2026, Qualcomm Technologies, Inc. and/or its subsidiaries.
#
# SPDX-License-Identifier: BSD-3-Clause
#
# Qualcomm mailbox framework
#
# Usage from platform makefile:
#
#   QTI_MBOX     := 1   # enable common core (required)
#   QTI_MBOX_QMP := 1   # enable QMP transport (optional)
#
# The platform must also provide:
#   - An implementation of plat_qti_mbox_get_data()
#   - A static channel configuration table
#     (see include/drivers/qti/mbox/qti_mbox_plat.h)
#

ifeq (${QTI_MBOX},1)

$(eval $(call add_define,QTI_MBOX))

MBOX_DRV_PATH	:= drivers/qti/mbox

BL31_SOURCES	+= ${MBOX_DRV_PATH}/qti_mbox.c

ifeq (${QTI_MBOX_QMP},1)
$(eval $(call add_define,QTI_MBOX_QMP))
BL31_SOURCES	+= ${MBOX_DRV_PATH}/qti_mbox_qmp.c
endif

ifeq (${QTI_MBOX_QMP_LITE},1)
$(eval $(call add_define,QTI_MBOX_QMP_LITE))
BL31_SOURCES	+= ${MBOX_DRV_PATH}/qti_mbox_qmp_lite.c
endif

endif # QTI_MBOX
