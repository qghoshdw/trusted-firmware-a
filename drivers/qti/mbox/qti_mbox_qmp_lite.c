// SPDX-License-Identifier: BSD-3-Clause
/*
 * Copyright (c) 2026, Qualcomm Technologies, Inc. and/or its subsidiaries.
 */

/*
 * QMP-Lite transport backend.
 *
 * QMP-Lite is a lightweight register-based mailbox protocol.  All descriptor
 * and mailbox fields are device MMIO registers; mmio_read_32/write_32 are
 * used for all accesses.  Payload bytes are packed/unpacked in local
 * variables (little-endian, 32-bit aligned).
 *
 * Protocol summary
 * ----------------
 * Each endpoint owns one 32-bit descriptor register.  The bit layout is
 * described in include/drivers/qti/mbox/qti_mbox_qmp_lite.h.
 *
 * The local mailbox region immediately follows the local descriptor register
 * (local_desc_base + 4).  The remote mailbox region immediately follows the
 * remote descriptor register (remote_desc_base + 4).
 *
 * Non-blocking design
 * -------------------
 *   send()    - writes the message and toggles LOCAL_TX; returns immediately.
 *               QTI_MBOX_EVT_TX_DONE is set by process() when the remote
 *               mirrors LOCAL_TX into REMOTE_TX_ACK.
 *   recv()    - reads the message when available; returns -EAGAIN otherwise.
 *               After reading, toggles LOCAL_RX_DONE to signal the remote.
 *   process() - dispatches to per-state handlers that advance the state
 *               machine and set events.
 *
 * Memory barriers
 * ---------------
 * dmbst()  - store barrier: used before publishing the descriptor register
 *            to ensure payload writes are visible first.
 * dmbld()  - load barrier: used after reading msg_len to ensure payload
 *            reads are ordered after the length observation.
 * dmbsy()  - full system barrier: used at the start of process() to ensure
 *            all previous local writes are visible before observing remote
 *            state.
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include <arch_helpers.h>
#include <common/debug.h>
#include <drivers/qti/mbox/qti_mbox.h>
#include <drivers/qti/mbox/qti_mbox_qmp_lite.h>
#include <lib/libc/errno.h>
#include <lib/mmio.h>

#include "qti_mbox_private.h"

static struct qti_mbox_qmp_lite_priv *qmp_lite_priv(struct qti_mbox_chan *chan)
{
	return (struct qti_mbox_qmp_lite_priv *)chan->cfg->transport_priv;
}

static uint32_t qmp_lite_read_remote(const struct qti_mbox_qmp_lite_priv *priv)
{
	return mmio_read_32(priv->cfg->remote_desc_base);
}

static void qmp_lite_publish(struct qti_mbox_qmp_lite_priv *priv)
{
	mmio_write_32(priv->cfg->local_desc_base, priv->local_desc);
}

/*
 * qmp_lite_signal_remote() - write the remote signal register.
 *
 * Issues dmbst() before the write to ensure all shared-memory publications
 * (descriptor and payload) are visible to the remote before the doorbell
 * fires.  If remote_signal.reg is 0, signaling is disabled.
 */
static void qmp_lite_signal_remote(const struct qti_mbox_qmp_lite_config *cfg)
{
	dmbst();

	if (cfg->remote_signal.reg != 0U)
		mmio_write_32(cfg->remote_signal.reg, cfg->remote_signal.value);
}

/*
 * qmp_lite_mbox_write() - write @size bytes from @buf to the local mailbox.
 *
 * The local mailbox starts at local_desc_base + sizeof(uint32_t).
 * Partial words are zero-padded to prevent information leakage.
 * Issues dmbst() at the end so the payload is visible before the caller
 * toggles LOCAL_TX in the descriptor register.
 */
static void qmp_lite_mbox_write(const struct qti_mbox_qmp_lite_priv *priv,
				const uint8_t *buf, uint32_t size)
{
	uintptr_t addr = priv->cfg->local_desc_base + sizeof(uint32_t);
	uint32_t full = size >> 2U;
	uint32_t rem = size & 3U;
	uint32_t word;
	uint32_t i;

	for (i = 0U; i < full; i++) {
		(void)memcpy(&word, buf, sizeof(word));
		mmio_write_32(addr, word);
		buf += sizeof(uint32_t);
		addr += sizeof(uint32_t);
	}
	if (rem != 0U) {
		word = 0U;
		(void)memcpy(&word, buf, rem);
		mmio_write_32(addr, word);
	}
	/* Payload must be visible before LOCAL_TX is toggled. */
	dmbst();
}

/*
 * qmp_lite_mbox_read() - read @size bytes from the remote mailbox into @buf.
 *
 * The remote mailbox starts at remote_desc_base + sizeof(uint32_t).
 * Issues dmbld() at the start to ensure the TX toggle observation (which
 * triggered this read) is ordered before the payload reads.
 */
static void qmp_lite_mbox_read(const struct qti_mbox_qmp_lite_priv *priv,
			       uint8_t *buf, uint32_t size)
{
	uintptr_t addr = priv->cfg->remote_desc_base + sizeof(uint32_t);
	uint32_t full = size >> 2U;
	uint32_t rem = size & 3U;
	uint32_t word;
	uint32_t i;

	/* Observe TX toggle before reading payload. */
	dmbld();
	for (i = 0U; i < full; i++) {
		word = mmio_read_32(addr);
		(void)memcpy(buf, &word, sizeof(word));
		buf += sizeof(uint32_t);
		addr += sizeof(uint32_t);
	}
	if (rem != 0U) {
		word = mmio_read_32(addr);
		(void)memcpy(buf, &word, rem);
	}
}

/*
 * bit_changed() - return true if the remote's bit differs from the local ack.
 *
 * For toggle bits: the remote flips its bit on each event; the local acks
 * by mirroring the current value.  A difference means a new event.
 * For level bits: a difference means the remote has changed state.
 */
static bool bit_changed(const struct qti_mbox_qmp_lite_priv *priv,
			uint32_t remote_bit, uint32_t ack_bit)
{
	return ((priv->remote_desc & remote_bit) != 0U) !=
	       ((priv->local_desc & ack_bit) != 0U);
}

/*
 * ack_bit() - mirror the remote's bit value into the local ack bit.
 *
 * Sets the local ack bit to 1 if the remote bit is 1, clears it otherwise.
 * Works for both level and toggle semantics.
 */
static void ack_bit(struct qti_mbox_qmp_lite_priv *priv, uint32_t remote_bit,
		    uint32_t ack)
{
	if ((priv->remote_desc & remote_bit) != 0U)
		priv->local_desc |= ack;
	else
		priv->local_desc &= ~ack;
}

/*
 * tx_acked() - return true if the remote has acked our last TX.
 *
 * The remote acks by mirroring LOCAL_TX into REMOTE_TX_ACK.  When both
 * bits match, the remote has consumed the message.
 */
static bool tx_acked(const struct qti_mbox_qmp_lite_priv *priv)
{
	return ((priv->local_desc & QMP_LITE_LOCAL_TX) != 0U) ==
	       ((priv->remote_desc & QMP_LITE_REMOTE_TX_ACK) != 0U);
}

/* --------------------------------------------------------------------------
 * Per-state process handlers
 * --------------------------------------------------------------------------
 */

/*
 * handle_link_down() - wait for the remote to assert its link state.
 *
 * When the remote asserts LOCAL_LINK_STATE (bit 0 of its descriptor), the
 * local endpoint responds by asserting its own LOCAL_LINK_STATE and
 * transitioning to LINK_NEGOTIATION.
 */
static void handle_link_down(struct qti_mbox_qmp_lite_priv *priv,
			     const struct qti_mbox_qmp_lite_config *cfg)
{
	if ((priv->remote_desc & QMP_LITE_LOCAL_LINK_STATE) != 0U) {
		priv->local_desc = QMP_LITE_LOCAL_LINK_STATE;
		qmp_lite_publish(priv);
		priv->state = QMP_LITE_STATE_LINK_NEGOTIATION;
		qmp_lite_signal_remote(cfg);
	}
}

/*
 * handle_link_negotiation() - exchange link state acknowledgments.
 *
 * Both endpoints must ack each other's link state before proceeding.
 * Once the local has acked the remote's link state and the remote's link
 * state is still asserted, the local asserts LOCAL_CH_STATE and transitions
 * to LOCAL_CONNECTING.
 */
static void handle_link_negotiation(struct qti_mbox_qmp_lite_priv *priv,
				    const struct qti_mbox_qmp_lite_config *cfg)
{
	bool updated = false;

	if ((priv->remote_desc & QMP_LITE_LOCAL_LINK_STATE) == 0U) {
		/* Remote link went down; reset to LINK_DOWN. */
		priv->local_desc = 0U;
		qmp_lite_publish(priv);
		priv->state = QMP_LITE_STATE_LINK_DOWN;
		qmp_lite_signal_remote(cfg);
		return;
	}
	if (bit_changed(priv, QMP_LITE_LOCAL_LINK_STATE,
			QMP_LITE_REMOTE_LINK_STATE_ACK)) {
		ack_bit(priv, QMP_LITE_LOCAL_LINK_STATE,
			QMP_LITE_REMOTE_LINK_STATE_ACK);
		updated = true;
	}
	if (((priv->local_desc & QMP_LITE_REMOTE_LINK_STATE_ACK) != 0U) &&
	    ((priv->remote_desc & QMP_LITE_LOCAL_LINK_STATE) != 0U)) {
		priv->local_desc |= QMP_LITE_LOCAL_CH_STATE;
		priv->state = QMP_LITE_STATE_LOCAL_CONNECTING;
		updated = true;
	}
	if (updated) {
		qmp_lite_publish(priv);
		qmp_lite_signal_remote(cfg);
	}
}

/*
 * handle_local_connecting() - exchange channel state acknowledgments.
 *
 * Both endpoints must ack each other's channel state.  Once both acks are
 * in place, the channel is fully connected and data transfer can begin.
 */
static void handle_local_connecting(struct qti_mbox_chan *chan,
				    struct qti_mbox_qmp_lite_priv *priv,
				    const struct qti_mbox_qmp_lite_config *cfg,
				    uint32_t *events)
{
	bool updated = false;

	if ((priv->remote_desc & QMP_LITE_LOCAL_LINK_STATE) == 0U) {
		*events |= QTI_MBOX_EVT_REMOTE_RESET |
			   QTI_MBOX_EVT_DISCONNECTED;
		priv->local_desc = 0U;
		qmp_lite_publish(priv);
		priv->state = QMP_LITE_STATE_LINK_DOWN;
		chan->mtu = 0U;
		qmp_lite_signal_remote(cfg);
		return;
	}
	if (bit_changed(priv, QMP_LITE_LOCAL_CH_STATE,
			QMP_LITE_REMOTE_CH_STATE_ACK)) {
		ack_bit(priv, QMP_LITE_LOCAL_CH_STATE,
			QMP_LITE_REMOTE_CH_STATE_ACK);
		updated = true;
	}
	if (updated) {
		qmp_lite_publish(priv);
		qmp_lite_signal_remote(cfg);
	}
	if (((priv->local_desc & QMP_LITE_REMOTE_CH_STATE_ACK) != 0U) &&
	    ((priv->remote_desc & QMP_LITE_LOCAL_CH_STATE) != 0U)) {
		priv->state = QMP_LITE_STATE_E2E_CONNECTED;
		chan->mtu = (size_t)priv->cfg->local_mbox_size;
		*events |= QTI_MBOX_EVT_CONNECTED;
	}
}

/*
 * handle_connected() - handle events in the fully connected state.
 *
 * Checks for:
 *   - Link or channel going down (REMOTE_RESET / DISCONNECTED)
 *   - TX ack: remote has mirrored LOCAL_TX into REMOTE_TX_ACK (TX_DONE)
 *   - RX_DONE ack: remote has mirrored our LOCAL_RX_DONE (ack our consume)
 *   - New RX message: remote has toggled its TX bit (RX_READY)
 */
static void handle_connected(struct qti_mbox_chan *chan,
			     struct qti_mbox_qmp_lite_priv *priv,
			     const struct qti_mbox_qmp_lite_config *cfg,
			     uint32_t *events)
{
	if ((priv->remote_desc & QMP_LITE_LOCAL_LINK_STATE) == 0U) {
		*events |= QTI_MBOX_EVT_REMOTE_RESET |
			   QTI_MBOX_EVT_DISCONNECTED;
		priv->local_desc = 0U;
		qmp_lite_publish(priv);
		priv->state = QMP_LITE_STATE_LINK_DOWN;
		priv->tx_pending = false;
		chan->mtu = 0U;
		qmp_lite_signal_remote(cfg);
		return;
	}
	if ((priv->remote_desc & QMP_LITE_LOCAL_CH_STATE) == 0U) {
		*events |= QTI_MBOX_EVT_DISCONNECTED;
		priv->local_desc &= ~(QMP_LITE_LOCAL_CH_STATE |
				      QMP_LITE_REMOTE_CH_STATE_ACK);
		qmp_lite_publish(priv);
		priv->state = QMP_LITE_STATE_LINK_NEGOTIATION;
		priv->tx_pending = false;
		chan->mtu = 0U;
		qmp_lite_signal_remote(cfg);
		return;
	}

	/* TX_DONE: remote has acked our TX toggle. */
	if (priv->tx_pending && tx_acked(priv)) {
		priv->tx_pending = false;
		*events |= QTI_MBOX_EVT_TX_DONE;
	}

	/*
	 * RX_DONE ack: remote has mirrored our LOCAL_RX_DONE toggle,
	 * confirming it has observed that we consumed its last message.
	 * Mirror the remote's ack back to complete the handshake.
	 */
	if (bit_changed(priv, QMP_LITE_LOCAL_RX_DONE,
			QMP_LITE_REMOTE_RX_DONE_ACK)) {
		ack_bit(priv, QMP_LITE_LOCAL_RX_DONE,
			QMP_LITE_REMOTE_RX_DONE_ACK);
		qmp_lite_publish(priv);
		qmp_lite_signal_remote(cfg);
	}

	/* RX_READY: remote has toggled its TX bit, indicating a new message. */
	if (bit_changed(priv, QMP_LITE_LOCAL_TX, QMP_LITE_REMOTE_TX_ACK))
		*events |= QTI_MBOX_EVT_RX_READY;
}

/* --------------------------------------------------------------------------
 * Transport operation callbacks
 * --------------------------------------------------------------------------
 */

static int qti_mbox_qmp_lite_init(struct qti_mbox_chan *chan)
{
	struct qti_mbox_qmp_lite_priv *priv;
	const struct qti_mbox_qmp_lite_config *cfg;

	priv = qmp_lite_priv(chan);
	cfg = (const struct qti_mbox_qmp_lite_config *)chan->cfg->transport_cfg;
	if (!cfg || !priv || cfg->local_desc_base == 0U ||
	    cfg->remote_desc_base == 0U)
		return -EINVAL;

	if (cfg->local_mbox_size == 0U || cfg->remote_mbox_size == 0U ||
	    cfg->local_mbox_size > QMP_LITE_MAX_MSG_SIZE ||
	    cfg->remote_mbox_size > QMP_LITE_MAX_MSG_SIZE)
		return -EINVAL;

	priv->cfg = cfg;
	priv->state = QMP_LITE_STATE_LINK_DOWN;
	priv->local_desc = 0U;
	priv->remote_desc = 0U;
	priv->tx_pending = false;

	/*
	 * Publish a cleared descriptor first so the remote sees a clean
	 * state, then assert LOCAL_LINK_STATE to initiate negotiation.
	 */
	qmp_lite_publish(priv);
	/* Cleared descriptor must be visible before LINK_STATE is asserted. */
	dmbst();
	priv->local_desc = QMP_LITE_LOCAL_LINK_STATE;
	qmp_lite_publish(priv);
	priv->state = QMP_LITE_STATE_LINK_NEGOTIATION;

	qmp_lite_signal_remote(cfg);

	return 0;
}

static void qti_mbox_qmp_lite_deinit(struct qti_mbox_chan *chan)
{
	struct qti_mbox_qmp_lite_priv *priv = qmp_lite_priv(chan);
	const struct qti_mbox_qmp_lite_config *cfg;

	if (!priv)
		return;

	if (!priv->cfg)
		return;
	cfg = priv->cfg;

	/* De-assert all bits; the remote will detect the link going down. */
	priv->local_desc = 0U;
	qmp_lite_publish(priv);
	priv->state = QMP_LITE_STATE_LINK_DOWN;
	priv->tx_pending = false;
	chan->mtu = 0U;
	qmp_lite_signal_remote(cfg);

	priv->cfg = NULL;
}

static int qti_mbox_qmp_lite_process(struct qti_mbox_chan *chan,
				     uint32_t *events)
{
	struct qti_mbox_qmp_lite_priv *priv = qmp_lite_priv(chan);
	const struct qti_mbox_qmp_lite_config *cfg;

	if (!priv->cfg)
		return -ENODEV;

	/*
	 * Full system barrier before observing remote state.  Ensures all
	 * previous local writes are visible to the remote and any pending
	 * load results are committed before the new observation window.
	 */
	dmbsy();

	cfg = priv->cfg;
	priv->remote_desc = qmp_lite_read_remote(priv);

	switch (priv->state) {
	case QMP_LITE_STATE_LINK_DOWN:
		handle_link_down(priv, cfg);
		break;
	case QMP_LITE_STATE_LINK_NEGOTIATION:
		handle_link_negotiation(priv, cfg);
		break;
	case QMP_LITE_STATE_LOCAL_CONNECTING:
		handle_local_connecting(chan, priv, cfg, events);
		break;
	case QMP_LITE_STATE_E2E_CONNECTED:
		handle_connected(chan, priv, cfg, events);
		break;
	default:
		return -EIO;
	}

	return 0;
}

static int qti_mbox_qmp_lite_send(struct qti_mbox_chan *chan, const void *buf,
				  size_t len)
{
	struct qti_mbox_qmp_lite_priv *priv = qmp_lite_priv(chan);
	const struct qti_mbox_qmp_lite_config *cfg;

	if (!priv->cfg)
		return -ENODEV;

	if (priv->state != QMP_LITE_STATE_E2E_CONNECTED)
		return -ENOTCONN;

	if (priv->tx_pending)
		/*
		 * Previous message not yet acked by the remote.
		 * The caller must wait for QTI_MBOX_EVT_TX_DONE.
		 */
		return -EBUSY;

	if (len > (size_t)priv->cfg->local_mbox_size)
		return -EMSGSIZE;

	cfg = priv->cfg;

	/* Write payload; dmbst() is issued inside qmp_lite_mbox_write(). */
	qmp_lite_mbox_write(priv, (const uint8_t *)buf, (uint32_t)len);

	/* Update msg_size field and toggle LOCAL_TX to signal the remote. */
	priv->local_desc &= ~QMP_LITE_MSG_SIZE_MASK;
	priv->local_desc |= ((uint32_t)len << QMP_LITE_MSG_SIZE_SHIFT) &
			    QMP_LITE_MSG_SIZE_MASK;
	priv->local_desc ^= QMP_LITE_LOCAL_TX;
	qmp_lite_publish(priv);

	priv->tx_pending = true;

	qmp_lite_signal_remote(cfg);

	return 0;
}

static int qti_mbox_qmp_lite_recv(struct qti_mbox_chan *chan, void *buf,
				  size_t *len)
{
	struct qti_mbox_qmp_lite_priv *priv = qmp_lite_priv(chan);
	const struct qti_mbox_qmp_lite_config *cfg;
	uint32_t remote_desc;
	uint32_t msg_size;

	if (!priv->cfg)
		return -ENODEV;

	if (priv->state != QMP_LITE_STATE_E2E_CONNECTED)
		return -EAGAIN;

	cfg = priv->cfg;
	remote_desc = qmp_lite_read_remote(priv);
	priv->remote_desc = remote_desc;

	/* Check if remote toggled TX bit (new message available). */
	if (!bit_changed(priv, QMP_LITE_LOCAL_TX, QMP_LITE_REMOTE_TX_ACK))
		return -EAGAIN;

	msg_size = (remote_desc & QMP_LITE_MSG_SIZE_MASK) >>
		   QMP_LITE_MSG_SIZE_SHIFT;

	if (msg_size == 0U || msg_size > cfg->remote_mbox_size) {
		ERROR("qti_mbox_qmp_lite: bad msg_size %u\n",
		      msg_size);
		return -EIO;
	}
	if ((size_t)msg_size > *len) {
		/* Preserve message; caller can retry with a larger buffer. */
		*len = (size_t)msg_size;
		return -ENOSPC;
	}

	/* qmp_lite_mbox_read() issues dmbld() before reading the payload. */
	qmp_lite_mbox_read(priv, (uint8_t *)buf, msg_size);
	*len = (size_t)msg_size;

	/*
	 * Load barrier: ensure all payload reads complete before returning
	 * ownership to the remote by updating the ack bits.
	 */
	dmbld();
	ack_bit(priv, QMP_LITE_LOCAL_TX, QMP_LITE_REMOTE_TX_ACK);
	priv->local_desc ^= QMP_LITE_LOCAL_RX_DONE;
	qmp_lite_publish(priv);
	qmp_lite_signal_remote(cfg);

	return 0;
}

static bool qti_mbox_qmp_lite_rx_pending(struct qti_mbox_chan *chan)
{
	const struct qti_mbox_qmp_lite_priv *priv = qmp_lite_priv(chan);
	uint32_t remote_desc;

	if (!priv->cfg || priv->state != QMP_LITE_STATE_E2E_CONNECTED)
		return false;
	remote_desc = mmio_read_32(priv->cfg->remote_desc_base);
	return ((remote_desc & QMP_LITE_LOCAL_TX) != 0U) !=
	       ((priv->local_desc & QMP_LITE_REMOTE_TX_ACK) != 0U);
}

const struct qti_mbox_ops qti_mbox_qmp_lite_ops = {
	.init = qti_mbox_qmp_lite_init,
	.deinit = qti_mbox_qmp_lite_deinit,
	.process = qti_mbox_qmp_lite_process,
	.send = qti_mbox_qmp_lite_send,
	.recv = qti_mbox_qmp_lite_recv,
	.rx_pending = qti_mbox_qmp_lite_rx_pending,
};
