/*
 * Driver for WCH PHY chip CH182.
 *
 * Copyright (C) 2026 Nanjing Qinheng Microelectronics Co., Ltd.
 * Web: http://wch.cn
 * Author: WCH <tech@wch.cn>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * Update Log:
 * V1.0 - initial version
 * V1.1 - add soft reset
 * V1.2 - add anti-interference monitoring function
 */

#define DEBUG
#define VERBOSE

#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/mii.h>
#include <linux/phy.h>
#include <linux/of.h>
#include <linux/device.h>
#include <linux/compiler.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/version.h>
#include <linux/workqueue.h>

#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 19, 0)
#ifndef READ_ONCE
#define READ_ONCE(x) ACCESS_ONCE(x)
#endif

#ifndef WRITE_ONCE
#define WRITE_ONCE(x, val)              \
	do {                            \
		ACCESS_ONCE(x) = (val); \
	} while (0)
#endif
#endif

#define DRIVER_AUTHOR "WCH"
#define DRIVER_DESC "WCH phy driver for CH182, etc."
#define VERSION_DESC "V1.2 On 2026.04"

/* WCH PHY ID */
#define PHY_ID_CH182D 0x737190D0

#define PHY_ID_CH182F2 0x73719020
#define PHY_ID_CH182F7 0x73719070
#define PHY_ID_CH182F8 0x73719080

#define PHY_ID_CH182H1 0x73719110
#define PHY_ID_CH182H2 0x73719120
#define PHY_ID_CH182H3 0x73719130
#define PHY_ID_CH182H6 0x73719160
#define PHY_ID_CH182H7 0x73719170
#define PHY_ID_CH182H8 0x73719180

#define WCH_PHY_ID_MASK 0xfffffff0

/* Operation Mode Strap Override */
#define PHY_PAG_SEL 0x1F

/*  Extended register */
#define PHY_PAG0 0x00
#define CH182_PHY_STATUS0 0x10
#define CH182_POLARITY_STATE BIT(12)
#define CH182_DESCR_LOCK_STATE BIT(9)
#define CH182_PCSR_100M_CTL 0x16
#define CH182_FORCE_100_OK BIT(5)
#define CH182_PWR_SAVE 0x18
#define CH182_PWR_RST_OK BIT(1)
#define CH182_INTERRUPT_IND 0x1E

#define PHY_PAG1 0x01
#define CH182_RX_CNT_CTRL 0x17
#define CH182_RX_CNT_EN BIT(12)

#define PHY_PAG7 0x07
#define CH182_CUST_LED_SET 0x11
#define CH182_INTERRUPT_MASK 0x13
#define CH182_INT_LINKCHG BIT(13)
#define CH182_CUSTOMIZED_LED BIT(3)

#define CH182_LINK_UP_INTVAL_MS 200
#define CH182_LINK_DOWN_INTVAL_MS 300
#define CH182_BMCR_RESET_CLEAR 0x3100
#define CH182_FLOW_DEBUG 0
#define CH182_REG_DEBUG 0
#define CH182_RECOVERY_DEBUG 0
#define CH182_DBG_FLOW "CH182_FLOW"
#define CH182_DBG_REG "CH182_REG"
#define CH182_DBG_RECOVERY "CH182_RECOVERY"

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 5, 0)
#define CH182_PHY_BUS(_phydev) ((_phydev)->mdio.bus)
#define CH182_PHY_ADDR(_phydev) ((_phydev)->mdio.addr)
#define CH182_PHY_DEV(_phydev) (&(_phydev)->mdio.dev)
#else
#define CH182_PHY_BUS(_phydev) ((_phydev)->bus)
#define CH182_PHY_ADDR(_phydev) ((_phydev)->addr)
#define CH182_PHY_DEV(_phydev) (&(_phydev)->dev)
#endif

struct ch182_led_type {
	u32 led_mode_reg;
	bool has_traditional_led;
	bool has_custom_led;
};

enum ch182_monitor_state {
	CH182_MONITOR_STOPPED,
	CH182_MONITOR_LINK_DOWN,
	CH182_MONITOR_LINK_UP,
};

/* led_tmode: traditional led mode.
 * led0_mode/led1_mode: custom led mode.
 */
struct ch182_priv {
	struct phy_device *phydev;
	const struct ch182_led_type *type;
	int led_tmode, led0_mode, led1_mode;
	bool has_valid_led_values;
	enum ch182_monitor_state monitor_state;
	unsigned int monitor_seq;
	struct mutex monitor_lock;
	struct workqueue_struct *wq;
	struct delayed_work monitor_work;
};

#if CH182_FLOW_DEBUG
#define ch182_flow_dbg(_phydev, _fmt, ...)                               \
	dev_info(CH182_PHY_DEV(_phydev), "%s %s: " _fmt, CH182_DBG_FLOW, \
		 __func__, ##__VA_ARGS__)
#else
#define ch182_flow_dbg(_phydev, _fmt, ...) \
	do {                               \
	} while (0)
#endif

#if CH182_REG_DEBUG
#define ch182_reg_dbg(_phydev, _fmt, ...)                               \
	dev_info(CH182_PHY_DEV(_phydev), "%s %s: " _fmt, CH182_DBG_REG, \
		 __func__, ##__VA_ARGS__)
#else
#define ch182_reg_dbg(_phydev, _fmt, ...) \
	do {                              \
	} while (0)
#endif

#if CH182_RECOVERY_DEBUG
#define ch182_recovery_dbg(_phydev, _fmt, ...)                               \
	dev_info(CH182_PHY_DEV(_phydev), "%s %s: " _fmt, CH182_DBG_RECOVERY, \
		 __func__, ##__VA_ARGS__)
#else
#define ch182_recovery_dbg(_phydev, _fmt, ...) \
	do {                                   \
	} while (0)
#endif

static const struct ch182_led_type ch182h8_led_type = {
	.led_mode_reg = CH182_INTERRUPT_MASK,
	.has_traditional_led = true,
	.has_custom_led = true,
};

static int ch182_check_link_stat(struct phy_device *phydev);
static int ch182_check_link_speed(struct phy_device *phydev);

static int ch182_mdio_read(struct phy_device *phydev, u32 regnum)
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 16, 0)
	return __phy_read(phydev, regnum);
#else
	struct mii_bus *bus = CH182_PHY_BUS(phydev);

	return bus->read(bus, CH182_PHY_ADDR(phydev), regnum);
#endif
}

static int ch182_mdio_write(struct phy_device *phydev, u32 regnum, u16 val)
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 16, 0)
	return __phy_write(phydev, regnum, val);
#else
	struct mii_bus *bus = CH182_PHY_BUS(phydev);

	return bus->write(bus, CH182_PHY_ADDR(phydev), regnum, val);
#endif
}

static int ch182_page_read(struct phy_device *phydev, int page, u32 regnum)
{
	struct mii_bus *bus = CH182_PHY_BUS(phydev);
	int ret;

	mutex_lock(&bus->mdio_lock);

	ch182_mdio_write(phydev, PHY_PAG_SEL, page);
	ret = ch182_mdio_read(phydev, regnum);
	ch182_mdio_write(phydev, PHY_PAG_SEL, PHY_PAG0);

	mutex_unlock(&bus->mdio_lock);

	return ret;
}

static int ch182_modify_paged(struct phy_device *phydev, int page, u32 regnum,
			      u16 mask, u16 set)
{
	struct mii_bus *bus = CH182_PHY_BUS(phydev);
	int val;

	mutex_lock(&bus->mdio_lock);

	ch182_mdio_write(phydev, PHY_PAG_SEL, page);
	val = ch182_mdio_read(phydev, regnum);
	if (val < 0)
		goto out;

	val &= ~mask;
	val |= set;
	ch182_mdio_write(phydev, regnum, val);
	val = 0;

out:
	ch182_mdio_write(phydev, PHY_PAG_SEL, PHY_PAG0);
	mutex_unlock(&bus->mdio_lock);

	return val;
}

static int ch182_setup_led_tmode(struct phy_device *phydev)
{
	struct ch182_priv *priv = phydev->priv;
	u32 reg;
	int val, ret;

	reg = priv->type->led_mode_reg;
	val = priv->led_tmode;

	ret = ch182_modify_paged(phydev, PHY_PAG7, reg, (0x3 << 4),
				 ((val & 0x3) << 4));
	if (ret < 0)
		phydev_err(phydev, "failed to set led mode\n");

	return ret;
}

static int ch182_setup_led_cmode(struct phy_device *phydev)
{
	struct ch182_priv *priv = phydev->priv;
	u32 reg;
	int val_0, val_1, ret;

	reg = CH182_CUST_LED_SET;
	val_0 = priv->led0_mode & 0x7;
	val_1 = (priv->led1_mode & 0x7) << 4;

	ret = ch182_modify_paged(phydev, PHY_PAG7, CH182_INTERRUPT_MASK,
				 CH182_CUSTOMIZED_LED, CH182_CUSTOMIZED_LED);
	if (ret < 0)
		goto out;

	ret = ch182_modify_paged(phydev, PHY_PAG7, reg, 0x77, (val_0 | val_1));

out:
	if (ret < 0)
		phydev_err(phydev, "failed to set led mode\n");

	return ret;
}

static int ch182_init_rxcnt(struct phy_device *phydev)
{
	int ret;

	ret = ch182_modify_paged(phydev, PHY_PAG1, CH182_RX_CNT_CTRL,
				 CH182_RX_CNT_EN, CH182_RX_CNT_EN);
	ch182_flow_dbg(phydev, "enable rx counter ret=%d\n", ret);
	if (ret < 0)
		phydev_err(phydev, "failed to enable rx counter\n");

	return ret;
}

static bool ch182_monitor_active(struct ch182_priv *priv,
				 enum ch182_monitor_state state,
				 unsigned int seq)
{
	/* A worker is valid only while its state and generation still match. */
	return READ_ONCE(priv->monitor_state) == state &&
	       READ_ONCE(priv->monitor_seq) == seq;
}

static void ch182_monitor_queue(struct ch182_priv *priv,
				enum ch182_monitor_state state,
				unsigned int delay_ms, bool allow_start)
{
	bool queue = false;

	mutex_lock(&priv->monitor_lock);

	if (priv->wq && state != CH182_MONITOR_STOPPED) {
		if (!allow_start &&
		    priv->monitor_state == CH182_MONITOR_STOPPED)
			goto out;

		/* Repeated same-state notifications must not postpone checks. */
		if (priv->monitor_state != state) {
			WRITE_ONCE(priv->monitor_state, state);
			WRITE_ONCE(priv->monitor_seq, priv->monitor_seq + 1);
			queue = true;
			ch182_flow_dbg(
				priv->phydev,
				"switch monitor state=%d seq=%u delay=%ums\n",
				state, priv->monitor_seq, delay_ms);
		} else if (!delayed_work_pending(&priv->monitor_work) &&
			   !work_busy(&priv->monitor_work.work)) {
			queue = true;
			ch182_flow_dbg(
				priv->phydev,
				"restart idle monitor state=%d seq=%u delay=%ums\n",
				state, priv->monitor_seq, delay_ms);
		}

		if (queue)
			mod_delayed_work(priv->wq, &priv->monitor_work,
					 msecs_to_jiffies(delay_ms));
	}

out:
	mutex_unlock(&priv->monitor_lock);
}

static void ch182_monitor_start(struct ch182_priv *priv,
				enum ch182_monitor_state state,
				unsigned int delay_ms)
{
	ch182_monitor_queue(priv, state, delay_ms, true);
}

static void ch182_monitor_set(struct ch182_priv *priv,
			      enum ch182_monitor_state state,
			      unsigned int delay_ms)
{
	ch182_monitor_queue(priv, state, delay_ms, false);
}

static void ch182_monitor_requeue(struct ch182_priv *priv,
				  enum ch182_monitor_state state,
				  unsigned int seq, unsigned int delay_ms)
{
	mutex_lock(&priv->monitor_lock);

	/* Requeue only if no link transition invalidated this worker. */
	if (ch182_monitor_active(priv, state, seq) && priv->wq)
		mod_delayed_work(priv->wq, &priv->monitor_work,
				 msecs_to_jiffies(delay_ms));

	mutex_unlock(&priv->monitor_lock);
}

static void ch182_monitor_stop(struct ch182_priv *priv)
{
	mutex_lock(&priv->monitor_lock);
	WRITE_ONCE(priv->monitor_state, CH182_MONITOR_STOPPED);
	WRITE_ONCE(priv->monitor_seq, priv->monitor_seq + 1);
	mutex_unlock(&priv->monitor_lock);

	/* Stop/remove paths must wait until the worker has fully exited. */
	cancel_delayed_work_sync(&priv->monitor_work);
}

static int ch182_config_init(struct phy_device *phydev)
{
	struct ch182_priv *priv = phydev->priv;
	int ret;

#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 4, 0)
	ret = genphy_config_init(phydev);
	if (ret < 0)
		return ret;
#endif

	if (priv->has_valid_led_values) {
		if (priv->led_tmode >= 0)
			ret = ch182_setup_led_tmode(phydev);
		else
			ret = ch182_setup_led_cmode(phydev);

		if (ret < 0)
			return ret;
	}

	ret = ch182_init_rxcnt(phydev);
	if (ret < 0)
		return ret;

	ch182_monitor_start(priv, CH182_MONITOR_LINK_DOWN,
			    CH182_LINK_DOWN_INTVAL_MS);

	return 0;
}

static void ch182_linkup_cfg(struct ch182_priv *priv)
{
	ch182_monitor_set(priv, CH182_MONITOR_LINK_UP, CH182_LINK_UP_INTVAL_MS);
}

static void ch182_linkdown_cfg(struct ch182_priv *priv)
{
	ch182_monitor_set(priv, CH182_MONITOR_LINK_DOWN,
			  CH182_LINK_DOWN_INTVAL_MS);
}

static void ch182_link_change(struct phy_device *phydev)
{
	struct ch182_priv *priv = phydev->priv;
	int link;

	if (!priv)
		return;

	if (READ_ONCE(priv->monitor_state) == CH182_MONITOR_STOPPED ||
	    !priv->wq)
		return;

	link = phydev->link;
	ch182_flow_dbg(phydev, "link notify link=%d speed=%d\n", link,
		       phydev->speed);

	if (link)
		ch182_linkup_cfg(priv);
	else
		ch182_linkdown_cfg(priv);
}

static int ch182_soft_reset(struct phy_device *phydev)
{
	int val;

	ch182_recovery_dbg(phydev, "trigger ch182_soft_reset\n");
	phy_write(phydev, MII_BMCR, BMCR_RESET);

	/* CH182 needs the reset bit to be cleared manually after reset-ready. */
	val = ch182_page_read(phydev, PHY_PAG0, CH182_PWR_SAVE);
	if (val < 0)
		return val;

	ch182_reg_dbg(phydev, "pwr_save=0x%04x\n", val & 0xffff);

	if (val & CH182_PWR_RST_OK)
		phy_write(phydev, MII_BMCR, CH182_BMCR_RESET_CLEAR);

	return 0;
}

static int ch182_set_link_interrupt_locked(struct phy_device *phydev,
					   bool enable)
{
	int val, ret;

	ret = ch182_mdio_write(phydev, PHY_PAG_SEL, PHY_PAG7);
	if (ret < 0)
		return ret;

	val = ch182_mdio_read(phydev, CH182_INTERRUPT_MASK);
	if (val < 0)
		return val;

	if (enable)
		val |= CH182_INT_LINKCHG;
	else
		val &= ~CH182_INT_LINKCHG;

	return ch182_mdio_write(phydev, CH182_INTERRUPT_MASK, val);
}

static int ch182_modify_page0_locked(struct phy_device *phydev, u32 regnum,
				     u16 mask, u16 set)
{
	int val, ret;

	ret = ch182_mdio_write(phydev, PHY_PAG_SEL, PHY_PAG0);
	if (ret < 0)
		return ret;

	val = ch182_mdio_read(phydev, regnum);
	if (val < 0)
		return val;

	val &= ~mask;
	val |= set;

	return ch182_mdio_write(phydev, regnum, val);
}

static void ch182_set_phypn(struct phy_device *phydev)
{
	struct mii_bus *bus = CH182_PHY_BUS(phydev);
	int ret, restore_ret;

	ch182_recovery_dbg(phydev, "trigger ch182_set_phypn\n");

	mutex_lock(&bus->mdio_lock);

	ret = ch182_set_link_interrupt_locked(phydev, false);
	if (ret < 0) {
		phydev_err(phydev, "failed to disable link interrupt: %d\n",
			   ret);
		goto out;
	}

	ret = ch182_modify_page0_locked(phydev, CH182_PCSR_100M_CTL,
					CH182_FORCE_100_OK, CH182_FORCE_100_OK);
	if (ret < 0) {
		phydev_err(phydev, "failed to set FORCE_100_OK: %d\n", ret);
		goto out;
	}

	ret = ch182_modify_page0_locked(phydev, CH182_PCSR_100M_CTL,
					CH182_FORCE_100_OK, 0);
	if (ret < 0) {
		phydev_err(phydev, "failed to clear FORCE_100_OK: %d\n", ret);
		goto out;
	}

	ret = ch182_mdio_read(phydev, CH182_INTERRUPT_IND);
	if (ret < 0) {
		phydev_err(phydev, "failed to read interrupt status: %d\n",
			   ret);
		goto out;
	}

	ret = ch182_set_link_interrupt_locked(phydev, true);
	if (ret < 0) {
		phydev_err(phydev, "failed to enable link interrupt: %d\n",
			   ret);
		goto out;
	}

out:
	restore_ret = ch182_mdio_write(phydev, PHY_PAG_SEL, PHY_PAG0);
	if (restore_ret < 0)
		phydev_err(phydev, "failed to restore PHY page0: %d\n",
			   restore_ret);

	mutex_unlock(&bus->mdio_lock);
}

static bool ch182_check_base_reg(struct phy_device *phydev)
{
	int val;

	val = phy_read(phydev, MII_EXPANSION);
	if (val < 0) {
		phydev_err(phydev, "failed to read expansion register\n");
		return false;
	}

	ch182_reg_dbg(phydev, "expansion=0x%04x\n", val & 0xffff);

	return !(val & 0x0001);
}

/* Link-up health check: verify PHY-side status before recovery. */
static void ch182_check_link(struct phy_device *phydev, struct ch182_priv *priv,
			     unsigned int seq)
{
	int val = 0, count = 0, i;
	int link, speed;

	for (i = 5; i > 0; i--) {
		val = ch182_page_read(phydev, PHY_PAG0, CH182_PHY_STATUS0);
		if (val < 0)
			return;

		ch182_reg_dbg(phydev, "phy_status0=0x%04x\n", val & 0xffff);

		if ((val & CH182_DESCR_LOCK_STATE) == 0)
			count++;
		else
			count = 0;

		msleep(1);
	}

	if (count == 5) {
		link = ch182_check_link_stat(phydev);
		speed = ch182_check_link_speed(phydev);
		ch182_flow_dbg(phydev,
			       "descr unlock count=%d link=%d speed=%d\n",
			       count, link, speed);
		if (link > 0 && speed > 0 &&
		    ch182_monitor_active(priv, CH182_MONITOR_LINK_UP, seq))
			ch182_soft_reset(phydev);
	}

	return;
}

static int ch182_check_link_stat(struct phy_device *phydev)
{
	int val;

	val = phy_read(phydev, MII_BMSR);
	if (val < 0)
		return val;

	if (!(val & BMSR_LSTATUS)) {
		val = phy_read(phydev, MII_BMSR);
		if (val < 0)
			return val;
	}

	ch182_reg_dbg(phydev, "bmsr=0x%04x link=%d\n", val & 0xffff,
		      !!(val & BMSR_LSTATUS));

	return !!(val & BMSR_LSTATUS);
}

/* Link-down conditioning path used before retrying link establishment. */
static void ch182_check_phypn(struct phy_device *phydev,
			      struct ch182_priv *priv, unsigned int seq,
			      bool flag)
{
	int phy_stat;
	int link;
	unsigned int delay_ms;
	bool retry;

	if (flag)
		delay_ms = 300;
	else
		delay_ms = 200;

	phy_stat = ch182_page_read(phydev, PHY_PAG0, CH182_PHY_STATUS0);
	if (phy_stat < 0) {
		phydev_err(phydev, "failed to read phy status0: %d\n",
			   phy_stat);
		return;
	}

	if (phy_stat & CH182_POLARITY_STATE) {
		/* Avoid holding phylib's state lock across the long settle delay. */
		mutex_unlock(&phydev->lock);
		msleep(delay_ms);
		mutex_lock(&phydev->lock);

		link = ch182_check_link_stat(phydev);
		ch182_reg_dbg(
			phydev,
			"polarity set, flag=%d status0=0x%04x link=%d delay=%ums\n",
			flag, phy_stat & 0xffff, link, delay_ms);
		if (link == 0) {
			retry = !flag;
			if (!retry)
				retry = ch182_check_base_reg(phydev);

			if (retry &&
			    ch182_monitor_active(priv, CH182_MONITOR_LINK_DOWN,
						 seq))
				ch182_set_phypn(phydev);
		}
	}

	return;
}

/* Select the link-down maintenance sequence from the current PHY mode. */
static void ch182_link_processing(struct phy_device *phydev,
				  struct ch182_priv *priv, unsigned int seq)
{
	int phy_bcr;

	phy_bcr = phy_read(phydev, MII_BMCR);
	if (phy_bcr < 0)
		return;

	ch182_reg_dbg(phydev, "bmcr=0x%04x\n", phy_bcr & 0xffff);

	if (phy_bcr & BMCR_ANENABLE)
		ch182_check_phypn(phydev, priv, seq, true);
	else if ((phy_bcr & BMCR_SPEED100) == 0)
		ch182_check_phypn(phydev, priv, seq, false);
}

static int ch182_monitor_snapshot(struct ch182_priv *priv,
				  enum ch182_monitor_state *state,
				  unsigned int *seq)
{
	mutex_lock(&priv->monitor_lock);

	if (priv->monitor_state == CH182_MONITOR_STOPPED) {
		mutex_unlock(&priv->monitor_lock);
		return -ECANCELED;
	}

	*state = priv->monitor_state;
	*seq = priv->monitor_seq;
	mutex_unlock(&priv->monitor_lock);

	return 0;
}

static void ch182_monitor_work(struct work_struct *work)
{
	struct ch182_priv *priv = container_of(to_delayed_work(work),
					       struct ch182_priv, monitor_work);
	struct phy_device *phydev = priv->phydev;
	enum ch182_monitor_state state;
	unsigned int seq;
	unsigned int delay_ms = 0;
	bool requeue = false;
	int link = 0;
	int speed = 0;

	if (ch182_monitor_snapshot(priv, &state, &seq))
		return;

	mutex_lock(&phydev->lock);

	if (!ch182_monitor_active(priv, state, seq))
		goto out;

	switch (state) {
	case CH182_MONITOR_LINK_DOWN:
		link = ch182_check_link_stat(phydev);
		if (link == 0) {
			ch182_link_processing(phydev, priv, seq);
			link = ch182_check_link_stat(phydev);
		}

		ch182_flow_dbg(phydev, "link-down worker link=%d seq=%u\n",
			       link, seq);

		if (link <= 0) {
			delay_ms = CH182_LINK_DOWN_INTVAL_MS;
			requeue = true;
		}
		break;
	case CH182_MONITOR_LINK_UP:
		speed = ch182_check_link_speed(phydev);
		if (speed > 0)
			ch182_check_link(phydev, priv, seq);

		link = ch182_check_link_stat(phydev);

		ch182_flow_dbg(phydev,
			       "link-up worker link=%d speed=%d seq=%u\n", link,
			       speed, seq);

		if (link != 0) {
			delay_ms = CH182_LINK_UP_INTVAL_MS;
			requeue = true;
		}
		break;
	default:
		break;
	}

out:
	mutex_unlock(&phydev->lock);

	if (requeue)
		ch182_monitor_requeue(priv, state, seq, delay_ms);
}

/* Use phylib's resolved speed for link-up monitoring. */
static int ch182_check_link_speed(struct phy_device *phydev)
{
	ch182_flow_dbg(phydev, "resolved speed=%d\n", phydev->speed);

	return phydev->speed == SPEED_100;
}

static int ch182_probe(struct phy_device *phydev)
{
	const struct ch182_led_type *type = phydev->drv->driver_data;
	const struct device_node *np = CH182_PHY_DEV(phydev)->of_node;
	struct ch182_priv *priv;
	u32 led_tmode = 0, led0_mode = 0, led1_mode = 0;
	bool has_led0 = false, has_led1 = false, has_tmode = false;
	int ret;

	dev_info(CH182_PHY_DEV(phydev), "ch182 phy probe, driver version: %s\n",
		 VERSION_DESC);

	priv = devm_kzalloc(CH182_PHY_DEV(phydev), sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	phydev->priv = priv;
	priv->phydev = phydev;
	priv->type = type;
	priv->has_valid_led_values = false;
	priv->led_tmode = -1;
	mutex_init(&priv->monitor_lock);
	WRITE_ONCE(priv->monitor_state, CH182_MONITOR_STOPPED);
	WRITE_ONCE(priv->monitor_seq, 0);
	priv->wq = alloc_workqueue("ch182_wq", WQ_MEM_RECLAIM, 1);
	if (!priv->wq) {
		phydev_err(phydev, "failed to create workqueue\n");
		return -ENOMEM;
	}

	INIT_DELAYED_WORK(&priv->monitor_work, ch182_monitor_work);

	ch182_flow_dbg(phydev, "probe done, workqueue ready\n");

	/* Try custom LED mode first */
	if (type->led_mode_reg && type->has_custom_led) {
		ret = of_property_read_u32(np, "wch,led0-mode", &led0_mode);
		if (!ret)
			has_led0 = true;

		ret = of_property_read_u32(np, "wch,led1-mode", &led1_mode);
		if (!ret)
			has_led1 = true;

		if (has_led0 && has_led1 && led0_mode <= 7 && led1_mode <= 7) {
			priv->led0_mode = led0_mode;
			priv->led1_mode = led1_mode;
			priv->has_valid_led_values = true;
			goto out;
		} else if (has_led0 || has_led1) {
			dev_warn(
				CH182_PHY_DEV(phydev),
				"invalid custom LED values, led0: 0x%02x, led1: 0x%02x\n",
				led0_mode, led1_mode);
		}
	}

	/* Try traditional LED mode if custom is not valid */
	if (type->led_mode_reg && type->has_traditional_led) {
		ret = of_property_read_u32(np, "wch,traditional-led",
					   &led_tmode);
		if (!ret)
			has_tmode = true;

		if (has_tmode && led_tmode <= 3) {
			priv->led_tmode = led_tmode;
			priv->has_valid_led_values = true;
		} else if (has_tmode) {
			dev_warn(CH182_PHY_DEV(phydev),
				 "invalid traditional LED mode: 0x%02x\n",
				 led_tmode);
		}
	}

out:
	return 0;
}

static void ch182_remove(struct phy_device *phydev)
{
	struct ch182_priv *priv = phydev->priv;
	struct workqueue_struct *wq;

	if (!priv)
		return;

	ch182_monitor_stop(priv);

	mutex_lock(&priv->monitor_lock);
	wq = priv->wq;
	priv->wq = NULL;
	mutex_unlock(&priv->monitor_lock);

	if (wq)
		destroy_workqueue(wq);
}

static int ch182_suspend(struct phy_device *phydev)
{
	struct ch182_priv *priv = phydev->priv;
	int ret;

	if (priv) {
		ch182_flow_dbg(phydev, "suspend\n");
		ch182_monitor_stop(priv);
	}

	ret = genphy_suspend(phydev);
	if (ret < 0 && priv) {
		if (phydev->link)
			ch182_monitor_start(priv, CH182_MONITOR_LINK_UP,
					    CH182_LINK_UP_INTVAL_MS);
		else
			ch182_monitor_start(priv, CH182_MONITOR_LINK_DOWN,
					    CH182_LINK_DOWN_INTVAL_MS);
	}

	return ret;
}

static int ch182_resume(struct phy_device *phydev)
{
	struct ch182_priv *priv = phydev->priv;
	int ret;

	ret = genphy_resume(phydev);
	if (ret < 0)
		return ret;

	if (priv) {
		ch182_flow_dbg(phydev, "resume\n");
		ch182_monitor_start(priv, CH182_MONITOR_LINK_DOWN,
				    CH182_LINK_DOWN_INTVAL_MS);
	}

	return 0;
}

static struct phy_driver ch182_driver[] = {
	{
		.phy_id = PHY_ID_CH182D,
		.name = "WCH CH182D Ethernet",
		.phy_id_mask = WCH_PHY_ID_MASK,
		.driver_data = &ch182h8_led_type,
		.probe = ch182_probe,
		.remove = ch182_remove,
		.config_init = ch182_config_init,
		.link_change_notify = ch182_link_change,
		.features = PHY_BASIC_FEATURES,
		.config_aneg = &genphy_config_aneg,
		.read_status = &genphy_read_status,
		.soft_reset = ch182_soft_reset,
		.suspend = ch182_suspend,
		.resume = ch182_resume,
	},
	{
		.phy_id = PHY_ID_CH182F2,
		.name = "WCH CH182F2 Ethernet",
		.phy_id_mask = WCH_PHY_ID_MASK,
		.driver_data = &ch182h8_led_type,
		.probe = ch182_probe,
		.remove = ch182_remove,
		.config_init = ch182_config_init,
		.link_change_notify = ch182_link_change,
		.features = PHY_BASIC_FEATURES,
		.config_aneg = &genphy_config_aneg,
		.read_status = &genphy_read_status,
		.soft_reset = ch182_soft_reset,
		.suspend = ch182_suspend,
		.resume = ch182_resume,
	},
	{
		.phy_id = PHY_ID_CH182F7,
		.name = "WCH CH182F7 Ethernet",
		.phy_id_mask = WCH_PHY_ID_MASK,
		.driver_data = &ch182h8_led_type,
		.probe = ch182_probe,
		.remove = ch182_remove,
		.config_init = ch182_config_init,
		.link_change_notify = ch182_link_change,
		.features = PHY_BASIC_FEATURES,
		.config_aneg = &genphy_config_aneg,
		.read_status = &genphy_read_status,
		.soft_reset = ch182_soft_reset,
		.suspend = ch182_suspend,
		.resume = ch182_resume,
	},
	{
		.phy_id = PHY_ID_CH182F8,
		.name = "WCH CH182F8 Ethernet",
		.phy_id_mask = WCH_PHY_ID_MASK,
		.driver_data = &ch182h8_led_type,
		.probe = ch182_probe,
		.remove = ch182_remove,
		.config_init = ch182_config_init,
		.link_change_notify = ch182_link_change,
		.features = PHY_BASIC_FEATURES,
		.config_aneg = &genphy_config_aneg,
		.read_status = &genphy_read_status,
		.soft_reset = ch182_soft_reset,
		.suspend = ch182_suspend,
		.resume = ch182_resume,
	},
	{
		.phy_id = PHY_ID_CH182H1,
		.name = "WCH CH182H1 Ethernet",
		.phy_id_mask = WCH_PHY_ID_MASK,
		.driver_data = &ch182h8_led_type,
		.probe = ch182_probe,
		.remove = ch182_remove,
		.config_init = ch182_config_init,
		.link_change_notify = ch182_link_change,
		.features = PHY_BASIC_FEATURES,
		.config_aneg = &genphy_config_aneg,
		.read_status = &genphy_read_status,
		.soft_reset = ch182_soft_reset,
		.suspend = ch182_suspend,
		.resume = ch182_resume,
	},
	{
		.phy_id = PHY_ID_CH182H2,
		.name = "WCH CH182H2 Ethernet",
		.phy_id_mask = WCH_PHY_ID_MASK,
		.driver_data = &ch182h8_led_type,
		.probe = ch182_probe,
		.remove = ch182_remove,
		.config_init = ch182_config_init,
		.link_change_notify = ch182_link_change,
		.features = PHY_BASIC_FEATURES,
		.config_aneg = &genphy_config_aneg,
		.read_status = &genphy_read_status,
		.soft_reset = ch182_soft_reset,
		.suspend = ch182_suspend,
		.resume = ch182_resume,
	},
	{
		.phy_id = PHY_ID_CH182H3,
		.name = "WCH CH182H3 Ethernet",
		.phy_id_mask = WCH_PHY_ID_MASK,
		.driver_data = &ch182h8_led_type,
		.probe = ch182_probe,
		.remove = ch182_remove,
		.config_init = ch182_config_init,
		.link_change_notify = ch182_link_change,
		.features = PHY_BASIC_FEATURES,
		.config_aneg = &genphy_config_aneg,
		.read_status = &genphy_read_status,
		.soft_reset = ch182_soft_reset,
		.suspend = ch182_suspend,
		.resume = ch182_resume,
	},
	{
		.phy_id = PHY_ID_CH182H6,
		.name = "WCH CH182H6 Ethernet",
		.phy_id_mask = WCH_PHY_ID_MASK,
		.driver_data = &ch182h8_led_type,
		.probe = ch182_probe,
		.remove = ch182_remove,
		.config_init = ch182_config_init,
		.link_change_notify = ch182_link_change,
		.features = PHY_BASIC_FEATURES,
		.config_aneg = &genphy_config_aneg,
		.read_status = &genphy_read_status,
		.soft_reset = ch182_soft_reset,
		.suspend = ch182_suspend,
		.resume = ch182_resume,
	},
	{
		.phy_id = PHY_ID_CH182H7,
		.name = "WCH CH182H7 Ethernet",
		.phy_id_mask = WCH_PHY_ID_MASK,
		.driver_data = &ch182h8_led_type,
		.probe = ch182_probe,
		.remove = ch182_remove,
		.config_init = ch182_config_init,
		.link_change_notify = ch182_link_change,
		.features = PHY_BASIC_FEATURES,
		.config_aneg = &genphy_config_aneg,
		.read_status = &genphy_read_status,
		.soft_reset = ch182_soft_reset,
		.suspend = ch182_suspend,
		.resume = ch182_resume,
	},
	{
		.phy_id = PHY_ID_CH182H8,
		.name = "WCH CH182H8 Ethernet",
		.phy_id_mask = WCH_PHY_ID_MASK,
		.driver_data = &ch182h8_led_type,
		.probe = ch182_probe,
		.remove = ch182_remove,
		.config_init = ch182_config_init,
		.link_change_notify = ch182_link_change,
		.features = PHY_BASIC_FEATURES,
		.config_aneg = &genphy_config_aneg,
		.read_status = &genphy_read_status,
		.soft_reset = ch182_soft_reset,
		.suspend = ch182_suspend,
		.resume = ch182_resume,
	}
};

module_phy_driver(ch182_driver);

static struct mdio_device_id __maybe_unused wch_tbl[] = {
	{ PHY_ID_CH182D, WCH_PHY_ID_MASK },
	{ PHY_ID_CH182F2, WCH_PHY_ID_MASK },
	{ PHY_ID_CH182F7, WCH_PHY_ID_MASK },
	{ PHY_ID_CH182F8, WCH_PHY_ID_MASK },
	{ PHY_ID_CH182H1, WCH_PHY_ID_MASK },
	{ PHY_ID_CH182H2, WCH_PHY_ID_MASK },
	{ PHY_ID_CH182H3, WCH_PHY_ID_MASK },
	{ PHY_ID_CH182H6, WCH_PHY_ID_MASK },
	{ PHY_ID_CH182H7, WCH_PHY_ID_MASK },
	{ PHY_ID_CH182H8, WCH_PHY_ID_MASK },
	{}
};

MODULE_DEVICE_TABLE(mdio, wch_tbl);

MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_VERSION(VERSION_DESC);
MODULE_LICENSE("GPL");
