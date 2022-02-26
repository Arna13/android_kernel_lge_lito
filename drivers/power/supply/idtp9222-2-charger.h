/*
 * IDTP9222-2 Wireless Power Receiver driver
 *
 * Copyright (C) 2020 LG Electronics, Inc
 *
 */

#ifndef __IDTP9222_2_CHARGER_H
#define __IDTP9222_2_CHARGER_H

/* Mask/Bit helpers */
#define _IDT_MASK(BITS, POS) \
	((unsigned char)(((1 << (BITS)) - 1) << (POS)))
#define IDT_MASK(LEFT_BIT_POS, RIGHT_BIT_POS) \
	_IDT_MASK((LEFT_BIT_POS) - (RIGHT_BIT_POS) + 1, \
		(RIGHT_BIT_POS))

#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/string.h>
#include <linux/of_gpio.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/interrupt.h>
#include <linux/pmic-voter.h>
#include <linux/power_supply.h>
#include "qcom/smb5-lib.h"
#ifdef CONFIG_LGE_PM_VENEER_PSY
#include "lge/veneer-primitives.h"
#endif

// Constants
#define IDTP9222_NAME_COMPATIBLE	"idt,p9222-charger"
#define IDTP9222_NAME_DRIVER		"idtp9222-charger"
#define IDTP9222_NAME_PSY		"wireless"

// Register addresses
#define REG_ADDR_FIRMWARE       0x06
#define REG_ADDR_STATUS_L       0x34
#define REG_ADDR_STATUS_H       0x35
#define REG_ADDR_INT_L          0x36
#define REG_ADDR_INT_H          0x37
#define REG_ADDR_OPMODE         0x3F
#define REG_ADDR_CHGSTAT        0x4E
#define REG_ADDR_EPT            0x4F
#define REG_ADDR_VADC_L         0x50
#define REG_ADDR_VADC_H         0x51
#define REG_ADDR_VOUT           0x52
#define REG_ADDR_VRECT_L        0x54
#define REG_ADDR_VRECT_H        0x55
#define REG_ADDR_IADC_L         0x58
#define REG_ADDR_IADC_H         0x59
#define REG_ADDR_OPFREQ_L       0x5C
#define REG_ADDR_OPFREQ_H       0x5D
#define REG_ADDR_COMMAND_L      0x62
#define REG_ADDR_COMMAND_H      0x63
#define REG_ADDR_GUARPWR        0xB4
#define REG_ADDR_POTPWR         0xB5
#define REG_ADDR_MPREQNP        0xBD
#define REG_ADDR_MPREQMP        0xBE
#define REG_ADDR_MPVRCALM1_L    0xE4
#define REG_ADDR_MPVRCALM1_H    0xE5
#define REG_ADDR_SPECREV        0x105
#define REG_ADDR_TXID           0x106

// For VOUT register
#define VOUT_V5P5               0x14
#define VOUT_V9P0               0x37
#define VOUT_V10P0              0x41
#define VOUT_V12P0              0x55
// For Guaranteed Power Register
#define POWER_10W               0x14
#define POWER_15W               0x1E
// For Operation mode register
#define OPMODE_MASK             IDT_MASK(7, 5)
#define OPMODE_SHIFT            5
#define OPMODE_AC_MISSING       0x0
#define OPMODE_WPC_BPP          0x1
#define OPMODE_WPC_EPP          0x2
#define OPMODE_PMA_SR1          0x3
#define OPMODE_PMA_SR1E         0x4
#define OPMODE_UNKNOWN          0x7
// For Status register
#define STAT_VOUT_SHIFT         7
#define EXTENDED_MODE           BIT(4)
// For command register
#define SEND_CHGSTAT            BIT(4)
#define SEND_EPT                BIT(3)
#define MCU_RESET               BIT(2)
// For EPT register
#define EPT_BY_EOC                      1
#define EPT_BY_OVERTEMP                 3
#define EPT_BY_NORESPONSE               8
#define EPT_BY_RESTART_POWER_TRANSFER   11

// For votables
#define DISABLE_BY_USB		"DISABLE_BY_USB"
#define DISABLE_BY_WA		"DISABLE_BY_WA"
#define DISABLE_BY_EOC		"DISABLE_BY_EOC"
#define USER_VOTER			"USER_VOTER"
#define WLC_CS100_VOTER		"WLC_CS100_VOTER"
#define WLC_THERMAL_VOTER	"WLC_THERMAL_VOTER"
#define WLC_MIN_VOTER		"WLC_MIN_VOTER"

// For Static Variables
#define TIMER_OVERHEAT_MS	3000
#define UNVOTING_TIMER_MS	5000
#define OFFLINE_TIMER_MS	5000
#define RECOVERY_TIMER_MS	1000
#define WORKAROUND_TXID		0x63
#define CONF_MIN_VOL_RANGE  3500000
#define I2C_RETRY_COUNT		5
#define I2C_RETRY_DELAY		10

enum idtp9222_print {
	IDT_ASSERT	= BIT(0),
	IDT_ERROR	= BIT(1),
	IDT_INTERRUPT	= BIT(2),
	IDT_MONITOR	= BIT(3),
	IDT_REGISTER	= BIT(4),
	IDT_RETURN	= BIT(5),
	IDT_UPDATE	= BIT(6),
	IDT_VERBOSE	= BIT(7),
};

enum idtp9222_opmode {
	UNKNOWN = 0,
	WPC,
	PMA,
};

struct idtp9222_fod {
	char addr;
	char value;
};

struct idtp9222_struct {
	/* idtp9222 descripters */
	struct power_supply	*wlc_psy;
	struct power_supply	*psy_battery;
	struct power_supply	*psy_dc;
	struct i2c_client	*wlc_client;
	struct votable		*wlc_disable;
	struct votable		*wlc_voltage;
	struct votable		*wlc_suspend;
	struct votable		*dc_icl_votable;
	struct device 		*wlc_device;

	struct wakeup_source	wlc_wakelock;

	/* idtp9222 work struct */
	struct delayed_work	worker_onpad;
	struct delayed_work	timer_maxinput;
	struct delayed_work	timer_setoff;
	struct delayed_work	timer_overheat;
	struct delayed_work	polling_log;

	/* shadow status */
	bool			status_onpad;		// opposite to gpio_detached
	bool			status_vrect;		// show gpio_vrect
	bool			status_dcin;		// presence of DCIN on PMIC side
	bool			status_full;		// it means EoC, not 100%
	bool			status_overheat;
	bool			status_done;
	int			capacity;
	int			capacity_raw;
	int			temperature;
	int			txid;
	int			guarpwr;
	int			potpwr;
	int			typec_mode;

	/* onpad flags */
	enum idtp9222_opmode	opmode_type;		// WPC or PMA
	bool			opmode_midpower;	// 9W or 5W
	bool			opmode_certi;		// Certificated or not

	/* for controling GPIOs */
	int			gpio_idtfault;		// MSM GPIO #52, DIR_IN(interrupt)
	int			gpio_detached;		// MSM GPIO #37, DIR_IN(interrupt)
	int			gpio_vrect;		// MSM GPIO #XX, DIR_IN(interrupt)
	int			gpio_disabled;		// PMI GPIO #09, DIR_OUT(command)

	/* FOD parameters */
	struct idtp9222_fod	*fod_bpp;
	struct idtp9222_fod	*fod_epp;
	int			size_fodbpp;
	int			size_fodepp;

	/* configuration from DT */
	int			configure_bppcurr;
	int			configure_eppcurr;
	int			configure_bppvolt;
	int			configure_eppvolt;
	int			configure_fullcurr;
	int			configure_overheat;	// shutdown threshold for overheat
	int			configure_recharge;	// recharge soc from fg
	int			configure_full;
	bool			configure_dualdisplay;
	bool			configure_chargedone;
	bool			configure_sysfs;	// making sysfs or not (for debug)
};

static bool idtp9222_is_onpad(struct idtp9222_struct *idtp9222);
static bool idtp9222_is_enabled(struct idtp9222_struct *idtp9222);
static bool idtp9222_is_vrect(struct idtp9222_struct *idtp9222);
static bool idtp9222_is_full(struct idtp9222_struct *idtp9222);

#endif /* __IDTP9222_2_CHARGER_H */
