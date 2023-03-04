#define pr_fmt(fmt)	"[Display][sw43103-ops:%s:%d] " fmt, __func__, __LINE__

#include "dsi_panel.h"
#include "lge_ddic_ops_helper.h"
#include "cm/lge_color_manager.h"
#include "brightness/lge_brightness_def.h"
#include "lge_dsi_panel.h"

#define ADDR_PTLAR 0x30
#define ADDR_PLTAC 0x31
#define ADDR_RDDISPM 0x3F
#define ADDR_ERR_DETECT 0x9F
#define ADDR_WRIECTL 0x55
#define ADDR_PWRCTL3 0xC3

extern int lge_ddic_dsi_panel_tx_cmd_set(struct dsi_panel *panel,
				enum lge_ddic_dsi_cmd_set_type type);
extern int lge_mdss_dsi_panel_cmd_read(struct dsi_panel *panel,
					u8 cmd, int cnt, char* ret_buf);
extern char* get_payload_addr(struct dsi_panel *panel, enum lge_ddic_dsi_cmd_set_type type, int position);
extern int get_payload_cnt(struct dsi_panel *panel, enum lge_ddic_dsi_cmd_set_type type, int position);
extern int lge_backlight_device_update_status(struct backlight_device *bd);

extern int dsi_panel_set_backlight(struct dsi_panel *panel, u32 bl_lvl);

const struct drs_res_info sw43103_res[3] = {
	{"fhd", 0, 1080, 2460},
};

#define IDX_DG_CTRL1 1
#define REG_DG_CTRL1 0xB4
#define NUM_DG_CTRL1 11
#define START_DG_CTRL1 2

#define IDX_DG_CTRL2 2
#define REG_DG_CTRL2 0xB5
#define NUM_DG_CTRL2 28
#define START_DG_CTRL2 1

#define OFFSET_DG_UPPER 3
#define OFFSET_DG_LOWER 9

#define STEP_DG_PRESET 5
#define NUM_DG_PRESET  35

static int rgb_preset[STEP_DG_PRESET][RGB_ALL] = {
	{12,  1, 23},
	{ 6,  1, 17},
	{ 0,  1, 11},
	{ 0,  7, 11},
	{ 0, 13, 11}
};

static char dg_ctrl1_values[NUM_DG_PRESET][OFFSET_DG_UPPER] = {
	{0x03, 0x05, 0xAF},
	{0x03, 0x01, 0x6B},
	{0x03, 0x01, 0x6B},
	{0x03, 0x01, 0x6B},
	{0x03, 0x01, 0x6B},
	{0x03, 0x01, 0x6B},
	{0x03, 0x01, 0x6B},
	{0x03, 0x01, 0x6B},
	{0x03, 0x01, 0x6B},
	{0x03, 0x01, 0x6B},
	{0x03, 0x01, 0x6B},
	{0x03, 0x01, 0x6B},
	{0x03, 0x01, 0x6B},
	{0x03, 0x01, 0x6B},
	{0x03, 0x01, 0x6B},
	{0x03, 0x01, 0x6B},
	{0x03, 0x01, 0x6B},
	{0x03, 0x01, 0x6B},
	{0x03, 0x01, 0x6B},
	{0x03, 0x01, 0x6B},
	{0x03, 0x01, 0x6B},
	{0x03, 0x01, 0x6B},
	{0x03, 0x01, 0x6B},
	{0x03, 0x01, 0x6B},
	{0x03, 0x01, 0x6B},
	{0x03, 0x01, 0x6B},
	{0x03, 0x01, 0x6B},
	{0x03, 0x01, 0x6B},
	{0x03, 0x01, 0x6B},
	{0x03, 0x01, 0x6B},
	{0x03, 0x01, 0x6B},
	{0x03, 0x01, 0x6B},
	{0x03, 0x01, 0x6B},
	{0x03, 0x01, 0x6B},
	{0x03, 0x01, 0x6B}
};

static char dg_ctrl2_values[NUM_DG_PRESET][OFFSET_DG_LOWER] = {
	{0x00, 0xFF, 0x80, 0x00, 0x80, 0x00, 0x80, 0x00, 0x80},
	{0x00, 0xFB, 0x7F, 0xFF, 0x7E, 0xFE, 0x7D, 0xFD, 0x7C},
	{0x00, 0xF8, 0x7F, 0xFE, 0x7D, 0xFC, 0x7C, 0xFB, 0x7A},
	{0x00, 0xF5, 0x7E, 0xFD, 0x7C, 0xFB, 0x7A, 0xF9, 0x77},
	{0x00, 0xF2, 0x7E, 0xFC, 0x7B, 0xF9, 0x78, 0xF6, 0x75},
	{0x00, 0xF0, 0x7E, 0xFC, 0x7A, 0xF8, 0x76, 0xF4, 0x72},
	{0x00, 0xEC, 0x7D, 0xFB, 0x79, 0xF6, 0x74, 0xF2, 0x6F},
	{0x00, 0xE9, 0x7D, 0xFA, 0x77, 0xF5, 0x72, 0xEF, 0x6D},
	{0x00, 0xE6, 0x7C, 0xF9, 0x76, 0xF3, 0x70, 0xED, 0x6A},
	{0x00, 0xE3, 0x7C, 0xF9, 0x75, 0xF2, 0x6E, 0xEB, 0x67},
	{0x00, 0xE0, 0x7C, 0xF8, 0x74, 0xF0, 0x6C, 0xE8, 0x64},
	{0x00, 0xDD, 0x7C, 0xF7, 0x73, 0xEF, 0x6B, 0xE6, 0x62},
	{0x00, 0xDA, 0x7B, 0xF6, 0x72, 0xED, 0x68, 0xE4, 0x5F},
	{0x00, 0xD7, 0x7B, 0xF6, 0x71, 0xEC, 0x67, 0xE2, 0x5D},
	{0x00, 0xD4, 0x7A, 0xF5, 0x6F, 0xEA, 0x65, 0xDF, 0x5A},
	{0x00, 0xD1, 0x7A, 0xF4, 0x6E, 0xE9, 0x63, 0xDD, 0x58},
	{0x00, 0xCE, 0x79, 0xF3, 0x6D, 0xE7, 0x61, 0xDB, 0x55},
	{0x00, 0xCB, 0x79, 0xF3, 0x6C, 0xE6, 0x60, 0xD9, 0x53},
	{0x00, 0xC9, 0x79, 0xF2, 0x6B, 0xE4, 0x5E, 0xD7, 0x50},
	{0x00, 0xC6, 0x78, 0xF1, 0x6A, 0xE3, 0x5C, 0xD5, 0x4E},
	{0x00, 0xC3, 0x78, 0xF1, 0x69, 0xE2, 0x5A, 0xD3, 0x4B},
	{0x00, 0xC0, 0x78, 0xF0, 0x68, 0xE0, 0x58, 0xD1, 0x49},
	{0x00, 0xBD, 0x77, 0xEF, 0x67, 0xDF, 0x57, 0xCF, 0x46},
	{0x00, 0xBA, 0x77, 0xEE, 0x66, 0xDD, 0x55, 0xCC, 0x44},
	{0x00, 0xB6, 0x77, 0xEE, 0x65, 0xDB, 0x52, 0xC9, 0x40},
	{0x00, 0xB3, 0x76, 0xED, 0x63, 0xDA, 0x50, 0xC7, 0x3D},
	{0x00, 0xB0, 0x76, 0xEC, 0x62, 0xD8, 0x4F, 0xC5, 0x3B},
	{0x00, 0xAD, 0x76, 0xEB, 0x61, 0xD7, 0x4D, 0xC2, 0x38},
	{0x00, 0xAA, 0x75, 0xEB, 0x60, 0xD6, 0x4B, 0xC0, 0x36},
	{0x00, 0xA7, 0x75, 0xEA, 0x5F, 0xD4, 0x49, 0xBE, 0x33},
	{0x00, 0xA4, 0x75, 0xE9, 0x5E, 0xD3, 0x47, 0xBC, 0x30},
	{0x00, 0xA1, 0x74, 0xE9, 0x5D, 0xD1, 0x45, 0xBA, 0x2E},
	{0x00, 0x9E, 0x74, 0xE8, 0x5C, 0xD0, 0x43, 0xB7, 0x2B},
	{0x00, 0x9B, 0x74, 0xE7, 0x5B, 0xCE, 0x42, 0xB5, 0x29},
	{0x00, 0x98, 0x73, 0xE6, 0x59, 0xCD, 0x40, 0xB3, 0x26}
};

#define NUM_SAT_CTRL 5
#define OFFSET_SAT_CTRL 12
#define REG_SAT_CTRL 0xB9

#define NUM_HUE_CTRL 5
#define OFFSET_HUE_CTRL 12
#define REG_HUE_CTRL 0xBB

#define NUM_SHA_CTRL 5
#define OFFSET_SHA_CTRL 11
#define REG_SHA_CTRL 0xB2

static char sat_ctrl_values[NUM_SAT_CTRL][OFFSET_SAT_CTRL] = {
	{0x29, 0x21, 0x21, 0x2B, 0x25, 0x29, 0x2A, 0x2A, 0x2A, 0x21, 0x29, 0x27},
	{0x2F, 0x27, 0x27, 0x31, 0x2B, 0x2F, 0x30, 0x30, 0x30, 0x27, 0x2F, 0x2D},
	{0x35, 0x2D, 0x2D, 0x37, 0x31, 0x35, 0x36, 0x36, 0x36, 0x2D, 0x35, 0x33},
	{0x3B, 0x33, 0x33, 0x3D, 0x37, 0x3B, 0x3C, 0x3C, 0x3C, 0x33, 0x3B, 0x39},
	{0x41, 0x39, 0x39, 0x43, 0x3D, 0x41, 0x42, 0x42, 0x42, 0x39, 0x41, 0x3F}
};

static char hue_ctrl_values[NUM_HUE_CTRL][OFFSET_HUE_CTRL] = {
	{0x70, 0x4A, 0x6B, 0x85, 0x55, 0x60, 0x86, 0x80, 0x80, 0x80, 0x73, 0x62},
	{0x78, 0x52, 0x73, 0x8D, 0x5D, 0x68, 0x8E, 0x88, 0x88, 0x88, 0x7B, 0x6A},
	{0x80, 0x5A, 0x7B, 0x95, 0x65, 0x70, 0x96, 0x90, 0x90, 0x90, 0x83, 0x72},
	{0x88, 0x62, 0x83, 0x9D, 0x6D, 0x78, 0x9E, 0x98, 0x98, 0x98, 0x8B, 0x7A},
	{0x90, 0x6A, 0x8B, 0xA5, 0x75, 0x80, 0xA6, 0xA0, 0xA0, 0xA0, 0x93, 0x82}
};

static char sha_ctrl_values[NUM_SHA_CTRL][OFFSET_SHA_CTRL] = {
	{0x89, 0x01, 0x01, 0x88, 0x24, 0x68, 0x22, 0x20, 0x04, 0x00, 0x00},
	{0x89, 0x0F, 0x0F, 0x88, 0x24, 0x68, 0x22, 0x20, 0x04, 0x00, 0x00},
	{0x89, 0x1F, 0x1F, 0x88, 0x24, 0x68, 0x22, 0x20, 0x04, 0x00, 0x00},
	{0x89, 0x2F, 0x2F, 0x88, 0x24, 0x68, 0x22, 0x20, 0x04, 0x00, 0x00},
	{0x89, 0x2F, 0x3F, 0x88, 0x24, 0x68, 0x22, 0x20, 0x04, 0x00, 0x00}
};

static void lge_set_custom_rgb_sw43103(struct dsi_panel *panel, bool send_cmd)
{
	int i = 0;
	int red_index, green_index, blue_index = 0;
	char *dgctl1_payload = NULL;
	char *dgctl2_payload = NULL;

	if (panel == NULL) {
		pr_err("Invalid input\n");
		return;
	}

	mutex_lock(&panel->panel_lock);

	//cm_rgb_step : 0~11
	//rgb_index 0~11 + 0~12
	if (panel->lge.screen_mode == screen_mode_oled_natural) {
		pr_info("Set default rgb step for screen_mode: %d\n", panel->lge.screen_mode);
		red_index   = rgb_preset[panel->lge.cm_preset_step][RED];
		green_index = rgb_preset[panel->lge.cm_preset_step][GREEN];
		blue_index  = rgb_preset[panel->lge.cm_preset_step][BLUE];
	} else {
		red_index   = rgb_preset[panel->lge.cm_preset_step][RED] + panel->lge.cm_red_step;
		green_index = rgb_preset[panel->lge.cm_preset_step][GREEN] + panel->lge.cm_green_step;
		blue_index  = rgb_preset[panel->lge.cm_preset_step][BLUE] + panel->lge.cm_blue_step;
	}

	pr_info("red_index=(%d) green_index=(%d) blue_index=(%d)\n", red_index, green_index, blue_index);

	dgctl1_payload = get_payload_addr(panel, LGE_DDIC_DSI_DISP_DG_COMMAND_DUMMY, IDX_DG_CTRL1);
	dgctl2_payload = get_payload_addr(panel, LGE_DDIC_DSI_DISP_DG_COMMAND_DUMMY, IDX_DG_CTRL2);

	if (!dgctl1_payload || !dgctl2_payload) {
		pr_err("LGE_DDIC_DSI_DISP_DG_COMMAND_DUMMY is NULL\n");
		mutex_unlock(&panel->panel_lock);
		return;
	}

	/*
	*	RU: RED_UPPER, BU: BLUE_UPPER, GU: GREEN_UPPER
	*	RL: RED_LOWER, BL: BLUE_LOWER, GL: GREEN_LOWER
	*
	* CTRL1(B4h): RU#1~3 GU#1~3 BU#1~3
	* CTRL2(B5h): RL#4~12 GL#4~12 BL#4~12
	*/

	// For RGB UPPER CTRL1
	for (i = 0; i < OFFSET_DG_UPPER; i++) {
		dgctl1_payload[i+START_DG_CTRL1] = dg_ctrl1_values[red_index][i];  //payload_ctrl1[2][3][4]
		dgctl1_payload[i+START_DG_CTRL1+OFFSET_DG_UPPER] = dg_ctrl1_values[green_index][i]; //payload_ctrl1[5][6][7]
		dgctl1_payload[i+START_DG_CTRL1+OFFSET_DG_UPPER*BLUE] = dg_ctrl1_values[blue_index][i]; //payload_ctrl1[8][9][10]
	}

	// FOR RGB LOWER CTRL2
	for (i = 0; i < OFFSET_DG_LOWER; i++) {
		dgctl2_payload[i+START_DG_CTRL2] = dg_ctrl2_values[red_index][i]; //payload_ctrl2[1]~[9]
		dgctl2_payload[i+START_DG_CTRL2+OFFSET_DG_LOWER] = dg_ctrl2_values[green_index][i]; //payload_ctrl2[10]~[18]
		dgctl2_payload[i+START_DG_CTRL2+OFFSET_DG_LOWER*BLUE] = dg_ctrl2_values[blue_index][i]; //payload_ctrl2[19]~[27]
	}

	for (i = 0; i < NUM_DG_CTRL1; i++) {
		pr_debug("Reg:0x%02x [%d:0x%02x]\n", REG_DG_CTRL1, i, dgctl1_payload[i]);
	}

	for (i = 0; i < NUM_DG_CTRL2; i++) {
		pr_debug("Reg:0x%02x [%d:0x%02x]\n", REG_DG_CTRL2, i, dgctl2_payload[i]);
	}

	if (send_cmd) {
		lge_ddic_dsi_panel_tx_cmd_set(panel, LGE_DDIC_DSI_DISP_DG_COMMAND_DUMMY);
	}

	mutex_unlock(&panel->panel_lock);
	return;
}

static void lge_display_control_store_sw43103(struct dsi_panel *panel, bool send_cmd)
{
	char *dispctrl1_payload = NULL;

	if(!panel) {
		pr_err("panel not exist\n");
		return;
	}

	mutex_lock(&panel->panel_lock);

	dispctrl1_payload = get_payload_addr(panel, LGE_DDIC_DSI_DISP_CTRL_COMMAND_1, 1);

	if (!dispctrl1_payload) {
		pr_err("LGE_DDIC_DSI_DISP_CTRL_COMMAND is NULL\n");
		mutex_unlock(&panel->panel_lock);
		return;
	}

	dispctrl1_payload[1] &= 0x00;

	/* CM_EN */
	dispctrl1_payload[1] |= panel->lge.color_manager_status << 2;
	/* GM_EN */
	dispctrl1_payload[1] |= panel->lge.dgc_status << 7;
	/* SHARPEN */
	dispctrl1_payload[1] |= panel->lge.sharpness_status << 6;

	pr_info("ctrl-command-1: 0x%02x", dispctrl1_payload[1]);

	if (send_cmd) {
		lge_ddic_dsi_panel_tx_cmd_set(panel, LGE_DDIC_DSI_DISP_CTRL_COMMAND_1);
	}

	mutex_unlock(&panel->panel_lock);
	return;
}

static void lge_set_screen_tune_sw43103(struct dsi_panel *panel)
{
	int i;
	int sat_index = 0;
	int hue_index = 0;
	int sha_index = 0;

	char *sat_payload = NULL;
	char *hue_payload = NULL;
	char *sha_payload = NULL;

	mutex_lock(&panel->panel_lock);

	sat_payload = get_payload_addr(panel, LGE_DDIC_DSI_SET_SATURATION, 1);
	hue_payload = get_payload_addr(panel, LGE_DDIC_DSI_SET_HUE, 1);
	sha_payload = get_payload_addr(panel, LGE_DDIC_DSI_SET_SHARPNESS, 1);

	if (!sat_payload) {
		pr_err("LGE_DDIC_DSI_SET_SATURATION is NULL\n");
		mutex_unlock(&panel->panel_lock);
		return;
	}

	if (!hue_payload) {
		pr_err("LGE_DDIC_DSI_SET_HUE is NULL\n");
		mutex_unlock(&panel->panel_lock);
		return;
	}

	if (!sha_payload) {
		pr_err("LGE_DDIC_DSI_SET_SHARPNESS is NULL\n");
		mutex_unlock(&panel->panel_lock);
		return;
	}

	sat_index = panel->lge.sc_sat_step;
	hue_index = panel->lge.sc_hue_step;
	sha_index = panel->lge.sc_sha_step;

	// SATURATION CTRL
	for (i = 0; i < OFFSET_SAT_CTRL; i++) {
		sat_payload[i+1] = sat_ctrl_values[sat_index][i];
	}

	// HUE CTRL
	for (i = 0; i < OFFSET_HUE_CTRL; i++) {
		hue_payload[i+1] = hue_ctrl_values[hue_index][i];
	}

	// SHARPNESS CTRL
	for (i = 0; i < OFFSET_SHA_CTRL; i++) {
		sha_payload[i+1] = sha_ctrl_values[sha_index][i];
	}

	for (i = 0; i < OFFSET_SAT_CTRL; i++) {
		pr_debug("Reg:0x%02x [%d:0x%02x]\n", REG_SAT_CTRL, i, sat_payload[i]);
	}
	for (i = 0; i < OFFSET_HUE_CTRL; i++) {
		pr_debug("Reg:0x%02x [%d:0x%02x]\n", REG_HUE_CTRL, i, hue_payload[i]);
	}
	for (i = 0; i < OFFSET_SHA_CTRL; i++) {
		pr_debug("Reg:0x%02x [%d:0x%02x]\n", REG_SHA_CTRL, i, sha_payload[i]);
	}

	lge_ddic_dsi_panel_tx_cmd_set(panel, LGE_DDIC_DSI_SET_SATURATION);
	lge_ddic_dsi_panel_tx_cmd_set(panel, LGE_DDIC_DSI_SET_HUE);
	lge_ddic_dsi_panel_tx_cmd_set(panel, LGE_DDIC_DSI_SET_SHARPNESS);

	mutex_unlock(&panel->panel_lock);

	return;
}

static void lge_set_screen_mode_sw43103(struct dsi_panel *panel, bool send_cmd)
{
	mutex_lock(&panel->panel_lock);

	pr_info("screen_mode %d\n", panel->lge.screen_mode);

	switch (panel->lge.screen_mode) {
	case screen_mode_oled_natural:
		panel->lge.dgc_status = 0x01;
		panel->lge.sharpness_status = 0x00;
		pr_info("natural mode\n");
		lge_ddic_dsi_panel_tx_cmd_set(panel, LGE_DDIC_DSI_CM_NATURAL);

		mutex_unlock(&panel->panel_lock);
		lge_set_custom_rgb_sw43103(panel, send_cmd);
		mutex_lock(&panel->panel_lock);
		break;
	case screen_mode_oled_vivid:
		panel->lge.dgc_status = 0x00;
		panel->lge.sharpness_status = 0x00;
		pr_info("vivid mode\n");
		lge_ddic_dsi_panel_tx_cmd_set(panel, LGE_DDIC_DSI_CM_VIVID);
		break;
	case screen_mode_oled_cinema:
		panel->lge.dgc_status = 0x01;
		panel->lge.sharpness_status = 0x00;
		pr_info("cinema mode\n");
		lge_ddic_dsi_panel_tx_cmd_set(panel, LGE_DDIC_DSI_CM_CINEMA);
		break;
	case screen_mode_oled_custom:
		pr_info("preset: %d, red: %d, green: %d, blue: %d\n",
				panel->lge.cm_preset_step, panel->lge.cm_red_step,
				panel->lge.cm_green_step, panel->lge.cm_blue_step);
		pr_info("saturation: %d, hue: %d, sharpness: %d\n",
				panel->lge.sc_sat_step, panel->lge.sc_hue_step, panel->lge.sc_sha_step);

		panel->lge.dgc_status = 0x01;
		panel->lge.sharpness_status = 0x01;

		lge_ddic_dsi_panel_tx_cmd_set(panel, LGE_DDIC_DSI_SET_SATURATION_DEFAULT);
		lge_ddic_dsi_panel_tx_cmd_set(panel, LGE_DDIC_DSI_SET_HUE_DEFAULT);
		lge_ddic_dsi_panel_tx_cmd_set(panel, LGE_DDIC_DSI_SET_WB_DEFAULT);

		mutex_unlock(&panel->panel_lock);
		lge_set_screen_tune_sw43103(panel);
		lge_set_custom_rgb_sw43103(panel, send_cmd);
		mutex_lock(&panel->panel_lock);
		break;
	default:
		panel->lge.dgc_status = 0x01;
		panel->lge.sharpness_status = 0x00;
		lge_ddic_dsi_panel_tx_cmd_set(panel, LGE_DDIC_DSI_SET_SATURATION_DEFAULT);
		lge_ddic_dsi_panel_tx_cmd_set(panel, LGE_DDIC_DSI_SET_HUE_DEFAULT);
		lge_ddic_dsi_panel_tx_cmd_set(panel, LGE_DDIC_DSI_SET_WB_DEFAULT);
		break;
	}

	mutex_unlock(&panel->panel_lock);
	lge_display_control_store_sw43103(panel, send_cmd);
	return;
}

static void lge_set_fp_lhbm_sw43103(struct dsi_panel *panel, int input)
{
	bool fp_aod_ctrl = false;

	mutex_lock(&panel->panel_lock);
	fp_aod_ctrl = lge_dsi_panel_is_power_on_lp(panel);

	if (panel == NULL) {
		mutex_unlock(&panel->panel_lock);
		pr_err("null ptr\n");
		return;
	}

	if((panel->lge.forced_lhbm == false) && (input == panel->lge.old_fp_lhbm_mode)) {
		mutex_unlock(&panel->panel_lock);
		pr_info("requested same state=%d\n", panel->lge.old_fp_lhbm_mode);
		return;
	}
	pr_err("lge_set_fp_lhbm_sw43103\n");

	switch (input) {
	case LGE_FP_LHBM_READY:
		panel->lge.lhbm_ready_enable = true;
		lge_ddic_dsi_panel_tx_cmd_set(panel, LGE_DDIC_DSI_FP_LHBM_READY);	//Enable dimming
		pr_info("[LHBM READY] set max brightness 0x6A9\n");
	break;
	case LGE_FP_LHBM_ON:
	case LGE_FP_LHBM_SM_ON:
	case LGE_FP_LHBM_FORCED_ON:
		if (fp_aod_ctrl) {
			lge_ddic_dsi_panel_tx_cmd_set(panel, LGE_DDIC_DSI_FP_LHBM_AOD_TO_FPS);
			pr_info("AOD to Normal mode\n");
		}
		pr_info("[LHBM ON] set max brightness 0x6A9\n");
		lge_ddic_dsi_panel_tx_cmd_set(panel, LGE_DDIC_DSI_FP_LHBM_ON);
		if (input == LGE_FP_LHBM_FORCED_ON) {
			if (panel->lge.old_fp_lhbm_mode == LGE_FP_LHBM_FORCED_ON)
				panel->lge.forced_lhbm = false;
			else
				panel->lge.forced_lhbm = true;
		}
	break;
	case LGE_FP_LHBM_OFF:
	case LGE_FP_LHBM_SM_OFF:
	case LGE_FP_LHBM_FORCED_OFF:
		lge_ddic_dsi_panel_tx_cmd_set(panel, LGE_DDIC_DSI_FP_LHBM_OFF);
		if (fp_aod_ctrl) {
			lge_ddic_dsi_panel_tx_cmd_set(panel, LGE_DDIC_DSI_FP_LHBM_FPS_TO_AOD);
			pr_info("Normal to AOD mode\n");
		}
		if (!panel->lge.lhbm_ready_enable) {
			pr_info("[LHBM OFF] current brightness = %d\n", panel->bl_config.bl_level);
			dsi_panel_set_backlight(panel, panel->bl_config.bl_level);
		}
		panel->lge.forced_lhbm = false;
	break;
	case LGE_FP_LHBM_EXIT:
	case LGE_FP_LHBM_FORCED_EXIT:
		pr_info("[LHBM EXIT] current brightness = %d\n", panel->bl_config.bl_level);
		panel->lge.lhbm_ready_enable = false;
		panel->lge.forced_lhbm = false;
		dsi_panel_set_backlight(panel, panel->bl_config.bl_level);

		lge_ddic_dsi_panel_tx_cmd_set(panel, LGE_DDIC_DSI_FP_LHBM_OFF);		//force off to avoid abnormal case
		lge_ddic_dsi_panel_tx_cmd_set(panel, LGE_DDIC_DSI_FP_LHBM_EXIT);	//Disable dimming

		if (lge_dsi_panel_is_power_on_interactive(panel)) {
			mutex_unlock(&panel->panel_lock);
			lge_set_screen_mode_sw43103(panel,true);
			mutex_lock(&panel->panel_lock);
		}
	break;
	default:
		pr_info("Not ready for another lhbm mode = %d\n", input);
	break;
	}

	panel->lge.old_fp_lhbm_mode = input;
	mutex_unlock(&panel->panel_lock);
	pr_info("set lhbm mode : %d \n", panel->lge.old_fp_lhbm_mode);

	return;
}

static void lge_set_video_enhancement_sw43103(struct dsi_panel *panel, int input)
{
	int rc = 0;

	mutex_lock(&panel->panel_lock);

	if (input) {
		rc = lge_ddic_dsi_panel_tx_cmd_set(panel, LGE_DDIC_DSI_VIDEO_ENHANCEMENT_ON);
		if (rc)
			pr_err("failed to send VIDEO_ENHANCEMENT_ON cmd, rc=%d\n", rc);
	}
	else {
		rc = lge_ddic_dsi_panel_tx_cmd_set(panel, LGE_DDIC_DSI_VIDEO_ENHANCEMENT_OFF);
		if (rc)
			pr_err("failed to send VIDEO_ENHANCEMENT_OFF cmd, rc=%d\n", rc);
		mutex_unlock(&panel->panel_lock);
		lge_set_screen_mode_sw43103(panel,true);
		mutex_lock(&panel->panel_lock);
	}
	mutex_unlock(&panel->panel_lock);

	pr_info("send cmds to %s the video enhancer \n",
		(input == true) ? "enable" : "disable");

	lge_backlight_device_update_status(panel->bl_config.raw_bd);
}

static void lge_irc_control_store_sw43103(struct dsi_panel *panel, bool enable)
{
	char *payload = NULL;

	if (!panel) {
		pr_err("panel not exist\n");
		return;
	}

	mutex_lock(&panel->panel_lock);

	payload = get_payload_addr(panel, LGE_DDIC_DSI_IRC_CTRL, 1);
	if (!payload) {
		pr_err("LGE_DDIC_DSI_IRC_CTRL is NULL\n");
		mutex_unlock(&panel->panel_lock);
		return;
	}

	payload[1] &= 0x60;
	panel->lge.irc_current_state = enable;
	payload[1] |= panel->lge.irc_current_state;

	pr_info("irc-command: 0x%02x", payload[1]);

	lge_ddic_dsi_panel_tx_cmd_set(panel, LGE_DDIC_DSI_IRC_CTRL);

	mutex_unlock(&panel->panel_lock);
}

static int lge_hdr_mode_set_sw43103(struct dsi_panel *panel, int input)
{
	bool hdr_mode = ((input > 0) ? true : false);

	mutex_lock(&panel->panel_lock);
	if (hdr_mode) {
		panel->lge.color_manager_status = 0;
		panel->lge.dgc_status = 0x00;
	} else {
		panel->lge.color_manager_status = 1;
	}
	mutex_unlock(&panel->panel_lock);
	pr_info("hdr=%s, cm=%s dgc=%s\n", (hdr_mode ? "set" : "unset"),
			((panel->lge.color_manager_status == 1) ? "enabled" : "disabled"),
			((panel->lge.dgc_status == 1) ? "enabled" : "disabled"));

	if (hdr_mode) {
		lge_irc_control_store_sw43103(panel, false);
		lge_display_control_store_sw43103(panel, true);
	} else {
		lge_irc_control_store_sw43103(panel, true);
		lge_set_screen_mode_sw43103(panel, true);
	}

	lge_backlight_device_update_status(panel->bl_config.raw_bd);

	return 0;
}

static void lge_vr_lp_mode_set_sw43103(struct dsi_panel *panel, int input)
{
	int rc = 0;
	bool enable = false;

	mutex_lock(&panel->panel_lock);

	panel->lge.vr_lp_mode = input;
	enable = !!panel->lge.vr_lp_mode;

	if (enable) {
		rc = lge_ddic_dsi_panel_tx_cmd_set(panel, LGE_DDIC_DSI_SET_VR_MODE_ON);
	} else {
		rc = lge_ddic_dsi_panel_tx_cmd_set(panel, LGE_DDIC_DSI_SET_VR_MODE_OFF);
	}
	mutex_unlock(&panel->panel_lock);

	pr_info("send cmds to %s vr_lp_set \n",	(input == true) ? "enable" : "disable");
}

static void lge_set_tc_perf_sw43103(struct dsi_panel *panel, int input)
{
	bool tc_perf_mode = ((input > 0) ? true : false);

	mutex_lock(&panel->panel_lock);
	if (tc_perf_mode)
		lge_ddic_dsi_panel_tx_cmd_set(panel, LGE_DDIC_DSI_TC_PERF_ON_COMMAND);
	else
		lge_ddic_dsi_panel_tx_cmd_set(panel, LGE_DDIC_DSI_TC_PERF_OFF_COMMAND);
	mutex_unlock(&panel->panel_lock);

	pr_info("set tc perf %s\n", (tc_perf_mode) ? "enable" : "disable");
}

struct lge_ddic_ops sw43103_ops = {
	/* aod */
	.store_aod_area = store_aod_area,
	.prepare_aod_cmds = NULL,
	.prepare_aod_area = NULL,
	.lge_check_vert_black_line = NULL,
	.lge_check_vert_white_line = NULL,
	.lge_check_vert_line_restore = NULL,
	/* brightness */
	.lge_bc_dim_set = NULL,
	.lge_set_therm_dim = NULL,
	.lge_get_brightness_dim = NULL,
	.lge_set_brightness_dim = NULL,
	/* image quality */
	.hdr_mode_set = lge_hdr_mode_set_sw43103,
	.lge_set_custom_rgb = lge_set_custom_rgb_sw43103,
	.lge_display_control_store = lge_display_control_store_sw43103,
	.lge_set_screen_tune = lge_set_screen_tune_sw43103,
	.lge_set_screen_mode = lge_set_screen_mode_sw43103,
	.sharpness_set = NULL,
	.lge_set_true_view_mode = NULL,
	.lge_set_video_enhancement = lge_set_video_enhancement_sw43103,
	.lge_vr_lp_mode_set = lge_vr_lp_mode_set_sw43103,
	.lge_set_tc_perf = lge_set_tc_perf_sw43103,
	/* drs */
	.get_current_res = NULL,
	.get_support_res = NULL,
	/* bist */
	.bist_ctrl = NULL,
	.release_bist = NULL,
	/* error detect */
	.err_detect_work = NULL,
	.err_detect_irq_handler = NULL,
	.set_err_detect_mask = NULL,
	/* pps */
	.set_pps_cmds = NULL,
	.unset_pps_cmds = NULL,
	/* irc */
	.set_irc_default_state = NULL,
	.set_irc_state = NULL,
	.get_irc_state = NULL,
	/* lhbm */
	.lge_set_fp_lhbm = lge_set_fp_lhbm_sw43103,
};
