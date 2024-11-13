/*
* SPDX-FileCopyrightText: 2021-2022 Unisoc (Shanghai) Technologies Co., Ltd
* SPDX-License-Identifier: GPL-2.0
*
* Copyright 2021-2022 Unisoc (Shanghai) Technologies Co., Ltd
*
* This program is free software; you can redistribute it and/or modify it
* under the terms of version 2 of the GNU General Public License
* as published by the Free Software Foundation.
*/

#ifndef __CHR_H__
#define __CHR_H__

#include <linux/delay.h>
#include <linux/err.h>
#include <linux/inet.h>
#include <linux/kernel.h>
#include <linux/kthread.h>
#include <linux/net.h>
#include <linux/sched.h>
#include <net/net_namespace.h>
#include <net/sock.h>
#include <uapi/asm-generic/errno.h>
#include <uapi/linux/in.h>

#define CHR_VERSION			1
#define CHR_ARR_SIZE			64
#define CHR_BUF_SIZE			1024
#define CHR_CP2_DATA_LEN		11
#define CHR_WAIT_TIMEOUT		2000
#define CHR_CONNECT_LIMIT		10

#define CHR_OPENERR_FLAGSET(A, B) 	(*A = B)

/*
 * struct evt_chr- the chr_evt data format from CP2 uploading
 *
 * @version: reserve for future
 * @evt_id: the chr_evt's id
 * @evt_id_subtype: reserve for future
 * @evt_content_len: the evt_content len
 * @evt_content: point to the struct such as "struct chr_open_error",
 *  CP2 define the evt_content size is 100bytes
 */
struct evt_chr {
	u8 version;
	u32 evt_id;
	u32 evt_id_subtype;
	u8 evt_content_len;
	u8 *evt_content;
} __packed;

/* used by driver to store CHR params*/
struct chr_driver_params {
	u16 refcnt;
	u32 evt_id;
	u8 version;
	u8 evt_content_len;
	u8 *evt_content;
};

struct chr_open_error {
	u8 reason_code; /* 0 is power_on err, 1 is download_ini err*/
};

struct chr_linkloss_disc_error {
	u8 reason_code; /* 1 is device power off, 2 is beacon loss */
};

struct chr_system_disc_error {
	u8 reason_code;
};

/*
 * struct chr_cmd - the data format of the buffer recvived from upper layer
 *
 * @evt_type: include "wcn_chr_set_event" and
 *  "wcn_chr_request_event"
 * @module: include "module=BSP","module=GNSS",
 *  "module=BT","module=WIFI";
 * @evt_id: chr_evt id
 * @set: 0 is close, 1 is open
 * @maxcount: maximum reporting limit count
 * @timerlimit: maximum reporting limit timer
 */
struct chr_cmd {
	u8 evt_type[18];
	u8 module[12];
	u32 evt_id;
	u32 set;
	u32 maxcount;
	u32 timerlimit;
};

/*
 * struct chr_refcnt_arr - this struct stored the number of occurrences of diff evt
 *
 * @open_err_cnt: save the cnt of open_err evt, open_err_cnt[x]
 *  represent the cnt of reason code is 'x'
 * @disc_linkloss_cnt: save the cnt of disc_linkloss evt,
 *  disc_linkloss_cntt[x] same meaning as above
 * @disc_systerr_cnt: save the cnt of disc_systerr evt,
 *  disc_systerr_cnt[x] same meaning as above
 */
struct chr_refcnt_arr {
	u16 open_err_cnt[CHR_ARR_SIZE];
	u16 disc_linkloss_cnt[CHR_ARR_SIZE];
	u16 disc_systerr_cnt[CHR_ARR_SIZE];
};

/*
 * the flag just used in sprd_iface_set_power to
 * determine whether open_err evt has occurred
 */
enum OPEN_ERR_LIST {
	OPEN_ERR_INIT = 0,
	OPEN_ERR_POWER_ON,
	OPEN_ERR_DOWNLOAD_INI
};

/* The flag is used to determine where is calling sprd_chr_deinit*/
enum CHR_DEINIT_TYPE {
	PROBE_DEINIT = 0,
	REMOVE_DEINIT
};

/*
 * struct cmd_chr_mode - this struct is the data format sent by driver to CP2
 *
 * @on_flag: 0 is disable chr modules, 1 is enable
 * @version: tht verison of chr
 * @chr_evt_id: tht evt id of chr_evt
 */
struct cmd_chr_mode {
	u8 on_flag;
	u8 version;
	u32 chr_evt_id[10];
} __packed;

/* The following are the evt_id for each chr_evt */
enum REPORT_CHR_LIST {
	EVT_CHR_WIFI_MIN = 0x11501,

	/* Error From Driver */
	EVT_CHR_DRV_MIN = EVT_CHR_WIFI_MIN,

	EVT_CHR_OPEN_ERR = EVT_CHR_DRV_MIN,

	EVT_CHR_DRV_MAX = 0X13000,
	/* Error From CP2 */
	EVT_CHR_FW_MIN = 0X13001,

	EVT_CHR_DISC_LINK_LOSS = EVT_CHR_FW_MIN,
	EVT_CHR_DISC_SYS_ERR,

	EVT_CHR_FW_MAX = 0X15000,

	EVT_CHR_WIFI_MAX = EVT_CHR_FW_MAX
};

/*
 * struct sprd_chr - this struct is contains most of the variables of CHR modules
 *
 * @sock_flag: indicates the status of Wi-Fi Drv chr client
 *  0 means haven't received any messages,
 *  1 is have received messages about open chr_evt,
 *  2 is have received messages about close all chr_evt
 * @thread_exit: indicates whether the chr_thread should exit
 *  0 means thread keeps running
 *  1 is thread should exit
 * @open_err_flag: the flag is used to determine whether open_err has occurred
 *  1 means power on failed
 *  2 means download_ini failed
 *  can be referenced "enum OPEN_ERR_LIST"
 * @thread_completed: use with the "sprd_chr_deinit"
 * @socket_completed: use with the "sprd_chr_deinit"
 * @priv: struct priv
 * @hif: struct hif
 * @chr_client_thread: the pointer of chr_client_thread
 * @chr_refcnt_arr: stored the number of occurrences of diff evt
 *  can be referenced "struct chr_refcnt_arr"
 * @chr_sock: the chr sock of wifi driver
 * @fw_cmd_list: recvived from upper layer to stores the chr_buf for CP2
 *  fw_cmd_list[x] represent the property of chr_evt which evt_id is "EVT_CHR_FW_MIN+x"
 * @fw_len: the current number of CP2 chr_evt that need to be monitored
 * @drv_cmd_list: recvived from upper layer to stores the chr_buf for Wi-Fi Drv
 *  drv_cmd_list[x] represent the property of chr_evt which evt_id is "EVT_CHR_DRV_MIN+x"
 * @drv_len: the current number of Wi-Fi Drv chr_evt that need to be monitored
 */
struct sprd_chr {
	u8 sock_flag;
	u8 thread_exit;
	u8 open_err_flag;
	struct completion thread_completed;
	struct completion socket_completed;
	struct sprd_priv *priv;
	struct sprd_hif *hif;

	struct task_struct *chr_client_thread;
	struct chr_refcnt_arr *chr_refcnt;
	struct socket *chr_sock;

	struct chr_cmd fw_cmd_list[CHR_ARR_SIZE];
	u32 fw_len;
	struct chr_cmd drv_cmd_list[CHR_ARR_SIZE];
	u32 drv_len;
};


int sprd_chr_init(struct sprd_chr *chr);
void sprd_chr_deinit(struct sprd_chr *chr, int exit_type);

/* This function is used to report chr_disconnect evt from CP2 */
void sprd_chr_report_disconnect(struct sprd_vif *vif, u8 version,
				u32 evt_id, u32 evt_id_subtype,
				u8 evt_content_len, u8 *evt_content);
/* This function is used to report chr_open_error evt from driver */
void sprd_chr_report_open_error(struct sprd_chr *chr, u32 evt_id,
				u8 err_code);
void sprd_chr_handle_open(struct sprd_chr *chr);
void sprd_chr_handle_power(struct sprd_chr *chr);
struct sprd_chr *sprd_chr_handle_probe(struct sprd_hif *hif);

#endif
