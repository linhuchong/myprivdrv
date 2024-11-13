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

//#include "common/common.h"
//#include "common/chip_ops.h"
#include "sprdwl.h"

/*
 * Sendmsg style is as follows:
 * "wcn_chr_ind_event,module=WIFI,
 * ref_count=%d,event_id=0x%x,
 * version=0x%x,event_content_len=%d,
 * char_info=**********"
 */
static int sprdwl_chr_sock_sendmsg(struct sprdwl_chr *chr, u8 *data)
{
	int ret;
	struct msghdr send_msg = {0};
	struct kvec send_vec = {0};

	send_vec.iov_base = data;
	send_vec.iov_len = 1024;

	pr_info("CHR: ready to sendmsg: %s\n", data);
	ret = kernel_sendmsg(chr->chr_sock, &send_msg, &send_vec, 1, CHR_BUF_SIZE);
	if (ret < 0) {
		pr_err("%s, CHR: sendmsg failed, release socket");
		if (chr->chr_sock)
			sock_release(chr->chr_sock);
		return -EINVAL;
	}

	return 0;
}

/* This function is used to fill chr_driver_params and sendbuf */
static void sprdwl_fill_chr_driver(struct chr_driver_params *chr_driver, u16 refcnt, u32 id,
			  	 u8 version, u8 content_len, u8 *content, u8 *buf)
{
	char temp[64] = {0};
	chr_driver->refcnt = refcnt;
	chr_driver->evt_id = id;
	chr_driver->version = version;
	chr_driver->evt_content_len = content_len;
	chr_driver->evt_content = content;

	if (id == EVT_CHR_DISC_LINK_LOSS || id == EVT_CHR_DISC_SYS_ERR
	    || id == EVT_CHR_OPEN_ERR) {
		snprintf(temp, sizeof(temp), "%u", (*content));
		snprintf(buf, CHR_BUF_SIZE, "wcn_chr_ind_event,module=WIFI,"
			"ref_count=%u,event_id=0x%x,version=0x%x,event_content_len=%d,"
			"char_info=%s", refcnt, id, version, (int)strlen(temp), temp);
	}
	return;
}

/* This function is used to report chr_disconnect evt from CP2 */
void sprdwl_chr_report_disconnect(struct sprdwl_vif *vif, u8 version,
				u32 evt_id, u32 evt_id_subtype,
				u8 evt_content_len, u8 *evt_content)
{
	int ret;
	u16 refcnt = 0;
	struct chr_linkloss_disc_error link_loss = {0};
	struct chr_system_disc_error system_err = {0};
	struct chr_driver_params chr_driver = {0};
	struct sprdwl_chr *chr = vif->priv->chr;
	u8 sendbuf[CHR_BUF_SIZE] = {0};
	u8 *pos = evt_content;

	if (*pos >= CHR_ARR_SIZE) {
		pr_info("%s, CHR: the content: %u is invalid, reporting not allowed",
			__func__, *pos);
		return;
	}

	if (evt_id == EVT_CHR_DISC_LINK_LOSS) {
		refcnt = ++chr->chr_refcnt->disc_linkloss_cnt[*pos];
		memcpy(&link_loss.reason_code, pos, sizeof(link_loss.reason_code));
		pr_info("%s: CHR: %s, ref_cnt=%u\n", __func__,
			link_loss.reason_code == 1 ? "Power off AP" : "Beacon Loss",
			refcnt);
	} else if (evt_id == EVT_CHR_DISC_SYS_ERR) {
		refcnt = ++chr->chr_refcnt->disc_systerr_cnt[*pos];
		memcpy(&system_err.reason_code, pos, sizeof(system_err.reason_code));
		pr_info("%s: CHR: SYSTEM_ERR_DISCONNECT, ref_cnt=%u\n", __func__, refcnt);
	}

	sprdwl_fill_chr_driver(&chr_driver, refcnt, evt_id, version,
			     evt_content_len, evt_content, sendbuf);

	if (chr->chr_sock) {
		ret = sprdwl_chr_sock_sendmsg(chr, sendbuf);
		if (ret)
			pr_err("CHR: wifi_driver_sendmsg failed with 0x%x\n", evt_id);
	} else {
		pr_err("CHR: connect been closed, can not send msg to server");
	}

	return;
}

/* This function is used to report chr_open_error evt from driver */
void sprdwl_chr_report_open_error(struct sprdwl_chr *chr, u32 evt_id, u8 err_code)
{
	int ret;
	u16 refcnt = 0;
	struct chr_open_error open_error = {0};
	struct chr_driver_params chr_driver = {0};
	u8 sendbuf[CHR_BUF_SIZE] = {0};

	if (err_code >= CHR_ARR_SIZE) {
		pr_info("%s, CHR: the err_code: %u is invalid, reporting not allowed",
			__func__, err_code);
		return;
	}

	refcnt = ++chr->chr_refcnt->open_err_cnt[err_code];
	open_error.reason_code = err_code;

	sprdwl_fill_chr_driver(&chr_driver, refcnt, evt_id, CHR_VERSION, 1, &err_code, sendbuf);

	if (chr->chr_sock) {
		ret = sprdwl_chr_sock_sendmsg(chr, sendbuf);
		if (ret)
			pr_err("CHR: wifi_driver_sendmsg failed with 0x%x\n", evt_id);
	} else {
		pr_err("CHR: connect been closed, can not send msg to server");
	}

	pr_info("%s: CHR: %s, ref_cnt=%u\n", __func__,
		open_error.reason_code == 0 ? "Power_on Err" : "Download_ini Err",
		refcnt);
	return;
}

/* This function is used to get the key val from string */
static inline void sprdwl_chr_get_cmdval(u32 *val, u8 *pos, u8 *key, int octal)
{
	u8 *temp = strstr(pos, key);

	if (!temp) {
		pr_info("%s, CHR: %s failed\n", __func__, key);
		return;
	}
	temp += strlen(key);

	*val = simple_strtoul(temp, NULL, octal);
}

/* this function is used to decode string */
static int sprdwl_chr_decode_str(struct chr_cmd *cmd_set, u8 *data)
{
	char temp_str[CHR_BUF_SIZE] = {0};
	u8 *pos = data;

	if (strstr(pos, "wcn_chr_set_event")) {
		memcpy(cmd_set->evt_type, pos, strlen("wcn_chr_set_event"));
		pos += strlen("wcn_chr_set_event,");
	}
	if (strstr(pos, "module=WIFI")) {
		memcpy(cmd_set->module, pos + strlen("module="), strlen("WIFI"));
		pos += strlen("module=WIFI,");
	}

	sprdwl_chr_get_cmdval(&cmd_set->evt_id, pos, "event_id=", 16);
	sprdwl_chr_get_cmdval(&cmd_set->set, pos, "set=", 10);
	sprdwl_chr_get_cmdval(&cmd_set->maxcount, pos, "maxcount=", 16);
	sprdwl_chr_get_cmdval(&cmd_set->timerlimit, pos, "tlimit=", 16);

	pr_info("CHR: decode_str: %s, %s, %#x, %d, %#x, %#x\n",
		cmd_set->evt_type, cmd_set->module, cmd_set->evt_id,
		cmd_set->set, cmd_set->maxcount, cmd_set->timerlimit);

	snprintf(temp_str, CHR_BUF_SIZE, "wcn_chr_set_event,module=WIFI,"
		"event_id=0x%x,set=%d,maxcount=0x%x,tlimit=0x%x", cmd_set->evt_id,
		cmd_set->set, cmd_set->maxcount, cmd_set->timerlimit);

	return strlen(temp_str);
}

/* this function is used to reset chr->cmd_list*/
static void sprdwl_chr_rebuild_cmdlist(struct sprdwl_chr *chr, struct chr_cmd *cmd_set)
{
	u32 index;

	if (cmd_set->evt_id >= EVT_CHR_DRV_MIN && cmd_set->evt_id <= EVT_CHR_DRV_MAX) {
		index = cmd_set->evt_id - EVT_CHR_DRV_MIN;

		if (index >= CHR_ARR_SIZE) {
			pr_err("%s, CHR: index:%u is invalid\n", __func__, index);
			return;
		}

		if (cmd_set->set == 1 && !chr->drv_cmd_list[index].set) {
			chr->drv_len = chr->drv_len + 1;
			pr_info("%s, CHR: start monitoring evt_id:%#x, drv_len: %u",
				__func__, cmd_set->evt_id, chr->drv_len);
		} else if (cmd_set->set == 0 && chr->drv_cmd_list[index].set) {
			chr->drv_len = chr->drv_len - 1;
			pr_info("%s, CHR: stop monitoring evt_id:%#x, drv_len: %u",
				__func__, cmd_set->evt_id, chr->drv_len);
		} else {
			pr_info("%s, CHR: adjust evt's params evt_id:%#x, drv_len: %u",
				__func__, cmd_set->evt_id, chr->drv_len);
		}
		memcpy(&chr->drv_cmd_list[index], cmd_set, sizeof(struct chr_cmd));

	} else if (cmd_set->evt_id >= EVT_CHR_FW_MIN && cmd_set->evt_id <= EVT_CHR_FW_MAX) {
		index = cmd_set->evt_id - EVT_CHR_FW_MIN;

		if (index >= CHR_ARR_SIZE) {
			pr_err("%s, CHR: index:%u is invalid\n", __func__, index);
			return;
		}

		if (cmd_set->set == 1 && !chr->fw_cmd_list[index].set) {
			chr->fw_len = chr->fw_len + 1;
			pr_info("%s, CHR: start monitoring evt_id:%#x, fw_len: %u",
				__func__, cmd_set->evt_id, chr->fw_len);
		} else if (cmd_set->set == 0 && chr->fw_cmd_list[index].set) {
			chr->fw_len = chr->fw_len - 1;
			pr_info("%s, CHR: stop monitoring evt_id:%#x, fw_len: %u",
				__func__, cmd_set->evt_id, chr->fw_len);
		} else {
			pr_info("%s, CHR: adjust evt's params evt_id:%#x, fw_len: %u",
				__func__, cmd_set->evt_id, chr->fw_len);
		}
		memcpy(&chr->fw_cmd_list[index], cmd_set, sizeof(struct chr_cmd));
	}

	return;
}

/* this function is used to determing whether CHR is disable*/
static inline int sprdwl_chr_set_sockflag(struct sprdwl_chr *chr, u8 *data)
{
	if (!strcmp("wcn_chr_disable", data)) {
		chr->fw_len = 0;
		chr->drv_len = 0;
		chr->sock_flag = 2;
		memset(&chr->fw_cmd_list, 0, sizeof(chr->fw_cmd_list));
		memset(&chr->drv_cmd_list, 0, sizeof(chr->drv_cmd_list));
		pr_info("CHR: disable all chr_evt, sock_flag set %u", chr->sock_flag);
		return -1;
	}
	chr->sock_flag = 1;
	pr_info("CHR: enable chr_evt, sock_flag set %u", chr->sock_flag);

	return 0;
}

/* This funtions is equivalent to "in_aton", using  "in_aton" can't pass google's whitelist check */
__be32 transform_aton(const char *str)
{
	unsigned int l;
	unsigned int val;
	int i;

	l = 0;
	for (i = 0; i < 4; i++) {
		l <<= 8;
		if (*str != '\0') {
			val = 0;
			while (*str != '\0' && *str != '.' && *str != '\n') {
				val *= 10;
				val += *str - '0';
				str++;
			}
			l |= val;
			if (*str != '\0')
				str++;
		}
	}
	return htonl(l);
}

extern struct sprdwl_intf *g_intf;
extern bool sprdwl_chip_is_on(struct sprdwl_intf *intf);
extern int sprdwl_set_chr(struct sprdwl_chr *chr);
static int sprdwl_chr_client_thread(void *params)
{
	int ret, sbuf_len, buf_pos;
	struct sockaddr_in s_addr;
	int connect_limit = 0;
	char recv_buf[CHR_BUF_SIZE] = {0};
	struct msghdr recv_msg = {0};
	struct kvec recv_vec = {0};
	struct chr_cmd command = {0};
	struct socket *sock = NULL;
	struct sprdwl_chr *chr;
	struct sprdwl_priv *priv;
	struct sprdwl_intf *intf = g_intf;

	chr = (struct sprdwl_chr *)params;
	priv = chr->priv;
/*
 * After receiving disable_chr each time,it's necessary
 * to establish a new connection with the upper.
 */
retry:

	ret = sock_create_kern(&init_net, AF_INET, SOCK_STREAM, 0, &sock);
	if (ret < 0) {
		pr_err("CHR: sock_client create failed %d\n", ret);
		chr->chr_client_thread = NULL;
		return -EINVAL;
	}

	chr->chr_sock = sock;
	s_addr.sin_family = AF_INET;
	s_addr.sin_port = htons(4758);
	s_addr.sin_addr.s_addr = transform_aton("127.0.0.1");

	pr_info("%s, CHR: wait the server starting", __func__);
	/* Optimize:block here while server not ready */
	while (1) {
		if (chr->thread_exit || connect_limit++ >= CHR_CONNECT_LIMIT) {
			pr_info("%s, CHR: stop wait connect, go exit!", __func__);
			goto exit;
		}
		complete(&chr->socket_completed);
		msleep(1000);

		ret = sock->ops->connect(sock, (struct sockaddr *)&s_addr,
					 sizeof(s_addr), 0);

		if (!ret)
			break;
	}
	pr_info("CHR: wifi_client connected\n");

	connect_limit = 0;
	recv_vec.iov_base = recv_buf;
	recv_vec.iov_len = CHR_BUF_SIZE;
/*
 * sock_flag = 2 means have recevied disable_chr,
 * it will break out here to establish
 * a new connection with upper
 */
	while (!kthread_should_stop() && chr->sock_flag != 2) {
		buf_pos = 0;
		memset(recv_buf, 0, sizeof(recv_buf));
		memset(&recv_msg, 0, sizeof(recv_msg));
		pr_info("CHR: wait for recv_msg");
		ret = kernel_recvmsg(sock, &recv_msg, &recv_vec, 1, CHR_BUF_SIZE, 0);

		if (unlikely(chr->thread_exit))
			goto exit;
		/* when an unknown err occurs in kernel_recvmsg,
		* a large amount of information will be printfed
		* cyclically, affecting the use of "kernel.log".
		* So go to exit.
		*/
		if (unlikely(ret <= 0)) {
			pr_err("%s, CHR: kernel_recvmsg faild, go to exit", __func__);
			goto exit;
		}

		pr_info("%s, CHR: recvmsg: %s", __func__, recv_buf);
		pr_info("CHR: msg_len is %d", (int)strlen(recv_buf));

		/* Multiple chr_evt may be sended through a single string */
		while (ret && recv_buf[buf_pos]) {
			if (sprdwl_chr_set_sockflag(chr, recv_buf))
				break;

			sbuf_len = sprdwl_chr_decode_str(&command, recv_buf + buf_pos);

			sprdwl_chr_rebuild_cmdlist(chr, &command);

			buf_pos += sbuf_len;
			memset(&command, 0, sizeof(command));
		}
		/*
		 * only when Wi-Fi is open, will cmd be sent to CP2,
		 * otherwise cmd will be saved
		 */
		if (sprdwl_chip_is_on(intf)) {
			ret = sprdwl_set_chr(chr);
			if (ret)
				pr_err("%s, CHR: set chr_cmd to CP2 failed", __func__);
		} else {
			pr_info("%s, CHR: Drop set_chr_cmd in case of power off"
				"save buf in chr_cmdlist", __func__);
		}
	}

	if (sock)
		sock_release(sock);
	chr->sock_flag = 0;
	pr_info("%s, CHR: init socket, try to connect server\n", __func__);

	goto retry;

exit:

	chr->thread_exit = 0;
	chr->chr_sock = NULL;
	sock_release(sock);
	sock = NULL;
	complete(&chr->thread_completed);
	pr_info("%s, CHR: exit client_thread\n", __func__);

	return 0;
}

void sprdwl_chr_handle_power(struct sprdwl_chr *chr)
{
/*
 * If driver have receied enable_chr and
 * required to monitor open_err evt then
 * start reporting the evt when power on err
 */
	if (chr->open_err_flag) {
		if (chr->sock_flag == 1 && chr->drv_cmd_list[0].set)
			sprdwl_chr_report_open_error(chr, EVT_CHR_OPEN_ERR, chr->open_err_flag - 1);
		else
			pr_info("%s, CHR: open err appears, but chr module is closed\n", __func__);

		CHR_OPENERR_FLAGSET(&chr->open_err_flag, OPEN_ERR_INIT);
	}
	return;
}

void sprdwl_chr_handle_open(struct sprdwl_chr *chr)
{
	int ret;
	/*
	 * Every time Wi-Fi is turned off,
	 * CP2 will clean up the global valrables
	 * that record the chr_evt to be monitored
	 */
	if (chr->fw_len) {
		pr_info("%s, CHR: set chr to CP2 each time open", __func__);
		ret = sprdwl_set_chr(chr);
		if (ret)
			pr_err("%s, CHR: set chr_cmd to CP2 failed", __func__);
	}

	/* if created chr_client_thread falied in sprd_iface_probe, try to create here */
	if (!chr->chr_sock) {
		pr_info("CHR: Creating chr_client_thread\n");
		ret = sprdwl_chr_init(chr);
		if (ret) {
			pr_err("%s chr init failed: %d\n", __func__, ret);
		}
	}
	return;
}

struct sprdwl_chr *sprdwl_chr_handle_probe(struct sprdwl_priv *priv)
{
	int ret;
	struct sprdwl_chr *chr = NULL;

	/* int the chr struct */
	chr = kzalloc(sizeof(*chr), GFP_KERNEL);
	if (!chr) {
		pr_info("%s, CHR: kzalloc chr failed", __func__);
		return NULL;
	}

	priv->chr = chr;
	chr->priv = priv;

	if (!chr->chr_sock) {
		pr_info("CHR: Creating chr_client_thread\n");
		ret = sprdwl_chr_init(chr);
		if (ret) {
			pr_err("%s, CHR: chr init failed: %d\n", __func__, ret);
		}
	}
	return chr;
}

int sprdwl_chr_init(struct sprdwl_chr *chr)
{
	struct chr_refcnt_arr *refcnt = NULL;

	/* Only init chr_refcnt one time */
	if (!chr->chr_refcnt) {
		refcnt = kzalloc(sizeof(*refcnt), GFP_KERNEL);
		if (!refcnt) {
			pr_info("%s, kzalloc refcnt failed", __func__);
			return -ENOMEM;
		}
		chr->chr_refcnt = refcnt;
	}

	chr->chr_client_thread = NULL;
	chr->chr_sock = NULL;

	pr_info("%s, CHR: ready to init the chr_client_thread", __func__);
	chr->chr_client_thread = kthread_create(sprdwl_chr_client_thread, chr, "wifi_driver_chr");
	if (IS_ERR_OR_NULL(chr->chr_client_thread)) {
		pr_err("CHR: client thread create failed\n");
		return -1;
	}
	init_completion(&chr->socket_completed);
	init_completion(&chr->thread_completed);
	wake_up_process(chr->chr_client_thread);

	return 0;
}

void sprdwl_chr_deinit(struct sprdwl_chr *chr, int exit_type)
{
	int ret;

	if (!chr) {
		pr_err("%s, CHR: struct chr has been free!", __func__);
		return;
	}

	/* wait the sprdwl_chr_client_thread entering the connect blocking status */
	ret = wait_for_completion_timeout(&chr->socket_completed, CHR_WAIT_TIMEOUT);

	if (!ret) {
		pr_err("%s, CHR: don't wait for the chr-thread to"
			"enter the connect blocking state", __func__);
		goto exit;
	}

	if (chr->chr_client_thread && chr->chr_sock) {
		reinit_completion(&chr->thread_completed);
		chr->thread_exit = 1;
		/*
		 * when sprd_iface_remove is running, sprdwl_chr_thread may have just
		 * received the msg from upper and is processing it at this time,
		 * and it needs to wait for its processing to complete before
		 * re-entering blocking.Only REMOVE_DEINIT need to be do this.
		 * The max long time is 20ms;
		 */
		if (exit_type == REMOVE_DEINIT) {
			msleep(100);
			chr->chr_sock->ops->shutdown(chr->chr_sock, SHUT_RDWR);
		}
		/* wait the sprdwl_chr_client_thread exit */
		ret = wait_for_completion_timeout(&chr->thread_completed, CHR_WAIT_TIMEOUT);

		if (!ret) {
			pr_err("%s, CHR: don't wait for the chr-thread to"
			       "enter the recvmsg blocking state", __func__);
			return;
		}
	}
	chr->chr_client_thread = NULL;

	if (chr->chr_refcnt) {
		kfree(chr->chr_refcnt);
		chr->chr_refcnt = NULL;
	}
exit:
	kfree(chr);
	chr = NULL;
	pr_info("%s, CHR: kfree struct chr, chr modules has been rmmod\n", __func__);
	return;
}

