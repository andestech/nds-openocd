/***************************************************************************
 *   Copyright (C) 2013 by Andes Technology                                *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.           *
 ***************************************************************************/
#include "aice_usb_pack_format.h"

/* AICE commands' pack/unpack functions */
struct aice_usb_cmmd_htda {
	unsigned char cmd;
	unsigned char length;
	unsigned char addr;
};

struct aice_usb_cmmd_htdb {
	unsigned char cmd;
	unsigned char length;
	unsigned char addr[4];
};

struct aice_usb_cmmd_htdc {
	unsigned char cmd;
	unsigned char length;
	unsigned char addr;
	unsigned char byte_data[4];
};

struct aice_usb_cmmd_htdd {
	unsigned char cmd;
	unsigned char length;
	unsigned char addr[4];
	unsigned char byte_data[4];
};

struct aice_usb_cmmd_htdma {
	unsigned char cmd;
	unsigned char target;
	unsigned char length;
	unsigned char addr;
};

struct aice_usb_cmmd_htdmb {
	unsigned char cmd;
	unsigned char target;
	unsigned char length;
	unsigned char reserved;
	unsigned char addr[4];
};

struct aice_usb_cmmd_htdmc {
	unsigned char cmd;
	unsigned char target;
	unsigned char length;
	unsigned char addr;
	unsigned char byte_data[4];
};

struct aice_usb_cmmd_htdmd {
	unsigned char cmd;
	unsigned char target;
	unsigned char length;
	unsigned char reserved;
	unsigned char addr[4];
	unsigned char byte_data[4];
};

struct aice_usb_cmmd_dtha {
	unsigned char cmdack;
	unsigned char length;
	unsigned char byte_data[4];
};

struct aice_usb_cmmd_dthb {
	unsigned char cmdack;
	unsigned char length;
};

struct aice_usb_cmmd_dthma {
	unsigned char cmdack;
	unsigned char target;
	unsigned char length;
	unsigned char reserved;
	unsigned char byte_data[4];
};

struct aice_usb_cmmd_dthmb {
	unsigned char cmdack;
	unsigned char target;
	unsigned char length;
	unsigned char reserved;
};

struct aice_usb_cmmd_dthxrw {
	unsigned char cmdack;
	unsigned char attr[4];
	unsigned char length[4];    // DTHXR only
	unsigned char byte_data[4]; // DTHXR only
};

struct aice_usb_cmmd_htdxrw {
	unsigned char cmd;
	unsigned char attr[4];
	unsigned char addr[8];
	unsigned char length[4];
	unsigned char byte_data[4]; // HTDXW only
};

static unsigned int aice_usb_cmmd_size[] = {
	AICE_CMDSIZE_HTDA,
	AICE_CMDSIZE_HTDB,
	AICE_CMDSIZE_HTDC,
	AICE_CMDSIZE_HTDD,
	AICE_CMDSIZE_HTDMA,
	AICE_CMDSIZE_HTDMB,
	AICE_CMDSIZE_HTDMC,
	AICE_CMDSIZE_HTDMD,
	AICE_CMDSIZE_DTHA,
	AICE_CMDSIZE_DTHB,
	AICE_CMDSIZE_DTHMA,
	AICE_CMDSIZE_DTHMB,
	AICE_CMDSIZE_HTDXR,
	AICE_CMDSIZE_HTDXW,
	AICE_CMDSIZE_DTHXR,
	AICE_CMDSIZE_DTHXW,
};

static void aice_pack_usb_cmd_htda(struct aice_usb_cmmd_info *pusb_cmmd_info)
{
	if (pusb_cmmd_info->cmdtype == AICE_CMDTYPE_HTDA) {
		struct aice_usb_cmmd_htda *pusb_cmmd_htda = (struct aice_usb_cmmd_htda *)pusb_cmmd_info->pusb_buffer;
		pusb_cmmd_htda->cmd = pusb_cmmd_info->cmd;
		pusb_cmmd_htda->length = pusb_cmmd_info->length;
		pusb_cmmd_htda->addr = (unsigned char)(pusb_cmmd_info->addr & 0xFF);
	}
	else {
		struct aice_usb_cmmd_htdma *pusb_cmmd_htdma = (struct aice_usb_cmmd_htdma *)pusb_cmmd_info->pusb_buffer;
		pusb_cmmd_htdma->cmd = pusb_cmmd_info->cmd;
		pusb_cmmd_htdma->target = pusb_cmmd_info->target;
		pusb_cmmd_htdma->length = pusb_cmmd_info->length;
		pusb_cmmd_htdma->addr = (unsigned char)(pusb_cmmd_info->addr & 0xFF);
	}
}

static void aice_pack_usb_cmd_htdb(struct aice_usb_cmmd_info *pusb_cmmd_info)
{
	unsigned char *paddr;
	if (pusb_cmmd_info->cmdtype == AICE_CMDTYPE_HTDB) {
		struct aice_usb_cmmd_htdb *pusb_cmmd_htdb = (struct aice_usb_cmmd_htdb *)pusb_cmmd_info->pusb_buffer;
		pusb_cmmd_htdb->cmd = pusb_cmmd_info->cmd;
		pusb_cmmd_htdb->length = pusb_cmmd_info->length;
		paddr = (unsigned char *)&pusb_cmmd_htdb->addr[0];
	}
	else {
		struct aice_usb_cmmd_htdmb *pusb_cmmd_htdmb = (struct aice_usb_cmmd_htdmb *)pusb_cmmd_info->pusb_buffer;
		pusb_cmmd_htdmb->cmd = pusb_cmmd_info->cmd;
		pusb_cmmd_htdmb->target = pusb_cmmd_info->target;
		pusb_cmmd_htdmb->length = pusb_cmmd_info->length;
		pusb_cmmd_htdmb->reserved = 0;
		paddr = (unsigned char *)&pusb_cmmd_htdmb->addr[0];
	}

	paddr[0] = (unsigned char)((pusb_cmmd_info->addr >> 24) & 0xFF);
	paddr[1] = (unsigned char)((pusb_cmmd_info->addr >> 16) & 0xFF);
	paddr[2] = (unsigned char)((pusb_cmmd_info->addr >> 8) & 0xFF);
	paddr[3] = (unsigned char)(pusb_cmmd_info->addr & 0xFF);
}

static void aice_pack_usb_cmd_htdc(struct aice_usb_cmmd_info *pusb_cmmd_info)
{
	unsigned char *pbyte_data;
	unsigned int i, word_cnt;
	unsigned char *pword_data = pusb_cmmd_info->pword_data;

	if (pusb_cmmd_info->cmdtype == AICE_CMDTYPE_HTDC) {
		struct aice_usb_cmmd_htdc *pusb_cmmd_htdc = (struct aice_usb_cmmd_htdc *)pusb_cmmd_info->pusb_buffer;
		pbyte_data = (unsigned char *)&pusb_cmmd_htdc->byte_data[0];
		pusb_cmmd_htdc->cmd = pusb_cmmd_info->cmd;
		pusb_cmmd_htdc->length = pusb_cmmd_info->length;
		pusb_cmmd_htdc->addr = (unsigned char)(pusb_cmmd_info->addr & 0xFF);
	}
	else {
		struct aice_usb_cmmd_htdmc *pusb_cmmd_htdmc = (struct aice_usb_cmmd_htdmc *)pusb_cmmd_info->pusb_buffer;
		pbyte_data = (unsigned char *)&pusb_cmmd_htdmc->byte_data[0];
		pusb_cmmd_htdmc->cmd = pusb_cmmd_info->cmd;
		pusb_cmmd_htdmc->target = pusb_cmmd_info->target;
		pusb_cmmd_htdmc->length = pusb_cmmd_info->length;
		pusb_cmmd_htdmc->addr = (unsigned char)(pusb_cmmd_info->addr & 0xFF);
	}
	word_cnt = (pusb_cmmd_info->length+1);
	for (i = 0 ; i < word_cnt ; i++) {
		if (pusb_cmmd_info->access_little_endian == 0) {
			pbyte_data[3] = (unsigned char)pword_data[3];
			pbyte_data[2] = (unsigned char)pword_data[2];
			pbyte_data[1] = (unsigned char)pword_data[1];
			pbyte_data[0] = (unsigned char)pword_data[0];
		} else {
			pbyte_data[0] = (unsigned char)pword_data[3];
			pbyte_data[1] = (unsigned char)pword_data[2];
			pbyte_data[2] = (unsigned char)pword_data[1];
			pbyte_data[3] = (unsigned char)pword_data[0];
		}
		pword_data += 4;
		pbyte_data += 4;
	}
}

static void aice_pack_usb_cmd_htdd(struct aice_usb_cmmd_info *pusb_cmmd_info)
{
	unsigned char *pbyte_data, *paddr;
	unsigned char *pword_data = pusb_cmmd_info->pword_data;
	unsigned int i, word_cnt;

	if (pusb_cmmd_info->cmdtype == AICE_CMDTYPE_HTDD) {
		struct aice_usb_cmmd_htdd *pusb_cmmd_htdd = (struct aice_usb_cmmd_htdd *)pusb_cmmd_info->pusb_buffer;
		pbyte_data = (unsigned char *)&pusb_cmmd_htdd->byte_data[0];
		paddr = (unsigned char *)&pusb_cmmd_htdd->addr[0];
		pusb_cmmd_htdd->cmd = pusb_cmmd_info->cmd;
		pusb_cmmd_htdd->length = pusb_cmmd_info->length;
	}
	else {
		struct aice_usb_cmmd_htdmd *pusb_cmmd_htdmd = (struct aice_usb_cmmd_htdmd *)pusb_cmmd_info->pusb_buffer;
		pbyte_data = (unsigned char *)&pusb_cmmd_htdmd->byte_data[0];
		paddr = (unsigned char *)&pusb_cmmd_htdmd->addr[0];
		pusb_cmmd_htdmd->cmd = pusb_cmmd_info->cmd;
		pusb_cmmd_htdmd->target = pusb_cmmd_info->target;
		pusb_cmmd_htdmd->length = pusb_cmmd_info->length;
		pusb_cmmd_htdmd->reserved = 0;
	}

	paddr[0] = (unsigned char)((pusb_cmmd_info->addr >> 24) & 0xFF);
	paddr[1] = (unsigned char)((pusb_cmmd_info->addr >> 16) & 0xFF);
	paddr[2] = (unsigned char)((pusb_cmmd_info->addr >> 8) & 0xFF);
	paddr[3] = (unsigned char)(pusb_cmmd_info->addr & 0xFF);
	word_cnt = (pusb_cmmd_info->length+1);
	for (i = 0 ; i < word_cnt ; i++) {
		if (pusb_cmmd_info->access_little_endian == 0) {
			pbyte_data[3] = (unsigned char)pword_data[3];
			pbyte_data[2] = (unsigned char)pword_data[2];
			pbyte_data[1] = (unsigned char)pword_data[1];
			pbyte_data[0] = (unsigned char)pword_data[0];
		} else {
			pbyte_data[0] = (unsigned char)pword_data[3];
			pbyte_data[1] = (unsigned char)pword_data[2];
			pbyte_data[2] = (unsigned char)pword_data[1];
			pbyte_data[3] = (unsigned char)pword_data[0];
		}
		pword_data += 4;
		pbyte_data += 4;
	}
}

static void aice_pack_usb_cmd_dtha(struct aice_usb_cmmd_info *pusb_cmmd_info)
{
	unsigned char *pbyte_data;
	unsigned char *pword_data = pusb_cmmd_info->pword_data;
	unsigned int i, word_cnt, buf_cnt;

	buf_cnt = (pusb_cmmd_info->length+1);
	if (pusb_cmmd_info->cmdtype == AICE_CMDTYPE_DTHA) {
		struct aice_usb_cmmd_dtha *pusb_cmmd_dtha = (struct aice_usb_cmmd_dtha *)pusb_cmmd_info->pusb_buffer;
		pusb_cmmd_info->cmd = pusb_cmmd_dtha->cmdack;
		pusb_cmmd_info->length = pusb_cmmd_dtha->length;
		pbyte_data = (unsigned char *)&pusb_cmmd_dtha->byte_data[0];
	}
	else {
		struct aice_usb_cmmd_dthma *pusb_cmmd_dthma = (struct aice_usb_cmmd_dthma *)pusb_cmmd_info->pusb_buffer;
		pusb_cmmd_info->cmd = pusb_cmmd_dthma->cmdack;
		pusb_cmmd_info->target = pusb_cmmd_dthma->target;
		pusb_cmmd_info->length = pusb_cmmd_dthma->length;
		pbyte_data = (unsigned char *)&pusb_cmmd_dthma->byte_data[0];
	}
	word_cnt = (pusb_cmmd_info->length+1);
	if (word_cnt > buf_cnt)
		word_cnt = buf_cnt;
	for (i = 0 ; i < word_cnt ; i++) {
		if (pusb_cmmd_info->access_little_endian == 0) {
			pword_data[3] = (unsigned char)pbyte_data[3];
			pword_data[2] = (unsigned char)pbyte_data[2];
			pword_data[1] = (unsigned char)pbyte_data[1];
			pword_data[0] = (unsigned char)pbyte_data[0];
		} else {
			pword_data[0] = (unsigned char)pbyte_data[3];
			pword_data[1] = (unsigned char)pbyte_data[2];
			pword_data[2] = (unsigned char)pbyte_data[1];
			pword_data[3] = (unsigned char)pbyte_data[0];
		}
		pword_data += 4;
		pbyte_data += 4;
	}
}

static void aice_pack_usb_cmd_dthb(struct aice_usb_cmmd_info *pusb_cmmd_info)
{
	if (pusb_cmmd_info->cmdtype == AICE_CMDTYPE_DTHB) {
		struct aice_usb_cmmd_dthb *pusb_cmmd_dthb = (struct aice_usb_cmmd_dthb *)pusb_cmmd_info->pusb_buffer;
		pusb_cmmd_info->cmd = pusb_cmmd_dthb->cmdack;
		pusb_cmmd_info->length = pusb_cmmd_dthb->length;
	}
	else {
		struct aice_usb_cmmd_dthmb *pusb_cmmd_dthmb = (struct aice_usb_cmmd_dthmb *)pusb_cmmd_info->pusb_buffer;
		pusb_cmmd_info->cmd = pusb_cmmd_dthmb->cmdack;
		pusb_cmmd_info->target = pusb_cmmd_dthmb->target;
		pusb_cmmd_info->length = pusb_cmmd_dthmb->length;
	}
}

static void aice_pack_usb_cmd_dthxrw(struct aice_usb_cmmd_info *pusb_cmmd_info)
{
	struct aice_usb_cmmd_dthxrw *pusb_cmmd_dthxrw = (struct aice_usb_cmmd_dthxrw *)pusb_cmmd_info->pusb_buffer;
	pusb_cmmd_info->cmd = pusb_cmmd_dthxrw->cmdack;

	if (pusb_cmmd_info->cmdtype == AICE_CMDTYPE_DTHXW)
		return;

	unsigned char *pword_data = pusb_cmmd_info->pword_data;
	while(1) {
		unsigned char *pbyte_data = (unsigned char *)&pusb_cmmd_dthxrw->byte_data[0];
		unsigned int i, word_cnt;
		unsigned int decoded_bytes = 0;

		// update attr
		pusb_cmmd_info->attr = pusb_cmmd_dthxrw->attr[0];
		for (i = 1 ; i < 4 ; i++) {
			pusb_cmmd_info->attr <<= 8;
			pusb_cmmd_info->attr |= pusb_cmmd_dthxrw->attr[i];
		}
		// update length
		pusb_cmmd_info->length = pusb_cmmd_dthxrw->length[0];
		for (i = 1 ; i < 4 ; i++) {
			pusb_cmmd_info->length <<= 8;
			pusb_cmmd_info->length |= pusb_cmmd_dthxrw->length[i];
		}

		word_cnt = (pusb_cmmd_info->length);
		for (i = 0 ; i < word_cnt ; i++) {
			if (pusb_cmmd_info->access_little_endian == 0) {
				pword_data[3] = (unsigned char)pbyte_data[3];
				pword_data[2] = (unsigned char)pbyte_data[2];
				pword_data[1] = (unsigned char)pbyte_data[1];
				pword_data[0] = (unsigned char)pbyte_data[0];
			} else {
				pword_data[0] = (unsigned char)pbyte_data[3];
				pword_data[1] = (unsigned char)pbyte_data[2];
				pword_data[2] = (unsigned char)pbyte_data[1];
				pword_data[3] = (unsigned char)pbyte_data[0];
			}
			pword_data += 4;
			pbyte_data += 4;
		}
		//LOG_DEBUG("attr=0x%x, length=0x%x", pusb_cmmd_info->attr, pusb_cmmd_info->length);
		if (pusb_cmmd_info->attr & 0x01) {
			return;
		}
		decoded_bytes += 9; // DTHXR header
		decoded_bytes += (word_cnt << 2);
		pusb_cmmd_dthxrw = (struct aice_usb_cmmd_dthxrw *)&pusb_cmmd_info->pusb_buffer[decoded_bytes];
	}
}

static void aice_pack_usb_cmd_htdxrw(struct aice_usb_cmmd_info *pusb_cmmd_info)
{
	unsigned int i, word_cnt, get_data;
	struct aice_usb_cmmd_htdxrw *pusb_cmmd_htdxrw = (struct aice_usb_cmmd_htdxrw *)pusb_cmmd_info->pusb_buffer;
	unsigned char *pword_data = pusb_cmmd_info->pword_data;
	unsigned char *pbyte_data = (unsigned char *)&pusb_cmmd_htdxrw->byte_data[0];
	pusb_cmmd_htdxrw->cmd = pusb_cmmd_info->cmd;

	// update attr
	get_data = pusb_cmmd_info->attr;
	for (i = 0 ; i < 4; i++) {
		pusb_cmmd_htdxrw->attr[3-i] = (get_data & 0xFF);
		get_data >>= 8;
	}
	// update length
	get_data = pusb_cmmd_info->length;
	for (i = 0 ; i < 4; i++) {
		pusb_cmmd_htdxrw->length[3-i] = (get_data & 0xFF);
		get_data >>= 8;
	}
	// update addr
	get_data = pusb_cmmd_info->hi_addr;
	for (i = 0 ; i < 4; i++) {
		pusb_cmmd_htdxrw->addr[3-i] = (get_data & 0xFF);
		get_data >>= 8;
	}
	get_data = pusb_cmmd_info->lo_addr;
	for (i = 0 ; i < 4; i++) {
		pusb_cmmd_htdxrw->addr[7-i] = (get_data & 0xFF);
		get_data >>= 8;
	}

	if (pusb_cmmd_info->cmdtype == AICE_CMDTYPE_HTDXR)
		return;

	word_cnt = (pusb_cmmd_info->length);
	for (i = 0 ; i < word_cnt ; i++) {
		if (pusb_cmmd_info->access_little_endian == 0) {
			pbyte_data[3] = (unsigned char)pword_data[3];
			pbyte_data[2] = (unsigned char)pword_data[2];
			pbyte_data[1] = (unsigned char)pword_data[1];
			pbyte_data[0] = (unsigned char)pword_data[0];
		} else {
			pbyte_data[0] = (unsigned char)pword_data[3];
			pbyte_data[1] = (unsigned char)pword_data[2];
			pbyte_data[2] = (unsigned char)pword_data[1];
			pbyte_data[3] = (unsigned char)pword_data[0];
		}
		pword_data += 4;
		pbyte_data += 4;
	}
}

void aice_pack_usb_cmd(struct aice_usb_cmmd_info *pusb_cmmd_info)
{
	if ((pusb_cmmd_info->cmdtype == AICE_CMDTYPE_HTDA) ||
			(pusb_cmmd_info->cmdtype == AICE_CMDTYPE_HTDMA))
		aice_pack_usb_cmd_htda(pusb_cmmd_info);
	else if ((pusb_cmmd_info->cmdtype == AICE_CMDTYPE_HTDB) ||
					 (pusb_cmmd_info->cmdtype == AICE_CMDTYPE_HTDMB))
		aice_pack_usb_cmd_htdb(pusb_cmmd_info);
	else if ((pusb_cmmd_info->cmdtype == AICE_CMDTYPE_HTDC) ||
					 (pusb_cmmd_info->cmdtype == AICE_CMDTYPE_HTDMC))
		aice_pack_usb_cmd_htdc(pusb_cmmd_info);
	else if ((pusb_cmmd_info->cmdtype == AICE_CMDTYPE_HTDD) ||
					 (pusb_cmmd_info->cmdtype == AICE_CMDTYPE_HTDMD))
		aice_pack_usb_cmd_htdd(pusb_cmmd_info);
	else if ((pusb_cmmd_info->cmdtype == AICE_CMDTYPE_DTHA) ||
					 (pusb_cmmd_info->cmdtype == AICE_CMDTYPE_DTHMA))
		aice_pack_usb_cmd_dtha(pusb_cmmd_info);
	else if ((pusb_cmmd_info->cmdtype == AICE_CMDTYPE_DTHB) ||
					 (pusb_cmmd_info->cmdtype == AICE_CMDTYPE_DTHMB))
		aice_pack_usb_cmd_dthb(pusb_cmmd_info);
	else if ((pusb_cmmd_info->cmdtype == AICE_CMDTYPE_DTHXR) ||
					 (pusb_cmmd_info->cmdtype == AICE_CMDTYPE_DTHXW))
		aice_pack_usb_cmd_dthxrw(pusb_cmmd_info);
	else if ((pusb_cmmd_info->cmdtype == AICE_CMDTYPE_HTDXR) ||
					 (pusb_cmmd_info->cmdtype == AICE_CMDTYPE_HTDXW))
		aice_pack_usb_cmd_htdxrw(pusb_cmmd_info);
}

unsigned int aice_get_usb_cmd_size(unsigned int usb_cmmd_type)
{
	return aice_usb_cmmd_size[usb_cmmd_type];
}
