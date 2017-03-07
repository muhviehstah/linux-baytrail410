/*
 * Support for Medifield PNW Camera Imaging ISP subsystem.
 *
 * Copyright (c) 2010 Intel Corporation. All Rights Reserved.
 *
 * Copyright (c) 2010 Silicon Hive www.siliconhive.com.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License version
 * 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 */
#ifndef __ATOMISP_INTERNAL_H__
#define __ATOMISP_INTERNAL_H__

#include <linux/atomisp_platform.h>
#include <linux/firmware.h>
#include <linux/kernel.h>
#include <linux/pm_qos.h>
#include <linux/idr.h>
#include <linux/slab.h>

#include <media/media-device.h>
#include <media/v4l2-subdev.h>

#include "ia_css_types.h"
#include "sh_css_legacy.h"

#include "atomisp_csi2.h"
#include "atomisp_file.h"
#include "atomisp_subdev.h"
#include "atomisp_tpg.h"

#include "gp_device.h"
#include "irq.h"

#define MULTI_STREAM_NUM	3
#define EXP_ID_WRAPAROUND       250

#define ATOMISP_MAJOR		0
#define ATOMISP_MINOR		5
#define ATOMISP_PATCHLEVEL	1

#define DRIVER_VERSION_STR	__stringify(ATOMISP_MAJOR) \
	"." __stringify(ATOMISP_MINOR) "." __stringify(ATOMISP_PATCHLEVEL)
#define DRIVER_VERSION		KERNEL_VERSION(ATOMISP_MAJOR, \
	ATOMISP_MINOR, ATOMISP_PATCHLEVEL)

#define ATOM_ISP_STEP_WIDTH	4
#define ATOM_ISP_STEP_HEIGHT	4

#define ATOM_ISP_MIN_WIDTH	4
#define ATOM_ISP_MIN_HEIGHT	4
#define ATOM_ISP_MAX_WIDTH	4352
#define ATOM_ISP_MAX_HEIGHT	3264

/* sub-QCIF resolution */
#define ATOM_RESOLUTION_SUBQCIF_WIDTH	128
#define ATOM_RESOLUTION_SUBQCIF_HEIGHT	96

#define ATOM_ISP_MAX_WIDTH_TMP	1280
#define ATOM_ISP_MAX_HEIGHT_TMP	720

#define ATOM_ISP_I2C_BUS_1	4
#define ATOM_ISP_I2C_BUS_2	5

#define ATOM_ISP_POWER_DOWN	0
#define ATOM_ISP_POWER_UP	1

#define ATOM_ISP_MAX_INPUTS	4

#define ATOMISP_SC_TYPE_SIZE	2

#define ATOMISP_ISP_TIMEOUT_DURATION		(2 * HZ) /* second */
#define ATOMISP_ISP_FILE_TIMEOUT_DURATION	(60 * HZ)
#define ATOMISP_ISP_MAX_TIMEOUT_COUNT	2

#define ATOMISP_CSS2_PIPE_MAX	2
#define ATOMISP_CSS_Q_DEPTH	3
#define ATOMISP_CSS_EVENTS_MAX  16
#define ATOMISP_CONT_RAW_FRAMES 10

#define ATOMISP_DELAYED_INIT_NOT_QUEUED	0
#define ATOMISP_DELAYED_INIT_QUEUED	1
#define ATOMISP_DELAYED_INIT_WORK_DONE	2
#define ATOMISP_DELAYED_INIT_DONE	2

#define ATOMISP_MIPI_BUFFER_SIZE	0x60000 // Intermediate MIPI Frame buffer size*32 bytes ~ 12MB

/*
 * Define how fast CPU should be able to serve ISP interrupts.
 * The bigger the value, the higher risk that the ISP is not
 * triggered sufficiently fast for it to process image during
 * vertical blanking time, increasing risk of dropped frames.
 * 1000 us is a reasonable value considering that the processing
 * time is typically ~2000 us.
 */
#define ATOMISP_MAX_ISR_LATENCY	1000



struct atomisp_css_event {
	enum ia_css_pipe_id pipe;
	struct ia_css_event event;
};

struct atomisp_input_subdev {
	unsigned int type;
	enum atomisp_camera_port port;
	int num_lanes;
	struct v4l2_subdev *camera;
	struct v4l2_subdev *motor;
	struct ia_css_morph_table *morph_table;
	struct ia_css_shading_table *shading_table;
	struct v4l2_frmsizeenum frame_size;

	/*
	 * To show this resource is used by
	 * which stream, in ISP multiple stream mode
	 */
	int used_by;
};

struct atomisp_freq_scaling_rule {
	unsigned int width;
	unsigned int height;
	unsigned short fps;
	unsigned int isp_freq;
	unsigned int run_mode;
};

enum atomisp_dfs_mode {
	ATOMISP_DFS_MODE_AUTO = 0,
	ATOMISP_DFS_MODE_LOW,
	ATOMISP_DFS_MODE_MAX,
};

struct atomisp_regs {
	/* PCI config space info */
	u16 pcicmdsts;
	u32 ispmmadr;
	u32 msicap;
	u32 msi_addr;
	u16 msi_data;
	u8 intr;
	u32 interrupt_control;
	u32 pmcs;
	u32 cg_dis;
	u32 i_control;

	/* I-Unit PHY related info */
	u32 csi_rcomp_config;
	u32 csi_afe_dly;
	u32 csi_control;

	/* New for MRFLD */
	u32 csi_afe_rcomp_config;
	u32 csi_afe_hs_control;
	u32 csi_deadline_control;
	u32 csi_access_viol;
};

struct atomisp_sw_contex {
	bool bypass;
	bool file_input;
	int  invalid_frame;
	int  invalid_vf_frame;
	int  invalid_s3a;
	int  invalid_dis;

	int power_state;
	int running_freq;
};

struct atomisp_acc_fw {
	struct ia_css_fw_info *fw;
	unsigned int handle;
	unsigned int flags;
	unsigned int type;
	struct {
		size_t length;
		unsigned long css_ptr;
	} args[N_IA_CSS_ISP_MEMORIES];
	struct list_head list;
};

struct atomisp_map {
	ia_css_ptr ptr;
	size_t length;
	struct list_head list;
	/* FIXME: should keep book which maps are currently used
	 * by binaries and not allow releasing those
	 * which are in use. Implement by reference counting.
	 */
};

#define ATOMISP_DEVICE_STREAMING_DISABLED	0
#define ATOMISP_DEVICE_STREAMING_ENABLED	1
#define ATOMISP_DEVICE_STREAMING_STOPPING	2
#define ATOMISP_DEVICE_STREAMING_STARTING       3


/*
 * ci device struct
 */
struct atomisp_device {
	struct pci_dev *pdev;
	struct device *dev;
	struct v4l2_device v4l2_dev;
	struct media_device media_dev;
	struct atomisp_platform_data *pdata;
	struct pci_dev *pci_root;
	const struct firmware *firmware;
	const struct firmware *aux_firmware;

	bool firmware_load_complete;

	struct pm_qos_request pm_qos;
	s32 max_isr_latency;

	struct {
		struct list_head fw;
		struct list_head memory_maps;
		struct ia_css_pipe *pipeline;
		bool extension_mode;
		struct ida ida;
		struct completion acc_done;
	} acc;

	struct ia_css_fw css_fw;

	/* ISP modules */
	struct atomisp_sub_device *isp_subdev;
	/*
	 * MRFLD has 3 CSI ports, while MFLD has only 2.
	 */
	struct atomisp_mipi_csi2_device csi2_port[ATOMISP_CAMERA_NR_PORTS];
	struct atomisp_tpg_device tpg;
	struct atomisp_file_device file_dev;

	/* Purpose of mutex is to protect and serialize use of isp data
	 * structures and css API calls. */
	struct mutex mutex;
	struct mutex vb2mutex;
	/*
	 * Serialise streamoff: mutex is dropped during streamoff to
	 * cancel the watchdog queue. MUST be acquired BEFORE
	 * "mutex".
	 */
	struct mutex streamoff_mutex;

	struct atomisp_input_subdev inputs[ATOM_ISP_MAX_INPUTS];
	struct v4l2_subdev *flash;
	struct v4l2_subdev *motor;

	struct atomisp_regs saved_regs;
	struct atomisp_sw_contex sw_contex;

	/* isp timeout status flag */
	bool isp_timeout;
	bool isp_fatal_error;
	struct workqueue_struct *wdt_work_queue;
	struct work_struct wdt_work;
	struct timer_list wdt;
	atomic_t wdt_count;
	unsigned int wdt_duration;	/* in jiffies */
	enum atomisp_frame_status fr_status;

	spinlock_t lock; /* Just for streaming below */

	bool need_gfx_throttle;

	int num_of_streams;
	int input_cnt;
	u64 raw_dmamask;

	/*
	 * Counts number of successful registered sub_devs
	 */
	int detected_snr_cnt;
	int mmu_base_addr;

	//fastboot related flags
	bool switch_fw_on_streamoff ;
	bool firmware_switched;
};


#define v4l2_dev_to_atomisp_device(dev) \
	container_of(dev, struct atomisp_device, v4l2_dev)

extern struct v4l2_device atomisp_dev;

extern void *atomisp_kernel_malloc(size_t bytes);

extern void atomisp_kernel_free(void *ptr);

#define MRFLD_FW_PATH_SMALL   "iaisp_2400_css.bin.small"
#define MRFLD_FW_PATH_BIG   "iaisp_2400_css.bin.big"
#define MRFLD_FW_PATH   MRFLD_FW_PATH_BIG

#endif /* __ATOMISP_INTERNAL_H__ */
