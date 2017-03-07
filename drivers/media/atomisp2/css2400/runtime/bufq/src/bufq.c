/*
 * Support for Intel Camera Imaging ISP subsystem.
 *
 * Copyright (c) 2010 - 2014 Intel Corporation. All Rights Reserved.
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

#include "assert_support.h"		/* assert */
#include "sp.h"
#include "ia_css_bufq.h"		/* Bufq API's */
#include "ia_css_queue.h"		/* ia_css_queue_t */
#include "sw_event_global.h"		/* Event IDs.*/
#include "ia_css_eventq.h"		/* ia_css_eventq_recv()*/
#include "ia_css_debug.h"		/* ia_css_debug_dtrace*/
#include "sh_css_internal.h"		/* sh_css_queue_type */
#include "sp_local.h"			/* sp_address_of */
#include "ia_css_util.h" 		/* ia_css_convert_errno()*/
#include "sh_css_firmware.h"		/* sh_css_sp_fw*/




/*********************************************************/
/* Global Queue objects used by CSS                      */
/*********************************************************/
struct sh_css_queues {
	/* Host2SP buffer queue */
	ia_css_queue_t host2sp_buffer_queue_handles
		[SH_CSS_MAX_SP_THREADS][SH_CSS_MAX_NUM_QUEUES];
	/* SP2Host buffer queue */
	ia_css_queue_t sp2host_buffer_queue_handles
		[SH_CSS_MAX_NUM_QUEUES];

	/* Host2SP event queue */
	ia_css_queue_t host2sp_event_queue_handle;

	/* SP2Host event queue */
	ia_css_queue_t sp2host_event_queue_handle;

	/* Tagger command queue */
	ia_css_queue_t host2sp_tag_cmd_queue_handle;
};

struct sh_css_queues  css_queues;


/*******************************************************
*** Static variables
********************************************************/
static int buffer_type_to_queue_id_map[SH_CSS_MAX_SP_THREADS][IA_CSS_NUM_DYNAMIC_BUFFER_TYPE];
static bool queue_availability[SH_CSS_MAX_SP_THREADS][SH_CSS_MAX_NUM_QUEUES];

/*******************************************************
*** Static functions
********************************************************/
static void map_buffer_type_to_queue_id(
	unsigned int thread_id,
	enum ia_css_buffer_type buf_type
	);
static void unmap_buffer_type_to_queue_id(
	unsigned int thread_id,
	enum ia_css_buffer_type buf_type
	);

static ia_css_queue_t *bufq_get_qhandle(
	enum sh_css_queue_type type,
	enum sh_css_queue_id id,
	int thread
	);

/*******************************************************
*** Public functions
********************************************************/
void ia_css_queue_map_init(void)
{
	unsigned int i, j;

	for (i = 0; i < SH_CSS_MAX_SP_THREADS; i++) {
		for (j = 0; j < SH_CSS_MAX_NUM_QUEUES; j++) {
			queue_availability[i][j] = true;
		}
	}

	for (i = 0; i < SH_CSS_MAX_SP_THREADS; i++) {
		for (j = 0; j < IA_CSS_NUM_DYNAMIC_BUFFER_TYPE; j++) {
			buffer_type_to_queue_id_map[i][j] = SH_CSS_INVALID_QUEUE_ID;
		}
	}
}

void ia_css_queue_map(
	unsigned int thread_id,
	enum ia_css_buffer_type buf_type,
	bool map)
{
	assert(buf_type < IA_CSS_NUM_DYNAMIC_BUFFER_TYPE);
	assert(thread_id < SH_CSS_MAX_SP_THREADS);

	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE,
		"ia_css_queue_map() enter: buf_type=%d, thread_id=%d\n", buf_type, thread_id);

	if(map) {
		map_buffer_type_to_queue_id(thread_id, buf_type);
	} else {
		unmap_buffer_type_to_queue_id(thread_id, buf_type);
	}
}

/**
 * @brief Query the internal queue ID.
 */
bool ia_css_query_internal_queue_id(
	enum ia_css_buffer_type buf_type,
	unsigned int thread_id,
	enum sh_css_queue_id *val)
{
	assert(buf_type < IA_CSS_NUM_DYNAMIC_BUFFER_TYPE);
	assert(thread_id < SH_CSS_MAX_SP_THREADS);
	assert(val != NULL);

	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE,
		"ia_css_query_internal_queue_id() enter: buf_type=%d, thread_id=%d\n", buf_type, thread_id);
	*val = buffer_type_to_queue_id_map[thread_id][buf_type];
	assert(*val != SH_CSS_INVALID_QUEUE_ID);
	assert(*val < SH_CSS_MAX_NUM_QUEUES);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE,
		"ia_css_query_internal_queue_id() leave: return_val=%d\n",
		*val);
	return true;
}

/*******************************************************
*** Static functions
********************************************************/
static void map_buffer_type_to_queue_id(
	unsigned int thread_id,
	enum ia_css_buffer_type buf_type)
{
	unsigned int i;

	assert(thread_id < SH_CSS_MAX_SP_THREADS);
	assert(buf_type < IA_CSS_NUM_DYNAMIC_BUFFER_TYPE);
	assert(buffer_type_to_queue_id_map[thread_id][buf_type] == SH_CSS_INVALID_QUEUE_ID);

	//queue 0 is reserved for parameter because it doesn't depend on events
	if (buf_type == IA_CSS_BUFFER_TYPE_PARAMETER_SET) {
		assert(queue_availability[thread_id][0] == true);
		queue_availability[thread_id][0] = false;
		buffer_type_to_queue_id_map[thread_id][buf_type] = IA_CSS_PARAMETER_SET_QUEUE_ID;
		return;
	}

	for (i = 1; i < SH_CSS_MAX_NUM_QUEUES; i++) {
		if (queue_availability[thread_id][i] == true) {
			queue_availability[thread_id][i] = false;
			buffer_type_to_queue_id_map[thread_id][buf_type] = i;
			break;
		}
	}

	assert(i != SH_CSS_MAX_NUM_QUEUES);
	return;
}

static void unmap_buffer_type_to_queue_id(
	unsigned int thread_id,
	enum ia_css_buffer_type buf_type)
{
	int queue_id;

	assert(thread_id < SH_CSS_MAX_SP_THREADS);
	assert(buf_type < IA_CSS_NUM_DYNAMIC_BUFFER_TYPE);
	assert(buffer_type_to_queue_id_map[thread_id][buf_type] != SH_CSS_INVALID_QUEUE_ID);

	queue_id = buffer_type_to_queue_id_map[thread_id][buf_type];
	buffer_type_to_queue_id_map[thread_id][buf_type] = SH_CSS_INVALID_QUEUE_ID;
	queue_availability[thread_id][queue_id] = true;
}


static ia_css_queue_t *bufq_get_qhandle(
	enum sh_css_queue_type type,
	enum sh_css_queue_id id,
	int thread)
{
	ia_css_queue_t *q = 0;

	switch (type) {
	case sh_css_host2sp_buffer_queue:
		if ((thread >= SH_CSS_MAX_SP_THREADS) || (thread < 0) ||
			(id == SH_CSS_INVALID_QUEUE_ID))
			break;
		q = &css_queues.host2sp_buffer_queue_handles[thread][id];
		break;
	case sh_css_sp2host_buffer_queue:
		if (id == SH_CSS_INVALID_QUEUE_ID)
			break;
		q = &css_queues.sp2host_buffer_queue_handles[id];
		break;
	case sh_css_host2sp_event_queue:
		q = &css_queues.host2sp_event_queue_handle;
		break;
	case sh_css_sp2host_event_queue:
		q = &css_queues.sp2host_event_queue_handle;
		break;
	case sh_css_host2sp_tag_cmd_queue:
		q = &css_queues.host2sp_tag_cmd_queue_handle;
		break;
	default:
		break;
	}

	return q;
}


enum ia_css_err ia_css_bufq_init(void)
{
	const struct ia_css_fw_info *fw;
	unsigned int HIVE_ADDR_ia_css_bufq_host_sp_queue;
	ia_css_queue_remote_t remoteq;
	int i, j;

	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ia_css_bufq_init() enter:\n");
	fw = &sh_css_sp_fw;

#ifdef C_RUN
	HIVE_ADDR_ia_css_bufq_host_sp_queue = (unsigned int)
		sp_address_of(ia_css_bufq_host_sp_queue);
#else
	HIVE_ADDR_ia_css_bufq_host_sp_queue = fw->info.sp.host_sp_queue;
#endif
	/* Setup queue location as SP and proc id as SP0_ID*/
	remoteq.location = IA_CSS_QUEUE_LOC_SP;
	remoteq.proc_id = SP0_ID;

	/* Setup all the local queue descriptors for Host2SP Buffer Queues */
	for (i = 0; i < SH_CSS_MAX_SP_THREADS; i++)
		for (j = 0; j < SH_CSS_MAX_NUM_QUEUES; j++) {
			remoteq.cb_desc_addr =
				HIVE_ADDR_ia_css_bufq_host_sp_queue
					+ offsetof(struct host_sp_queues,
					 host2sp_buffer_queues_desc[i][j]);

			remoteq.cb_elems_addr =
				HIVE_ADDR_ia_css_bufq_host_sp_queue
					+ offsetof(struct host_sp_queues,
					 host2sp_buffer_queues_elems[i][j]);

			/* Initialize the queue instance and obtain handle */
			ia_css_queue_remote_init(
				&css_queues.host2sp_buffer_queue_handles[i][j],
				&remoteq);
		}

	/* Setup all the local queue descriptors for SP2Host Buffer Queues */
	for (i = 0; i < SH_CSS_MAX_NUM_QUEUES; i++) {
		remoteq.cb_desc_addr = HIVE_ADDR_ia_css_bufq_host_sp_queue
			+ offsetof(struct host_sp_queues,
				 sp2host_buffer_queues_desc[i]);

		remoteq.cb_elems_addr = HIVE_ADDR_ia_css_bufq_host_sp_queue
			+ offsetof(struct host_sp_queues,
				 sp2host_buffer_queues_elems[i]);

		/* Initialize the queue instance and obtain handle */
		ia_css_queue_remote_init(
			&css_queues.sp2host_buffer_queue_handles[i],
			&remoteq);
	}

	/* Host2SP queues event queue*/
	remoteq.cb_desc_addr = HIVE_ADDR_ia_css_bufq_host_sp_queue +
		offsetof(struct host_sp_queues, host2sp_event_queue_desc);
	remoteq.cb_elems_addr = HIVE_ADDR_ia_css_bufq_host_sp_queue +
		offsetof(struct host_sp_queues, host2sp_event_queue_elems);
	/* Initialize the queue instance and obtain handle */
	ia_css_queue_remote_init(&css_queues.host2sp_event_queue_handle,
		&remoteq);

	/* SP2Host queues event queue*/
	remoteq.cb_desc_addr = HIVE_ADDR_ia_css_bufq_host_sp_queue +
		offsetof(struct host_sp_queues, sp2host_event_queue_desc);
	remoteq.cb_elems_addr = HIVE_ADDR_ia_css_bufq_host_sp_queue +
		offsetof(struct host_sp_queues, sp2host_event_queue_elems);
	/* Initialize the queue instance and obtain handle */
	ia_css_queue_remote_init(&css_queues.sp2host_event_queue_handle,
		&remoteq);

	/* Host2SP tagger command queue */
	remoteq.cb_desc_addr = HIVE_ADDR_ia_css_bufq_host_sp_queue +
		offsetof(struct host_sp_queues, host2sp_tag_cmd_queue_desc);
	remoteq.cb_elems_addr = HIVE_ADDR_ia_css_bufq_host_sp_queue +
		offsetof(struct host_sp_queues, host2sp_tag_cmd_queue_elems);
	/* Initialize the queue instance and obtain handle */
	ia_css_queue_remote_init(&css_queues.host2sp_tag_cmd_queue_handle,
		&remoteq);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ia_css_bufq_init() leave:\n");

	return IA_CSS_SUCCESS;
}

enum ia_css_err ia_css_bufq_enqueue_buffer(
	int thread_index,
	int queue_id,
	uint32_t item)
{
	enum ia_css_err return_err = IA_CSS_SUCCESS;
	ia_css_queue_t *q;
	int error;

	if ((thread_index >= SH_CSS_MAX_SP_THREADS) || (thread_index < 0) ||
			(queue_id == SH_CSS_INVALID_QUEUE_ID))
		return IA_CSS_ERR_INVALID_ARGUMENTS;

	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE,
		"ia_css_bufq_enqueue_buffer() enter: thread_index = %d, queue_id=%d\n",
		thread_index,
		queue_id);

	/* Get the queue for communication */
	q = bufq_get_qhandle(sh_css_host2sp_buffer_queue,
		queue_id,
		thread_index);
	if (q != NULL) {
		error = ia_css_queue_enqueue(q, item);
		return_err = ia_css_convert_errno(error);
	} else {
		/* Error as the queue is not initialized */
		return_err = IA_CSS_ERR_RESOURCE_NOT_AVAILABLE;
	}

	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE,
		"ia_css_bufq_enqueue_buffer() leave: return_err = %d\n", return_err);

	return return_err;
}

enum ia_css_err ia_css_bufq_dequeue_buffer(
	int queue_id,
	uint32_t *item)
{
	enum ia_css_err return_err;
	int error = 0;
	ia_css_queue_t *q;

	if ((item == NULL) || (queue_id == SH_CSS_INVALID_QUEUE_ID))
		return IA_CSS_ERR_INVALID_ARGUMENTS;

	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE,
		"ia_css_bufq_dequeue_buffer() enter: queue_id = %d\n",
		queue_id);

	q = bufq_get_qhandle(sh_css_sp2host_buffer_queue,
		queue_id,
		-1);
	if (q != NULL) {
		error = ia_css_queue_dequeue(q, item);
		return_err = ia_css_convert_errno(error);
	} else {
		/* Error as the queue is not initialized */
		return_err = IA_CSS_ERR_RESOURCE_NOT_AVAILABLE;
	}

	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE,
		"ia_css_bufq_dequeue_buffer() leave: return_err = %d item = %p\n",
		return_err,
		item);
	return return_err;
}

enum ia_css_err ia_css_bufq_enqueue_event(
	uint8_t evt_id,
	uint8_t evt_payload_0,
	uint8_t evt_payload_1,
	uint8_t evt_payload_2)
{
	enum ia_css_err return_err;
	int error = 0;
	ia_css_queue_t *q;

	q = bufq_get_qhandle(sh_css_host2sp_event_queue, -1, -1);
	if (NULL == q) {
		/* Error as the queue is not initialized */
		ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE,
			"ia_css_bufq_enqueue_event() leaving: queue not available\n");
		return IA_CSS_ERR_RESOURCE_NOT_AVAILABLE;
	}

	error = ia_css_eventq_send(q,
			evt_id, evt_payload_0, evt_payload_1, evt_payload_2);

	return_err = ia_css_convert_errno(error);

	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE,
		"ia_css_bufq_enqueue_event() leave: return_err = %d\n", return_err);

	return return_err;
}

enum  ia_css_err ia_css_bufq_dequeue_event(
	uint8_t item[BUFQ_EVENT_SIZE])
{
	enum ia_css_err return_err;
	int error = 0;
	ia_css_queue_t *q;

	if (item == NULL)
		return IA_CSS_ERR_INVALID_ARGUMENTS;

	q = bufq_get_qhandle(sh_css_sp2host_event_queue, -1, -1);
	if (NULL == q) {
		/* Error as the queue is not initialized */
		ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE,
			"ia_css_dequeue_event() leaving: Not available\n");
			return IA_CSS_ERR_RESOURCE_NOT_AVAILABLE;
	}
	error = ia_css_eventq_recv(q, item);

	return_err = ia_css_convert_errno(error);

	return return_err;

}

enum ia_css_err ia_css_bufq_enqueue_tag_cmd(
	uint32_t item)
{
	enum ia_css_err return_err;
	int error = 0;
	ia_css_queue_t *q;

	q = bufq_get_qhandle(sh_css_host2sp_tag_cmd_queue, -1, -1);
	if (NULL == q) {
		/* Error as the queue is not initialized */
		ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE,
			"ia_css_bufq_enqueue_tag_cmd() leaving: queue not available\n");
		return IA_CSS_ERR_RESOURCE_NOT_AVAILABLE;
	}
	if (q != NULL) {
		error = ia_css_queue_enqueue(q, item);
		return_err = ia_css_convert_errno(error);
	} else {
		/* Error as the queue is not initialized */
		return_err = IA_CSS_ERR_RESOURCE_NOT_AVAILABLE;
	}

	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE,
		"ia_css_bufq_enqueue_tag_cmd() leave: return_err = %d\n", return_err);

	return return_err;
}

enum ia_css_err ia_css_bufq_deinit(void)
{
	return IA_CSS_SUCCESS;
}
