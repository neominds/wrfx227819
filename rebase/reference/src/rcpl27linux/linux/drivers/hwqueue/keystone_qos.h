/*
 * Keystone Accumulator driver
 *
 * Copyright (C) 2012 Texas Instruments Incorporated - http://www.ti.com
 * Contact: Sandeep Paulraj <s-paulraj@ti.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 */

#ifndef __KEYSTONE_QOS_FW_H
#define __KEYSTONE_QOS_FW_H

#define QOS_MAX_INPUTS			128

#define QMSS_CLK_RATE			327680000

#define QOS_RETCODE_SUCCESS		1

#define QOS_COMMAND_TIMEOUT		20	/* msecs */
#define QOS_COMMAND_DELAY		10	/* usecs */

#define QOS_CMD_GET_QUEUE_BASE		0x80
#define QOS_CMD_SET_QUEUE_BASE		0x81
#define QOS_CMD_SET_TIMER_CONFIG	0x82
#define QOS_CMD_ENABLE_PORT		0x90
#define QOS_CMD_PORT_SHADOW		0x91
#define QOS_CMD_STATS_REQUEST		0x92

#define QOS_MAGIC_DROPSCHED		0x8020

#define QOS_QUEUE_BASE_DROP_SCHED	BIT(8)
#define QOS_QUEUE_BASE_QOS_SCHED	0

#define QOS_ENABLE			BIT(8)
#define QOS_DISABLE			0

#define QOS_COPY_ACTIVE_TO_SHADOW	0
#define	QOS_COPY_SHADOW_TO_ACTIVE	BIT(8)

#define QOS_DROP_SCHED_CFG		4
#define QOS_DROP_SCHED_ENABLE		BIT(24)

#define QOS_DEFAULT_OVERHEAD_BYTES	24

#define QOS_CREDITS_PACKET_SHIFT	20
#define QOS_CREDITS_BYTE_SHIFT		11

#define QOS_BYTE_NORMALIZATION_FACTOR	3000
#define QOS_PACKET_NORMALIZATION_FACTOR	2

#define	QOS_MAX_WEIGHT			SZ_1M

#define to_qnode(_n)	container_of(_n, struct khwq_qos_tree_node, node)

enum khwq_qos_normalize {
	DIVIDE,
	MULTIPLY
};

enum khwq_qos_accounting_type {
	QOS_PACKET_ACCT,
	QOS_BYTE_ACCT
};

enum khwq_qos_drop_mode {
	QOS_TAILDROP,
	QOS_RED,
};

enum khwq_qos_tree_node_type {
	QOS_NODE_PRIO,
	QOS_NODE_WRR,
	QOS_NODE_DEFAULT,
};

enum khwq_qos_shadow_type {
	QOS_SCHED_PORT_CFG,
	QOS_DROP_CFG_PROF,
	QOS_DROP_QUEUE_CFG,
	QOS_DROP_OUT_PROF,

	QOS_MAX_SHADOW	/* last */
};

enum khwq_qos_control_type {
	QOS_CONTROL_ENABLE,
	QOS_CONTROL_GET_INPUT,
};

struct khwq_qos_shadow {
	enum khwq_qos_shadow_type	  type;
	struct khwq_qos_info		 *info;
	const char			 *name;
	void				 *data;
	unsigned long			 *dirty;
	unsigned long			 *avail;
	unsigned long			 *running;
	int				  start, count, size;
	int		(*sync)(struct khwq_qos_shadow *shadow, int idx);
	int		(*control)(struct khwq_qos_shadow *shadow,
				   enum khwq_qos_control_type ctrl,
				   int idx, u32 arg);
};

struct khwq_qos_stats {
	enum khwq_qos_shadow_type	  type;
	const char			 *name;
	void				 *data;
	unsigned long			 *dirty;
	unsigned long			 *avail;
	unsigned long			 *running;
	int				  start, count, size;
	int		(*get_stats)(struct khwq_qos_stats *stats, int idx);
};

struct khwq_qos_drop_policy {
	const char			*name;
	struct khwq_qos_info		*info;
	bool				 usecount;
	enum khwq_qos_accounting_type	 acct;
	enum khwq_qos_drop_mode		 mode;
	u32				 limit;
	u32				 red_low;
	u32				 red_high;
	u32				 half_life;
	u32				 max_drop_prob;
	int				 drop_cfg_idx;
	struct list_head		 list;
	struct kobject			 kobj;
};

struct khwq_qos_info {
	spinlock_t			 lock;
	struct khwq_device		*kdev;
	struct khwq_pdsp_info		*pdsp;
	u32				 refcount;
	struct khwq_qos_shadow		 shadows[QOS_MAX_SHADOW];
	struct khwq_qos_stats		 stats;
	struct ktree			 qos_tree;
	struct list_head		 drop_policies;
	struct list_head		 stats_classes;
	struct khwq_qos_drop_policy	*default_drop_policy;
	
	struct dentry			*root_dir;
	struct dentry			*config_profiles;
	struct dentry			*out_profiles;
	struct dentry			*queue_configs;
	struct dentry			*port_configs;

	int	 sched_port_queue_base,
		 drop_sched_queue_base,
		 inputs_per_port,
		 ticks_per_sec,
		 pdsp_id;

	struct {
		u8		 int_num;
		u8		 qos_ticks;
		u8		 drop_ticks;
		u32		 seed[3];
	} drop_cfg;

	struct timer_list		 timer;

	struct kobject			 *kobj;
	struct kobject			 *kobj_stats;
	struct kobject			 *kobj_policies;
};

struct khwq_qos_stats_class {
	const char			*name;
	struct khwq_qos_info		*info;
	struct list_head		 list;
	int				 stats_block_idx;
	bool				 usecount;
	struct kobject			 kobj;
};

struct khwq_qos_input_queue {
	bool				 valid;
	u32				 queue;
	int				 drop_queue_idx;
};

struct khwq_qos_tree_node {
	struct khwq_qos_tree_node	*parent;
	struct khwq_qos_info		*info;
	struct ktree_node		 node;
	enum khwq_qos_tree_node_type	 type;
	u32				 weight;
	u32				 priority;
	enum khwq_qos_accounting_type	 acct;
	const char			*name;
	int				 overhead_bytes;
	int				 output_rate;
	int				 burst_size;
	int				 num_input_queues;
	u32				 input_queues[QOS_MAX_INPUTS];
	struct khwq_qos_input_queue	 input_queue[QOS_MAX_INPUTS];
	struct khwq_qos_drop_policy	*drop_policy;
	struct khwq_qos_stats_class	*stats_class;

	int	 child_port_count;	/* children that need ports	*/
	int	 child_count;		/* number of children		*/
	int	 parent_input;		/* input number of parent	*/
	u32	 child_weight_min;	/* min of child weights		*/
	u32	 child_weight_max;	/* max of child weights		*/
	u32	 child_weight[4];
	u32	 child_weight_sum;	/* sum of child weights		*/
	bool	 is_drop_input;		/* indicates that child's output
					   feeds to the drop sched	*/
	bool	 has_sched_port;	/* does this port need a sched?	*/
	int	 output_queue;		/* from DT or calculated	*/

	/* allocated resources */
	int	 sched_port_idx;	/* inherited by default nodes	*/
	int	 drop_out_idx;		/* inherited by default nodes	*/
	int	 drop_queue_idx[QOS_MAX_INPUTS];

	struct kobject			 kobj;
};

struct khwq_semaphore_regs {
	u32		sem;
};

struct khwq_push_stats_regs {
	u32		stats_info;
	u32		bytes_forwarded;
	u32		bytes_discarded;
	u32		packets_forwarded;
	u32		packets_discarded;
};

struct khwq_query_stats_regs {
	u32		bytes_forwarded_lsw;
	u32		bytes_forwarded_msw;
	u32		bytes_discarded_lsw;
	u32		bytes_discarded_msw;
	u32		packets_forwarded;
	u32		packets_discarded;
};

#define QOS_SHADOW_OFFSET	0x40
#define QOS_PUSH_PROXY_OFFSET	0x2e0
#define QOS_STATS_OFFSET	0x300
#define QOS_MAGIC_OFFSET	0x1ff8
#define QOS_VERSION_OFFSET	0x1ffc

#define khwq_qos_id_to_idx(idx)		((idx) & 0xffff)
#define khwq_qos_id_to_pdsp(idx)	((idx) >> 16)
#define khwq_qos_make_id(pdsp, idx)	((pdsp) << 16 | (idx))
#define khwq_qos_id_to_queue(info, idx)		\
	((info)->drop_sched_queue_base + khwq_qos_id_to_idx(idx))

int khwq_qos_alloc(struct khwq_qos_info *info, enum khwq_qos_shadow_type type);
int khwq_qos_free(struct khwq_qos_info *info, enum khwq_qos_shadow_type type,
		    int idx);
int khwq_qos_control(struct khwq_qos_info *info, enum khwq_qos_shadow_type type,
		       enum khwq_qos_control_type ctrl, int idx, u32 arg,
		       bool internal);
int khwq_qos_sync(struct khwq_qos_info *info, enum khwq_qos_shadow_type type,
		    int idx, bool internal);
int khwq_qos_get(struct khwq_qos_info *info, enum khwq_qos_shadow_type type,
		   const char *name, int idx, int offset, int startbit,
		   int nbits, u32 *value);
int khwq_qos_set(struct khwq_qos_info *info, enum khwq_qos_shadow_type type,
		   const char *name, int idx, int offset, int startbit,
		   int nbits, bool sync, u32 value, bool internal);

#define DEFINE_SHADOW(_type, _field)					       \
static inline int khwq_qos_control_##_field(struct khwq_qos_info *info,        \
					    enum khwq_qos_control_type ctrl,   \
					    int idx, u32 arg)		       \
{									       \
	return khwq_qos_control(info, _type, ctrl, idx, arg, false);	       \
}									       \
static inline int khwq_qos_sync_##_field(struct khwq_qos_info *info,	       \
					 int idx)			       \
{									       \
	return khwq_qos_sync(info, _type, idx, false);		       \
}

DEFINE_SHADOW(QOS_DROP_CFG_PROF,	drop_cfg);
DEFINE_SHADOW(QOS_DROP_OUT_PROF,	drop_out);
DEFINE_SHADOW(QOS_SCHED_PORT_CFG,	sched_port);
DEFINE_SHADOW(QOS_DROP_QUEUE_CFG,	drop_queue);

#define DEFINE_ALLOC(_type, _field)					       \
static inline int khwq_qos_alloc_##_field(struct khwq_qos_info *info)	       \
{									       \
	return khwq_qos_alloc(info, _type);				       \
}									       \
static inline int khwq_qos_free_##_field(struct khwq_qos_info *info,	       \
					 int idx)			       \
{									       \
	return khwq_qos_free(info, _type, idx);			       \
}

DEFINE_ALLOC(QOS_DROP_CFG_PROF,	 drop_cfg);
DEFINE_ALLOC(QOS_DROP_OUT_PROF,	 drop_out);
DEFINE_ALLOC(QOS_SCHED_PORT_CFG, sched_port);

#define DEFINE_FIELD_U32(_type, _field, _offset, _startbit, _nbits)	 \
static inline int khwq_qos_get_##_field(struct khwq_qos_info *info,	 \
					int idx, u32 *value)		 \
{									 \
	return khwq_qos_get(info, _type, #_field, idx, _offset,	 \
			      _startbit, _nbits, value);		 \
}									 \
static inline int khwq_qos_set_##_field(struct khwq_qos_info *info,	 \
					int idx, u32 value, bool sync)	 \
{									 \
	return khwq_qos_set(info, _type, #_field, idx, _offset,	 \
			      _startbit, _nbits, sync, value, false);	 \
}									 \
static inline int __khwq_qos_set_##_field(struct khwq_qos_info *info,	 \
					int idx, u32 value, bool sync)	 \
{									 \
	return khwq_qos_set(info, _type, #_field, idx, _offset,	 \
			      _startbit, _nbits, sync, value, true);	 \
}

#define DEFINE_FIELD_U32_ARRAY(_type, _field, _offset, _size)		 \
static inline int khwq_qos_get_##_field(struct khwq_qos_info *info,	 \
					int idx, int elem, u32 *value)	 \
{									 \
	int ofs = _offset + elem * _size;				 \
	return khwq_qos_get(info, _type, #_field, idx, ofs, 0, 32,	 \
			      value);					 \
}									 \
static inline int khwq_qos_set_##_field(struct khwq_qos_info *info,	 \
				int idx, int elem, u32 value, bool sync) \
{									 \
	int ofs = _offset + elem * _size;				 \
	return khwq_qos_set(info, _type, #_field, idx, ofs, 0, 32,	 \
			      sync, value, false);			 \
}									 \
static inline int __khwq_qos_set_##_field(struct khwq_qos_info *info,	 \
				int idx, int elem, u32 value, bool sync) \
{									 \
	int ofs = _offset + elem * _size;				 \
	return khwq_qos_set(info, _type, #_field, idx, ofs, 0, 32,	 \
			      sync, value, true);			 \
}

DEFINE_FIELD_U32(QOS_DROP_CFG_PROF, drop_cfg_unit_flags,   0x00,  0,  8)
DEFINE_FIELD_U32(QOS_DROP_CFG_PROF, drop_cfg_mode,         0x00,  8,  8)
DEFINE_FIELD_U32(QOS_DROP_CFG_PROF, drop_cfg_time_const,   0x00, 16,  8)
DEFINE_FIELD_U32(QOS_DROP_CFG_PROF, drop_cfg_tail_thresh,  0x04,  0, 32)
DEFINE_FIELD_U32(QOS_DROP_CFG_PROF, drop_cfg_red_low,      0x08,  0, 32)
DEFINE_FIELD_U32(QOS_DROP_CFG_PROF, drop_cfg_red_high,     0x0c,  0, 32)
DEFINE_FIELD_U32(QOS_DROP_CFG_PROF, drop_cfg_thresh_recip, 0x10,  0, 32)

DEFINE_FIELD_U32(QOS_DROP_OUT_PROF, drop_out_queue_number, 0x00,  0, 16)
DEFINE_FIELD_U32(QOS_DROP_OUT_PROF, drop_out_red_prob,     0x00, 16, 16)
DEFINE_FIELD_U32(QOS_DROP_OUT_PROF, drop_out_cfg_prof_idx, 0x04,  0,  8)
DEFINE_FIELD_U32(QOS_DROP_OUT_PROF, drop_out_enable,       0x04,  8,  8)
DEFINE_FIELD_U32(QOS_DROP_OUT_PROF, drop_out_avg_depth,    0x08,  0, 32)

DEFINE_FIELD_U32(QOS_DROP_QUEUE_CFG, drop_q_out_prof_idx,  0x00,  0,  8)
DEFINE_FIELD_U32(QOS_DROP_QUEUE_CFG, drop_q_stat_blk_idx,  0x00,  8,  8)
DEFINE_FIELD_U32(QOS_DROP_QUEUE_CFG, drop_q_stat_irq_pair_idx, 0x00, 16,  8)
DEFINE_FIELD_U32(QOS_DROP_QUEUE_CFG, drop_q_valid,         0x00, 24,  8)

DEFINE_FIELD_U32(QOS_SCHED_PORT_CFG, sched_unit_flags,     0x00,  0,  8)
DEFINE_FIELD_U32(QOS_SCHED_PORT_CFG, sched_group_count,    0x00,  8,  8)
DEFINE_FIELD_U32(QOS_SCHED_PORT_CFG, sched_out_queue,      0x00, 16, 16)
DEFINE_FIELD_U32(QOS_SCHED_PORT_CFG, sched_overhead_bytes, 0x04,  0,  8)
DEFINE_FIELD_U32(QOS_SCHED_PORT_CFG, sched_out_throttle,   0x04, 16, 16)
DEFINE_FIELD_U32(QOS_SCHED_PORT_CFG, sched_cir_credit,     0x08,  0, 32)
DEFINE_FIELD_U32(QOS_SCHED_PORT_CFG, sched_cir_max,        0x0c,  0, 32)
DEFINE_FIELD_U32(QOS_SCHED_PORT_CFG, sched_total_q_count,  0x24,  0,  8)
DEFINE_FIELD_U32(QOS_SCHED_PORT_CFG, sched_sp_q_count,     0x24,  8,  8)
DEFINE_FIELD_U32(QOS_SCHED_PORT_CFG, sched_wrr_q_count,    0x24, 16,  8)

DEFINE_FIELD_U32_ARRAY(QOS_SCHED_PORT_CFG, sched_wrr_credit,  0x28, 0x8)
DEFINE_FIELD_U32_ARRAY(QOS_SCHED_PORT_CFG, sched_cong_thresh, 0x2c, 0x8)

int khwq_qos_start(struct khwq_qos_info *info);
int khwq_qos_stop(struct khwq_qos_info *info);
int khwq_qos_tree_start(struct khwq_qos_info *info);
int khwq_qos_tree_stop(struct khwq_qos_info *info);

#endif

