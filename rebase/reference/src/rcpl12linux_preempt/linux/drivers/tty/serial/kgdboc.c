/*
 * Based on the same principle as kgdboe using the NETPOLL api, this
 * driver uses a console polling api to implement a gdb serial inteface
 * which is multiplexed on a console port.
 *
 * Maintainer: Jason Wessel <jason.wessel@windriver.com>
 *
 * 2007-2008 (c) Jason Wessel - Wind River Systems, Inc.
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */
#include <linux/kernel.h>
#include <linux/ctype.h>
#include <linux/kgdb.h>
#include <linux/kdb.h>
#include <linux/tty.h>
#include <linux/kthread.h>
#include <linux/console.h>
#include <linux/vt_kern.h>
#include <linux/input.h>
#include <linux/module.h>
#include <linux/usb.h>

#define MAX_CONFIG_LEN		40
#define MAX_CHAR_RING		512

static struct kgdb_io		kgdboc_io_ops;

/* -1 = init not run yet, 0 = unconfigured, 1 = configured. */
static int configured		= -1;

static char config[MAX_CONFIG_LEN];
static char config_try_later;
static struct kparam_string kps = {
	.string			= config,
	.maxlen			= MAX_CONFIG_LEN,
};

static int kgdboc_use_kms;  /* 1 if we use kernel mode switching */
static struct tty_driver	*kgdb_tty_driver;
static int			kgdb_tty_line;
static struct file		*kgdb_filp;

#ifdef CONFIG_KDB_KEYBOARD
static int kgdboc_reset_connect(struct input_handler *handler,
				struct input_dev *dev,
				const struct input_device_id *id)
{
	input_reset_device(dev);

	/* Retrun an error - we do not want to bind, just to reset */
	return -ENODEV;
}

static void kgdboc_reset_disconnect(struct input_handle *handle)
{
	/* We do not expect anyone to actually bind to us */
	BUG();
}

static const struct input_device_id kgdboc_reset_ids[] = {
	{
		.flags = INPUT_DEVICE_ID_MATCH_EVBIT,
		.evbit = { BIT_MASK(EV_KEY) },
	},
	{ }
};

static struct input_handler kgdboc_reset_handler = {
	.connect	= kgdboc_reset_connect,
	.disconnect	= kgdboc_reset_disconnect,
	.name		= "kgdboc_reset",
	.id_table	= kgdboc_reset_ids,
};

static DEFINE_MUTEX(kgdboc_reset_mutex);

static void kgdboc_restore_input_helper(struct work_struct *dummy)
{
	/*
	 * We need to take a mutex to prevent several instances of
	 * this work running on different CPUs so they don't try
	 * to register again already registered handler.
	 */
	mutex_lock(&kgdboc_reset_mutex);

	if (input_register_handler(&kgdboc_reset_handler) == 0)
		input_unregister_handler(&kgdboc_reset_handler);

	mutex_unlock(&kgdboc_reset_mutex);
}

static DECLARE_WORK(kgdboc_restore_input_work, kgdboc_restore_input_helper);

static struct task_struct *delay_schedule_work_task = NULL;
static int do_reschedule_work = 0;
static int do_restore_input(void *dummy);

static void _kgdboc_restore_input(void)
{
	if (likely(system_state == SYSTEM_RUNNING)) {
		schedule_work(&kgdboc_restore_input_work);
#ifdef CONFIG_KDB_USB
		usb_poll_irq_schedule_flush();
#endif /* CONFIG_KDB_USB */
	}
}

static void kgdboc_restore_input(void)
{
	do_reschedule_work = 1;
}

static int kgdboc_register_kbd(char **cptr)
{
	if (delay_schedule_work_task == NULL) {
		delay_schedule_work_task = kthread_create(do_restore_input, NULL, "kgdboc_do_restore_input");
		if (delay_schedule_work_task)
			wake_up_process(delay_schedule_work_task);
	}

	if (strncmp(*cptr, "kbd", 3) == 0 ||
		strncmp(*cptr, "kdb", 3) == 0) {
		if (kdb_poll_idx < KDB_POLL_FUNC_MAX) {
			kdb_poll_funcs[kdb_poll_idx] = kdb_get_kbd_char;
			kdb_poll_idx++;
#ifdef CONFIG_KDB_USB
			if (kdb_poll_idx < KDB_POLL_FUNC_MAX) {
				kdb_poll_funcs[kdb_poll_idx] = kdb_get_usb_char;
				kdb_poll_idx++;
			}
#endif /* CONFIG_KDB_USB */
			if (cptr[0][3] == ',')
				*cptr += 4;
			else
				return 1;
		}
	}
	return 0;
}

static void kgdboc_unregister_kbd(void)
{
	int i;

	for (i = 0; i < kdb_poll_idx; i++) {
#ifdef CONFIG_KDB_USB
		if (kdb_poll_funcs[i] == kdb_get_kbd_char ||
			kdb_poll_funcs[i] == kdb_get_usb_char) {
#else /* ! CONFIG_KDB_USB */
		if (kdb_poll_funcs[i] == kdb_get_kbd_char) {
#endif /* CONFIG_KDB_USB */
			kdb_poll_idx--;
			kdb_poll_funcs[i] = kdb_poll_funcs[kdb_poll_idx];
			kdb_poll_funcs[kdb_poll_idx] = NULL;
			i--;
		}
	}
	flush_work(&kgdboc_restore_input_work);
}

static int do_restore_input(void *dummy)
{
	for (;;) {
		if (do_reschedule_work) {
			do_reschedule_work = 0;
			_kgdboc_restore_input();
		}
		schedule_timeout_interruptible(HZ);
	}

	return 0;
}

#else /* ! CONFIG_KDB_KEYBOARD */
#define kgdboc_register_kbd(x) 0
#define kgdboc_unregister_kbd()
#define kgdboc_restore_input()
#endif /* ! CONFIG_KDB_KEYBOARD */

static int kgdboc_option_setup(char *opt)
{
	if (strlen(opt) >= MAX_CONFIG_LEN) {
		printk(KERN_ERR "kgdboc: config string too long\n");
		return -ENOSPC;
	}
	strcpy(config, opt);

	return 0;
}

static int buffered_char = -1;
static u8 ch_ring[MAX_CHAR_RING];
static int ch_head;
static int ch_tail;
static u8 break_char;
static int no_polled_breaks;
static int schedule_breakpoints;

/* Return 1 if a the next layer up should discard the character,
 * else return 0
 */
static int kgdboc_rx_callback(u8 c)
{
	if (likely(atomic_read(&kgdb_active) == -1 &&
		   !kgdb_is_waiting_on_breakpoint())) {
		if (no_polled_breaks)
			return 0;
		if (c == break_char ||
		    (c == '$' && !kgdb_connected && break_char == 0x03)) {
			if (c == '$')
				buffered_char = c;
			if (schedule_breakpoints)
				kgdb_schedule_breakpoint();
			else
				kgdb_breakpoint();
			return 1;
		}
		return 0;
	}
	/* Assume the debugger is active and store the characters in an
	 * array which will be decremented at a later point.
	 */
	ch_ring[ch_tail] = c;
	ch_tail++;
	if (ch_tail >= MAX_CHAR_RING)
		ch_tail = 0;
	if (ch_tail == ch_head)
		printk(KERN_CRIT "kgdboc: ERROR rx buffer overflow\n");

	return 1;
}

__setup("kgdboc=", kgdboc_option_setup);

static void cleanup_kgdboc(void)
{
	if (kgdb_unregister_nmi_console())
		return;
	kgdboc_unregister_kbd();
	if (configured == 1)
		kgdb_unregister_io_module(&kgdboc_io_ops);
}

static void release_kgdboc_tty(void)
{
	if (kgdb_tty_driver)
		kgdb_tty_driver->ops->poll_init(kgdb_tty_driver, kgdb_tty_line,
						NULL, (void *)-1);
	if (kgdb_filp)
		tty_console_poll_close(&kgdb_filp);
	kgdb_tty_driver = NULL;
}

static int configure_kgdboc(void)
{
	struct tty_driver *p;
	int tty_line = 0;
	int err;
	char *cptr = config;
	struct console *cons;
	char *str;

	err = kgdboc_option_setup(config);
	if (err || !strlen(config) || isspace(config[0]))
		goto noconfig;

	err = -ENODEV;
	/* If a driver was previously configured remove it now */
	release_kgdboc_tty();
	kgdboc_io_ops.is_console = 0;
	kgdb_tty_driver = NULL;

	kgdboc_use_kms = 0;
	if (strncmp(cptr, "kms,", 4) == 0) {
		cptr += 4;
		kgdboc_use_kms = 1;
	}

	if (kgdboc_register_kbd(&cptr))
		goto do_register;

	p = tty_find_polling_driver(cptr, &tty_line);
	if (!p) {
		config_try_later = config[0];
		goto noconfig;
	}

	cons = console_drivers;
	while (cons) {
		int idx;
		if (cons->device && cons->device(cons, &idx) == p &&
		    idx == tty_line) {
			kgdboc_io_ops.is_console = 1;
			break;
		}
		cons = cons->next;
	}

	kgdb_tty_driver = p;
	kgdb_tty_line = tty_line;
	/* Set defaults and parse optional configuration information */
	no_polled_breaks = 0;
	schedule_breakpoints = 1;
	break_char = 0x03;
	if (strstr(config, ",n"))
		no_polled_breaks = 1;
	if (strstr(config, ",B"))
		schedule_breakpoints = 0;
	str = strstr(config, ",c");
	if (str)
		if (kstrtou8(str+2, 0, &break_char))
			break_char = 0x03;
	str = strrchr(config, ','); /* pointer to baud for init callback */
	if (str) {
		str++;
		if (!(*str >= '0' && *str <= '9'))
			str = NULL;
	}
	/* Initialize the HW level driver for polling */
	if (p->ops->poll_init(p, tty_line, str, kgdboc_rx_callback))
		goto noconfig;

	/* Open the port and obtain a tty which call low level driver startup */
	if (tty_console_poll_open(kgdb_tty_driver, &kgdb_filp,
				  kgdb_tty_line) != 0)
		goto noconfig;

do_register:
	err = kgdb_register_io_module(&kgdboc_io_ops);
	if (err)
		goto noconfig;

	err = kgdb_register_nmi_console();
	if (err)
		goto nmi_con_failed;

	configured = 1;
	ch_head = 0;
	ch_tail = 0;

	return 0;

nmi_con_failed:
	kgdb_unregister_io_module(&kgdboc_io_ops);
noconfig:
	kgdboc_unregister_kbd();
	release_kgdboc_tty();
	config[0] = 0;
	configured = 0;
	cleanup_kgdboc();

	return err;
}

#ifdef CONFIG_KGDB_SERIAL_CONSOLE
void kgdboc_init_hook(void)
{
	if (config_try_later && configured == 0) {
		config[0] = config_try_later;
		configure_kgdboc();
	}
}
#endif

static int __init init_kgdboc(void)
{
	/* Already configured? */
	if (configured == 1)
		return 0;

	return configure_kgdboc();
}

static int kgdboc_get_char(void)
{
	int ret;

	if (!kgdb_tty_driver)
		return -1;
	if (buffered_char >= 0)
		return xchg(&buffered_char, -1);

	/* A return of -2 means use the poll character ring */
	if (ch_head != ch_tail) {
		ret = ch_ring[ch_head];
		ch_head++;
		if (ch_head >= MAX_CHAR_RING)
			ch_head = 0;
		return ret;
	}

	ret = kgdb_tty_driver->ops->poll_get_char(kgdb_tty_driver,
						  kgdb_tty_line);
	if (ret != -2)
		return ret;

	return -1;
}

static void kgdboc_put_char(u8 chr)
{
	if (!kgdb_tty_driver)
		return;
	kgdb_tty_driver->ops->poll_put_char(kgdb_tty_driver,
					kgdb_tty_line, chr);
}

static int param_set_kgdboc_var(const char *kmessage, struct kernel_param *kp)
{
	int len = strlen(kmessage);

	if (len >= MAX_CONFIG_LEN) {
		printk(KERN_ERR "kgdboc: config string too long\n");
		return -ENOSPC;
	}

	/* Only copy in the string if the init function has not run yet */
	if (configured < 0) {
		strcpy(config, kmessage);
		return 0;
	}

	if (kgdb_connected) {
		printk(KERN_ERR
		       "kgdboc: Cannot reconfigure while KGDB is connected.\n");

		return -EBUSY;
	}

	strcpy(config, kmessage);
	/* Chop out \n char as a result of echo */
	if (config[len - 1] == '\n')
		config[len - 1] = '\0';

	if (configured == 1)
		cleanup_kgdboc();

	/* Go and configure with the new params. */
	return configure_kgdboc();
}

static int dbg_restore_graphics;

static void kgdboc_pre_exp_handler(void)
{
	if (!dbg_restore_graphics && kgdboc_use_kms) {
		dbg_restore_graphics = 1;
		con_debug_enter(vc_cons[fg_console].d);
	}
	/* Increment the module count when the debugger is active */
	if (!kgdb_connected)
		try_module_get(THIS_MODULE);
}

static void kgdboc_post_exp_handler(void)
{
	/* decrement the module count when the debugger detaches */
	if (!kgdb_connected)
		module_put(THIS_MODULE);
	if (kgdboc_use_kms && dbg_restore_graphics) {
		dbg_restore_graphics = 0;
		con_debug_leave();
	}
	ch_head = 0;
	ch_tail = 0;
	kgdboc_restore_input();
}

static struct kgdb_io kgdboc_io_ops = {
	.name			= "kgdboc",
	.read_char		= kgdboc_get_char,
	.write_char		= kgdboc_put_char,
	.pre_exception		= kgdboc_pre_exp_handler,
	.post_exception		= kgdboc_post_exp_handler,
};

#ifdef CONFIG_KGDB_SERIAL_CONSOLE
/* This is only available if kgdboc is a built in for early debugging */
static int __init kgdboc_early_init(char *opt)
{
	/* save the first character of the config string because the
	 * init routine can destroy it.
	 */
	char save_ch;

	kgdboc_option_setup(opt);
	save_ch = config[0];
	init_kgdboc();
	config[0] = save_ch;
	return 0;
}

early_param("ekgdboc", kgdboc_early_init);
#endif /* CONFIG_KGDB_SERIAL_CONSOLE */

module_init(init_kgdboc);
module_exit(cleanup_kgdboc);
module_param_call(kgdboc, param_set_kgdboc_var, param_get_string, &kps, 0644);
/* The optional paramters to the config string are:
 * ,n == no monitoring the port for a break char
 * ,B == monitor the port for a break char and issue a breakpoint in line
 * ,c### == Use an alternate break character 1-255 instead of ^C (3)
 * The baud parameter must always be last, if used
 * ,baud == A baud rate parameter IE: 115200n81
 */
MODULE_PARM_DESC(kgdboc, "<serial_device>[,n][,B][,c###][,baud]");
MODULE_DESCRIPTION("KGDB Console TTY Driver");
MODULE_LICENSE("GPL");
