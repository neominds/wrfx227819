/*
 * kdb USB keyboard driver
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.
 *
 * Copyright (c) 1999-2006 Silicon Graphics, Inc.  All Rights Reserved.
 * Copyright (c) 2009-2012 Wind River Systems, Inc.  All Rights Reserved.
 */

#include <linux/kdb.h>
#include <linux/keyboard.h>
#include <linux/usb.h>
#include <linux/module.h>

static unsigned char kdb_usb_keycode[256] = {
	  0,   0,   0,   0,  30,  48,  46,  32,  18,  33,  34,  35,  23,
	 36,  37,  38,  50,  49,  24,  25,  16,  19,  31,  20,  22,  47,
	 17,  45,  21,  44,   2,   3,   4,   5,   6,   7,   8,   9,  10,
	 11,  28,   1,  14,  15,  57,  12,  13,  26,  27,  43,  84,  39,
	 40,  41,  51,  52,  53,  58,  59,  60,  61,  62,  63,  64,  65,
	 66,  67,  68,  87,  88,  99,  70, 119, 110, 102, 104, 111, 107,
	109, 106, 105, 108, 103,  69,  98,  55,  74,  78,  96,  79,  80,
	 81,  75,  76,  77,  71,  72,  73,  82,  83,  86, 127, 116, 117,
	 85,  89,  90,  91,  92,  93,  94,  95, 120, 121, 122, 123, 134,
	138, 130, 132, 128, 129, 131, 137, 133, 135, 136, 113, 115, 114,
	  0,   0,   0, 124,   0, 181, 182, 183, 184, 185, 186, 187, 188,
	189, 190, 191, 192, 193, 194, 195, 196, 197, 198,   0,   0,   0,
	  0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
	  0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
	  0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
	  0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
	  0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
	  0,   0,   0,  29,  42,  56, 125,  97,  54, 100, 126, 164, 166,
	165, 163, 161, 115, 114, 113, 150, 158, 159, 128, 136, 177, 178,
	176, 142, 152, 173, 140,
};

#define MAX_KEYBOARDS	8
static struct usb_device *kdb_keyboard_dev[8];

#define KDB_USB_RING_MAX 4
static int kdb_usbkbd_head;
static int kdb_usbkbd_tail;
static int kdb_usbkbd_ring[KDB_USB_RING_MAX];
static bool kdb_usbkbd_caps_lock;

static void push_usbkbd_key(int keycode)
{
	int next = kdb_usbkbd_head + 1;

	if (next >= KDB_USB_RING_MAX)
		next = 0;
	if (next == kdb_usbkbd_tail)
		return;

	kdb_usbkbd_ring[kdb_usbkbd_head] = keycode;
	kdb_usbkbd_head = next;
}

static int kdb_pop_usbkbd_key(void)
{
	int next = kdb_usbkbd_tail + 1;
	int ret = kdb_usbkbd_ring[kdb_usbkbd_tail];

	if (kdb_usbkbd_tail == kdb_usbkbd_head)
		return -1;
	if (next >= KDB_USB_RING_MAX)
		next = 0;
	kdb_usbkbd_tail = next;
	return ret;
}

#define MAX_KEYS_DOWN 4
static int kbdusb_keys_down[MAX_KEYS_DOWN];
static int kbdusb_idx;

/*
 * This function receive input from usb keyboard devices
 */
void kdb_put_usb_char(char *buffer, struct usb_device *dev)
{
	unsigned char keycode, spec;
	int i, j, found;

	/* Mark keys up if they are no longer down */
	for (i = 0; i < kbdusb_idx; i++) {
		for (j = 0; j < MAX_KEYS_DOWN; j++) {
			if (kbdusb_keys_down[i] == buffer[2+j])
				break;
		}
		if (j == MAX_KEYS_DOWN) {
			kbdusb_idx--;
			kbdusb_keys_down[i] = kbdusb_keys_down[kbdusb_idx];
			i--;
			if (kbdusb_idx == 0)
				break;
		}
	}

	for (i = 0; i < MAX_KEYS_DOWN; i++) {
		if (!buffer[2+i])
			break;

		keycode = buffer[2+i];
		buffer[2+i] = 0;
		spec = buffer[0];
		buffer[0] = 0;

		/* if the key was previously down, skip it */
		found = 0;
		for (j = 0; j < kbdusb_idx; j++)
			if (keycode == kbdusb_keys_down[j]) {
				found = 1;
				break;
			}
		if (found)
			continue;

		if (kbdusb_idx < MAX_KEYS_DOWN) {
			kbdusb_keys_down[kbdusb_idx] = keycode;
			kbdusb_idx++;
		}
		/* A normal key is pressed, decode it */
		if (keycode)
			keycode = kdb_usb_keycode[keycode];

		/* 2 Keys pressed at one time ? */
		if (spec && keycode) {
			switch (spec) {
			case 0x2:
			case 0x20: /* Shift */
				push_usbkbd_key(key_maps[1][keycode]);
				break;
			case 0x1:
			case 0x10: /* Ctrl */
				push_usbkbd_key(key_maps[4][keycode]);
				break;
			case 0x4:
			case 0x40: /* Alt */
				break;
			}
		} else if (keycode) { /* If only one key pressed */
			switch (keycode) {
			case 0x1C: /* Enter */
				push_usbkbd_key(13);
				break;
			case 0x3A: /* Capslock */
				kdb_usbkbd_caps_lock = !kdb_usbkbd_caps_lock;
				break;
			case 0x0E: /* Backspace */
				push_usbkbd_key(8);
				break;
			case 0x0F: /* TAB */
				push_usbkbd_key(9);
				break;
			case 0x77: /* Pause */
				break;
			default:
				if (!kdb_usbkbd_caps_lock)
					push_usbkbd_key(plain_map[keycode]);
				else
					push_usbkbd_key(key_maps[1][keycode]);
				break;
			}
		}
	}
}

void kdb_keyboard_attach(struct usb_device *dev)
{
	int i;

	for (i = 0; i < MAX_KEYBOARDS; i++)
		if (kdb_keyboard_dev[i] == NULL) {
			kdb_keyboard_dev[i] = dev;
			break;
		}
}

void kdb_keyboard_detach(struct usb_device *dev)
{
	int i;

	for (i = 0; i < MAX_KEYBOARDS; i++)
		if (kdb_keyboard_dev[i] == dev) {
			kdb_keyboard_dev[i] = NULL;
			break;
		}
}

int kdb_get_usb_char(void)
{
	int ret = kdb_pop_usbkbd_key();
	int i;

	if (ret >= 0)
		return ret;
	for (i = 0; i < MAX_KEYBOARDS; i++)
		if (kdb_keyboard_dev[i])
			usb_poll_irq(kdb_keyboard_dev[i]);
	return kdb_pop_usbkbd_key();
}
EXPORT_SYMBOL_GPL(kdb_get_usb_char);
