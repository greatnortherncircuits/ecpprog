/*
 *  iceprog -- simple programming tool for FTDI-based Lattice iCE programmers
 *
 *  Copyright (C) 2015  Clifford Wolf <clifford@clifford.at>
 *  Copyright (C) 2018  Piotr Esden-Tempski <piotr@esden.net>
 *
 *  Permission to use, copy, modify, and/or distribute this software for any
 *  purpose with or without fee is hereby granted, provided that the above
 *  copyright notice and this permission notice appear in all copies.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 *  WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 *  MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 *  ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 *  WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 *  ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 *  OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 *
 *  Relevant Documents:
 *  -------------------
 *  http://www.ftdichip.com/Support/Documents/AppNotes/AN_108_Command_Processor_for_MPSSE_and_MCU_Host_Bus_Emulation_Modes.pdf
 */

#define _GNU_SOURCE

#include <ftdi.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#ifndef _WIN32
#include <unistd.h>
#endif

#include "mpsse.h"

// ---------------------------------------------------------
// MPSSE / FTDI definitions
// ---------------------------------------------------------

/* FTDI bank pinout typically used for iCE dev boards/
 * BUS IO | Signal | Control
 * -------+--------+--------------
 * xDBUS0 |    SCK | MPSSE
 * xDBUS1 |   MOSI | MPSSE
 * xDBUS2 |   MISO | MPSSE
 * xDBUS3 |     nc |
 * xDBUS4 |     CS | GPIO
 * xDBUS5 |     nc |
 * xDBUS6 |  CDONE | GPIO
 * xDBUS7 | CRESET | GPIO
 */

struct ftdi_context mpsse_ftdic;
bool mpsse_ftdic_open = false;
bool mpsse_ftdic_latency_set = false;
unsigned char mpsse_ftdi_latency;
int mpsse_error_last = 0;
extern void qprintf ( const char * format, ... );

// ---------------------------------------------------------
// MPSSE / FTDI function implementations
// ---------------------------------------------------------

void mpsse_check_rx()
{
	uint8_t cnt = 0;
	while (1) {
		uint8_t data;
		int rc = ftdi_read_data(&mpsse_ftdic, &data, 1);
		if (rc <= 0)
			break;
        qprintf( "unexpected rx byte: %02X\n", data);
		cnt++;

		if(cnt > 32)
			break;
	}
}

void mpsse_error(int status)
{
    mpsse_error_last = status;
	//mpsse_check_rx();
	if (mpsse_ftdic_open) {
		if (mpsse_ftdic_latency_set)
			ftdi_set_latency_timer(&mpsse_ftdic, mpsse_ftdi_latency);
		ftdi_usb_close(&mpsse_ftdic);
	}
	ftdi_deinit(&mpsse_ftdic);
}

uint8_t mpsse_recv_byte()
{
	uint8_t data;
	while (1) {
		int rc = ftdi_read_data(&mpsse_ftdic, &data, 1);
		if (rc < 0) {
            qprintf( "Read error.\n");
			mpsse_error(2);
            return 0;
		}
		if (rc == 1)
			break;
		usleep(100);
	}
	return data;
}

void mpsse_send_byte(uint8_t data)
{
	int rc = ftdi_write_data(&mpsse_ftdic, &data, 1);
	if (rc != 1) {
        qprintf( "Write error (single byte, rc=%d, expected %d)(%s).\n", rc, 1, ftdi_get_error_string(&mpsse_ftdic));
		mpsse_error(2);
        return;
	}
}


void mpsse_xfer(uint8_t* data_buffer, uint16_t send_length, uint16_t receive_length)
{
    if(mpsse_error_last != 0) {
        return;
    }
	if(send_length){
		int rc = ftdi_write_data(&mpsse_ftdic, data_buffer, send_length);
		if (rc != send_length) {
            qprintf( "Write error (rc=%d, expected %d)[%s]\n", rc, 1, ftdi_get_error_string(&mpsse_ftdic));
			mpsse_error(2);
            return;
		}
	}

	if(receive_length){
		/* Calls to ftdi_read_data may return with less data than requested if it wasn't ready. 
		 * We stay in this while loop to collect all the data that we expect. */
		uint16_t rx_len = 0;
		while(rx_len != receive_length){
			int rc = ftdi_read_data(&mpsse_ftdic, data_buffer + rx_len, receive_length - rx_len);
			if (rc < 0) {
                qprintf( "Read error (rc=%d)[%s]\n", rc, ftdi_get_error_string(&mpsse_ftdic));
				mpsse_error(2);
                return;
			}else{
				rx_len += rc;
			}
		}
	}
}

int mpsse_init(int ifnum, const char *devstr, int clkdiv)
{
	enum ftdi_interface ftdi_ifnum = INTERFACE_A;

	switch (ifnum) {
		case 0:
			ftdi_ifnum = INTERFACE_A;
			break;
		case 1:
			ftdi_ifnum = INTERFACE_B;
			break;
		case 2:
			ftdi_ifnum = INTERFACE_C;
			break;
		case 3:
			ftdi_ifnum = INTERFACE_D;
			break;
		default:
			ftdi_ifnum = INTERFACE_A;
			break;
	}

	ftdi_init(&mpsse_ftdic);
	ftdi_set_interface(&mpsse_ftdic, ftdi_ifnum);

	if (devstr != NULL) {
		if (ftdi_usb_open_string(&mpsse_ftdic, devstr)) {
            qprintf( "Can't find iCE FTDI USB device (device string %s).\n", devstr);
			mpsse_error(2);
            return -1;
		}
	} else {
		if (ftdi_usb_open(&mpsse_ftdic, 0x0403, 0x6010) && ftdi_usb_open(&mpsse_ftdic, 0x0403, 0x6014)) {
            qprintf( "Can't find iCE FTDI USB device (vendor_id 0x0403, device_id 0x6010 or 0x6014).\n");
			mpsse_error(2);
            return -1;
		}
	}

	mpsse_ftdic_open = true;

	if (ftdi_usb_reset(&mpsse_ftdic)) {
        qprintf( "Failed to reset iCE FTDI USB device.\n");
		mpsse_error(2);
        return -1;
	}

	if (ftdi_usb_purge_buffers(&mpsse_ftdic)) {
        qprintf( "Failed to purge buffers on iCE FTDI USB device.\n");
		mpsse_error(2);
        return -1;
	}

	if (ftdi_get_latency_timer(&mpsse_ftdic, &mpsse_ftdi_latency) < 0) {
        qprintf( "Failed to get latency timer (%s).\n", ftdi_get_error_string(&mpsse_ftdic));
		mpsse_error(2);
        return -1;
	}

	/* 1 is the fastest polling, it means 1 kHz polling */
	if (ftdi_set_latency_timer(&mpsse_ftdic, 1) < 0) {
        qprintf( "Failed to set latency timer (%s).\n", ftdi_get_error_string(&mpsse_ftdic));
		mpsse_error(2);
        return -1;
	}

	mpsse_ftdic_latency_set = true;

	/* Enter MPSSE (Multi-Protocol Synchronous Serial Engine) mode. Set all pins to output. */
	if (ftdi_set_bitmode(&mpsse_ftdic, 0xff, BITMODE_MPSSE) < 0) {
        qprintf( "Failed to set BITMODE_MPSSE on FTDI USB device.\n");
		mpsse_error(2);
        return -1;
	}

	int rc = ftdi_usb_purge_buffers(&mpsse_ftdic);
	if (rc != 0) {
        qprintf( "Purge error.\n");
		mpsse_error(2);
        return -1;
	}

	mpsse_send_byte(MC_TCK_X5);

        // set clock - actual clock is 6MHz/(clkdiv)
        mpsse_send_byte(MC_SET_CLK_DIV);
        mpsse_send_byte((clkdiv-1) & 0xff);
        mpsse_send_byte((clkdiv-1) >> 8);

	mpsse_send_byte(MC_SETB_LOW);
	mpsse_send_byte(0x08); /* Value */
	mpsse_send_byte(0x0B); /* Direction */
    return 0;
}

void mpsse_close(void)
{
	ftdi_set_latency_timer(&mpsse_ftdic, mpsse_ftdi_latency);
	ftdi_disable_bitbang(&mpsse_ftdic);
	ftdi_usb_close(&mpsse_ftdic);
	ftdi_deinit(&mpsse_ftdic);
}
