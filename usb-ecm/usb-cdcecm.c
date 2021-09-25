/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2010 Gareth McMullin <gareth@blacksphere.co.nz>
 * Copyright (C) 2015 Piotr Esden-Tempski <piotr@esden.net>
 * Copyright (C) 2018 Seb Holzapfel <schnommus@gmail.com>
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */

/**
 * \addtogroup Examples
 *
 * This example implements a USB CDC-ECM device (Ethernet NIC)
 *
 * TODO: Edit this text
 * When data is recieved, it will toggle the green LED and echo the data.
 * The red LED is toggled constantly and a string is sent over USB every
 * time the LED changes state as a heartbeat.
 */

#include <libopencm3/cm3/common.h>
#include <libopencm3/cm3/vector.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/usb/usbd.h>
#include <libopencm3/usb/cdc.h>
#include <libopencm3/efm32/wdog.h>
#include <libopencm3/efm32/gpio.h>
#include <libopencm3/efm32/cmu.h>

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// Makes this program compatible with Toboot-V2.0
#include <toboot.h>
TOBOOT_CONFIGURATION(0);
//TOBOOT_CONFIGURATION(TOBOOT_CONFIG_FLAG_AUTORUN); // Uncomment to boot directly to this app, instead of the DFU bootloader.

/* Default AHB (core clock) frequency of Tomu board */
//#define AHB_FREQUENCY 14000000

#define LED_GREEN_PORT GPIOA
#define LED_GREEN_PIN  GPIO0
#define LED_RED_PORT   GPIOB
#define LED_RED_PIN    GPIO7

#define VENDOR_ID                 0x1209    /* pid.code */
#define PRODUCT_ID                0x70b1    /* Assigned to Tomu project */
#define DEVICE_VER                0x0BEB    /* Program version */


#define NUM_USB_STRINGS 7
// CDC ECM definitions, which should probably be added to libopencm3 "usb/cdc.h".
#define USB_CDC_INTERFACE_SUBCLASS_ECM 0x06
#define USB_CDC_DESCRIPTOR_SUBTYPE_ETHERNET 0x0F 

#define USB_CDC_REQ_SET_ETHERNET_PACKET_FILTER 0x43 // Required by ECM standard
// TODO: These are optional
#define USB_CDC_REQ_SET_ETHERNET_MULTICAST_FILTERS 0x40
#define USB_CDC_REQ_SET_ETHERNET_POWER_MANAGEMENT_PATTERN_FILTER 0x41
#define USB_CDC_REQ_GET_ETHERNET_POWER_MANAGEMENT_PATTERN_FILTER 0x42
#define USB_CDC_REQ_GET_ETHERNET_STATISTIC 0x44

#define USB_CDC_ECM_NOTIFICATION_NETWORK_CONNECTION 0x0;
#define USB_CDC_ECM_NOTIFICATION_RESPONSE_AVAILABLE 0x1;
#define USB_CDC_ECM_NOTIFICATION_CONNECTION_SPEED_CHANGE 0x2A;

#define CDC_ECM_DATA_OUT_EP 0x01
#define CDC_ECM_DATA_IN_EP 0x81
// TODO: EFM32HG only supports 3 IN Endpoints, and I need CDC ACM Nofitication, CDC ACM Data, ECM Data.
// TODO: Add #ifdef. If CDC ACM is removed, I can add this optional notification EP for CDC ECM.
#define CDC_ECM_NOTIFY_EP 0x83
#define CDC_ACM_DATA_OUT_EP 0x02
#define CDC_ACM_DATA_IN_EP 0x82
#define CDC_ACM_NOTIFY_EP 0x83

#define CDC_ECM_COMM_INTERFACE_NUM 0
#define CDC_ECM_DATA_INTERFACE_NUM 1
#define CDC_ACM_COMM_INTERFACE_NUM 2
#define CDC_ACM_DATA_INTERFACE_NUM 3

//TODO: Experiment
// #define CDC_ACM_COMM_INTERFACE_NUM 0
// #define CDC_ACM_DATA_INTERFACE_NUM 1


struct usb_cdc_notification_header {
	uint8_t bmRequestType;
	uint8_t bNotificationCode;
	uint8_t wValue;
	uint8_t wIndex;
	uint8_t wLength;
} __attribute__((packed));

struct usb_cdc_notification_speed_change {
	struct usb_cdc_notification_header notify_header;
	uint32_t dlbitrate;
	uint32_t ulbitrate;
} __attribute__((packed));

struct usb_cdc_ecm_descriptor {
	uint8_t bFunctionLength;
	uint8_t bDescriptorType;
	uint8_t bDescriptorSubtype;
	uint8_t iMACAddress;
	uint32_t bmEthernetStatistics;
	uint16_t wMaxSegmentSize;
	uint16_t wNumberMCFilters;
	uint8_t bNumberPowerFilters;
} __attribute__((packed));

#define USB_PUTS_DELAY_USEC 2000

static volatile bool g_usbd_is_connected = false;
static usbd_device *g_usbd_dev = 0;

static const struct usb_device_descriptor dev = {
	.bLength = USB_DT_DEVICE_SIZE,
	.bDescriptorType = USB_DT_DEVICE,
	.bcdUSB = 0x0200,
	.bDeviceClass = USB_CLASS_CDC,
	// .bDeviceSubClass = 0, // Unused in CDC device
	.bDeviceSubClass = USB_CDC_INTERFACE_SUBCLASS_ECM, // TODO: Pointless, but this way in Blackberry
	.bDeviceProtocol = 0, // Unused in CDC device
	.bMaxPacketSize0 = 64,
	.idVendor = VENDOR_ID,
	.idProduct = PRODUCT_ID,
	.bcdDevice = DEVICE_VER,
	.iManufacturer = 1,
	.iProduct = 2,
	.iSerialNumber = 3,
	.bNumConfigurations = 1,
};

static const struct usb_endpoint_descriptor ecm_comm_endp[] = {{
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = CDC_ECM_NOTIFY_EP, // In
	.bmAttributes = USB_ENDPOINT_ATTR_INTERRUPT,
	.wMaxPacketSize = 16,
	// .bInterval = 255,
	.bInterval = 4 // Making it 4 like Blackberry
}};

static const struct usb_endpoint_descriptor ecm_data_endp[] = {{
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = CDC_ECM_DATA_IN_EP, // In
	.bmAttributes = USB_ENDPOINT_ATTR_BULK,
	.wMaxPacketSize = 64,
	// .bInterval = 1,
	.bInterval = 0 // Making it 0 like Blackberry

}, {
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = CDC_ECM_DATA_OUT_EP, // Out
	.bmAttributes = USB_ENDPOINT_ATTR_BULK,
	.wMaxPacketSize = 64,
	// .bInterval = 1,
	.bInterval = 0 // Making it 0 like Blackberry
}};

 // This notification endpoint isn't implemented. According to CDC spec it is
 // optional, but its absence causes a NULL pointer dereference in Linux cdc_acm driver.
static const struct usb_endpoint_descriptor acm_comm_endp[] = {{
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = CDC_ACM_NOTIFY_EP,
	.bmAttributes = USB_ENDPOINT_ATTR_INTERRUPT,
	.wMaxPacketSize = 16,
	.bInterval = 255,
}};

static const struct usb_endpoint_descriptor acm_data_endp[] = {{
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = CDC_ACM_DATA_OUT_EP,
	.bmAttributes = USB_ENDPOINT_ATTR_BULK,
	.wMaxPacketSize = 64,
	.bInterval = 1,
}, {
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = CDC_ACM_DATA_IN_EP,
	.bmAttributes = USB_ENDPOINT_ATTR_BULK,
	.wMaxPacketSize = 64,
	.bInterval = 1,
}};


static const struct {
	struct usb_cdc_header_descriptor header;
	struct usb_cdc_union_descriptor cdc_union;
	struct usb_cdc_ecm_descriptor ecm;
} __attribute__((packed)) cdcecm_interface_functional_descriptors = {
	.header = {
		.bFunctionLength = sizeof(struct usb_cdc_header_descriptor),
		.bDescriptorType = CS_INTERFACE,
		.bDescriptorSubtype = USB_CDC_TYPE_HEADER,
		.bcdCDC = 0x0110,
	},    
	.cdc_union = {
		.bFunctionLength = sizeof(struct usb_cdc_union_descriptor),
		.bDescriptorType = CS_INTERFACE,
		.bDescriptorSubtype = USB_CDC_TYPE_UNION,
		.bControlInterface = CDC_ECM_COMM_INTERFACE_NUM,
		.bSubordinateInterface0 = CDC_ECM_DATA_INTERFACE_NUM,
	 },
	 .ecm = {
		 .bFunctionLength = sizeof(struct usb_cdc_ecm_descriptor),
		 .bDescriptorType = CS_INTERFACE,
		 .bDescriptorSubtype = USB_CDC_DESCRIPTOR_SUBTYPE_ETHERNET,
		 .iMACAddress = 4,
		 .bmEthernetStatistics = 0x0,
		 .wMaxSegmentSize = 1514,
		 .wNumberMCFilters = 0,
		 .bNumberPowerFilters = 0,        
	 }
};


static const struct {
	struct usb_cdc_header_descriptor header;
	struct usb_cdc_call_management_descriptor call_mgmt;
	struct usb_cdc_acm_descriptor acm;
	struct usb_cdc_union_descriptor cdc_union;
} __attribute__((packed)) cdcacm_functional_descriptors = {
	.header = {
		.bFunctionLength = sizeof(struct usb_cdc_header_descriptor),
		.bDescriptorType = CS_INTERFACE,
		.bDescriptorSubtype = USB_CDC_TYPE_HEADER,
		.bcdCDC = 0x0110,
	},
	.call_mgmt = {
		.bFunctionLength = sizeof(struct usb_cdc_call_management_descriptor),
		.bDescriptorType = CS_INTERFACE,
		.bDescriptorSubtype = USB_CDC_TYPE_CALL_MANAGEMENT,
		.bmCapabilities = 0,
		.bDataInterface = CDC_ACM_DATA_INTERFACE_NUM,
	},
	.acm = {
		.bFunctionLength = sizeof(struct usb_cdc_acm_descriptor),
		.bDescriptorType = CS_INTERFACE,
		.bDescriptorSubtype = USB_CDC_TYPE_ACM,
		.bmCapabilities = 6,
	},
	.cdc_union = {
		.bFunctionLength = sizeof(struct usb_cdc_union_descriptor),
		.bDescriptorType = CS_INTERFACE,
		.bDescriptorSubtype = USB_CDC_TYPE_UNION,
		.bControlInterface = CDC_ACM_COMM_INTERFACE_NUM,
		.bSubordinateInterface0 = CDC_ACM_DATA_INTERFACE_NUM,
	 }
};

static const struct usb_interface_descriptor ecm_comm_iface[] = {{
	.bLength = USB_DT_INTERFACE_SIZE,
	.bDescriptorType = USB_DT_INTERFACE,
	.bInterfaceNumber = CDC_ECM_COMM_INTERFACE_NUM,
	.bAlternateSetting = 0,
	.bNumEndpoints = 0, // TODO: When CDC is removed, this should be 1
	.bInterfaceClass = USB_CLASS_CDC,
	.bInterfaceSubClass = USB_CDC_INTERFACE_SUBCLASS_ECM,
	.bInterfaceProtocol = 0x0,
	.iInterface = 5,

	// .endpoint = ecm_comm_endp, // TODO: #ifdef no ACM, not enough endpoints
	.endpoint = NULL,

	.extra = &cdcecm_interface_functional_descriptors,
	.extralen = sizeof(cdcecm_interface_functional_descriptors)
}};

static const struct usb_interface_descriptor ecm_data_iface[] = {
	{
		.bLength = USB_DT_INTERFACE_SIZE,
		.bDescriptorType = USB_DT_INTERFACE,
		.bInterfaceNumber = CDC_ECM_DATA_INTERFACE_NUM,
		.bAlternateSetting = 0,
		.bNumEndpoints = 0,
		.bInterfaceClass = USB_CLASS_DATA,
		.bInterfaceSubClass = 0,
		.bInterfaceProtocol = 0,
		.iInterface = 6,
		.extra = NULL,
		.extralen = 0,
	},
	{
		.bLength = USB_DT_INTERFACE_SIZE,
		.bDescriptorType = USB_DT_INTERFACE,
		.bInterfaceNumber = CDC_ECM_DATA_INTERFACE_NUM,
		.bAlternateSetting = 1,
		.bNumEndpoints = 2,
		.bInterfaceClass = USB_CLASS_DATA,
		.bInterfaceSubClass = 0,
		.bInterfaceProtocol = 0,
		.iInterface = 7,
		.extra = NULL,
		.extralen = 0,
		.endpoint = ecm_data_endp,
	}
};


static const struct usb_interface_descriptor acm_comm_iface[] = {{
	.bLength = USB_DT_INTERFACE_SIZE,
	.bDescriptorType = USB_DT_INTERFACE,
	.bInterfaceNumber = CDC_ACM_COMM_INTERFACE_NUM,
	.bAlternateSetting = 0,
	.bNumEndpoints = 1,
	.bInterfaceClass = USB_CLASS_CDC,
	.bInterfaceSubClass = USB_CDC_SUBCLASS_ACM,
	.bInterfaceProtocol = USB_CDC_PROTOCOL_AT,
	.iInterface = 0,

	.endpoint = acm_comm_endp,
	
	.extra = &cdcacm_functional_descriptors,
	.extralen = sizeof(cdcacm_functional_descriptors)
}};

static const struct usb_interface_descriptor acm_data_iface[] = {{
	.bLength = USB_DT_INTERFACE_SIZE,
	.bDescriptorType = USB_DT_INTERFACE,
	.bInterfaceNumber = CDC_ACM_DATA_INTERFACE_NUM,
	.bAlternateSetting = 0,
	.bNumEndpoints = 2,
	.bInterfaceClass = USB_CLASS_DATA,
	.bInterfaceSubClass = 0,
	.bInterfaceProtocol = 0,
	.iInterface = 0,

	.endpoint = acm_data_endp,
}};


// Must be defined to be allocated
// This is needed to store the current altsetting in case of more than one is defined
uint8_t ecm_data_iface_cur_altsetting = 0;

static const struct usb_interface ecm_ifaces[] = {
	{
	.num_altsetting = 1,
	.cur_altsetting = NULL,
	.altsetting = ecm_comm_iface,
	},
	{
	.num_altsetting = 2,
	.cur_altsetting = &ecm_data_iface_cur_altsetting,
	.altsetting = ecm_data_iface,
	}
};

static const struct usb_interface acm_ifaces[] = {
	{
	.num_altsetting = 1,
	.cur_altsetting = NULL,
	.altsetting = acm_comm_iface,
	},
	{
	.num_altsetting = 1,
	.cur_altsetting = NULL,
	.altsetting = acm_data_iface,
	}
};

static const struct usb_interface ecm_acm_ifaces[] = {
	{
	.num_altsetting = 1,
	.cur_altsetting = NULL,
	.altsetting = ecm_comm_iface,
	},
	{
	.num_altsetting = 2,
	.cur_altsetting = &ecm_data_iface_cur_altsetting,
	.altsetting = ecm_data_iface,
	},
	{
	.num_altsetting = 1,
	.cur_altsetting = NULL,
	.altsetting = acm_comm_iface,
	},
	{
	.num_altsetting = 1,
	.cur_altsetting = NULL,
	.altsetting = acm_data_iface,
	}
};

static const struct usb_config_descriptor ecm_config = {
	.bLength = USB_DT_CONFIGURATION_SIZE,
	.bDescriptorType = USB_DT_CONFIGURATION,
	.wTotalLength = 0, // Can be anything, it is updated automatically when the usb code prepares the descriptor
	.bNumInterfaces = 2, // CDC ECM control + data
	.bConfigurationValue = 1,
	.iConfiguration = 0,
	.bmAttributes = 0x80, // Bus powered
	.bMaxPower = 0x32,
	.interface = ecm_ifaces
};

static const struct usb_config_descriptor acm_config = {
	.bLength = USB_DT_CONFIGURATION_SIZE,
	.bDescriptorType = USB_DT_CONFIGURATION,
	.wTotalLength = 0, // Can be anything, it is updated automatically when the usb code prepares the descriptor
	.bNumInterfaces = 2, // CDC ACM control + data
	.bConfigurationValue = 1,
	.iConfiguration = 0,
	.bmAttributes = 0x80, // Bus powered
	.bMaxPower = 0x32,
	.interface = acm_ifaces
};

static const struct usb_config_descriptor ecm_acm_config = {
	.bLength = USB_DT_CONFIGURATION_SIZE,
	.bDescriptorType = USB_DT_CONFIGURATION,
	.wTotalLength = 0, // Can be anything, it is updated automatically when the usb code prepares the descriptor
	.bNumInterfaces = 4, // CDC ECM control + data, CDC ACM control + data
	.bConfigurationValue = 1,
	.iConfiguration = 0,
	.bmAttributes = 0x80, // Bus powered
	.bMaxPower = 0x32,
	.interface = ecm_acm_ifaces
};


static const char* usb_strings[NUM_USB_STRINGS] = {
	"Tomu", // Manufacturer
	"CDC-ECM Demo", // Product
	"DEMO", // SerialNumber
	"4CFCAA123BEB", // MAC Address - for testing purposes only!
	"CDC-ECM Interface", // CDC ECM control interface name
	"CDC-ECM Data Dummy", // CDC ECM data (first alternate) interface name
	"CDC-ECM Data" // CDC ECM data (second alternate) interface name
};

/* This busywait loop is roughly accurate when running at 24 MHz. */
void udelay_busy(uint32_t usecs)
{
	while (usecs --> 0) {
		/* This inner loop is 3 instructions, one of which is a branch.
		 * This gives us 4 cycles total.
		 * We want to sleep for 1 usec, and there are cycles per usec at 24 MHz.
		 * Therefore, loop 6 times, as 6*4=24.
		 */
		asm("mov   r1, #6");
		asm("retry:");
		asm("sub r1, #1");
		asm("bne retry");
		asm("nop");
	}
}

/* Buffer to be used for control requests. */
static uint8_t usbd_control_buffer[128];

//TODO: Add cases for ECM specific controll requests (ECM document 6.2), e.g. SetEthernetPacketFilter ?
static enum usbd_request_return_codes cdc_control_request(usbd_device *usbd_dev, struct usb_setup_data *req, uint8_t **buf,
		uint16_t *len, void (**complete)(usbd_device *usbd_dev, struct usb_setup_data *req))
{
	(void)complete;
	(void)buf;
	(void)usbd_dev;

	switch(req->bRequest) {
		
		case USB_CDC_REQ_SET_CONTROL_LINE_STATE: {
			g_usbd_is_connected = req->wValue & 1; // Check RTS bit
			
			if (!g_usbd_is_connected) { // Note: GPIO polarity is inverted
				gpio_set(LED_GREEN_PORT, LED_GREEN_PIN);
			} else {
				gpio_clear(LED_GREEN_PORT, LED_GREEN_PIN);
			}

			return USBD_REQ_HANDLED;
			}
		break;
		case USB_CDC_REQ_SET_LINE_CODING: 
			if (*len < sizeof(struct usb_cdc_line_coding))
				return USBD_REQ_NOTSUPP;
			return USBD_REQ_HANDLED;        
		break;
		
		case USB_CDC_REQ_SET_ETHERNET_MULTICAST_FILTERS:
		case USB_CDC_REQ_SET_ETHERNET_POWER_MANAGEMENT_PATTERN_FILTER:
		case USB_CDC_REQ_GET_ETHERNET_POWER_MANAGEMENT_PATTERN_FILTER:
		case USB_CDC_REQ_GET_ETHERNET_STATISTIC:
			return USBD_REQ_NOTSUPP;
		break;
		case USB_CDC_REQ_SET_ETHERNET_PACKET_FILTER:
			return USBD_REQ_HANDLED;
		break;
	}
	
	return USBD_REQ_NOTSUPP;
}

//TODO: Might not be needed, not commmands to control ECM. Maybe set IP and restart?

// Simple callback that echoes whatever is sent
static void cdcacm_data_rx_cb(usbd_device *usbd_dev, uint8_t ep)
{
	(void)ep;	

	char buf[64];
	int len = usbd_ep_read_packet(usbd_dev, CDC_ACM_DATA_OUT_EP, buf, sizeof(buf));

	if (len) {

		gpio_toggle(LED_RED_PORT, LED_RED_PIN);  

		if (buf[0] == '\r') {
			buf[1] = '\n';
			buf[2] = '\0';
			len++;
		}
		usbd_ep_write_packet(usbd_dev, CDC_ACM_DATA_IN_EP, buf, len);
		buf[len] = 0;
	}
}

static void usb_puts(char *s) {
	if (g_usbd_is_connected) {
		usbd_ep_write_packet(g_usbd_dev, CDC_ACM_DATA_IN_EP, s, strnlen(s, 64));
	}
}

static void cdcecm_data_rx_cb(usbd_device *usbd_dev, uint8_t ep)
{
	(void)ep;
	(void)usbd_dev;

	gpio_toggle(LED_RED_PORT, LED_RED_PIN); // TODO: Light the Red LED for debug

	usb_puts("\r\nECM Data Rx");
	udelay_busy(USB_PUTS_DELAY_USEC);


/*
	char buf[64];
	int len = usbd_ep_read_packet(usbd_dev, 0x04, buf, sizeof(buf)); //TODO: Hardcoded 0x04 ECM Data Out endpoint (from Host)
*/
	// TODO: keep reading until no more bulk data, or packet is 1514, and save to gloval packet buffer
	// TODO: How do I know the bulk transaction ended? See USB CDC ECM document, section 3.3.1 Segment Delineation
	// TODO: Basically, the last packet will be < 64. If the frame is a multiple of 64, there will be a zero-length packet.

	/*
	if (len == 64) {
		usb_puts("\r\nReceived full bulk packet");
	   	udelay_busy(USB_PUTS_DELAY_USEC);
	} else {
		usb_puts("\r\nReceived bulk packet size ");
	   	udelay_busy(USB_PUTS_DELAY_USEC);
		char string_buffer[3];
		itoa(len, string_buffer, 10);
		usb_puts(string_buffer);
   		udelay_busy(USB_PUTS_DELAY_USEC);
	}
	*/

	// TODO: Convert the data from binary to hex string and write to CDC ACM Data In endpoint
	// TODO: Write the Ethernet packet we received, no more than 64 bytes at a time since the endpoint buffer is 64
	// usbd_ep_write_packet(usbd_dev, 0x82, epbuf, len); //TODO: Hardcoded 0x82 ACM Data In endpoint (to Host)
 
}

static void cdc_altsetting_cc(usbd_device *usbd_dev, uint16_t wIndex, uint16_t wValue)
{
	(void)usbd_dev;

	if (wIndex != CDC_ECM_DATA_INTERFACE_NUM) {
		return;
	}

	if (wValue != 1) { // The ECM Data alternate interface
		return;
	}

	// Set alternate ECM data interface

	// TODO: If I remove the optional notification endpoint, this can be removed as well.
	return; // TODO: #ifdef no ACM
	// TODO: Add #ifdef to enable/disable the ECM notification endpoint

	struct usb_cdc_notification_header notify_buf;
	notify_buf.bmRequestType = 0xA1; // TODO: Not magic number
	notify_buf.wIndex = wIndex;
	// Send a notification for NetworkConnection + ConnectionSpeedChange
	// See USB CDC document 6.3.1
	notify_buf.bNotificationCode = USB_CDC_ECM_NOTIFICATION_NETWORK_CONNECTION;
	notify_buf.wLength = 0;        
	notify_buf.wValue = 1; // 1 = Connected, 0 = Disconnected.
	usbd_ep_write_packet(g_usbd_dev, CDC_ECM_NOTIFY_EP, &notify_buf, sizeof(notify_buf));
	
	udelay_busy(1000); // TODO: Needed?

	struct usb_cdc_notification_speed_change notify_speed_change;
	notify_speed_change.notify_header.bmRequestType = 0xA1; // TODO: Don't use a magic number
	notify_speed_change.notify_header.bNotificationCode = USB_CDC_ECM_NOTIFICATION_CONNECTION_SPEED_CHANGE;
	notify_speed_change.notify_header.wIndex = wIndex;
	notify_speed_change.notify_header.wValue = 0;
	notify_speed_change.notify_header.wLength = 8;
	notify_speed_change.dlbitrate = 10000000; // 10 Mbps
	notify_speed_change.ulbitrate = 10000000; // 10 Mbps
	usbd_ep_write_packet(g_usbd_dev, CDC_ECM_NOTIFY_EP, &notify_speed_change, sizeof(notify_speed_change));

	return;
}


static void cdc_set_config(usbd_device *usbd_dev, uint16_t wValue)
{
	(void)wValue;

	// usbd_ep_setup(usbd_dev, CDC_ECM_NOTIFY_EP, USB_ENDPOINT_ATTR_INTERRUPT, 16, 0);
	// TODO: Restore
	usbd_ep_setup(usbd_dev, CDC_ECM_DATA_OUT_EP, USB_ENDPOINT_ATTR_BULK, 64, cdcecm_data_rx_cb);
	usbd_ep_setup(usbd_dev, CDC_ECM_DATA_IN_EP, USB_ENDPOINT_ATTR_BULK, 64, 0);

	usbd_ep_setup(usbd_dev, CDC_ACM_DATA_OUT_EP, USB_ENDPOINT_ATTR_BULK, 64, cdcacm_data_rx_cb);
	usbd_ep_setup(usbd_dev, CDC_ACM_DATA_IN_EP, USB_ENDPOINT_ATTR_BULK, 64, 0);
	usbd_ep_setup(usbd_dev, CDC_ACM_NOTIFY_EP, USB_ENDPOINT_ATTR_INTERRUPT, 16, 0);

	// TODO: Restore when adding ECM back, use #ifdef since it's only useful for notification
	// usbd_register_set_altsetting_callback(g_usbd_dev, cdc_altsetting_cc);

	// The specified callback will be called if (type == (bmRequestType & type_mask)).

	//TODO: Add CDC ECM codes if applies here
	usbd_register_control_callback(
				usbd_dev,
				USB_REQ_TYPE_CLASS | USB_REQ_TYPE_INTERFACE,
				USB_REQ_TYPE_TYPE | USB_REQ_TYPE_RECIPIENT,
				cdc_control_request);
//TODO: Restore for ECM control

	usbd_register_control_callback(
				usbd_dev,
				USB_REQ_TYPE_IN | USB_REQ_TYPE_CLASS | USB_REQ_TYPE_INTERFACE,
				USB_REQ_TYPE_DIRECTION | USB_REQ_TYPE_TYPE | USB_REQ_TYPE_RECIPIENT,
				cdc_control_request);

}


void usb_isr(void)
{
	usbd_poll(g_usbd_dev);
}

void hard_fault_handler(void)
{
	while(1);
}

int main(void)
{
	bool line_was_connected = false;

	/* Disable the watchdog that the bootloader started. */
	WDOG_CTRL = 0;

	/* GPIO peripheral clock is necessary for us to set up the GPIO pins as outputs */
	cmu_periph_clock_enable(CMU_GPIO);

	/* Set up both LEDs as outputs */
	gpio_mode_setup(LED_RED_PORT, GPIO_MODE_WIRED_AND, LED_RED_PIN);
	gpio_mode_setup(LED_GREEN_PORT, GPIO_MODE_WIRED_AND, LED_GREEN_PIN);

	gpio_set(LED_RED_PORT, LED_RED_PIN); // Turn off red LED
	gpio_set(LED_GREEN_PORT, LED_GREEN_PIN); // Turn off green LED

	/* Configure the USB core & stack */
	g_usbd_dev = usbd_init(&efm32hg_usb_driver, &dev, &ecm_acm_config, usb_strings, NUM_USB_STRINGS, usbd_control_buffer, sizeof(usbd_control_buffer));
	usbd_register_set_config_callback(g_usbd_dev, cdc_set_config);
	
	/* Set the CPU Core to run from the trimmed USB clock, divided by 2.
	 * This will give the CPU Core a frequency of 24 MHz +/- 1% */
	CMU_CMD = (5 << 0);
	while (! (CMU_STATUS & (1 << 26)))
		;

	/* Enable USB IRQs */
	nvic_enable_irq(NVIC_USB_IRQ);

	while(1) {
		
		if (line_was_connected != g_usbd_is_connected) {
			if (g_usbd_is_connected) {
				udelay_busy(USB_PUTS_DELAY_USEC);
				usb_puts("\r\nUSB CDC ACM Connected!\r\n");
				udelay_busy(USB_PUTS_DELAY_USEC);
			}
			line_was_connected = g_usbd_is_connected;
		}

		// TODO: Remove this, find another use for the LEDs
		usb_puts("Beep3\n\r");            
		// gpio_toggle(LED_RED_PORT, LED_RED_PIN);  
		udelay_busy(500000);
	}
}
