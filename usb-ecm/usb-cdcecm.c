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

//#include "toboot.h"
//TOBOOT_CONFIGURATION(0);

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
    .bDeviceSubClass = 0, // Unused in CDC device
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

/*
 * This notification endpoint isn't implemented. According to CDC spec its
 * optional, but its absence causes a NULL pointer dereference in Linux
 * cdc_acm driver.
 */
static const struct usb_endpoint_descriptor acm_comm_endp[] = {{
    .bLength = USB_DT_ENDPOINT_SIZE,
    .bDescriptorType = USB_DT_ENDPOINT,
    .bEndpointAddress = 0x83, // In
    .bmAttributes = USB_ENDPOINT_ATTR_INTERRUPT,
    .wMaxPacketSize = 16,
    .bInterval = 255,
}};

static const struct usb_endpoint_descriptor acm_data_endp[] = {{
    .bLength = USB_DT_ENDPOINT_SIZE,
    .bDescriptorType = USB_DT_ENDPOINT,
    .bEndpointAddress = 0x01, // Out
    .bmAttributes = USB_ENDPOINT_ATTR_BULK,
    .wMaxPacketSize = 64,
    .bInterval = 1,
}, {
    .bLength = USB_DT_ENDPOINT_SIZE,
    .bDescriptorType = USB_DT_ENDPOINT,
    .bEndpointAddress = 0x82, // In
    .bmAttributes = USB_ENDPOINT_ATTR_BULK,
    .wMaxPacketSize = 64,
    .bInterval = 1,
}};

static const struct usb_endpoint_descriptor ecm_comm_endp[] = {{
    .bLength = USB_DT_ENDPOINT_SIZE,
    .bDescriptorType = USB_DT_ENDPOINT,
    .bEndpointAddress = 0x86, // In
    .bmAttributes = USB_ENDPOINT_ATTR_INTERRUPT,
    .wMaxPacketSize = 16,
    .bInterval = 255,
}};

static const struct usb_endpoint_descriptor ecm_data_endp[] = {{
    .bLength = USB_DT_ENDPOINT_SIZE,
    .bDescriptorType = USB_DT_ENDPOINT,
    .bEndpointAddress = 0x85, // In
    .bmAttributes = USB_ENDPOINT_ATTR_BULK,
    .wMaxPacketSize = 64,
    .bInterval = 1,

}, {
    .bLength = USB_DT_ENDPOINT_SIZE,
    .bDescriptorType = USB_DT_ENDPOINT,
    .bEndpointAddress = 0x04, // Out
    .bmAttributes = USB_ENDPOINT_ATTR_BULK,
    .wMaxPacketSize = 64,
    .bInterval = 1,
}};

static const struct {
    struct usb_cdc_header_descriptor header;
    struct usb_cdc_call_management_descriptor call_mgmt;
    struct usb_cdc_acm_descriptor acm;
    struct usb_cdc_union_descriptor cdc_union;
} __attribute__((packed)) cdcacm_interface_functional_descriptors = {
    .header = {
        .bFunctionLength = sizeof(struct usb_cdc_header_descriptor),
        .bDescriptorType = CS_INTERFACE,
        .bDescriptorSubtype = USB_CDC_TYPE_HEADER,
        .bcdCDC = 0x0110,
    },
    .call_mgmt = {
        .bFunctionLength =
            sizeof(struct usb_cdc_call_management_descriptor),
        .bDescriptorType = CS_INTERFACE,
        .bDescriptorSubtype = USB_CDC_TYPE_CALL_MANAGEMENT,
        .bmCapabilities = 0,
        .bDataInterface = 1,
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
        .bControlInterface = 0,
        .bSubordinateInterface0 = 1,
     }
};

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
        .bControlInterface = 2,
        .bSubordinateInterface0 = 3,
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


static const struct usb_interface_descriptor acm_comm_iface[] = {{
    .bLength = USB_DT_INTERFACE_SIZE,
    .bDescriptorType = USB_DT_INTERFACE,
    .bInterfaceNumber = 0,
    .bAlternateSetting = 0,
    .bNumEndpoints = 1, // Should be 1
    .bInterfaceClass = USB_CLASS_CDC,
    .bInterfaceSubClass = USB_CDC_SUBCLASS_ACM,
    .bInterfaceProtocol = USB_CDC_PROTOCOL_AT,
    .iInterface = 0,

    .endpoint = acm_comm_endp,

    .extra = &cdcacm_interface_functional_descriptors,
    .extralen = sizeof(cdcacm_interface_functional_descriptors)
}};

static const struct usb_interface_descriptor acm_data_iface[] = {{
    .bLength = USB_DT_INTERFACE_SIZE,
    .bDescriptorType = USB_DT_INTERFACE,
    .bInterfaceNumber = 1,
    .bAlternateSetting = 0,
    .bNumEndpoints = 2, // Should be 2
    .bInterfaceClass = USB_CLASS_DATA,
    .bInterfaceSubClass = 0,
    .bInterfaceProtocol = 0,
    .iInterface = 0,

    .endpoint = acm_data_endp,
}};


static const struct usb_interface_descriptor ecm_comm_iface[] = {{
    .bLength = USB_DT_INTERFACE_SIZE,
    .bDescriptorType = USB_DT_INTERFACE,
    .bInterfaceNumber = 2,
    .bAlternateSetting = 0,
    .bNumEndpoints = 1,
    .bInterfaceClass = USB_CLASS_CDC,
    .bInterfaceSubClass = USB_CDC_INTERFACE_SUBCLASS_ECM,
    .bInterfaceProtocol = 0x0,
    .iInterface = 5,

    .endpoint = ecm_comm_endp,

    .extra = &cdcecm_interface_functional_descriptors,
    .extralen = sizeof(cdcecm_interface_functional_descriptors)
}};

static const struct usb_interface_descriptor ecm_data_iface[] = {
    {
        .bLength = USB_DT_INTERFACE_SIZE,
        .bDescriptorType = USB_DT_INTERFACE,
        .bInterfaceNumber = 3,
        .bAlternateSetting = 0,
        .bNumEndpoints = 0,
        .bInterfaceClass = USB_CLASS_DATA,
        .bInterfaceSubClass = 0,
        .bInterfaceProtocol = 0,
        .iInterface = 6,
    },
    {
        .bLength = USB_DT_INTERFACE_SIZE,
        .bDescriptorType = USB_DT_INTERFACE,
        .bInterfaceNumber = 3,
        .bAlternateSetting = 1,
        .bNumEndpoints = 2,
        .bInterfaceClass = USB_CLASS_DATA,
        .bInterfaceSubClass = 0,
        .bInterfaceProtocol = 0,
        .iInterface = 7,
        .endpoint = ecm_data_endp,
    }
};

static const struct usb_interface ifaces[] = {
    {
    .num_altsetting = 1,
    .altsetting = acm_comm_iface,
    },
    {
    .num_altsetting = 1,
    .altsetting = acm_data_iface,
    },
    {
    .num_altsetting = 1,
    .altsetting = ecm_comm_iface,
    },
    {
    .num_altsetting = 2,
    .altsetting = ecm_data_iface,
    }
};

static const struct usb_interface ecm_ifaces[] = {
    {
    .num_altsetting = 1,
    .altsetting = ecm_comm_iface,
    },
    {
    .num_altsetting = 2,
    .altsetting = ecm_data_iface,
    }
};

static const struct usb_config_descriptor config = {
    .bLength = USB_DT_CONFIGURATION_SIZE,
    .bDescriptorType = USB_DT_CONFIGURATION,
    .wTotalLength = 0,
    .bNumInterfaces = 2, // Should be 4
    .bConfigurationValue = 1,
    .iConfiguration = 0,
    .bmAttributes = 0x80,
    .bMaxPower = 0x32,

    // .interface = ifaces, //TODO
    .interface = ecm_ifaces
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

//TODO: Should I add cases for ECM specific controll requests (ECM document 6.2), e.g. SetEthernetPacketFilter ?
static enum usbd_request_return_codes cdcacm_control_request(usbd_device *usbd_dev, struct usb_setup_data *req, uint8_t **buf,
        uint16_t *len, void (**complete)(usbd_device *usbd_dev, struct usb_setup_data *req))
{
    (void)complete;
    (void)buf;
    (void)usbd_dev;

    switch(req->bRequest) {
    case USB_CDC_REQ_SET_CONTROL_LINE_STATE: {
        g_usbd_is_connected = req->wValue & 1; /* Check RTS bit */
        if (!g_usbd_is_connected) /* Note: GPIO polarity is inverted */
            gpio_set(LED_GREEN_PORT, LED_GREEN_PIN);
        else
            gpio_clear(LED_GREEN_PORT, LED_GREEN_PIN);
        return USBD_REQ_HANDLED;
        }
    case USB_CDC_REQ_SET_LINE_CODING: 
        if(*len < sizeof(struct usb_cdc_line_coding))
            return 0;

        return USBD_REQ_HANDLED;
    }
    return 0;
}

//TODO: Copy the better shell, from opticspy or pwm-shell?
//TODO: Might not be needed, not commmands to control ECM. Maybe set IP and restart?

/* Simple callback that echoes whatever is sent */
static void cdcacm_data_rx_cb(usbd_device *usbd_dev, uint8_t ep)
{
    (void)ep;

    char buf[64];
    int len = usbd_ep_read_packet(usbd_dev, 0x01, buf, sizeof(buf)); //TODO: Hardcoded 0x01 ACM Data Out endpoint (from Host)

    if (len) {
        if (buf[0] == '\r') {
            buf[1] = '\n';
            buf[2] = '\0';
            len++;
        }
        usbd_ep_write_packet(usbd_dev, 0x82, buf, len); //TODO: Hardcoded 0x82 ACM Data In endpoint (to Host)
        buf[len] = 0;
    }
}

static void usb_puts(char *s);

static void cdcecm_data_rx_cb(usbd_device *usbd_dev, uint8_t ep)
{
    (void)ep;

    char buf[64];
    int len = usbd_ep_read_packet(usbd_dev, 0x04, buf, sizeof(buf)); //TODO: Hardcoded 0x04 ECM Data Out endpoint (from Host)

    // TODO: keep reading until no more bulk data, or packet is 1514, and save to gloval packet buffer
    // TODO: How do I know the bulk transaction ended? See USB CDC ECM document, section 3.3.1 Segment Delineation
    // TODO: Basically, the last packet will be < 64. If the frame is a multiple of 64, there will be a zero-length packet.

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
    

    // TODO: Convert the data from binary to hex string and write to CDC ACM Data In endpoint
    // TODO: Write the Ethernet packet we received, no more than 64 bytes at a time since the endpoint buffer is 64
    // usbd_ep_write_packet(usbd_dev, 0x82, epbuf, len); //TODO: Hardcoded 0x82 ACM Data In endpoint (to Host)
 
}

static void cdcacm_set_config(usbd_device *usbd_dev, uint16_t wValue)
{
    (void)wValue;

    // TODO: Use #define for endpoint constants

    usbd_ep_setup(usbd_dev, 0x01, USB_ENDPOINT_ATTR_BULK, 64, cdcacm_data_rx_cb);
    usbd_ep_setup(usbd_dev, 0x82, USB_ENDPOINT_ATTR_BULK, 64, 0);
    usbd_ep_setup(usbd_dev, 0x83, USB_ENDPOINT_ATTR_INTERRUPT, 16, 0);

    usbd_ep_setup(usbd_dev, 0x04, USB_ENDPOINT_ATTR_BULK, 64, cdcecm_data_rx_cb);
    usbd_ep_setup(usbd_dev, 0x85, USB_ENDPOINT_ATTR_BULK, 64, 0);
    usbd_ep_setup(usbd_dev, 0x86, USB_ENDPOINT_ATTR_INTERRUPT, 16, 0);

    usbd_register_control_callback(
                usbd_dev,
                USB_REQ_TYPE_CLASS | USB_REQ_TYPE_INTERFACE,
                USB_REQ_TYPE_TYPE | USB_REQ_TYPE_RECIPIENT,
                cdcacm_control_request);
}


static void usb_puts(char *s) {
    if (g_usbd_is_connected) {
        usbd_ep_write_packet(g_usbd_dev, 0x82, s, strnlen(s, 64));
    }
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

    /* Configure the USB core & stack */
    g_usbd_dev = usbd_init(&efm32hg_usb_driver, &dev, &config, usb_strings, NUM_USB_STRINGS, usbd_control_buffer, sizeof(usbd_control_buffer));
    usbd_register_set_config_callback(g_usbd_dev, cdcacm_set_config);

    //TODO: Add a call to usbd_register_set_altsetting_callback, and implement a callback to just print which interface + altsetting was selected

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
                udelay_busy(2000);
                usb_puts("\r\nHello world!\r\n");
                udelay_busy(2000);
            }
            line_was_connected = g_usbd_is_connected;
        }

        // TODO: Remove this, find another use for the LEDs
        usb_puts("toggling LED\n\r");            
        gpio_toggle(LED_RED_PORT, LED_RED_PIN);  
        udelay_busy(300000);
    }
}
