/**
 * \addtogroup Examples
 *
 * This example implements a USB CDC-NCM device (Ethernet NIC)
 *
 * TODO: Edit this text
 * When data is recieved, it will toggle the green LED and echo the data.
 * The red LED is toggled constantly and a string is sent over USB every
 * time the LED changes state as a heartbeat.
 */

#include <libopencm3/cm3/common.h>
#include <libopencm3/cm3/vector.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/cm3/systick.h>
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

/* Systick interrupt frequency, Hz */
#define SYSTICK_FREQUENCY 1000

/* USB (core clock) frequency of Tomu board */
#define USB_CLK_FREQUENCY 24000000

#define LED_GREEN_PORT GPIOA
#define LED_GREEN_PIN  GPIO0
#define LED_RED_PORT   GPIOB
#define LED_RED_PIN    GPIO7

#define VENDOR_ID                 0x1209    /* pid.code */
#define PRODUCT_ID                0x70b1    /* Assigned to Tomu project */
#define DEVICE_VER                0x0BEB    /* Program version */

#define EP_WRITE_RETRY_DELAY_USECS 50

#define NUM_USB_STRINGS 7
// CDC NCM definitions, which should probably be added to libopencm3 "usb/cdc.h".
#define USB_CDC_DESCRIPTOR_SUBTYPE_ETHERNET 0x0F
#define USB_CDC_DESCRIPTOR_SUBTYPE_NCM 0x1A
#define USB_CDC_NCM_COMM_INTERFACE_SUBCLASS 0x0D
#define NCM_DATA_CLASS_INTERFACE_PROTOCOL 0x01 // Network Transfer Block

// NCM class-specific requests
// Required NCM requests
#define USB_CDC_REQ_GET_NTB_PARAMETERS 0x80
#define USB_CDC_REQ_GET_NTB_INPUT_SIZE 0x85
#define USB_CDC_REQ_SET_NTB_INPUT_SIZE 0x86
// Optional NCM requests
#define USB_CDC_REQ_SET_ETHERNET_MULTICAST_FILTERS 0x40
#define USB_CDC_REQ_SET_ETHERNET_POWER_MANAGEMENT_PATTERN_FILTER 0x41
#define USB_CDC_REQ_GET_ETHERNET_POWER_MANAGEMENT_PATTERN_FILTER 0x42
#define USB_CDC_REQ_SET_ETHERNET_PACKET_FILTER 0x43 // Optional for NCM, was required by ECM.
#define USB_CDC_REQ_GET_ETHERNET_STATISTIC 0x44
#define USB_CDC_REQ_GET_NET_ADDRESS 0x81
#define USB_CDC_REQ_SET_NET_ADDRESS 0x82
#define USB_CDC_REQ_GET_NTB_FORMAT 0x83
#define USB_CDC_REQ_SET_NTB_FORMAT 0x84
#define USB_CDC_REQ_GET_MAX_DATAGRAM_SIZE 0x87
#define USB_CDC_REQ_SET_MAX_DATAGRAM_SIZE 0x88
#define USB_CDC_REQ_GET_CRC_MODE 0x89
#define USB_CDC_REQ_SET_CRC_MODE 0x8A

#define USB_CDC_ECM_NOTIFICATION_NETWORK_CONNECTION 0x0
#define USB_CDC_ECM_NOTIFICATION_RESPONSE_AVAILABLE 0x1
#define USB_CDC_ECM_NOTIFICATION_CONNECTION_SPEED_CHANGE 0x2A

// EFM32HG supports 3 IN and 3 OUT Endpoints.
#define CDC_NCM_DATA_OUT_EP 0x01
#define CDC_NCM_DATA_IN_EP 0x81
#define CDC_NCM_NOTIFY_EP 0x83

#define CDC_NCM_COMM_INTERFACE_NUM 0
#define CDC_NCM_DATA_INTERFACE_NUM 1

#define MAX_ICMP_FRAME 128   // Plenty for normal pings (most are < 100 bytes total)

struct usb_cdc_notification_header {
	uint8_t bmRequestType;
	uint8_t bNotificationCode;
	uint16_t wValue;
	uint16_t wIndex;
	uint16_t wLength;
} __attribute__((packed));

struct usb_cdc_notification_speed_change {
	struct usb_cdc_notification_header notify_header;
	uint32_t dlbitrate;
	uint32_t ulbitrate;
} __attribute__((packed));

// CDC NCM 1.1 5.4 Ethernet Networking Functional Descriptor
struct usb_cdc_ethernet_descriptor {
	uint8_t bFunctionLength;
	uint8_t bDescriptorType;
	uint8_t bDescriptorSubtype;
	uint8_t iMACAddress;
	uint32_t bmEthernetStatistics;
	uint16_t wMaxSegmentSize;
	uint16_t wNumberMCFilters;
	uint8_t bNumberPowerFilters;
} __attribute__((packed));

// CDC NCM 1.1 6.2.1 NCM Functional Descriptor
struct usb_cdc_ncm_descriptor {
	uint8_t bFunctionLength;
	uint8_t bDescriptorType;
	uint8_t bDescriptorSubtype;
	uint16_t bcdNcmVersion;
	uint8_t bmNetworkCapabilities;
} __attribute__((packed));

#define USB_PUTS_DELAY_USEC 2000

static usbd_device *g_usbd_dev = 0;

/* RX buffer for full NTB (one transfer) - matches our advertised dwNtbOutMaxSize */
// TODO: Rename to g_ ?
static uint8_t ntb_rx_buf[600]; // TODO: Can it be reduced to 1542 from 2048? Consider alignment.
static volatile uint16_t ntb_rx_len = 0; // TODO: should this be volatile?

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

static const struct usb_endpoint_descriptor ncm_comm_endp[] = {{
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = CDC_NCM_NOTIFY_EP, // In
	.bmAttributes = USB_ENDPOINT_ATTR_INTERRUPT,
	.wMaxPacketSize = 16,
	.bInterval = 255,	
}};

static const struct usb_endpoint_descriptor ncm_data_endp[] = {{
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = CDC_NCM_DATA_IN_EP, // In
	.bmAttributes = USB_ENDPOINT_ATTR_BULK,
	.wMaxPacketSize = 64,
	// .bInterval = 1,
	.bInterval = 0 // TODO: Making it 0 like Blackberry

}, {
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = CDC_NCM_DATA_OUT_EP, // Out
	.bmAttributes = USB_ENDPOINT_ATTR_BULK,
	.wMaxPacketSize = 64,
	// .bInterval = 1,
	.bInterval = 0 // TODO: Making it 0 like Blackberry
}};

// See: Table 6-1: NCM Communication Interface Descriptor Requirements
static const struct {
	struct usb_cdc_header_descriptor header;
	struct usb_cdc_union_descriptor cdc_union;
	struct usb_cdc_ethernet_descriptor cdc_ethernet;
	struct usb_cdc_ncm_descriptor cdc_ncm;
} __attribute__((packed)) cdcncm_interface_functional_descriptors = {
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
		.bControlInterface = CDC_NCM_COMM_INTERFACE_NUM,
		.bSubordinateInterface0 = CDC_NCM_DATA_INTERFACE_NUM,
	 },
	 .cdc_ethernet = {
		.bFunctionLength = sizeof(struct usb_cdc_ethernet_descriptor),
		.bDescriptorType = CS_INTERFACE,
		.bDescriptorSubtype = USB_CDC_DESCRIPTOR_SUBTYPE_ETHERNET,
		.iMACAddress = 4,
		.bmEthernetStatistics = 0x0,
		.wMaxSegmentSize = 1514,
		.wNumberMCFilters = 0,
		.bNumberPowerFilters = 0,        
	 },
	.cdc_ncm = {
		.bFunctionLength = sizeof(struct usb_cdc_ncm_descriptor),
		.bDescriptorType = CS_INTERFACE,
		.bDescriptorSubtype = USB_CDC_DESCRIPTOR_SUBTYPE_NCM,
		.bcdNcmVersion = 0x0100,
		.bmNetworkCapabilities = 0x02, // Bitmap. Currently setting only D1 (GetNetAddress/SetNetAddress)
	 }
};

static const struct usb_interface_descriptor ncm_comm_iface[] = {{
	.bLength = USB_DT_INTERFACE_SIZE,
	.bDescriptorType = USB_DT_INTERFACE,
	.bInterfaceNumber = CDC_NCM_COMM_INTERFACE_NUM,
	.bAlternateSetting = 0,
	.bNumEndpoints = 1,
	.bInterfaceClass = USB_CLASS_CDC,
	.bInterfaceSubClass = USB_CDC_NCM_COMM_INTERFACE_SUBCLASS,
	.bInterfaceProtocol = 0x0,
	.iInterface = 5,
	.endpoint = ncm_comm_endp,
	.extra = &cdcncm_interface_functional_descriptors,
	.extralen = sizeof(cdcncm_interface_functional_descriptors)
}};

static const struct usb_interface_descriptor ncm_data_iface[] = {
	{
		.bLength = USB_DT_INTERFACE_SIZE,
		.bDescriptorType = USB_DT_INTERFACE,
		.bInterfaceNumber = CDC_NCM_DATA_INTERFACE_NUM,
		.bAlternateSetting = 0,
		.bNumEndpoints = 0,
		.bInterfaceClass = USB_CLASS_DATA,
		.bInterfaceSubClass = 0,
		.bInterfaceProtocol = NCM_DATA_CLASS_INTERFACE_PROTOCOL,
		.iInterface = 6,
		.extra = NULL,
		.extralen = 0,
	},
	{
		.bLength = USB_DT_INTERFACE_SIZE,
		.bDescriptorType = USB_DT_INTERFACE,
		.bInterfaceNumber = CDC_NCM_DATA_INTERFACE_NUM,
		.bAlternateSetting = 1,
		.bNumEndpoints = 2,
		.bInterfaceClass = USB_CLASS_DATA,
		.bInterfaceSubClass = 0,
		.bInterfaceProtocol = NCM_DATA_CLASS_INTERFACE_PROTOCOL,
		.iInterface = 7,
		.extra = NULL,
		.extralen = 0,
		.endpoint = ncm_data_endp,
	}
};

// Must be defined to be allocated
// This is needed to store the current altsetting in case of more than one is defined
uint8_t ncm_data_iface_cur_altsetting = 0;

static const struct usb_interface ncm_ifaces[] = {
	{
	.num_altsetting = 1,
	.cur_altsetting = NULL,
	.altsetting = ncm_comm_iface,
	},
	{
	.num_altsetting = 2,
	.cur_altsetting = &ncm_data_iface_cur_altsetting,
	.altsetting = ncm_data_iface,
	}
};

static const struct usb_config_descriptor ncm_config = {
	.bLength = USB_DT_CONFIGURATION_SIZE,
	.bDescriptorType = USB_DT_CONFIGURATION,
	.wTotalLength = 0, // Can be anything, it is updated automatically when the usb code prepares the descriptor
	.bNumInterfaces = 2, // CDC NCM control + data
	.bConfigurationValue = 1,
	.iConfiguration = 0,
	.bmAttributes = 0x80, // Bus powered
	.bMaxPower = 0x32,
	.interface = ncm_ifaces
};

static const char* usb_strings[NUM_USB_STRINGS] = {
	"Tomu", // Manufacturer
	"CDC-NCM Demo", // Product
	"DEMO", // SerialNumber
	"4CFCAA123BEB", // MAC Address - for testing purposes only!
	"CDC-NCM Interface", // CDC NCM control interface name
	"CDC-NCM Data Dummy", // CDC NCM data (first alternate) interface name
	"CDC-NCM Data" // CDC NCM data (second alternate) interface name
};

// This busywait loop is roughly accurate when running at 24 MHz.
void udelay_busy(uint32_t usecs)
{
	while (usecs --> 0) {
		/* This inner loop is 3 instructions, one of which is a branch.
		 * This gives us 4 cycles total.
		 * We want to sleep for 1 usec, and there are cycles per usec at 24 MHz.
		 * Therefore, loop 6 times, as 6*4=24.
		 */
		asm volatile(
			"mov   r1, #6\n"
			"retry:\n"
			"sub   r1, #1\n"
			"bne   retry\n"
			"nop"
			: : : "r1"
		);
	}
}

/* Buffer to be used for control requests. */
static uint8_t usbd_control_buffer[128];

// Minimal NTB parameters we advertise (NCM 1.0, NTB-16 only, single frame, 2 KiB buffers)
// CDC NCM 1.1 7.2.1 GetNtbParameters Data - NTB Parameter Structure (Table 7-3)
// Fields are stored as Little Endian
static const uint8_t ntb_parameters[0x1C] = {
	0x1C, 0x00,                     /* wLength */
	0x01, 0x00,                     /* bmNtbFormatsSupported = NTB-16 only */
	0x00, 0x08, 0x00, 0x00,         /* dwNtbInMaxSize  = 2048 bytes */
	0x04, 0x00,                     /* wNdpInDivisor */
	0x00, 0x00,                     /* wNdpInPayloadRemainder */
	0x04, 0x00,                     /* wNdpInAlignment */
	0x00, 0x00,                     /* Reserved, padding. */
	0x00, 0x08, 0x00, 0x00,         /* dwNtbOutMaxSize = 2048 bytes */
	0x04, 0x00,                     /* wNdpOutDivisor */
	0x00, 0x00,                     /* wNdpOutPayloadRemainder */
	0x04, 0x00,                     /* wNdpOutAlignment */
	0x01, 0x00                      /* wNtbOutMaxDatagrams = 1 (only one frame per OUT NTB) */
};

// Current NTB input size the host told us (we'll store it)
static uint32_t g_ntb_in_max_size = 2048;

// MAC Address in network byte order
static uint8_t g_mac_address[6] = {0x4C, 0xFC, 0xAA, 0x12, 0x3B, 0xEB};
static uint8_t g_server_mac_address[6] = {0x02, 0x00, 0x00, 0x00, 0x00, 0x01};
// Default IP addresses for host NIC and "remote server" we simulate.
static uint8_t g_host_ip_address[4] = {192, 168, 7, 2};
static uint8_t g_server_ip_address[4] = {192, 168, 7, 1};

static uint32_t rx_count = 0, tx_count = 0;

static enum usbd_request_return_codes cdc_control_request(usbd_device *usbd_dev, struct usb_setup_data *req, uint8_t **buf,
		uint16_t *len, void (**complete)(usbd_device *usbd_dev, struct usb_setup_data *req))
{
	(void)complete;
	(void)usbd_dev;

	switch(req->bRequest) {
		// NCM control requests (CDC NCM 1.1 document, section 7.2)

		// Optional NCM requests we do not support:
		case USB_CDC_REQ_SET_ETHERNET_MULTICAST_FILTERS:
		case USB_CDC_REQ_SET_ETHERNET_POWER_MANAGEMENT_PATTERN_FILTER:
		case USB_CDC_REQ_GET_ETHERNET_POWER_MANAGEMENT_PATTERN_FILTER:
		case USB_CDC_REQ_SET_ETHERNET_PACKET_FILTER:
		case USB_CDC_REQ_GET_ETHERNET_STATISTIC:
		case USB_CDC_REQ_GET_NTB_FORMAT:
		case USB_CDC_REQ_SET_NTB_FORMAT:
		case USB_CDC_REQ_GET_MAX_DATAGRAM_SIZE:
		case USB_CDC_REQ_SET_MAX_DATAGRAM_SIZE:
		case USB_CDC_REQ_GET_CRC_MODE:
		case USB_CDC_REQ_SET_CRC_MODE:
			return USBD_REQ_NOTSUPP;
		break;
	
		// How replies work in libopencm3:
		// *buf: Point this to the memory buffer containing the data you want to send.
		// *len: Set this to the number of bytes to be sent.
		// Make sure buf is still allocated when the function returns, e.g. static global buffer.
		// Set the "*complete" callback if actions are needed after the Control request is complete, acknowledged by the Host.

		// Required NCM requests we must support:
		case USB_CDC_REQ_GET_NTB_PARAMETERS:
			*buf = (uint8_t *) ntb_parameters;
			*len = sizeof(ntb_parameters);
			return USBD_REQ_HANDLED;
		break;

		case USB_CDC_REQ_GET_NTB_INPUT_SIZE:
			*buf = (uint8_t *)&g_ntb_in_max_size;     /* little-endian 32-bit */
			*len = 4;
			return USBD_REQ_HANDLED;
		break;

		case USB_CDC_REQ_SET_NTB_INPUT_SIZE:
			if (*len == 4) {
				g_ntb_in_max_size = *(uint32_t *)(*buf);  // dwNtbInMaxSize from Host
				return USBD_REQ_HANDLED;
			} else {
				return USBD_REQ_NOTSUPP;
			}
		break;

		// Support NET ADDRESS, since we set bit D1 in bmNetworkCapabilities.
		case USB_CDC_REQ_GET_NET_ADDRESS:
			*buf = (uint8_t *) &g_mac_address;
			*len = sizeof(g_mac_address);
			return USBD_REQ_HANDLED;
		break;
		case USB_CDC_REQ_SET_NET_ADDRESS:
			if (*len == sizeof(g_mac_address)) {
				memcpy(g_mac_address, (uint8_t *)(*buf), sizeof(g_mac_address));
				return USBD_REQ_HANDLED;
			} else {
				return USBD_REQ_NOTSUPP;
			}
		break;		
	}
	
	return USBD_REQ_NOTSUPP;
}

//TODO: Might use these later
/*
static char nibble_to_hexchar(uint8_t nibble) {
	if (nibble < 10) {
		return (char) (nibble + '0');
	} else if (nibble < 16) {
		return (char) (nibble - 10 + 'A');
	} else {
		return 'X';
	}
}

static void buf_to_hexstring(uint8_t* source, char* dest, unsigned int start_index, unsigned int end_index) {
	for (unsigned int current_byte_index = start_index; current_byte_index <= end_index; ++current_byte_index) {
		dest[(current_byte_index - start_index)*2] = nibble_to_hexchar(source[current_byte_index] >> 4); // MSB nibble
		dest[(current_byte_index - start_index)*2 + 1] = nibble_to_hexchar(source[current_byte_index] & 0xF); // LSB nibble
	}
}
*/

/* Internet checksum (RFC 1071 / RFC 791)
 * Used for IPv4 header, ICMP, and (if needed later) UDP/TCP.
 * The caller must zero the checksum field before calling.
 */
static uint16_t internet_checksum(const uint8_t *buf, size_t len)
{
    uint32_t sum = 0;
    const uint16_t *p = (const uint16_t *)buf;

    while (len > 1) {
        sum += *p++;
        len -= 2;
    }
    if (len) {
        sum += *(const uint8_t *)p;
    }

    sum = (sum >> 16) + (sum & 0xFFFF);
    sum += (sum >> 16);
    return ~sum;
}

/* Minimal NTB-16 TX helper - builds a one-frame NTB and sends it */
static void ncm_send_frame(const uint8_t *frame, uint16_t frame_len)
{
    if (frame_len == 0 || frame_len > 1514) return;

    // Simple one-frame NTB-16
    // TODO: 2048 bytes matches our advertised dwNtbOutMaxSize, although we only used headers (12+16) + Ethernet frame (1514)
	// TODO: Perhaps this can be reduced to 1542 from 2048? Consider alignment.
	static uint8_t ntb_buf[600];
	uint16_t ntb_len = 0;

    /* NTH16 - "NCMH" */
    ntb_buf[ntb_len++] = 'N'; ntb_buf[ntb_len++] = 'C'; ntb_buf[ntb_len++] = 'M'; ntb_buf[ntb_len++] = 'H';
    ntb_buf[ntb_len++] = 0x0C; ntb_buf[ntb_len++] = 0x00;  /* wHeaderLength */
    ntb_buf[ntb_len++] = 0x00; ntb_buf[ntb_len++] = 0x00;  /* wSequence (we ignore) */ //TODO: Can we ignore?
    ntb_buf[ntb_len++] = 0x00; ntb_buf[ntb_len++] = 0x00;  /* wBlockLength will be filled later */
    ntb_buf[ntb_len++] = 0x0C; ntb_buf[ntb_len++] = 0x00;  /* wNdpIndex = 12 (right after NTH) */

    /* NDP16 - "NCM0" */
    ntb_buf[ntb_len++] = 'N'; ntb_buf[ntb_len++] = 'C'; ntb_buf[ntb_len++] = 'M'; ntb_buf[ntb_len++] = '0';
    ntb_buf[ntb_len++] = 0x10; ntb_buf[ntb_len++] = 0x00;  /* wLength = 16 (one entry + zero) */
    ntb_buf[ntb_len++] = 0x00; ntb_buf[ntb_len++] = 0x00;  /* wNextNdpIndex = 0 */
    /* First datagram entry */
    ntb_buf[ntb_len++] = 0x1C; ntb_buf[ntb_len++] = 0x00;  /* wDatagramIndex = 12+16 (after NTB+NDP) */
    ntb_buf[ntb_len++] = frame_len & 0xFF;
    ntb_buf[ntb_len++] = (frame_len >> 8) & 0xFF;          /* wDatagramLength */
    /* Zero terminator */
    ntb_buf[ntb_len++] = 0x00; ntb_buf[ntb_len++] = 0x00;
    ntb_buf[ntb_len++] = 0x00; ntb_buf[ntb_len++] = 0x00;

    /* Copy the actual Ethernet frame */
    memcpy(ntb_buf + ntb_len, frame, frame_len);
    ntb_len += frame_len;

    /* Fix wBlockLength in NTH */
    ntb_buf[8] = ntb_len & 0xFF;
    ntb_buf[9] = (ntb_len >> 8) & 0xFF;

    // Send in 64-byte chunks (required on full-speed USB)
	uint16_t sent = 0;
	while (sent < ntb_len) {
		uint16_t chunk = (ntb_len - sent) > 64 ? 64 : (ntb_len - sent);
		uint16_t return_value = usbd_ep_write_packet(g_usbd_dev, CDC_NCM_DATA_IN_EP, ntb_buf + sent, chunk);
		while (!return_value) {
			udelay_busy(EP_WRITE_RETRY_DELAY_USECS);
			return_value = usbd_ep_write_packet(g_usbd_dev, CDC_NCM_DATA_IN_EP, ntb_buf + sent, chunk);
		}
		sent += chunk;
	}
	
	tx_count++;
    gpio_toggle(LED_GREEN_PORT, LED_GREEN_PIN);  /* visual TX feedback */
}

// UDP debug packet
static void send_udp_debug(void)
{
	uint8_t packet[128];
    uint16_t len = 0;

    /* Ethernet header (14 bytes) */
    memcpy(packet + len, g_mac_address, 6);          len += 6;   // dst = Host NIC
    memcpy(packet + len, g_server_mac_address, 6);   len += 6;   // src = Tomu (remote)
    packet[len++] = 0x08; packet[len++] = 0x00;                 // EtherType = IPv4

	/* IPv4 header (20 bytes) */
    uint16_t ip_hdr_start = len;
    packet[len++] = 0x45; packet[len++] = 0x00;
    packet[len++] = 0x00; packet[len++] = 0x00;   // total length (fix later)
    packet[len++] = 0x00; packet[len++] = 0x01;   // Identification
    packet[len++] = 0x00; packet[len++] = 0x00;   // Flags + Fragment Offset
    packet[len++] = 0x40; packet[len++] = 0x11;   // TTL, protocol = UDP
    packet[len++] = 0x00; packet[len++] = 0x00;   // checksum (zero for now)
    memcpy(packet + len, g_server_ip_address, 4); len += 4;   // src IP
    memcpy(packet + len, g_host_ip_address, 4);   len += 4;   // dst IP

	/* UDP header (8 bytes) */
	//TODO: Make destination port a constant define instead of hard-coded?
    packet[len++] = 0x04; packet[len++] = 0xD2;                 // src port 1234
    packet[len++] = 0x04; packet[len++] = 0xD2;                 // dst port 1234
    packet[len++] = 0x00; packet[len++] = 0x00;                 // UDP Length (fix later)
    packet[len++] = 0x00; packet[len++] = 0x00;                 // UDP checksum = 0 (allowed)

	/* Payload */
    char status[16];
    strcpy((char*)packet + len, "RX:"); len += 3;
    itoa(rx_count, status, 10);
    strcpy((char*)packet + len, status); len += strlen(status);
    strcpy((char*)packet + len, " TX:"); len += 4;
    itoa(tx_count, status, 10);
    strcpy((char*)packet + len, status); len += strlen(status);
    packet[len++] = '\r'; packet[len++] = '\n';

	/* Fix lengths */
    uint16_t ip_total = len - 14; // Skip 14 bytes of Ethernet header
    packet[16] = (ip_total >> 8) & 0xFF;
    packet[17] = ip_total & 0xFF;

    uint16_t udp_len = len - 34;   // 14 (Eth) + 20 (IP)
    packet[38] = (udp_len >> 8) & 0xFF;
    packet[39] = udp_len & 0xFF;

	/* Calculate and insert IPv4 header checksum */
    uint16_t csum = internet_checksum(packet + ip_hdr_start, 20);
    packet[ip_hdr_start + 10] = csum & 0xFF;
    packet[ip_hdr_start + 11] = (csum >> 8) & 0xFF;

    ncm_send_frame(packet, len);
}

/* Minimal NTB-16 parser - called only when a complete NTB has been received */
static void ncm_parse_and_echo_ntb(void)
{
    if (ntb_rx_len < 12) return;

    /* Check NTH16 signature "NCMH" */
    if (ntb_rx_buf[0] != 'N' || ntb_rx_buf[1] != 'C' || ntb_rx_buf[2] != 'M' || ntb_rx_buf[3] != 'H') {
        goto reset;
    }

    /* Get NDP index from NTH (offset 10) */
    uint16_t ndp_idx = ntb_rx_buf[10] | (ntb_rx_buf[11] << 8);
    if (ndp_idx + 12 > ntb_rx_len) {
		goto reset;
	}

    /* Check NDP16 signature "NCM0" */
    if (ntb_rx_buf[ndp_idx] != 'N' || ntb_rx_buf[ndp_idx+1] != 'C' ||
        ntb_rx_buf[ndp_idx+2] != 'M' || ntb_rx_buf[ndp_idx+3] != '0') {
        goto reset;
    }

    /* First datagram pointer (offset 8 inside NDP) */
    uint16_t frame_offset = ntb_rx_buf[ndp_idx+8] | (ntb_rx_buf[ndp_idx+9] << 8);
    uint16_t frame_len    = ntb_rx_buf[ndp_idx+10] | (ntb_rx_buf[ndp_idx+11] << 8);

    if (frame_offset == 0 || frame_len == 0 || frame_offset + frame_len > ntb_rx_len) {
        goto reset;
    }

    // Valid Ethernet frame
	rx_count++;
	gpio_toggle(LED_RED_PORT, LED_RED_PIN); // TODO: Debug

	uint8_t* packet = &ntb_rx_buf[frame_offset];

	/* Check for ARP request for our IP */
    if (frame_len >= 42 && 
        packet[12] == 0x08 && packet[13] == 0x06 &&           // EtherType ARP
        packet[21] == 0x01 &&                                 // opcode = request
        memcmp(packet + 38, g_server_ip_address, 4) == 0) {   // target IP = ours
        
        /* Build ARP reply */
        uint8_t arp_reply[42];
		memcpy(arp_reply, g_mac_address, 6);                // dst = host
        memcpy(arp_reply + 6, g_server_mac_address, 6);     // src = us		
        arp_reply[12] = 0x08; arp_reply[13] = 0x06;         // EtherType ARP
        arp_reply[14] = 0x00; arp_reply[15] = 0x01;         // HW type Ethernet
        arp_reply[16] = 0x08; arp_reply[17] = 0x00;         // protocol IP
        arp_reply[18] = 6; arp_reply[19] = 4;               // HW / proto length
        arp_reply[20] = 0x00; arp_reply[21] = 0x02;         // opcode = reply
        memcpy(arp_reply + 22, g_server_mac_address, 6);    // sender MAC
        memcpy(arp_reply + 28, g_server_ip_address, 4);     // sender IP
        memcpy(arp_reply + 32, packet + 22, 6);           	// target MAC (request sender MAC)
        memcpy(arp_reply + 38, packet + 28, 4);             // target IP (request sender IP)

        ncm_send_frame(arp_reply, 42);
        goto reset;
    }

	/* ICMP Echo Request to our IP? (minimal valid frame = 42 bytes) */
    if (frame_len >= 42 && frame_len <= MAX_ICMP_FRAME &&
        packet[12] == 0x08 && packet[13] == 0x00 &&   /* EtherType = IPv4 */
        packet[23] == 0x01 &&                          /* IP protocol = ICMP */
        packet[34] == 0x08 &&                          /* ICMP type = Echo Request */
        memcmp(packet + 30, g_server_ip_address, 4) == 0) {  /* dst IP == us */

        uint8_t reply[frame_len];
        memcpy(reply, packet, frame_len);

        /* Swap MACs (Host NIC <-> Tomu server) */
        memcpy(reply, packet + 6, 6);                    /* dst = original src */
        memcpy(reply + 6, g_server_mac_address, 6);      /* src = us */

        /* Swap IPs */
        memcpy(reply + 26, packet + 30, 4);              /* src = our IP */
        memcpy(reply + 30, packet + 26, 4);              /* dst = original src IP */

        reply[34] = 0x00;   /* change to Echo Reply (type 0) */
        /* code stays 0, identifier + sequence number are preserved by memcpy */

        /* IP header checksum (field is at offset 24/25 in the IP header) */
        reply[24] = 0;
        reply[25] = 0;
        uint16_t ip_csum = internet_checksum(reply + 14, 20);
        reply[24] = ip_csum & 0xFF;
        reply[25] = (ip_csum >> 8) & 0xFF;

        /* ICMP checksum (over ICMP header + data) */
        reply[36] = 0;
        reply[37] = 0;
        uint16_t icmp_csum = internet_checksum(reply + 34, frame_len - 34);
        reply[36] = icmp_csum & 0xFF;
        reply[37] = (icmp_csum >> 8) & 0xFF;

        ncm_send_frame(reply, frame_len);
        goto reset;
    }

	/* Echo the frame back as a proper NTB */
    ncm_send_frame(ntb_rx_buf + frame_offset, frame_len);

reset:
    ntb_rx_len = 0;   /* ready for next NTB */
}

static void cdcncm_data_rx_cb(usbd_device *usbd_dev, uint8_t ep)
{
	(void)ep;
    (void)usbd_dev;

    uint8_t packet_buf[64];
    uint16_t len = usbd_ep_read_packet(usbd_dev, CDC_NCM_DATA_OUT_EP, packet_buf, sizeof(packet_buf));

    if (len == 0) {                     /* Zero-length packet = end of NTB */
        if (ntb_rx_len > 0) {
            ncm_parse_and_echo_ntb();
        }
        return;
    }

    /* Append this packet to the current NTB */
    if (ntb_rx_len + len > sizeof(ntb_rx_buf)) { // Overflow
        ntb_rx_len = 0;
        return;
    }

    memcpy(ntb_rx_buf + ntb_rx_len, packet_buf, len);
    ntb_rx_len += len;

    /* If this was a short packet (< 64 bytes), the NTB is complete */
    if (len < 64) {
        ncm_parse_and_echo_ntb();
    }

	return;
}

static void cdc_altsetting_cc(usbd_device *usbd_dev, uint16_t wIndex, uint16_t wValue)
{
	(void)usbd_dev;

	if (wIndex != CDC_NCM_DATA_INTERFACE_NUM) {
		return;
	}

	if (wValue != 1) { // The NCM Data alternate interface
		return;
	}

	// Set alternate NCM data interface
	
	// NCM 1.1 9.1 Notification Sequencing
	// NCM functions are required to send ConnectionSpeedChange and NetworkConnection notifications in a specific order. 

	// Send a notification for ConnectionSpeedChange + NetworkConnection
	uint16_t return_value;

	// ConnectionSpeedChange - USB CDC document 6.3.3
	struct usb_cdc_notification_speed_change speed = {
		.notify_header = {
			.bmRequestType = 0xA1, // Value 10100001B from CDC document.
			.bNotificationCode = USB_CDC_ECM_NOTIFICATION_CONNECTION_SPEED_CHANGE,
			.wValue = 0,
			.wIndex = CDC_NCM_DATA_INTERFACE_NUM,
			.wLength = 8,
		},
		.dlbitrate = 10000000, // 10 Mbps
		.ulbitrate = 10000000, // 10 Mbps
	};

	// TODO: Refactor enpoint writing to a function to avoid duplicating this loop

	return_value = usbd_ep_write_packet(g_usbd_dev, CDC_NCM_NOTIFY_EP, &speed, sizeof(speed));
	// The endpoint might be busy transmitting, wait a little and retry.
	while (!return_value) {
		udelay_busy(EP_WRITE_RETRY_DELAY_USECS);
		return_value = usbd_ep_write_packet(g_usbd_dev, CDC_NCM_NOTIFY_EP, &speed, sizeof(speed));
	}

	// NetworkConnection - USB CDC document 6.3.1
	struct usb_cdc_notification_header connection = {
		.bmRequestType = 0xA1, // Value 10100001B from CDC document.
		.bNotificationCode = USB_CDC_ECM_NOTIFICATION_NETWORK_CONNECTION,
		.wValue = 1, // 1 = Connected, 0 = Disconnected.
		.wIndex = CDC_NCM_DATA_INTERFACE_NUM,
		.wLength = 0,
	};

	return_value = usbd_ep_write_packet(g_usbd_dev, CDC_NCM_NOTIFY_EP, &connection, sizeof(connection));
	// The endpoint might be busy transmitting, wait a little and retry.
	while (!return_value) {
		udelay_busy(EP_WRITE_RETRY_DELAY_USECS);
		return_value = usbd_ep_write_packet(g_usbd_dev, CDC_NCM_NOTIFY_EP, &connection, sizeof(connection));
	}

	gpio_clear(LED_GREEN_PORT, LED_GREEN_PIN);   // solid green = link up

	rx_count = 0;
	tx_count = 0;

	// TODO: Implement reset logic, as specified in NCM 1.1:
	// 9.2 Using Alternate Settings to Reset an NCM Function

	return;
}

static void cdc_set_config(usbd_device *usbd_dev, uint16_t wValue)
{
	(void)wValue;

	usbd_ep_setup(usbd_dev, CDC_NCM_NOTIFY_EP, USB_ENDPOINT_ATTR_INTERRUPT, 16, 0);
	usbd_ep_setup(usbd_dev, CDC_NCM_DATA_OUT_EP, USB_ENDPOINT_ATTR_BULK, 64, cdcncm_data_rx_cb);
	usbd_ep_setup(usbd_dev, CDC_NCM_DATA_IN_EP, USB_ENDPOINT_ATTR_BULK, 64, 0);

	usbd_register_set_altsetting_callback(g_usbd_dev, cdc_altsetting_cc);

	// The specified callback will be called if (type == (bmRequestType & type_mask)).

//TODO: Why is this done twice?

	usbd_register_control_callback(
				usbd_dev,
				USB_REQ_TYPE_CLASS | USB_REQ_TYPE_INTERFACE,
				USB_REQ_TYPE_TYPE | USB_REQ_TYPE_RECIPIENT,
				cdc_control_request);

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

void sys_tick_handler(void)
{	
	// TODO: Service uIP timers

	static uint16_t tick_counter = 0;

	if (tick_counter >= 5000) { // Every 5 seconds
		send_udp_debug();
		tick_counter = 0;
	}
	
	++tick_counter;
}

int main(void)
{
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
	g_usbd_dev = usbd_init(&efm32hg_usb_driver, &dev, &ncm_config, usb_strings, NUM_USB_STRINGS, usbd_control_buffer, sizeof(usbd_control_buffer));
	usbd_register_set_config_callback(g_usbd_dev, cdc_set_config);
	
	// The call to usbd_init called efm32hg_usbd_init, which set up and enabled the USB clock.

	/* Set the CPU Core to run from the trimmed USB clock, divided by 2.
	 * This will give the CPU Core a frequency of 24 MHz +/- 1% */
	CMU_CMD = CMU_CMD_HFCLKSEL(5);
    while (! (CMU_STATUS & CMU_STATUS_USHFRCODIV2SEL))
		;

	/* Enable USB IRQs */
	nvic_enable_irq(NVIC_USB_IRQ);

    /* Configure the system tick, at lower priority than USB IRQ */
    systick_set_frequency(SYSTICK_FREQUENCY, USB_CLK_FREQUENCY);
    systick_counter_enable();
    systick_interrupt_enable();
    nvic_set_priority(NVIC_SYSTICK_IRQ, 0x10);


	while(1) {
		;
	}
}
