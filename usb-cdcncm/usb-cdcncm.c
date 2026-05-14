/**
 * \addtogroup Examples
 *
 * This example implements a USB CDC-NCM device (Ethernet NIC) + HTTP web server.
 * The USB host OS (Windows/Linux etc.) can interact with this server, on the NIC network.
 * The following protocols are implemented:
 * ARP reply: sends the "server" MAC address matching its requested IP address.
 * ICMP Echo (Ping) reply.
 * A UDP packet with Rx/Tx stats is regulary sent to the host (unless disabled).
 * An HTTP (over TCP/IP port 80) web server serves an HTML page + favicon icon.
 * Unknown GET paths return a 404 page.
 * When a valid packet is recieved, the red LED is toggled (unless disabled).
 * When a packet is transmitted, the green LED is toggled (unless disabled).
 */

#include <libopencm3/cm3/common.h>
#include <libopencm3/cm3/vector.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/cortex.h>
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

// Simple one-frame NTB-16
// 2048 bytes is our advertised dwNtbOutMaxSize, although we only require NCM headers (12+16) + Ethernet frame (1514) (disregarding alignment)
// Currently allocating less than maximum, enough for our largest sent packets, to save RAM.
// Increase these values if larger packets are required.
#define NTB_BUF_SIZE 1000 	// For Tx/Rx of packets
#define MAX_TCP_FRAME 960   // Contains HTTP response, must be smaller than (ntb_tx_buf - NCM headers) == 28 bytes.
#define MAX_ICMP_FRAME 128  // Plenty for normal pings (most are < 100 bytes total)

#define UDP_SRC_PORT 1234
#define UDP_DST_PORT 1234
#define HTML_AUTO_REFRESH

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

// RX buffer for full NTB (one transfer)
static uint8_t ntb_rx_buf[NTB_BUF_SIZE];
static volatile uint16_t ntb_rx_len = 0;

static uint8_t ntb_tx_buf[NTB_BUF_SIZE];
static uint8_t tcp_packet[MAX_TCP_FRAME];

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
	.bInterval = 0

}, {
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = CDC_NCM_DATA_OUT_EP, // Out
	.bmAttributes = USB_ENDPOINT_ATTR_BULK,
	.wMaxPacketSize = 64,
	// .bInterval = 1,
	.bInterval = 0
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

void write_packet_when_ready(usbd_device *usbd_dev, uint8_t addr, const void *buf, uint16_t len)
{
	uint16_t return_value = usbd_ep_write_packet(usbd_dev, addr, buf, len);
	while (!return_value) {
		udelay_busy(EP_WRITE_RETRY_DELAY_USECS);
		return_value = usbd_ep_write_packet(usbd_dev, addr, buf, len);
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
static uint8_t g_host_ip_address[4] = {192, 168, 7, 2}; // Used to send UDP stats packet
static uint8_t g_server_ip_address[4] = {192, 168, 7, 1};

static uint32_t rx_count = 0, tx_count = 0;
static uint32_t uptime_seconds = 0;

static volatile bool led_visualization = true;
static volatile bool udp_debug_enabled = true;

/* Simple TCP connection state for single client */
static bool tcp_in_session = false;
static uint32_t tcp_our_seq = 0x12345678;  /* Our initial sequence number */

// Not strictly required, since our packets never fragment, but for good measure (RFC 791) 
static uint16_t ip_id_counter = 0;

typedef enum {
    HTTP_CONTENT_MAIN_PAGE,
    HTTP_CONTENT_FAVICON_PNG,
	HTTP_CONTENT_404,
    // Add additional content types or pages here
} http_content_type_t;

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

/* TCP checksum (RFC 793) — pseudo-header + TCP header/data */
static uint16_t tcp_checksum(const uint8_t *tcp_buf, uint16_t tcp_len,
                             const uint8_t *src_ip, const uint8_t *dst_ip)
{
    uint32_t sum = 0;

    /* Pseudo header (src IP, dst IP, 0, proto=6, TCP len) */
    sum += (src_ip[0] << 8) | src_ip[1];
    sum += (src_ip[2] << 8) | src_ip[3];
    sum += (dst_ip[0] << 8) | dst_ip[1];
    sum += (dst_ip[2] << 8) | dst_ip[3];
    sum += 0x0006;   /* protocol = TCP */
    sum += tcp_len;

    /* TCP header + payload */
    for (uint16_t i = 0; i < tcp_len; i += 2) {
        if (i + 1 < tcp_len)
            sum += (tcp_buf[i] << 8) | tcp_buf[i + 1];
        else
            sum += tcp_buf[i] << 8;
    }

    sum = (sum >> 16) + (sum & 0xFFFF);
    sum += (sum >> 16);
    return ~sum;
}

// Write a unique number to the IP header Identification field
static inline void write_ip_identification(uint8_t *buf, uint16_t *offset) {
    buf[(*offset)++] = (ip_id_counter >> 8) & 0xFF;
    buf[(*offset)++] = ip_id_counter & 0xFF;
    ip_id_counter++;
}

/* Minimal NTB-16 TX helper - builds a one-frame NTB and sends it */
static void ncm_send_frame(const uint8_t *frame, uint16_t frame_len)
{
    if (frame_len == 0 || frame_len > 1514) return;

	/* Temporarily disable interrupts to prevent reentrancy from systick/USB */
    uint32_t mask = cm_mask_interrupts(1);

	uint16_t ntb_len = 0;

    /* NTH16 - "NCMH" */
    ntb_tx_buf[ntb_len++] = 'N'; ntb_tx_buf[ntb_len++] = 'C'; ntb_tx_buf[ntb_len++] = 'M'; ntb_tx_buf[ntb_len++] = 'H';
    ntb_tx_buf[ntb_len++] = 0x0C; ntb_tx_buf[ntb_len++] = 0x00;  /* wHeaderLength */
    ntb_tx_buf[ntb_len++] = 0x00; ntb_tx_buf[ntb_len++] = 0x00;  /* wSequence ignored, assuming the host driver doesn't care */ 
    ntb_tx_buf[ntb_len++] = 0x00; ntb_tx_buf[ntb_len++] = 0x00;  /* wBlockLength will be filled later */
    ntb_tx_buf[ntb_len++] = 0x0C; ntb_tx_buf[ntb_len++] = 0x00;  /* wNdpIndex = 12 (right after NTH) */

    /* NDP16 - "NCM0" */
    ntb_tx_buf[ntb_len++] = 'N'; ntb_tx_buf[ntb_len++] = 'C'; ntb_tx_buf[ntb_len++] = 'M'; ntb_tx_buf[ntb_len++] = '0';
    ntb_tx_buf[ntb_len++] = 0x10; ntb_tx_buf[ntb_len++] = 0x00;  /* wLength = 16 (one entry + zero) */
    ntb_tx_buf[ntb_len++] = 0x00; ntb_tx_buf[ntb_len++] = 0x00;  /* wNextNdpIndex = 0 */
    /* First datagram entry */
    ntb_tx_buf[ntb_len++] = 0x1C; ntb_tx_buf[ntb_len++] = 0x00;  /* wDatagramIndex = 12+16 (after NTB+NDP) */
    ntb_tx_buf[ntb_len++] = frame_len & 0xFF;
    ntb_tx_buf[ntb_len++] = (frame_len >> 8) & 0xFF;          /* wDatagramLength */
    /* Zero terminator */
    ntb_tx_buf[ntb_len++] = 0x00; ntb_tx_buf[ntb_len++] = 0x00;
    ntb_tx_buf[ntb_len++] = 0x00; ntb_tx_buf[ntb_len++] = 0x00;

    /* Copy the actual Ethernet frame */
    memcpy(ntb_tx_buf + ntb_len, frame, frame_len);
    ntb_len += frame_len;

    /* Fix wBlockLength in NTH */
    ntb_tx_buf[8] = ntb_len & 0xFF;
    ntb_tx_buf[9] = (ntb_len >> 8) & 0xFF;

    // Send in 64-byte chunks (required on full-speed USB)
	uint16_t sent = 0;
	while (sent < ntb_len) {
		uint16_t chunk = (ntb_len - sent) > 64 ? 64 : (ntb_len - sent);
		write_packet_when_ready(g_usbd_dev, CDC_NCM_DATA_IN_EP, ntb_tx_buf + sent, chunk);
		sent += chunk;
	}
	
	tx_count++;
	if (led_visualization) {
    	gpio_toggle(LED_GREEN_PORT, LED_GREEN_PIN); // Toggle Green LED for frame Tx
	}
    
    /* Restore previous interrupt state */
    cm_mask_interrupts(mask);
}

// UDP debug packet
static void send_udp_debug(void)
{
	uint8_t udp_packet[71]; // 42 bytes header + max payload
    uint16_t len = 0;

    /* Ethernet header (14 bytes) */
    memcpy(udp_packet + len, g_mac_address, 6);          len += 6;   // dst = Host NIC
    memcpy(udp_packet + len, g_server_mac_address, 6);   len += 6;   // src = Tomu (remote)
    udp_packet[len++] = 0x08; udp_packet[len++] = 0x00;                 // EtherType = IPv4

	/* IPv4 header (20 bytes) */
    uint16_t ip_hdr_start = len;
    udp_packet[len++] = 0x45; udp_packet[len++] = 0x00;
    udp_packet[len++] = 0x00; udp_packet[len++] = 0x00;   // total length (fix later)
	write_ip_identification(udp_packet, &len);    
	udp_packet[len++] = 0x00; udp_packet[len++] = 0x00;   // Flags + Fragment Offset
    udp_packet[len++] = 0x40; udp_packet[len++] = 0x11;   // TTL, protocol = UDP
    udp_packet[len++] = 0x00; udp_packet[len++] = 0x00;   // checksum (zero for now)
    memcpy(udp_packet + len, g_server_ip_address, 4); len += 4;   // src IP
    memcpy(udp_packet + len, g_host_ip_address, 4);   len += 4;   // dst IP

	/* UDP header (8 bytes) */   	
	udp_packet[len++] = (UDP_SRC_PORT >> 8) & 0xFF;
	udp_packet[len++] = UDP_SRC_PORT & 0xFF;

	udp_packet[len++] = (UDP_DST_PORT >> 8) & 0xFF;
	udp_packet[len++] = UDP_DST_PORT & 0xFF;

    udp_packet[len++] = 0x00; udp_packet[len++] = 0x00;		// UDP Length (fixed later)
    udp_packet[len++] = 0x00; udp_packet[len++] = 0x00;		// UDP checksum = 0 (allowed)

	/* Payload */
    char status[16];
    strcpy((char*)udp_packet + len, "RX:"); len += 3;
    itoa(rx_count, status, 10);
    strcpy((char*)udp_packet + len, status); len += strlen(status);
    strcpy((char*)udp_packet + len, " TX:"); len += 4;
    itoa(tx_count, status, 10);
    strcpy((char*)udp_packet + len, status); len += strlen(status);
    udp_packet[len++] = '\r'; udp_packet[len++] = '\n';

	/* Fix lengths */
    uint16_t ip_total = len - 14; // Skip 14 bytes of Ethernet header
    udp_packet[16] = (ip_total >> 8) & 0xFF;
    udp_packet[17] = ip_total & 0xFF;

    uint16_t udp_len = len - 34;   // 14 (Eth) + 20 (IP)
    udp_packet[38] = (udp_len >> 8) & 0xFF;
    udp_packet[39] = udp_len & 0xFF;

	/* Calculate and insert IPv4 header checksum */
    uint16_t csum = internet_checksum(udp_packet + ip_hdr_start, 20);
    udp_packet[ip_hdr_start + 10] = csum & 0xFF;
    udp_packet[ip_hdr_start + 11] = (csum >> 8) & 0xFF;

    ncm_send_frame(udp_packet, len);
}

/* Generic TCP packet sender */
static void send_tcp_packet(uint8_t *incoming_eth,
                            uint32_t client_seq,
							uint32_t client_ack,
                            uint8_t tcp_flags,
                            bool include_timestamp)
{
    uint8_t packet[66];
    uint16_t len = 0;

    /* Ethernet */
    memcpy(packet + len, incoming_eth + 6, 6); len += 6;
    memcpy(packet + len, g_server_mac_address, 6); len += 6;
    packet[len++] = 0x08; packet[len++] = 0x00;

    /* IP */
    uint16_t ip_start = len;
    packet[len++] = 0x45; packet[len++] = 0x00;
    packet[len++] = 0x00; packet[len++] = include_timestamp ? 0x34 : 0x28;
    write_ip_identification(packet, &len);
    packet[len++] = 0x00; packet[len++] = 0x00;
    packet[len++] = 0x40; packet[len++] = 0x06;
    packet[len++] = 0x00; packet[len++] = 0x00;
    memcpy(packet + len, g_server_ip_address, 4); len += 4;
    memcpy(packet + len, incoming_eth + 14 + 12, 4); len += 4;

    /* TCP */
    uint16_t tcp_start = len;
    packet[len++] = 0x00; packet[len++] = 0x50;
    packet[len++] = incoming_eth[34]; packet[len++] = incoming_eth[35];

    /* Sequence number */
	uint32_t our_seq = include_timestamp ? 0x12345678 : client_ack;
    packet[len++] = (our_seq >> 24) & 0xFF;
    packet[len++] = (our_seq >> 16) & 0xFF;
    packet[len++] = (our_seq >> 8) & 0xFF;
    packet[len++] = our_seq & 0xFF;

    /* Ack */
    uint32_t ack = client_seq + 1;
    packet[len++] = (ack >> 24) & 0xFF;
    packet[len++] = (ack >> 16) & 0xFF;
    packet[len++] = (ack >> 8) & 0xFF;
    packet[len++] = ack & 0xFF;

    /* Data offset + flags */
    packet[len++] = include_timestamp ? 0x80 : 0x50;
    packet[len++] = tcp_flags;

    packet[len++] = 0x16; packet[len++] = 0xD0; /* window */

    packet[len++] = 0x00; packet[len++] = 0x00; /* checksums */
    packet[len++] = 0x00; packet[len++] = 0x00;

    /* Timestamp option (only for SYN+ACK) */
    if (include_timestamp) {
        packet[len++] = 0x01; packet[len++] = 0x01;
        packet[len++] = 0x08; packet[len++] = 0x0A;
        packet[len++] = 0x00; packet[len++] = 0x00; packet[len++] = 0x00; packet[len++] = 0x00;

        uint32_t client_tsval = 0;
        uint8_t *opts = incoming_eth + 14 + 20 + 20;
        if (opts[0] == 0x02 && opts[1] == 0x04 &&
            opts[4] == 0x04 && opts[5] == 0x02 &&
            opts[6] == 0x08 && opts[7] == 0x0A) {
            client_tsval = ((uint32_t)opts[8] << 24) | ((uint32_t)opts[9] << 16) |
                           ((uint32_t)opts[10] << 8) | opts[11];
        }
        packet[len++] = (client_tsval >> 24) & 0xFF;
        packet[len++] = (client_tsval >> 16) & 0xFF;
        packet[len++] = (client_tsval >> 8) & 0xFF;
        packet[len++] = client_tsval & 0xFF;
    }

    /* Checksums */
    uint16_t ip_csum = internet_checksum(packet + ip_start, 20);
    packet[ip_start + 10] = ip_csum & 0xFF;
    packet[ip_start + 11] = (ip_csum >> 8) & 0xFF;

    uint16_t tcp_len = len - tcp_start;
    uint16_t tcp_csum = tcp_checksum(packet + tcp_start, tcp_len,
                                     g_server_ip_address, incoming_eth + 14 + 12);
    packet[tcp_start + 16] = (tcp_csum >> 8) & 0xFF;
    packet[tcp_start + 17] = tcp_csum & 0xFF;

    ncm_send_frame(packet, len);
}

/* Builds HTTP header + body into buf.
 * Returns the number of bytes written for the HTTP content. */
static uint16_t build_http_content(http_content_type_t type, uint8_t *buf)
{
    uint16_t len = 0;

    if (type == HTTP_CONTENT_MAIN_PAGE) {
        /* Write header up to Content-Length: */
        strcpy((char*)buf + len, 
               "HTTP/1.0 200 OK\r\nContent-Type: text/html\r\nConnection: close\r\nContent-Length: ");
        len += strlen((char*)buf + len);

        uint16_t content_length_value_pos = len;   // where the number will go

        /* Build body right after */
        uint16_t body_start = len;

        /* HTML body */
        strcpy((char*)buf + len, "<html><body style='font-family:monospace;background:#111;color:#0f0'>"); len += strlen((char*)buf + len);
		#ifdef HTML_AUTO_REFRESH
		strcpy((char*)buf + len, "<meta http-equiv='refresh' content='5'>"); len += strlen((char*)buf + len);
		#endif
        strcpy((char*)buf + len, "<h1>Tomu NCM</h1>"); len += strlen((char*)buf + len);
        
		strcpy((char*)buf + len, "<p>rx="); len += strlen((char*)buf + len);
        itoa(rx_count, (char*)buf + len, 10); len += strlen((char*)buf + len);
        strcpy((char*)buf + len, " tx="); len += strlen((char*)buf + len);
        itoa(tx_count, (char*)buf + len, 10); len += strlen((char*)buf + len);
        
		strcpy((char*)buf + len, "<form action='/' method='GET'>"); len += strlen((char*)buf + len);   
  
		strcpy((char*)buf + len, "<p><input type='hidden' name='led' value='0'>"); len += strlen((char*)buf + len);
		strcpy((char*)buf + len, "<label><input type='checkbox' name='led' value='1'"); len += strlen((char*)buf + len);
		if (led_visualization) { strcpy((char*)buf + len, " checked"); len += 8; }
		strcpy((char*)buf + len, "> Rx/Tx LED</label></p>"); len += strlen((char*)buf + len);
		
		strcpy((char*)buf + len, "<p><input type='hidden' name='udp' value='0'>"); len += strlen((char*)buf + len);
		strcpy((char*)buf + len, "<label><input type='checkbox' name='udp' value='1'"); len += strlen((char*)buf + len);
		if (udp_debug_enabled) { strcpy((char*)buf + len, " checked"); len += 8; }
		strcpy((char*)buf + len, "> UDP Stats</label></p>"); len += strlen((char*)buf + len);

		strcpy((char*)buf + len, "<p><button type='submit' style='font-family:monospace; background:#222; color:#0f0; border:1px solid #0f0; padding:4px 14px; cursor:pointer;'>Apply</button></p>");
		len += strlen((char*)buf + len);
		
		strcpy((char*)buf + len, "</form>"); len += strlen((char*)buf + len);

		strcpy((char*)buf + len, "</p><p>Uptime: "); len += strlen((char*)buf + len);
        uint32_t h = uptime_seconds / 3600, m = (uptime_seconds % 3600)/60, s = uptime_seconds % 60;
        char tb[16];
        itoa(h, tb, 10); strcpy((char*)buf + len, tb); len += strlen(tb);
        strcpy((char*)buf + len, ":"); len += 1;
        if (m < 10) { strcpy((char*)buf + len, "0"); len += 1; }
        itoa(m, tb, 10); strcpy((char*)buf + len, tb); len += strlen(tb);
        strcpy((char*)buf + len, ":"); len += 1;
        if (s < 10) { strcpy((char*)buf + len, "0"); len += 1; }
        itoa(s, tb, 10); strcpy((char*)buf + len, tb); len += strlen(tb);

        strcpy((char*)buf + len, "</p><p><small><a href='https://github.com/im-tomu/tomu-quickstart/tree/master/usb-cdcncm' style='color:#0f0;'>USB CDC-NCM on EFM32HG309</a></small></p></body></html>");
        len += strlen((char*)buf + len);

        uint16_t body_len = len - body_start;

        /* Shift body to make room for number + \r\n\r\n */
        char clen[16];
        int num_len = strlen(itoa(body_len, clen, 10));
        int shift = num_len + 4;

        memmove(buf + body_start + shift, buf + body_start, body_len);

        /* Write the number and terminator into the gap */
        memcpy(buf + content_length_value_pos, clen, num_len);
        memcpy(buf + content_length_value_pos + num_len, "\r\n\r\n", 4);

        /* Update len to cover header + body */
        len = content_length_value_pos + num_len + 4 + body_len;
    } else if (type == HTTP_CONTENT_FAVICON_PNG) {
		static const uint8_t favicon_data[] = {
			0x89, 0x50, 0x4e, 0x47, 0x0d, 0x0a, 0x1a, 0x0a, 0x00, 0x00, 0x00, 0x0d,
			0x49, 0x48, 0x44, 0x52, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x10,
			0x10, 0x02, 0x00, 0x00, 0x00, 0xc0, 0x01, 0xb4, 0x75, 0x00, 0x00, 0x00,
			0x97, 0x49, 0x44, 0x41, 0x54, 0x38, 0xcb, 0x63, 0x64, 0x60, 0xf8, 0xff,
			0x9f, 0x81, 0x86, 0x80, 0x89, 0x96, 0x86, 0x0f, 0x0f, 0x0b, 0x58, 0x70,
			0x49, 0x08, 0x84, 0xca, 0x24, 0x0a, 0x84, 0x32, 0xee, 0xf9, 0xb6, 0x91,
			0x71, 0x0f, 0x7e, 0x23, 0xbe, 0xe5, 0xb5, 0xfd, 0xfd, 0x96, 0xf7, 0xb3,
			0x31, 0xe3, 0xe3, 0xcf, 0x46, 0x12, 0x2c, 0xf8, 0xc1, 0x99, 0xdf, 0xf0,
			0x83, 0x93, 0xe1, 0xe8, 0xaf, 0x8f, 0x0c, 0x47, 0x99, 0xdd, 0x1e, 0xa8,
			0x30, 0xbb, 0xb1, 0x7d, 0x5d, 0x3b, 0x87, 0xed, 0xeb, 0x3f, 0x43, 0x49,
			0x9f, 0x7f, 0x86, 0x3f, 0xa7, 0x44, 0x7d, 0xf9, 0x39, 0x05, 0xa2, 0xf2,
			0xcf, 0x6f, 0x63, 0xb7, 0x3f, 0xbf, 0x19, 0x1a, 0x19, 0x18, 0x18, 0x1a,
			0xb1, 0x1a, 0x05, 0x49, 0x45, 0xf8, 0x48, 0x96, 0x4b, 0x7b, 0x3a, 0x59,
			0x2e, 0x09, 0x0a, 0x0a, 0x0a, 0x0a, 0x0a, 0xf2, 0x26, 0xbb, 0xcc, 0xe2,
			0x4d, 0x26, 0x46, 0x17, 0x84, 0x1c, 0xfa, 0x91, 0x3c, 0x6a, 0x01, 0x41,
			0xc0, 0x38, 0x5a, 0x16, 0x0d, 0xb8, 0x05, 0x00, 0xcd, 0x84, 0x47, 0xe7,
			0xc3, 0x88, 0xd9, 0x26, 0x00, 0x00, 0x00, 0x00, 0x49, 0x45, 0x4e, 0x44,
			0xae, 0x42, 0x60, 0x82		
		};

		strcpy((char*)buf + len, "HTTP/1.0 200 OK\r\nContent-Type: image/png\r\nContent-Length: ");
		len += strlen((char*)buf + len);

		char clen[16];
		itoa(sizeof(favicon_data), clen, 10);
		strcpy((char*)buf + len, clen);
		len += strlen(clen);

		strcpy((char*)buf + len, "\r\n\r\n");
		len += 4;

		memcpy(buf + len, favicon_data, sizeof(favicon_data));
		len += sizeof(favicon_data);
	} else if (type == HTTP_CONTENT_404) {
    	static const char page_404[] =
        "<html><body style='font-family:monospace;background:#111;color:#0f0'>"
		"<pre>"
		"                 ###      404       \n"
		"  404          ##:-*#               \n"
		"               #######              \n"
		"                 ##           ##***#\n"
		"            ############      #*--+#\n"
		"        ##*+=------====++*##  ######\n"
		"        ##.:--------------##    ### \n"
		"#####   ##.--+**+==+**=---##    ### \n"
		"#=-=*#  ##:=+..... ....==-##    ### \n"
		"######  ##-== :+*. .*+.==-##  ####  \n"
		"  ##    ##-=+:.......:-+=-#######   \n"
		"  ####  ##---=++++++++=---####      \n"
		"    ######---------------=##        \n"
		"       ###-------404-----=##    404 \n"
		"        ##---------------=##        \n"
		"        ####################        \n"
		"  404       ####    #####           \n"
		"           #*==*#  #*==*##          "
        "</pre>"
        "</body></html>";

		strcpy((char*)buf + len, "HTTP/1.0 404 Not Found\r\nContent-Type: text/html\r\nContent-Length: ");
		len += strlen((char*)buf + len);

		char clen[16];
		itoa(sizeof(page_404) - 1, clen, 10);           // -1 because of the null terminator
		strcpy((char*)buf + len, clen);
		len += strlen(clen);

		strcpy((char*)buf + len, "\r\n\r\n");
		len += 4;

		strcpy((char*)buf + len, page_404);
		len += strlen(page_404);
	}

    return len;
}

static void send_http_response(uint8_t *incoming_eth, uint32_t client_seq, uint32_t client_ack, uint16_t payload_len, http_content_type_t content_type)
{
    (void)client_ack;
    uint16_t len = 0;

    /* Ethernet + IP + TCP headers (same as before) */
    memcpy(tcp_packet + len, incoming_eth + 6, 6); len += 6;
    memcpy(tcp_packet + len, g_server_mac_address, 6); len += 6;
    tcp_packet[len++] = 0x08; tcp_packet[len++] = 0x00;

    uint16_t ip_start = len;
    tcp_packet[len++] = 0x45; tcp_packet[len++] = 0x00;
    tcp_packet[len++] = 0x00; tcp_packet[len++] = 0x00; /* placeholder */
    write_ip_identification(tcp_packet, &len);
    tcp_packet[len++] = 0x00; tcp_packet[len++] = 0x00;
    tcp_packet[len++] = 0x40; tcp_packet[len++] = 0x06;
    tcp_packet[len++] = 0x00; tcp_packet[len++] = 0x00;
    memcpy(tcp_packet + len, g_server_ip_address, 4); len += 4;
    memcpy(tcp_packet + len, incoming_eth + 14 + 12, 4); len += 4;

    uint16_t tcp_start = len;
    tcp_packet[len++] = 0x00; tcp_packet[len++] = 0x50;
    tcp_packet[len++] = incoming_eth[34]; tcp_packet[len++] = incoming_eth[35];

    uint32_t our_seq = tcp_our_seq + 1;
    tcp_packet[len++] = (our_seq >> 24) & 0xFF;
    tcp_packet[len++] = (our_seq >> 16) & 0xFF;
    tcp_packet[len++] = (our_seq >> 8) & 0xFF;
    tcp_packet[len++] = our_seq & 0xFF;

    uint32_t ack_val = client_seq + payload_len;
    tcp_packet[len++] = (ack_val >> 24) & 0xFF;
    tcp_packet[len++] = (ack_val >> 16) & 0xFF;
    tcp_packet[len++] = (ack_val >> 8) & 0xFF;
    tcp_packet[len++] = ack_val & 0xFF;

    tcp_packet[len++] = 0x50;
    tcp_packet[len++] = 0x19;  // PSH + ACK + FIN
    tcp_packet[len++] = 0x16; tcp_packet[len++] = 0xD0;
    tcp_packet[len++] = 0x00; tcp_packet[len++] = 0x00;
    tcp_packet[len++] = 0x00; tcp_packet[len++] = 0x00;

   	/* Build HTTP content directly into tcp_packet */
    uint16_t http_len = build_http_content(content_type, tcp_packet + len);
    len += http_len;

    /* Fix IP length */
    uint16_t ip_total = len - ip_start;
    tcp_packet[ip_start + 2] = ip_total >> 8;
    tcp_packet[ip_start + 3] = ip_total & 0xFF;

    /* Checksums */
    uint16_t ip_csum = internet_checksum(tcp_packet + ip_start, 20);
    tcp_packet[ip_start + 10] = ip_csum & 0xFF;
    tcp_packet[ip_start + 11] = (ip_csum >> 8) & 0xFF;

    uint16_t tcp_total = len - tcp_start;
    uint16_t tcp_csum = tcp_checksum(tcp_packet + tcp_start, tcp_total,
                                     g_server_ip_address, incoming_eth + 14 + 12);
    tcp_packet[tcp_start + 16] = (tcp_csum >> 8) & 0xFF;
    tcp_packet[tcp_start + 17] = tcp_csum & 0xFF;

    ncm_send_frame(tcp_packet, len);
    tcp_in_session = false;
}

/* Minimal NTB-16 parser - called only when a complete NTB has been received */
static void ncm_parse_ntb(void)
{
	if (ntb_rx_len < 12) {
		goto reset;
	}

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
	if (led_visualization) {
    	gpio_toggle(LED_RED_PORT, LED_RED_PIN); // Toggle Red LED for frame Rx
	}

	uint8_t *eth = &ntb_rx_buf[frame_offset];
    uint16_t eth_type = (eth[12] << 8) | eth[13];

	switch (eth_type) {
    case 0x0806: /* ARP */
        if (frame_len >= 42 &&
            eth[21] == 0x01 && // Opcode == request
            memcmp(eth + 38, g_server_ip_address, 4) == 0) { // target IP = ours

            uint8_t arp[42];
            memcpy(arp, eth + 6, 6); // Destination = Sender MAC
            memcpy(arp + 6, g_server_mac_address, 6); // Source MAC
            arp[12] = 0x08; arp[13] = 0x06; // EtherType ARP
            arp[14] = 0x00; arp[15] = 0x01; // HW type Ethernet
            arp[16] = 0x08; arp[17] = 0x00; // protocol IP
            arp[18] = 6; arp[19] = 4; // HW / proto length
            arp[20] = 0x00; arp[21] = 0x02; // opcode = reply
            memcpy(arp + 22, g_server_mac_address, 6); // sender MAC
            memcpy(arp + 28, g_server_ip_address, 4); // sender IP
            memcpy(arp + 32, eth + 22, 6); // target MAC (request sender MAC)
            memcpy(arp + 38, eth + 28, 4); // target IP (request sender IP)

            ncm_send_frame(arp, 42);
        }
        break;

    case 0x0800: /* IPv4 */
        if (frame_len >= 42 && frame_len <= MAX_ICMP_FRAME &&
            eth[23] == 0x01 && // IP protocol == ICMP
            eth[34] == 0x08 && // ICMP type == Echo Request
            memcmp(eth + 30, g_server_ip_address, 4) == 0) { // dst IP == us

            uint8_t reply[frame_len];
            memcpy(reply, eth, frame_len);

			// Swap MACs (Host NIC <-> Tomu server)
            memcpy(reply, eth + 6, 6); // dst = original src
            memcpy(reply + 6, g_server_mac_address, 6); // src = us
            // Swap IPs
			memcpy(reply + 26, eth + 30, 4); // src = our IP
            memcpy(reply + 30, eth + 26, 4); // dst = original src IP

            reply[34] = 0x00; // change to Echo Reply (type 0)

            reply[24] = 0; reply[25] = 0;
            uint16_t ip_csum = internet_checksum(reply + 14, 20);
            reply[24] = ip_csum & 0xFF;
            reply[25] = (ip_csum >> 8) & 0xFF;

            reply[36] = 0; reply[37] = 0;
            uint16_t icmp_csum = internet_checksum(reply + 34, frame_len - 34);
            reply[36] = icmp_csum & 0xFF;
            reply[37] = (icmp_csum >> 8) & 0xFF;

            ncm_send_frame(reply, frame_len);
        } else if (eth[23] == 0x06) { /* TCP */
			uint8_t ip_header_len = (eth[14] & 0x0F) * 4;
			uint8_t *tcp = eth + 14 + ip_header_len;

			uint16_t dst_port = (tcp[2] << 8) | tcp[3];
			if (dst_port == 80) {
				uint8_t flags = tcp[13];
				uint32_t seq = ((uint32_t)tcp[4] << 24) |
							((uint32_t)tcp[5] << 16) |
							((uint32_t)tcp[6] << 8) |
							tcp[7];
				uint32_t ack = ((uint32_t)tcp[8] << 24) |
							((uint32_t)tcp[9] << 16) |
							((uint32_t)tcp[10] << 8) |
							tcp[11];
				uint16_t tcp_hdr_len = ((tcp[12] >> 4) & 0x0F) * 4;
				uint8_t *payload = tcp + tcp_hdr_len;
				uint16_t payload_len = frame_len - (14 + ip_header_len + tcp_hdr_len);

				if (flags & 0x02) { /* SYN */
					send_tcp_packet(eth, seq, 0, 0x12, true); // Send SYN+ACK
				} else if (flags & 0x01) { /* FIN (client wants to close) */
            		send_tcp_packet(eth, seq, ack, 0x11, false); // Send FIN+ACK
        		} else if ((flags & 0x10) && payload_len > 4) { /* ACK + data */
					if (strncmp((char*)payload, "GET /favicon", 12) == 0) {
    					send_http_response(eth, seq, ack, payload_len, HTTP_CONTENT_FAVICON_PNG);
					} else if (strncmp((char*)payload, "GET /", 5) == 0) {
						char *path = (char*)payload + 5;   // skip "GET /"
						if (path[0] == ' ' || path[0] == '?') {
							// Parse query parameters (very simple parser, assumes parameter order)
							char *query = strstr((char*)payload, "?");			
							if (query) {
								// Find the end of the query string (before " HTTP" or \r)
								// Done to avoid parsing values in "Referer" field
								char *query_end = strstr(query, " HTTP");
								if (!query_end) query_end = strstr(query, "\r");
								
								if (query_end) {
									char saved = *query_end;
									*query_end = '\0';   // temporarily terminate the query string

									// Now safely parse only the query parameters
									if (strstr(query, "led=0")) led_visualization = false;
									if (strstr(query, "led=1")) led_visualization = true;
									if (strstr(query, "udp=0")) udp_debug_enabled = false;
									if (strstr(query, "udp=1")) udp_debug_enabled = true;

									*query_end = saved;  // restore original character
								}
							}

							if (!led_visualization) {
								gpio_set(LED_RED_PORT, LED_RED_PIN); // Turn off red LED
								gpio_set(LED_GREEN_PORT, LED_GREEN_PIN); // Turn off green LED
							}
							
							send_http_response(eth, seq, ack, payload_len, HTTP_CONTENT_MAIN_PAGE);
						} else { // Unsupported "GET /" page.
							send_http_response(eth, seq, ack, payload_len, HTTP_CONTENT_404);
						}
					}
				}
			}
   		}
        break;

    default:
        /* Unknown EtherType — drop */
        break;
    }

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
            ncm_parse_ntb();
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
        ncm_parse_ntb();
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

	write_packet_when_ready(g_usbd_dev, CDC_NCM_NOTIFY_EP, &speed, sizeof(speed));

	// NetworkConnection - USB CDC document 6.3.1
	struct usb_cdc_notification_header connection = {
		.bmRequestType = 0xA1, // Value 10100001B from CDC document.
		.bNotificationCode = USB_CDC_ECM_NOTIFICATION_NETWORK_CONNECTION,
		.wValue = 1, // 1 = Connected, 0 = Disconnected.
		.wIndex = CDC_NCM_DATA_INTERFACE_NUM,
		.wLength = 0,
	};

	write_packet_when_ready(g_usbd_dev, CDC_NCM_NOTIFY_EP, &connection, sizeof(connection));

	gpio_clear(LED_GREEN_PORT, LED_GREEN_PIN); // Solid green = link up

	// Reset counters, see also: NCM 1.1 9.2 Using Alternate Settings to Reset an NCM Function
	rx_count = 0;
	tx_count = 0;

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

	// IN/OUT control requests handled in the same callback function (no need for USB_REQ_TYPE_DIRECTION in mask)
	usbd_register_control_callback(
				usbd_dev,
				USB_REQ_TYPE_CLASS | USB_REQ_TYPE_INTERFACE,
				USB_REQ_TYPE_TYPE | USB_REQ_TYPE_RECIPIENT,
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
	static uint16_t tick_counter = 0;

	tick_counter++;

    if (tick_counter >= 1000) {           // every second
        uptime_seconds++;
        tick_counter = 0;
    }

    if ((tick_counter == 0) && (uptime_seconds % 5 == 0) && udp_debug_enabled) {   // every 5 seconds
        send_udp_debug();
    }
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
