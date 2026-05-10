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

#define USB_CDC_ECM_NOTIFICATION_NETWORK_CONNECTION 0x0;
#define USB_CDC_ECM_NOTIFICATION_RESPONSE_AVAILABLE 0x1;
#define USB_CDC_ECM_NOTIFICATION_CONNECTION_SPEED_CHANGE 0x2A;

// EFM32HG supports 3 IN and 3 OUT Endpoints.
#define CDC_NCM_DATA_OUT_EP 0x01
#define CDC_NCM_DATA_IN_EP 0x81
#define CDC_NCM_NOTIFY_EP 0x83

#define CDC_NCM_COMM_INTERFACE_NUM 0
#define CDC_NCM_DATA_INTERFACE_NUM 1

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

// CDC NCM11 5.4 Ethernet Networking Functional Descriptor
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

// CDC NCM11 6.2.1 NCM Functional Descriptor
struct usb_cdc_ncm_descriptor {
	uint8_t bFunctionLength;
	uint8_t bDescriptorType;
	uint8_t bDescriptorSubtype;
	uint16_t bcdNcmVersion;
	uint8_t bmNetworkCapabilities;
} __attribute__((packed));

#define USB_PUTS_DELAY_USEC 2000

static volatile bool g_usbd_is_connected = false;
static usbd_device *g_usbd_dev = 0;

static volatile uint16_t g_frame_len = 0;
static uint8_t g_ethernet_frame[1514];

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

// TODO: Fix clobbering as done in my other programs

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

static enum usbd_request_return_codes cdc_control_request(usbd_device *usbd_dev, struct usb_setup_data *req, uint8_t **buf,
		uint16_t *len, void (**complete)(usbd_device *usbd_dev, struct usb_setup_data *req))
{
	(void)complete;
	(void)buf;
	(void)usbd_dev;

	switch(req->bRequest) {
		
		//TODO: Probably not relevant for NCM, only ACM which we don't include.
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

		// NCM control requests (NCM11 document, section 7.2)

		// Optional NCM requests we do not support: (TODO: Support some of them)
		case USB_CDC_REQ_SET_ETHERNET_MULTICAST_FILTERS:
		case USB_CDC_REQ_SET_ETHERNET_POWER_MANAGEMENT_PATTERN_FILTER:
		case USB_CDC_REQ_GET_ETHERNET_POWER_MANAGEMENT_PATTERN_FILTER:
		case USB_CDC_REQ_SET_ETHERNET_PACKET_FILTER:
		case USB_CDC_REQ_GET_ETHERNET_STATISTIC:
		// TODO: Support NET ADDRESS, since I set D1 in bmNetworkCapabilities
		case USB_CDC_REQ_GET_NET_ADDRESS:
		case USB_CDC_REQ_SET_NET_ADDRESS:
		case USB_CDC_REQ_GET_NTB_FORMAT:
		case USB_CDC_REQ_SET_NTB_FORMAT:
		case USB_CDC_REQ_GET_MAX_DATAGRAM_SIZE:
		case USB_CDC_REQ_SET_MAX_DATAGRAM_SIZE:
		case USB_CDC_REQ_GET_CRC_MODE:
		case USB_CDC_REQ_SET_CRC_MODE:
			return USBD_REQ_NOTSUPP;
		break;
		
		// Required NCM requests we must support:
		case USB_CDC_REQ_GET_NTB_PARAMETERS:
			//TODO: Implement logic, and return reply on the control endpoint:
			// *buf: Point this to the memory buffer containing the data you want to send.
			// *len: Set this to the number of bytes to be sent.
			// Make sure buf is still allocated when the function returns:
			// Use global buffer (static uint8_t) with the largest reply expected (0x1C for NTB params)
			// Or to a const array, e.g.: *buf = (uint8_t *)"MyDevice123";
			// Or  static const uint8_t my_numerical_data[] = { 0xDE, 0xAD, 0xBE, 0xEF };
			//    *buf = (uint8_t *)my_numerical_data;
			// If the data is not static (even if it's const), some compilers might still put it on the stack as a temporary local variable. 
			// Always use static inside the function, or define the array outside the function at the top of your file.
			// Set the "complete" callback if actions are needed after the Control request is complete, acknowledged by the Host.
			// e.g. freeing memory, applying settings, triggering actions.

			/*
			static void my_done_callback(usbd_device *usbd_dev, struct usb_setup_data *req) {
			// This runs AFTER the data is sent and acknowledged
			gpio_toggle(GPIOA, GPIO5); 
			}

			static int my_control_callback(usbd_device *usbd_dev, struct usb_setup_data *req, 
									uint8_t **buf, uint16_t *len, 
									void (**complete)(usbd_device *, struct usb_setup_data *)) 
			{
				if (req->bRequest == MY_GET_DATA_REQ) {
					*buf = my_data_buffer;  // Point to the data to return
					*len = sizeof(my_data_buffer);
					*complete = my_done_callback; // Register the post-transfer action
					return 1; // Request Handled
				}
			return 0; // Request Not Handled
			}
			*/
			

			return USBD_REQ_HANDLED;
		break;

		case USB_CDC_REQ_GET_NTB_INPUT_SIZE:
			//TODO: Implement logic, and return reply.
			return USBD_REQ_HANDLED;
		break;

		case USB_CDC_REQ_SET_NTB_INPUT_SIZE:
			//TODO: Implement logic
			return USBD_REQ_HANDLED;
		break;
	}
	
	return USBD_REQ_NOTSUPP;
}

//TODO: Add delay in case the endpoint is busy, as done in my other programs.
// TODO: Remove this function or convert to hard-coded UDP send
static void usb_puts(char *s) {
	if (g_usbd_is_connected) {
		gpio_toggle(LED_GREEN_PORT, LED_GREEN_PIN); // TODO: Toggle green LED
		// usbd_ep_write_packet(g_usbd_dev, CDC_ACM_DATA_IN_EP, s, strnlen(s, 64));
	}
}

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

static void handle_ethernet_frame() {

//TODO: This function might be too long for the USB interrupt, with all the delays
//TODO: I might need to add locking, or double-buffer the Ethernet frame buf

	char len_string[5];
	itoa(g_frame_len, len_string, 10);

	usb_puts("\r\nFrame len: ");
	udelay_busy(USB_PUTS_DELAY_USEC);
	
	usb_puts(len_string);
	udelay_busy(USB_PUTS_DELAY_USEC);


	usb_puts("\r\nData: ");
	udelay_busy(USB_PUTS_DELAY_USEC);

	// For each byte, we write 2 Hex chars.
	// We can only print 64 chars at a time, so we read 32 bytes.

	char char_buf[64];

	uint8_t print_len;
	for (unsigned int start_index = 0; start_index < g_frame_len; start_index+=32) {
		if ((g_frame_len - start_index) >= 32) {
			print_len = 32;
		} else {
			print_len = (g_frame_len - start_index); // Print only remaining bytes
			char_buf[print_len*2] = '\0'; // Null terminate, for usb_puts strnlen.
		}
		buf_to_hexstring(g_ethernet_frame, char_buf, start_index, start_index + print_len - 1);
		usb_puts(char_buf);
		udelay_busy(USB_PUTS_DELAY_USEC);
	}

	g_frame_len = 0;
}

static void cdcncm_data_rx_cb(usbd_device *usbd_dev, uint8_t ep)
{
	(void)ep;
	(void)usbd_dev;

	gpio_toggle(LED_RED_PORT, LED_RED_PIN); // TODO: Light the Red LED for debug

	//TODO: NTB logic

	uint8_t packet_buf[64];
	uint16_t len = usbd_ep_read_packet(usbd_dev, CDC_NCM_DATA_OUT_EP, packet_buf, sizeof(packet_buf));


	char packet_len_buf[3];

	usb_puts("\r\nLen: ");
	udelay_busy(USB_PUTS_DELAY_USEC);
	itoa(len, packet_len_buf, 10);
	usb_puts(packet_len_buf);
	udelay_busy(USB_PUTS_DELAY_USEC);

	// See USB CDC ECM document, section 3.3.1 Segment Delineation.
	// Basically, the last packet will be < 64. If the frame is a multiple of 64, there will be a zero-length packet.

	if (len == 0) { // Zero-length packet
		if (g_frame_len) { // Frame has data
			handle_ethernet_frame();			
		} else {
			usb_puts("\r\nUnexpected 0-len packet");
			udelay_busy(USB_PUTS_DELAY_USEC);
		}
		return;
	}

	if ((g_frame_len + len) > (sizeof(g_ethernet_frame))) {
		g_frame_len = 0; // Drop frame data
		usb_puts("\r\nError: Frame overflow");
		udelay_busy(USB_PUTS_DELAY_USEC);	
	}
	
	memcpy(g_ethernet_frame + g_frame_len, packet_buf, len);
	g_frame_len+=len;
	if (len < 64) { // Last packet in a frame
		handle_ethernet_frame();
		return;
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
	
	// NCM11 9.1 Notification Sequencing
	// NCM functions are required to send ConnectionSpeedChange and NetworkConnection notifications in a specific order. 

	// Send a notification for ConnectionSpeedChange + NetworkConnection

	// ConnectionSpeedChange - USB CDC document 6.3.3
	struct usb_cdc_notification_speed_change notify_speed_change;
	notify_speed_change.notify_header.bmRequestType = 0xA1; // Value 10100001B from CDC document.
	notify_speed_change.notify_header.bNotificationCode = USB_CDC_ECM_NOTIFICATION_CONNECTION_SPEED_CHANGE;
	notify_speed_change.notify_header.wIndex = wIndex;
	notify_speed_change.notify_header.wValue = 0;
	notify_speed_change.notify_header.wLength = 8;
	notify_speed_change.dlbitrate = 10000000; // 10 Mbps
	notify_speed_change.ulbitrate = 10000000; // 10 Mbps
	usbd_ep_write_packet(g_usbd_dev, CDC_NCM_NOTIFY_EP, &notify_speed_change, sizeof(notify_speed_change));

	udelay_busy(1000); // TODO: Needed? Modify to loop on the output port if it's busy?

	// NetworkConnection - USB CDC document 6.3.1
	struct usb_cdc_notification_header notify_buf;
	notify_buf.bmRequestType = 0xA1; // Value 10100001B from CDC document.
	notify_buf.wIndex = wIndex;
	notify_buf.bNotificationCode = USB_CDC_ECM_NOTIFICATION_NETWORK_CONNECTION;
	notify_buf.wLength = 0;        
	notify_buf.wValue = 1; // 1 = Connected, 0 = Disconnected.
	usbd_ep_write_packet(g_usbd_dev, CDC_NCM_NOTIFY_EP, &notify_buf, sizeof(notify_buf));

	// TODO: Implement reset logic, as specified in NCM11:
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
		usb_puts("\r\nSysTick\r\n");
		tick_counter = 0;
	}
	
	++tick_counter;
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
		
		// TODO: Modify, we don't support CDC ACM config.
		if (line_was_connected != g_usbd_is_connected) {
			if (g_usbd_is_connected) {
				udelay_busy(USB_PUTS_DELAY_USEC);
				// usb_puts("\r\nUSB CDC ACM Connected!\r\n");
				udelay_busy(USB_PUTS_DELAY_USEC);
			}
			line_was_connected = g_usbd_is_connected;
		}

	}
}
