/**
 *TODO: Update this text
 * This example implements a USB CDC-ACM device (aka Virtual Serial Port)
 * to demonstrate the use of the USB device stack.
 *
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
#include <libopencm3/efm32/timer.h>

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

// Make this program compatible with Toboot-V2.0
#include <toboot.h>
TOBOOT_CONFIGURATION(0);
//TOBOOT_CONFIGURATION(TOBOOT_CONFIG_FLAG_AUTORUN); // Uncomment to boot directly to this app, instead of the DFU bootloader.

// Serial prompt
#define PROMPT          "CMD> "

#define LED_GREEN_PORT  GPIOA
#define LED_GREEN_PIN   GPIO0
#define LED_RED_PORT    GPIOB
#define LED_RED_PIN     GPIO7

#define VENDOR_ID       0x1209  // pid.code
#define PRODUCT_ID      0x70b1  // Assigned to Tomu project
#define DEVICE_VER      0x0BEB  // Program version

#define BAUD_RATE       19200
#define BIT_TIME_US     (1000000/BAUD_RATE)    // Bit time in uS

#define USB_PUTS_DELAY_USEC 5000

#define TIMER_INPUT_CLOCK_FREQUENCY 24000000
#define LED_PWM_TIMER_TOP_CCV 100
#define LED_PWM_TIMER_PRESCALER TIMER_CTRL_PRESC_DIV16 // TODO: Is this a good value? Should I decrease for accuracy?
#define LED_PWM_TIMER_CYCLES_PER_SECOND ((LED_PWM_TIMER_TOP_CCV + 1) * (1 << LED_PWM_TIMER_PRESCALER))
#define LED_PWM_TIMER_CYCLES_PER_MILLISECOND (uint32_t) ((((float) TIMER_INPUT_CLOCK_FREQUENCY / LED_PWM_TIMER_CYCLES_PER_SECOND) + 500) / 1000)

#define CMD_GREEN_LED_FLAG 'G'
#define CMD_RED_LED_FLAG 'R'
#define CMD_DEBUG_PRINTS_FLAG 'D'
#define CMD_SERIAL_ECHO_FLAG 'E'
#define CMD_TEST_MODE 'T'

#define NUM_LED_PARAMS 7
#define LED_TEST_MODE_MAX 4

enum led_colour {
	GREEN_LED = 0,
	RED_LED = 1
};

enum led_fade_phase {
	LED_PHASE_LOW = 0,
	LED_PHASE_RAMP_UP = 1,
	LED_PHASE_RAMP_DOWN = 2,
	LED_PHASE_HIGH = 3
};

struct led_pwm_cfg
{
	// Fade cycle durations in milliseconds
	uint32_t ramp_up_ms;
	uint32_t ramp_down_ms;
	uint32_t high_duration_ms;
	uint32_t low_duration_ms;
	// Brightness percentage values, 0-100.
	uint8_t min_brightness;
	uint8_t max_brightness;
	// Current fade cycle phase
	enum led_fade_phase current_phase;
};

struct led_pwm_cfg g_green_led_cfg, g_red_led_cfg;

static volatile bool g_usbd_is_connected = false;
static usbd_device *g_usbd_dev = 0;
static uint8_t g_current_command[1024];
static uint8_t g_new_command[1024];
static volatile uint32_t g_new_command_len;
static volatile bool g_should_use_new_command;
static volatile bool g_debug_prints_enabled = true;
static volatile bool g_serial_echo_enabled = true;
static volatile bool g_green_counters_reset = false;
static volatile bool g_red_counters_reset = false;


static const struct usb_device_descriptor dev =
{
	.bLength = USB_DT_DEVICE_SIZE,
	.bDescriptorType = USB_DT_DEVICE,
	.bcdUSB = 0x0200,
	.bDeviceClass = USB_CLASS_CDC,
	.bDeviceSubClass = 0,
	.bDeviceProtocol = 0,
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
static const struct usb_endpoint_descriptor comm_endp[] =
{{
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = 0x83,
	.bmAttributes = USB_ENDPOINT_ATTR_INTERRUPT,
	.wMaxPacketSize = 16,
	.bInterval = 255,
}};

static const struct usb_endpoint_descriptor data_endp[] =
{{
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = 0x01,
	.bmAttributes = USB_ENDPOINT_ATTR_BULK,
	.wMaxPacketSize = 64,
	.bInterval = 1,
}, {
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = 0x82,
	.bmAttributes = USB_ENDPOINT_ATTR_BULK,
	.wMaxPacketSize = 64,
	.bInterval = 1,
}};

static const struct {
	struct usb_cdc_header_descriptor header;
	struct usb_cdc_call_management_descriptor call_mgmt;
	struct usb_cdc_acm_descriptor acm;
	struct usb_cdc_union_descriptor cdc_union;
} __attribute__((packed)) cdcacm_functional_descriptors =
{
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

static const struct usb_interface_descriptor comm_iface[] =
{{
	.bLength = USB_DT_INTERFACE_SIZE,
	.bDescriptorType = USB_DT_INTERFACE,
	.bInterfaceNumber = 0,
	.bAlternateSetting = 0,
	.bNumEndpoints = 1,
	.bInterfaceClass = USB_CLASS_CDC,
	.bInterfaceSubClass = USB_CDC_SUBCLASS_ACM,
	.bInterfaceProtocol = USB_CDC_PROTOCOL_AT,
	.iInterface = 0,

	.endpoint = comm_endp,

	.extra = &cdcacm_functional_descriptors,
	.extralen = sizeof(cdcacm_functional_descriptors)
}};

static const struct usb_interface_descriptor data_iface[] =
{{
	.bLength = USB_DT_INTERFACE_SIZE,
	.bDescriptorType = USB_DT_INTERFACE,
	.bInterfaceNumber = 1,
	.bAlternateSetting = 0,
	.bNumEndpoints = 2,
	.bInterfaceClass = USB_CLASS_DATA,
	.bInterfaceSubClass = 0,
	.bInterfaceProtocol = 0,
	.iInterface = 0,

	.endpoint = data_endp,
}};

static const struct usb_interface ifaces[] =
{{
	.num_altsetting = 1,
	.altsetting = comm_iface,
}, {
	.num_altsetting = 1,
	.altsetting = data_iface,
}};

static const struct usb_config_descriptor config =
{
	.bLength = USB_DT_CONFIGURATION_SIZE,
	.bDescriptorType = USB_DT_CONFIGURATION,
	.wTotalLength = 0,
	.bNumInterfaces = 2,
	.bConfigurationValue = 1,
	.iConfiguration = 0,
	.bmAttributes = 0x80,
	.bMaxPower = 0x32,

	.interface = ifaces,
};

static const char *usb_strings[] =
{
	"Tomu",
	"Serial Shell",
	"LED PWM",
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
		asm("mov   r1, #6");
		asm("retry:");
		asm("sub r1, #1");
		asm("bne retry");
		asm("nop");
	}
}

/* Buffer to be used for control requests. */
static uint8_t usbd_control_buffer[128];

static enum usbd_request_return_codes cdcacm_control_request(usbd_device *usbd_dev, struct usb_setup_data *req, uint8_t **buf,
		uint16_t *len, void (**complete)(usbd_device *usbd_dev, struct usb_setup_data *req))
{
	(void)complete;
	(void)buf;
	(void)usbd_dev;

	switch(req->bRequest) {
	case USB_CDC_REQ_SET_CONTROL_LINE_STATE:
		g_usbd_is_connected = req->wValue & 1; // Check RTS bit
		return USBD_REQ_HANDLED;
	break;
	case USB_CDC_REQ_SET_LINE_CODING: 
		if(*len < sizeof(struct usb_cdc_line_coding))
			return 0;

		return USBD_REQ_HANDLED;
	break;
	}
	
	return 0;
}

// Rx callback: Echoes back the input, stores the received command.
static void cdcacm_data_rx_cb(usbd_device *usbd_dev, uint8_t ep)
{
	(void)ep;

	char buf[64];
	char output[64];
	uint32_t len = usbd_ep_read_packet(usbd_dev, 0x01, buf, sizeof(buf));

	if (len) {
		// Look for '\r' and append '\n'
		uint32_t i;
		uint32_t new_len = 0;
		for (i = 0; (i < len) && (new_len < sizeof(buf)); i++) {
			if (buf[i] == '\r') {
				output[new_len++] = buf[i];
				if (new_len >= sizeof(output))
					new_len--;
				output[new_len++] = '\n';
				if (new_len >= sizeof(output))
					new_len--;

				// Switch over to using the new message.
				g_should_use_new_command = true;
			}
			// For backspace characters, go back one space, print a 'space' to overwrite
			// the character, then move the cursor back by one.
			else if ((buf[i] == 0x7f) || (buf[i] == '\b')) {
				if (g_new_command_len > 0) {
					output[new_len++] = '\b';
					if (new_len >= sizeof(output))
						new_len--;
					output[new_len++] = ' ';
					if (new_len >= sizeof(output))
						new_len--;
					output[new_len++] = '\b';
					if (new_len >= sizeof(output))
						new_len--;
					g_new_command[--g_new_command_len] = '\0';
				}
				continue;
			}
			// For printable characters, put them in the new-message buffer.
			else if (isprint(buf[i])) {
				g_new_command[g_new_command_len++] = buf[i];
				output[new_len++] = buf[i];
				if (new_len >= sizeof(output))
					new_len--;
			}
		}
		if (g_serial_echo_enabled) {
			usbd_ep_write_packet(usbd_dev, 0x82, output, new_len);
		}
		output[new_len] = 0;
	}
}

static void cdcacm_set_config(usbd_device *usbd_dev, uint16_t wValue)
{
	(void)wValue;

	usbd_ep_setup(usbd_dev, 0x01, USB_ENDPOINT_ATTR_BULK, 64, cdcacm_data_rx_cb);
	usbd_ep_setup(usbd_dev, 0x82, USB_ENDPOINT_ATTR_BULK, 64, 0);
	usbd_ep_setup(usbd_dev, 0x83, USB_ENDPOINT_ATTR_INTERRUPT, 16, 0);

	usbd_register_control_callback(
				usbd_dev,
				USB_REQ_TYPE_CLASS | USB_REQ_TYPE_INTERFACE,
				USB_REQ_TYPE_TYPE | USB_REQ_TYPE_RECIPIENT,
				cdcacm_control_request);
}

static void usb_puts(char *s)
{
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

inline static void set_led_timer_ccvb(enum led_colour led, uint32_t value)
{
	switch (led) {
		case GREEN_LED:
			TIMER0_CC0_CCVB = value;
			return;
		break;
		case RED_LED:
			TIMER1_CC0_CCVB = value;
			return;
		break;
		default: // Should never happen
			// usb_puts("ERROR SET LED TIMER CCVB\r\n"); // Commented to simplify inline function
			return;
		break;
	}
}

// TODO: Remove this timer?
void timer2_isr(void)
{
	#define CCV_INCREMENT 10

	static unsigned int heartbeat_count = 0;	
	char message_buf[8];

	// Clear overflow interrupt flag
	TIMER2_IFC = TIMER_IFC_OF;

	++heartbeat_count;
	itoa(heartbeat_count, message_buf, 10);

/*
	char my_itoa_buf[8] = "0000000";
	//my_itoa_buf[7] = '\0';	
	int current_index = 6;
	unsigned int num_to_convert = 9102349;

    while ((num_to_convert) && (current_index >= 0)) {	    
        my_itoa_buf[current_index] = '0' + (num_to_convert % 10);
		--current_index;
		num_to_convert /= 10;
    }
*/

	//TODO: Consecutive calls to usb_puts are rarely executed correctly.
	//TODO: More often than not, only the first call is executed.
	//TODO: Requires a delay, to allow the USB driver to finish writing to the endpoint, see HID keyboard example.
	//TODO: Test delay values, how low can I go?
	//TODO: Wrap usb_puts with a function that add a delay?

	if (g_debug_prints_enabled) {
		usb_puts("\r\nHeartbeat counter: ");
		udelay_busy(USB_PUTS_DELAY_USEC);
		usb_puts(message_buf);
		udelay_busy(USB_PUTS_DELAY_USEC);
	}
	
	// TODO: Testing toggle TIMER1 PWM compare value
	// Using the buffered version for CCV to avoid glitches.
	// Timer will wait until a PWM period is completed before setting the new compare value.
//	static int direction = 1;

/*
	if (g_debug_prints_enabled) {
		usb_puts("\r\nRED LED Direction: ");
		udelay_busy(USB_PUTS_DELAY_USEC);
		if (direction > 0) {
			usb_puts("UP");
		} else {
			usb_puts("DOWN");
		}
		udelay_busy(USB_PUTS_DELAY_USEC);
	}
*/

/*
	// Modify Red LED PWM duty cycle
	TIMER1_CC0_CCVB = TIMER1_CC0_CCV + direction*CCV_INCREMENT;
	if (TIMER1_CC0_CCVB >= LED_PWM_TIMER_TOP_CCV) {
		TIMER1_CC0_CCVB = LED_PWM_TIMER_TOP_CCV;
		direction = -1;
	} else if ((int32_t) (TIMER1_CC0_CCVB) <= 0) {
		TIMER1_CC0_CCVB = 0;
		direction = 1;
	}
*/
}

static void setup_timer2(void)
{
	cmu_periph_clock_enable(CMU_TIMER2);

	// Initialize TIMER2
	timer_set_clock_prescaler(TIMER2, TIMER_CTRL_PRESC_DIV1024);	
	// Top value of 24000, for div 1024, for 24 MHz clock is 1 Hz
	// So 48000 is once every 2 seconds
	timer_set_top(TIMER2, 48000); //TODO: Don't use magic number
    
    TIMER2_IEN = TIMER_IEN_OF;
    TIMER2_CNT = 0;

    // Enable TIMER2 interrupt
    nvic_enable_irq(NVIC_TIMER2_IRQ);

	timer_start(TIMER2);
}

void timer1_isr(void)
{
	// Clear overflow interrupt flag
	TIMER1_IFC = TIMER_IFC_OF;

	// TODO: Add fade state machine, as done for green LED

}

static void setup_timer1(void)
{
	cmu_periph_clock_enable(CMU_TIMER1);
	timer_set_clock_prescaler(TIMER1, LED_PWM_TIMER_PRESCALER);
	timer_set_top(TIMER1, LED_PWM_TIMER_TOP_CCV);
    
	TIMER1_CC0_CCV = 0; // PWM compare value
	TIMER1_CNT = 0; // Initial counter value

    TIMER1_IEN = TIMER_IEN_OF; // Interrupt enable: Overflow
    
	TIMER1_CC0_CTRL	|= TIMER_CC_CTRL_OUTINV; // Invert the output, our LEDs are active-low
	TIMER1_CC0_CTRL |= TIMER_CC_CTRL_MODE(TIMER_CC_CTRL_MODE_PWM);	

	// Set TIM1_CC0 output to Red LED GPIO's location
	// RED LED GPIO = PB7, TIM1_CC0 LOCATION #3
	TIMER1_ROUTE |= TIMER_ROUTE_LOCATION_LOCx(TIMER_ROUTE_LOCATION_LOC3);
	TIMER1_ROUTE |= TIMER_ROUTE_CC0PEN; // Enable output CC0 for TIMER1
    
    nvic_enable_irq(NVIC_TIMER1_IRQ); // Enable TIMER1 interrupt

	timer_start(TIMER1);
}

void timer0_isr(void)
{
	// Clear overflow interrupt flag
	TIMER0_IFC = TIMER_IFC_OF;

	static timer_cycles = 0;
	static cycle_millis = 0;
	enum led_colour led = GREEN_LED;

	if (g_green_counters_reset) {
		g_green_counters_reset = false;
		timer_cycles = 0;
		cycle_millis = 0;
	}

	++timer_cycles;

	if (!(timer_cycles % LED_PWM_TIMER_CYCLES_PER_MILLISECOND)) {
		++cycle_millis;
		timer_cycles = 0;
	}

	// TODO: refactor all of this into a function, for reuse with Red LED.
	// TODO: Maybe cycle_millis should be volatile global, one per LED.
	// TODO: Alternatively, the function will receive a pointer to cycle_millis and update it.

	struct led_pwm_cfg* led_cfg = &g_green_led_cfg;
	uint8_t brightness_delta = led_cfg->max_brightness - led_cfg->min_brightness;

	switch (led_cfg->current_phase) {
		case LED_PHASE_LOW:
			if (cycle_millis >= led_cfg->low_duration_ms) {
				led_cfg->current_phase = LED_PHASE_RAMP_UP;
				cycle_millis = 0;				
			} else {
				// Not necessary, but smooths the transition when setting the current phase from outside this function.
				set_led_timer_ccvb(led, led_cfg->min_brightness);
			}
		break;
		case LED_PHASE_RAMP_UP:
			if (cycle_millis >= led_cfg->ramp_up_ms) {
				led_cfg->current_phase = LED_PHASE_HIGH;
				set_led_timer_ccvb(led, led_cfg->max_brightness);
				cycle_millis = 0;
			} else {
				set_led_timer_ccvb(led, led_cfg->min_brightness + (brightness_delta *  cycle_millis / led_cfg->ramp_up_ms));
			}
		break;
		case LED_PHASE_HIGH:
			if (cycle_millis >= led_cfg->high_duration_ms) {
				led_cfg->current_phase = LED_PHASE_RAMP_DOWN;
				cycle_millis = 0;
			} else {
				// Not necessary, but smooths the transition when setting the current phase from outside this function.
				set_led_timer_ccvb(led, led_cfg->max_brightness);
			}
		break;
		case LED_PHASE_RAMP_DOWN:
			if (cycle_millis >= led_cfg->ramp_down_ms) {
				led_cfg->current_phase = LED_PHASE_LOW;
				set_led_timer_ccvb(led, led_cfg->min_brightness);
				cycle_millis = 0;
			} else {
				set_led_timer_ccvb(led, led_cfg->max_brightness - (brightness_delta *  cycle_millis / led_cfg->ramp_down_ms));
			}
		break;
		default: // Should never happen
			usb_puts("\r\nERROR LED PHASE!\r\n");
		break;
	}

}

static void setup_timer0(void)
{
	cmu_periph_clock_enable(CMU_TIMER0);
	timer_set_clock_prescaler(TIMER0, LED_PWM_TIMER_PRESCALER);
	timer_set_top(TIMER0, LED_PWM_TIMER_TOP_CCV);

	TIMER0_CC0_CCV = 0; // PWM compare value
	TIMER0_CNT = 0; // Initial counter value

    TIMER0_IEN = TIMER_IEN_OF; // Interrupt enable: Overflow

	TIMER0_CC0_CTRL	|= TIMER_CC_CTRL_OUTINV; // Invert the output, our LEDs are active-low
	TIMER0_CC0_CTRL |= TIMER_CC_CTRL_MODE(TIMER_CC_CTRL_MODE_PWM);	

	// Set TIM0_CC0 output to Green LED GPIO's location
	// GREEN LED GPIO = PA0, TIM0_CC0 LOC #0/1/4, also TIM0_CC1 LOCATION #6.
	TIMER0_ROUTE |= TIMER_ROUTE_LOCATION_LOCx(TIMER_ROUTE_LOCATION_LOC0);
	TIMER0_ROUTE |= TIMER_ROUTE_CC0PEN; // Enable output CC0 for TIMER0

	nvic_enable_irq(NVIC_TIMER0_IRQ); // Enable TIMER0 interrupt

	timer_start(TIMER0);
}


static void led_timer_stop(enum led_colour led)
{
	switch (led) {
		case GREEN_LED:
			timer_stop(TIMER0);
			TIMER0_CNT = 0;
		break;
		case RED_LED:
			timer_stop(TIMER1);
			TIMER1_CNT = 0;
		break;
		default: // Should never happen
			usb_puts("ERROR LED TIMER STOP\r\n");
			return;
		break;
	}
}

static void led_timer_start(enum led_colour led)
{
	switch (led) {
		case GREEN_LED:
			timer_start(TIMER0);
		break;
		case RED_LED:
			timer_start(TIMER1);
		break;
		default: // Should never happen
			usb_puts("ERROR LED TIMER START\r\n");
			return;
		break;
	}
}

static void led_isr_counters_reset(enum led_colour led)
{
	switch (led) {
		case GREEN_LED:
			g_green_counters_reset = true;
		break;
		case RED_LED:
			g_red_counters_reset = true;
		break;
		default: // Should never happen
			usb_puts("ERROR LED COUNTERS RESET\r\n");
			return;
		break;
	}
}

static void set_led_test_mode(struct led_pwm_cfg* led_cfg, uint32_t mode)
{
	if (led_cfg == NULL) {
		usb_puts("ERROR LED TEST\r\n");
		return;
	}

	switch (mode) {
		case 0: // LED breathing demo, low power indication
			led_cfg->min_brightness = 0;
			led_cfg->max_brightness = 100;
			led_cfg->low_duration_ms = 4000;
			led_cfg->high_duration_ms = 500;
			led_cfg->ramp_up_ms = 1000;
			led_cfg->ramp_down_ms = 1000; 
			led_cfg->current_phase = LED_PHASE_LOW;
		break;
		case 1: // LED blink demo
			led_cfg->min_brightness = 0;
			led_cfg->max_brightness = 100;
			led_cfg->low_duration_ms = 1000;
			led_cfg->high_duration_ms = 1000;
			led_cfg->ramp_up_ms = 0;
			led_cfg->ramp_down_ms = 0;
			led_cfg->current_phase = LED_PHASE_LOW;
		break;
		case 2: // LED constant ON demo
			led_cfg->min_brightness = 100;
			led_cfg->max_brightness = 100;
			led_cfg->low_duration_ms = 0;
			led_cfg->high_duration_ms = 1000;
			led_cfg->ramp_up_ms = 0;
			led_cfg->ramp_down_ms = 0;
			led_cfg->current_phase = LED_PHASE_HIGH;
		break;
		case 3: // LED constant OFF demo
			led_cfg->max_brightness = 0;
			led_cfg->min_brightness = 0;
			led_cfg->low_duration_ms = 1000;
			led_cfg->high_duration_ms = 0;
			led_cfg->ramp_up_ms = 0;
			led_cfg->ramp_down_ms = 0;
			led_cfg->current_phase = LED_PHASE_LOW;
		break;
		case 4: // LED symmetric min to max
			led_cfg->max_brightness = 100;
			led_cfg->min_brightness = 0;
			led_cfg->low_duration_ms = 1000;
			led_cfg->high_duration_ms = 1000;
			led_cfg->ramp_up_ms = 10000;
			led_cfg->ramp_down_ms = 10000;
			led_cfg->current_phase = LED_PHASE_LOW;
		break;
		default:
			usb_puts("ERROR CMD TEST\r\n");
			return;
		break;
	}

	if (g_debug_prints_enabled) {
		char message_buf[4];
		usb_puts("\r\nSet test mode: ");
		udelay_busy(USB_PUTS_DELAY_USEC);
		itoa(mode, message_buf, 10);
		usb_puts(message_buf);
	}
}

// Returns: true on successful parse, false otherwise.
static bool parse_flag(uint8_t flag, uint8_t param_char)
{
	bool* flag_p = NULL;

	switch (flag) {
	case CMD_DEBUG_PRINTS_FLAG:
		flag_p = &g_debug_prints_enabled;
	break;
	case CMD_SERIAL_ECHO_FLAG:
		flag_p = &g_serial_echo_enabled;
	break;
	default:
		usb_puts("ERROR FLAG TYPE\r\n"); // Should never happen
		return false;
	}

	if (flag_p == NULL) { // Should never happen
		usb_puts("ERROR FLAG POINTER\r\n");
		return false;
	}

	if (param_char == (uint8_t) '1') {
		*flag_p = true;
		return true;
	} else if (param_char == (uint8_t) '0') {
		*flag_p = false;
		return true;
	} else {
		usb_puts("ERROR PARSE FLAG\r\n");
		return false;
	}

	return false; // Shouldn't get here
}

// Returns: true on successful parse, false otherwise.
// Is successful, the value of last_char is the index of the last element parsed in the buffer
static bool parse_led_params(enum led_colour led, uint8_t* buffer, uint32_t len, uint32_t* last_char)
{
	uint32_t current_char_index = 0;
	uint32_t current_string_index = 0;
	struct led_pwm_cfg* led_cfg = NULL;
	uint8_t* param_strings[NUM_LED_PARAMS];

	if ((buffer == NULL) || (last_char == NULL) || (len == 0)) {
		usb_puts("ERROR PARSE LED PARAMS\r\n");
		return false;
	}

	switch (led) {
		case GREEN_LED:
			led_cfg = &g_green_led_cfg;
		break;
		case RED_LED:
			led_cfg = &g_red_led_cfg;
		break;
		default: // Should never happen
			usb_puts("ERROR PARSE LED PARAMS\r\n");
			return;
		break;
	}

	while (current_char_index < len) {
		// TODO: Go through all the whitespace using isspace, find a string, add it's start value to an array.
		// TODO: When you find the next whitespace, set it as NULL termination '\0'
		// TODO: If the last parameter string is the last index of the buffer, there is no room for whitespace.
		// TODO: Handle this case somehow... will I be forced to allocase buffers for all params?

		++current_char_index;
	}

	// TODO: We expect at least 7 strings: min max high low rampup rampdown phase
	if (current_string_index < (NUM_LED_PARAMS - 1)) {
		usb_puts("ERROR: Missing LED parameters!\r\n");
		return false;
	}

	// TODO: After all strings are found, convert each one using strtol (or atoi)
	// TODO: Verify the number is legal, i.e. percentage 0-100, phase type (letter?)
	// TODO: Verify the number isn't negative for milliseconds, or just treat it as uint32_t, very large positive.
	// TODO: Set the converted number to the appropriate cfg setting

	// TODO: Set the value of last_char to the last char index included in a string, not 

	return true;
}

static void handle_command(uint8_t* cmd_buffer, uint32_t buffer_len)
{
	static uint8_t test_mode = 0;
	uint8_t current_flag;
	enum led_colour current_led;

	if ((cmd_buffer == NULL) || (buffer_len == 0)) {
		return;
	}

	uint32_t current_char = 0;
	while (current_char < buffer_len) {

		if (isspace(cmd_buffer[current_char])) {
			++current_char;
			continue;
		}

		switch (toupper(cmd_buffer[current_char])) {
			case CMD_TEST_MODE:

				// LED configuration should stop the timer, then set config values, then start timer.
				// TODO: Generelize into a function which accept enum led_colour, so the function would know which TIMER to restart, etc.
				// TODO: But then I wouldn't be able to sync the LEDs, so maybe don't include the timer stuff in cfg function.

				// TODO: Currently test mode is only for Green LED
				led_timer_stop(GREEN_LED);
				set_led_test_mode(&g_green_led_cfg, test_mode);
				led_isr_counters_reset(GREEN_LED);
				led_timer_start(GREEN_LED);

				if (test_mode >= LED_TEST_MODE_MAX) {
					test_mode = 0;
				} else {
					++test_mode;
				}
			break;
			case CMD_DEBUG_PRINTS_FLAG:
			case CMD_SERIAL_ECHO_FLAG:
				current_flag = (uint8_t) toupper(cmd_buffer[current_char]);
				++current_char;
				if (current_char >= buffer_len) {
					usb_puts("Missing flag param: 0/1\r\n");
					return; // Don't parse the rest of the input
				}
				if (!parse_flag(current_flag, cmd_buffer[current_char])) {
					return; // Don't parse the rest of the input
				}
			break;
			case CMD_GREEN_LED_FLAG:
			case CMD_RED_LED_FLAG:

				if (toupper(cmd_buffer[current_char]) == CMD_GREEN_LED_FLAG) {
					current_led = GREEN_LED;
				} else {
					current_led = RED_LED;
				}

				++current_char;
				if (current_char >= buffer_len) {
					usb_puts("ERROR: Missing LED params!\r\n");
					return; // Don't parse the rest of the input
				}

				led_timer_stop(current_led);
				
				if (cmd_buffer[current_char] == ' ') {
					if (!parse_led_params(current_led, &(cmd_buffer[current_char]), buffer_len - current_char, &current_char)) {
						// Restore the timer, will use unmodified settings.
						led_isr_counters_reset(current_led);
						led_timer_start(current_led);
						return; // Don't parse the rest of the input
					}
				}

				// TODO: Switch case of different LED param letters here? X N H L U D P


				led_isr_counters_reset(current_led);
				led_timer_start(current_led);

			// TODO: Maybe instead of parsing a string with 7 different parameters, I'll first implement setting each one seperately?
			// TODO: E.g. GX100 GN0 GH1500 GL800 GU1000 GD2000 GPL
			// TODO: Set Green LED max, min, high_ms, low_ms, rampup_ms, rampdown_ms, phase (L/H/U/D)
			// TODO: If 'G' is followed by a space, I can parse the full command, otherwise just one param.

			break;
			default:
				usb_puts("ERROR Parsing Command\r\n");
				return;
		} // Switch

		++current_char;
	}

/* // TODO: Remove this
	if (g_debug_prints_enabled) {
		usb_puts("Command received\r\n");
		udelay_busy(USB_PUTS_DELAY_USEC);
	}
*/
	// TODO: Parse the command.
	// TODO: Add function to parse LED config and return success/failure
	// TODO: G MIN_% MAX_% LOW_MS RAMPUP_MS HIGH_MS RAMPDOWN_MS R MIN_% …
	// TODO: Add function to pase a flag (input 0/1) and return flag value or failure
	// TODO: Or maybe send the address of the bool to set, and have the function handle everything?
	// TODO: D1 D0 E1 E0


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
	// LED GPIO polarity is reveresed, gpio_set will turn off LEDs.
	gpio_set(LED_RED_PORT, LED_RED_PIN);
	gpio_set(LED_GREEN_PORT, LED_GREEN_PIN);

	/* Configure the USB core & stack */
	g_usbd_dev = usbd_init(&efm32hg_usb_driver, &dev, &config, usb_strings, 3, usbd_control_buffer, sizeof(usbd_control_buffer));
	usbd_register_set_config_callback(g_usbd_dev, cdcacm_set_config);

	/* Set the CPU Core to run from the trimmed USB clock, divided by 2.
	 * This will give the CPU Core a frequency of 24 MHz +/- 1% */
	CMU_CMD = CMU_CMD_HFCLKSEL(5);
	while (! (CMU_STATUS & CMU_STATUS_USHFRCODIV2SEL))
		;

	// Enable USB IRQs
	nvic_enable_irq(NVIC_USB_IRQ);

	// TODO: Should probably leave all values initialized to 0, and have the Timer as Off by default.

	// Initialize LED fade values
	set_led_test_mode(&g_green_led_cfg, 0); 
	set_led_test_mode(&g_red_led_cfg, 0); 

	setup_timer0();
	setup_timer1();
	setup_timer2();

	while (1) {

		if (line_was_connected != g_usbd_is_connected) {
			if (g_usbd_is_connected) {
				// Wait for the terminal to appear
				udelay_busy(1000);
				usb_puts("\r\nEnter command, followed by Return:\r\n" PROMPT);
			}
			line_was_connected = g_usbd_is_connected;
		}

		if (!g_usbd_is_connected)
			continue;

		// Handle the received command, if there's a new one available.
		if (g_should_use_new_command) {
			memset(g_current_command, 0, sizeof(g_current_command));

			// Print a different message depending on whether the new message is blank.
			if (g_new_command_len) {
				memcpy(g_current_command, g_new_command, g_new_command_len);
				handle_command(g_current_command, g_new_command_len);
			}
			else {
				usb_puts("\r\n" PROMPT);
			}

			g_should_use_new_command = false;
			g_new_command_len = 0;
		}
	}
}
