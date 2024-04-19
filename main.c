/*___________________________________________________________________________________________________

Title:
	AVR LED IR remote lamp

Description:
	This code is designed for controlling LED brightness and color using an IR remote control.
  It utilizes software PWM for smooth LED dimming and toggling of RGB LEDs.

Author:
 	QuintusJoyal
_____________________________________________________________________________________________________*/

#include <avr/io.h> // AVR standard I/O definitions
#include <util/delay.h> // Functions to create time delays
#include <avr/interrupt.h> // AVR interrupt handling functions
// #include "./src/libs/uart.h" // UART communication library
#include "./src/libs/IRremote.h" // IR remote control library
#include "./src/libs/softwarePWM.h" // Software PWM library

#define F_CPU 16000000UL // Define CPU frequency
#define BAUDRATE 9600 // Define baud rate for UART communication

#define VOL_UP 25 // IR command code for volume up
#define VOL_DOWN 22 // IR command code for volume down
#define CH_UP 21 // IR command code for channel up
#define CH_DOWN 7 // IR command code for channel down
#define POWER 69 // IR command code for power

#define BRIGHTNESS_STEP (uint8_t) 15 // Step size for adjusting brightness

uint8_t brightness = 20; // Initial brightness level
uint8_t level = 0; // Initial LED level

// Function to set LED brightness according to level
void LED_brightness() {
  for (uint8_t i = 0; i < level; i++) {
    softwarePWM_Set(i, brightness);
  }
}

// Function to turn on a specific LED
void LED_turn_on(uint8_t led) {
  softwarePWM_Set(led, brightness);
}

// Function to turn off a specific LED
void LED_turn_off(uint8_t led) {
  softwarePWM_Set(led, 0);
}

// Function to toggle RGB LEDs based on level
void RGB_toggle() {
  uint8_t m = (level - 1) % 3;
  softwarePWM_Set(4, brightness * (m == 0));
  softwarePWM_Set(5, brightness * (m == 1));
  softwarePWM_Set(6, brightness * (m == 2));
}

// Function to increase volume level
void vol_up() {
  if (level > 6) { 
    RGB_toggle(); // Toggle RGB LEDs
    level = (level <= 9) ? level + 1 : 7; // Increment level, limit at 7
    return;
  }
  LED_turn_on(level++); // Turn on LED at current level
}

// Function to decrease volume level
void vol_down() {
  if (level > 7) {
    --level;
    RGB_toggle(); // Toggle RGB LEDs
    return;
  }
  if (level == 7) {
    // Turn on RGB LEDs at level 7
    LED_turn_on(4);
    LED_turn_on(5);
    LED_turn_on(6);
    --level;
    return;
  }
  LED_turn_off(level--); // Turn off LED at current level
  if (level == UINT8_MAX) level = 0; // Wrap level around if overflow
}

// Function to increase brightness
void ch_up() {
  brightness = (brightness + BRIGHTNESS_STEP < UINT8_MAX) ? 
    brightness + (level > 0) * BRIGHTNESS_STEP : UINT8_MAX; // Increase brightness, limit at maximum
  LED_brightness(); // Adjust LED brightness
}

// Function to decrease brightness
void ch_down() {
  brightness = (brightness - BRIGHTNESS_STEP > 0) ? 
    brightness - (level > 0) * BRIGHTNESS_STEP : 0; // Decrease brightness, limit at minimum
  LED_brightness(); // Adjust LED brightness
}

// Function to turn off all LEDs
void power() {
  for (int8_t i = 6; i >= 0; i--) LED_turn_off(i); // Turn off all LEDs
  level = 0; // Reset LED levels
}

// Function to control LEDs based on received IR command
void LED_control(uint16_t command) {
  switch (command) {
    case VOL_UP: vol_up(); break; // Volume up command
    case VOL_DOWN: vol_down(); break; // Volume down command
    case CH_UP: ch_up(); break; // Channel up command
    case CH_DOWN: ch_down(); break; // Channel down command
    case POWER: power(); break; // Power command
    default: break;
  }
}

int main()
{
  uint16_t address = 0; // Variable to store IR address
  uint16_t command = 0; // Variable to store IR command

  IR_init(); // Initialize IR remote control
  // uart_init(); // Initialize UART communication
  softwarePWM_Init(); // Initialize software PWM

  // uart_puts("Well I'm working here!\r\n"); // Send message indicating successful initialization
  while (1) {
    if (IR_codeAvailable()) { // Check if IR code is available
      if (!IR_isRepeatCode()) { // Check if IR code is not a repeat
        IR_getCode(&address, &command); // Get IR address and command
        LED_control(command); // Control LEDs based on received command
        // uart_putU16(address); // Send IR address over UART
        // uart_puts(", ");
        // uart_putU16(command); // Send IR command over UART
        // uart_puts(", ");
        // uart_putU8(level); // Send current LED level over UART
        // uart_puts("\r\n");
      }
    }
  }
}
