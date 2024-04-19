/*___________________________________________________________________________________________________

Title:
	IRremote.h v1.0

Description:
	Library for sending and receiving remote controller codes on AVR devices
	
	For complete details visit:
	https://www.programming-electronics-diy.xyz/2022/08/ir-remote-control-library-for-avr.html

Author:
 	Liviu Istrate
	istrateliviu24@yahoo.com
	www.programming-electronics-diy.xyz

Donate:
	Software development takes time and effort so if you find this useful consider a small donation at:
	paypal.me/alientransducer
_____________________________________________________________________________________________________*/


/* ----------------------------- LICENSE - GNU GPL v3 -----------------------------------------------

* This license must be included in any redistribution.

* Copyright (c) 2022 Liviu Istrate, www.programming-electronics-diy.xyz (istrateliviu24@yahoo.com)

* Project URL: https://www.programming-electronics-diy.xyz/2022/08/ir-remote-control-library-for-avr.html

* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.

* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.

* You should have received a copy of the GNU General Public License
* along with this program. If not, see <https://www.gnu.org/licenses/>.

--------------------------------- END OF LICENSE --------------------------------------------------*/

#ifndef IRREMOTE_H_
#define IRREMOTE_H_

//-----------------------------------------------------------------------------
// Includes
//-----------------------------------------------------------------------------
#include <avr/io.h>
#include <util/atomic.h>
#include <avr/interrupt.h>
#include <util/delay.h>

//-----------------------------------------------------------------------------
// Macros User SETUP
//-----------------------------------------------------------------------------
// Input Capture Pin (must be an ICP)
#define ICP_DDR						DDRB
#define ICP_PORT					PORTB
#define ICP_PIN						PB0

// OCRA pin used for transmitting
#define IR_CARRIER_DDR				DDRB
#define IR_CARRIER_PORT				PORTB
#define IR_CARRIER_PIN				PB1

// Size of the receiver buffer that holds the pulse length in us
//#define IR_RX_BUFFER_SIZE			68 // increase this if the main loop is slow

//-----------------------------------------------------------------------------
// Macros
//-----------------------------------------------------------------------------
// See the Timer calculator spreadsheet for results based on prescaler selection
#if F_CPU >= 16000000
	#define IR_TIMER_PRESCALER		64
#elif F_CPU >= 8000000
	#define IR_TIMER_PRESCALER		8
#else
	#define IR_TIMER_PRESCALER		1
#endif

// One timer value in microseconds
#define IR_TICK_TIME				(1.0 / (F_CPU / (float)IR_TIMER_PRESCALER)) * 1000000 // convert to microseconds
#define IR_TIMER_OVERFLOW			IR_TICK_TIME * 65536 // us
#define IR_OCR_38KHZ				(F_CPU / (2 * 1 * 38000))
#define IR_OCR_DUTYCYCLE			IR_OCR_38KHZ / 4 // 0.25% duty cycle

// Minimum and maximum time in microseconds
// NEC
#define IR_NEC_AGC_MIN_TIME				9000
#define IR_NEC_AGC_MAX_TIME				9300
#define IR_NEC_LONG_PAUSE_MIN_TIME		4300
#define IR_NEC_LONG_PAUSE_MAX_TIME		4600
#define IR_NEC_SHORT_PAUSE_MIN_TIME		2000
#define IR_NEC_SHORT_PAUSE_MAX_TIME		2300

// RC-5
#define IR_RC5_MIN_TIME					880
#define IR_RC5_MAX_TIME					980
#define IR_RC5_JOINED_BITS_TIME			1500

// IR protocols
#define IR_PROTOCOL_NONE				0
#define IR_PROTOCOL_NEC					1
#define IR_PROTOCOL_NEC_EXTENDED		2
#define IR_PROTOCOL_RC5					3
#define IR_PROTOCOL_RC5_EXTENDED		4

typedef uint8_t	bool;
#define true	1
#define false	0

// Timer 1
#define TCCRnA				TCCR1A
#define TCCRnB				TCCR1B
#define COMnA1				COM1A1
#define WGMn3				WGM13
#define ICNCn				ICNC1
#define CSn1				CS11
#define CSn0				CS10
#define TIMSKn				TIMSK1
#define ICIEn				ICIE1
#define TOIEn				TOIE1
#define ICRn				ICR1
#define OCRnA				OCR1A
#define ICESn				ICES1
#define TIMERn_CAPT_vect	TIMER1_CAPT_vect
#define TIMERn_OVF_vect		TIMER1_OVF_vect


// By default the output of IR receiver is high.
// When the carrier frequency is detected, the output will go low.
// So to make the IR receiver output low, the carrier must be on.
#define IR_OUTPUT_LOW(){\
	TCCRnA |= (1 << COMnA1);\
	TCNT1 = 0;\
}

#define IR_OUTPUT_HIGH(){\
	TCCRnA &= ~(1 << COMnA1);\
	TCNT1 = 0;\
}

//-----------------------------------------------------------------------------
// Global VARIABLES
//-----------------------------------------------------------------------------
typedef enum {
	IR_STAGE_PULSE_1,
	IR_STAGE_PULSE_2,
	IR_STAGE_PULSE_3,
	IR_STAGE_PULSE_4,
	IR_STAGE_PULSE_5,
	IR_STAGE_PULSE_6,
	IR_STAGE_ADDRESS,
	IR_STAGE_DATA_BITS,
	IR_STAGE_STOP_BIT,
	IR_STAGE_END
} IR_Stage;

// Receiver
static uint32_t IR_dataRX;
static uint16_t IR_address;
static uint16_t IR_command;
static uint8_t IR_protocolType;
static uint8_t IR_protocolTypeExt;
static uint8_t IR_RXexpectedBits;
static uint8_t IR_stageRX;
static uint8_t IR_ToggleBit;
static uint8_t IR_waitNextPulse;
static bool IR_RepeatCode;
static bool IR_frameStart;
volatile static uint16_t IR_Timestamp;
volatile static uint8_t IR_TimerOverflows;

//-----------------------------------------------------------------------------
// Function PROTOTYPES
//-----------------------------------------------------------------------------
void IR_init(void);
bool IR_codeAvailable(void);
bool IR_isRepeatCode(void);
uint8_t IR_getToggleBit(void);
uint8_t IR_getProtocol(void);
void IR_getCode(uint16_t* address, uint16_t* command);
void IR_sendCode(uint16_t address, uint16_t command, uint8_t toggle, uint8_t protocol);
void IR_disable(void);

static void IR_resetRX(void);


//-----------------------------------------------------------------------------
// Functions
//-----------------------------------------------------------------------------

/*-----------------------------------------------------------------------------
	
------------------------------------------------------------------------------*/
void IR_init(void){
	// Set Input Capture pin as input
	ICP_DDR &= ~(1 << ICP_PIN);
	
	// Pin used to generate the 38kHz carrier signal
	IR_CARRIER_PORT &= ~(1 << IR_CARRIER_PIN);
	IR_CARRIER_DDR |= (1 << IR_CARRIER_PIN);
	
	/* Set Input Capture Pin */
	// Input Capture Noise Canceler
	// Input Capture Edge Select Falling Edge
	TCCRnB = (1 << ICNCn);
	
	// Set prescaler
	#if IR_TIMER_PRESCALER == 64
		TCCRnB |= (1 << CSn1) | (1 << CSn0);
	#elif IR_TIMER_PRESCALER == 8
		TCCRnB |= (1 << CSn1);
	#elif IR_TIMER_PRESCALER == 1
		TCCRnB |= (1 << CSn0);
	#endif
	
	// Timer/Counter 1, Input Capture Interrupt Enable
	// Timer/Counter 1, Overflow Interrupt Enable
	TIMSKn = (1 << ICIEn) | (1 << TOIEn);

	sei();
}


/*-----------------------------------------------------------------------------
	Returns true when new code is available. The decoding is also done here.
------------------------------------------------------------------------------*/
bool IR_codeAvailable(void){
	bool code_received = false;
	uint8_t command_inv;
	IR_protocolTypeExt = IR_protocolType;
	
	if(IR_stageRX == IR_STAGE_END){
		if(IR_protocolType == IR_PROTOCOL_NEC){
			if(IR_dataRX == 0){
				// Repeat code received
				code_received = true;
				goto EXIT;
			}
			
			// Save lower 2 bytes - the address
			IR_address = IR_dataRX;
			// Save the command
			IR_command = IR_dataRX >> 16;
			IR_command &= 0x00FF;
			command_inv = IR_dataRX >> 24;
			command_inv = ~command_inv;
			
			// Check if extended protocol
			if((IR_address & 0x00FF) == ~(IR_address & 0xFF00)){
				IR_address &= 0x00FF;
			}else{
				IR_protocolTypeExt = IR_PROTOCOL_NEC_EXTENDED;
			}
			
			// Validate the command
			if(IR_command == command_inv){
				code_received = true;
			}
			
		}else if(IR_protocolType == IR_PROTOCOL_RC5){
			code_received = true;
			
			IR_address = (IR_dataRX >> 6) & 0x1F;
			IR_command = IR_dataRX & 0x3F;
			IR_ToggleBit = (IR_dataRX >> 11) & 0x01;
			
			// Second Start bit
			if(((IR_dataRX >> 12) & 0x01) == 0){
				IR_protocolTypeExt = IR_PROTOCOL_RC5_EXTENDED;
				IR_command |= (1 << 6);
			}
		}
		
		EXIT:
		IR_stageRX = IR_STAGE_PULSE_1;
		IR_resetRX();
	}
	
	if(code_received) return true;
	return false;
}


/*-----------------------------------------------------------------------------
	When IR_codeAvailable() returns true, this function can be used to
	obtain the decoded address and command.
	
	address, command	pointer to variables where to store decoded data
------------------------------------------------------------------------------*/
void IR_getCode(uint16_t* address, uint16_t* command){
	*address = IR_address;
	*command = IR_command;
	IR_address = IR_command = 0;
}


/*-----------------------------------------------------------------------------
	RC-5 protocol only.
	Returns the toggle bit: 0 or 1.
------------------------------------------------------------------------------*/
uint8_t IR_getToggleBit(void){
	return IR_ToggleBit;
}


/*-----------------------------------------------------------------------------
	Returns the protocol type defined by the following macros:
	
	IR_PROTOCOL_NONE				0
	IR_PROTOCOL_NEC					1
	IR_PROTOCOL_NEC_EXTENDED		2
	IR_PROTOCOL_RC5					3
	IR_PROTOCOL_RC5_EXTENDED		4
------------------------------------------------------------------------------*/
uint8_t IR_getProtocol(void){
	return IR_protocolTypeExt;
}


/*-----------------------------------------------------------------------------
	NEC protocol only.
	Returns true if a repeat code is received.
------------------------------------------------------------------------------*/
bool IR_isRepeatCode(void){
	bool buff = IR_RepeatCode;
	IR_RepeatCode = false;
	return buff;
}


/*-----------------------------------------------------------------------------
	Encodes and sends data using delays. The input capture pin interrupt 
	is disabled during this time and the timer is used to generate the carrier
	on a particular pin. The modulation is done by disabling the timer.
	
	toggle			The toggle bit, 0 or 1 (for RC-5 only).
					For NEC a 1 means a repeat code.
	protocol		One of the defined protocols
------------------------------------------------------------------------------*/
void IR_sendCode(uint16_t address, uint16_t command, uint8_t toggle, uint8_t protocol){
	uint32_t frame = 0;
	
	// Block ICP ISR
	IR_stageRX = IR_STAGE_END;

	// Generate 38kHz carrier
	// PWM, Phase and Frequency	Correct, Mode 8
	// Clear OC1A/OC1B on Compare Match when up-counting.
	// Set OC1A/OC1B on Compare Match when down-counting.
	// Prescaler 1
	TCCRnA = (1 << COMnA1);
	TCCRnB = (1 << WGMn3) | (1 << CSn0);
	TIMSKn = 0;
	ICRn = IR_OCR_38KHZ; // 38kHz
	OCRnA = IR_OCR_DUTYCYCLE;

	if((protocol == IR_PROTOCOL_NEC) || (protocol == IR_PROTOCOL_NEC_EXTENDED)){
		frame = (uint32_t)command << 16;
		frame |= (uint32_t)(~command) << 24;
		frame |= address; // extended 16-bits address
		
		if(protocol == IR_PROTOCOL_NEC) frame |= (~address) << 8;
		
		// Send leader code
		// AGC pulse - low for x ms
		IR_OUTPUT_LOW();
		_delay_ms(9);
	
		// Long or short pause - high x ms
		IR_OUTPUT_HIGH();
	
		if(toggle){
			_delay_us(2250);
			goto END_CODE;
		}else{
			_delay_us(4500);
		}
	
		// Send code - LSB first
		for(uint8_t i = 0; i < 32; i++){
			if(((uint32_t)1 << i) & frame){
				// Send 1
				IR_OUTPUT_LOW();
				_delay_us(562); // low pulse
				IR_OUTPUT_HIGH();
				_delay_us(1687); // high pulse
			}else{
				// Send 0
				IR_OUTPUT_LOW();
				_delay_us(562); // low pulse
				IR_OUTPUT_HIGH();
				_delay_us(562); // high pulse
			}
		}
	
		END_CODE:
	
		// Stop bit
		IR_OUTPUT_LOW();
		_delay_us(562); // low pulse
		IR_OUTPUT_HIGH(); // idle high
		
	}else if((protocol == IR_PROTOCOL_RC5) || (protocol == IR_PROTOCOL_RC5_EXTENDED)){
		// Set Start bit 1 to 1
		frame = (1 << 13);
		
		// Set Start bit 2
		frame |= (1 << 12); // set bit to 1
		
		if(protocol == IR_PROTOCOL_RC5_EXTENDED){
			// Set bit to 0 that represents a 1 but inverted
			if(command & (1 << 6)) frame &= ~(1 << 12);
		}
		
		// Set toggle bit
		if(toggle) frame |= (1 << 11);
		
		// Address
		frame |= address << 6;
		
		// Command
		frame |= command;
		
		// Send code - MSB first
		for(uint8_t i = 0; i < 14; i++){
			if(((uint16_t)1 << (13 - i)) & frame){
				// Send 1
				IR_OUTPUT_HIGH();
				_delay_us(889); // low pulse
				IR_OUTPUT_LOW();
				_delay_us(889); // high pulse
			}else{
				// Send 0
				IR_OUTPUT_LOW();
				_delay_us(889); // low pulse
				IR_OUTPUT_HIGH();
				_delay_us(889); // high pulse
			}
		}
	}
	
	// Restore Timer for Receive mode
	TCCRnA = 0;
	IR_init();
	IR_stageRX = IR_STAGE_PULSE_1;
}


/*-----------------------------------------------------------------------------
	Disable the Timer and associated interrupts.
------------------------------------------------------------------------------*/
void IR_disable(void){
	TIMSKn = 0;
	TCCRnB = 0;
}


static void IR_resetRX(void){
	IR_dataRX = 0;
	IR_RXexpectedBits = 0;
	IR_waitNextPulse = 0;
	IR_stageRX = IR_STAGE_PULSE_1;
}


//-----------------------------------------------------------------------------
// ISR Handlers
//-----------------------------------------------------------------------------
// Timer/Counter Capture Event
// ~35us @ 16MHz

/*-----------------------------------------------------------------------------
	The protocol type is defined here but not the extended ones. 
	If the protocol is extended is decided in the decoder function.
	This ISR triggers on every pulse then the pulses are converted to bits
	that are stored in a 32-bit variable used by the decoder function.
------------------------------------------------------------------------------*/
ISR(TIMERn_CAPT_vect){
	uint16_t pulse_length;
	uint16_t IR_TimestampPrev;
	uint8_t pulse_level;
	
	if(IR_stageRX == IR_STAGE_END) return;
	
	// Save previous timestamp
	IR_TimestampPrev = IR_Timestamp;

	// Read TCNT1 timestamp
	IR_Timestamp = ICRn;
	
	// Select the opposite edge to trigger
	TCCRnB ^= (1 << ICESn);
	pulse_level = TCCRnB & (1<<ICESn);
	
	// Calculate time difference in timer ticks
	if(IR_TimerOverflows){
		pulse_length = (65535 - IR_TimestampPrev) + IR_Timestamp;
		IR_TimerOverflows = 0;
	}else{
		pulse_length = IR_Timestamp - IR_TimestampPrev;
	}
	
	pulse_length = pulse_length * IR_TICK_TIME;
	
	// Reset protocol and decoding after long idle period
	// Depending on the protocol used, this can also hold half of start bit
	if(pulse_length > 15000){
		// This prevents collecting data in the middle of the frame
		IR_frameStart = true;
		IR_resetRX();
		return;
	}
	
	
	// Detect protocol type by checking leader code
	switch(IR_stageRX){
		case IR_STAGE_PULSE_1:
			IR_frameStart = false;
			IR_protocolType = 0;
			IR_stageRX = IR_STAGE_PULSE_2;
		
			if(pulse_length < IR_NEC_AGC_MAX_TIME && pulse_length > IR_NEC_AGC_MIN_TIME){
				// NEC protocol, AGC pulse
				IR_protocolType = IR_PROTOCOL_NEC;
			}else if(pulse_level == 0 && pulse_length > IR_RC5_MIN_TIME && pulse_length < IR_RC5_MAX_TIME){
				// RC-5 protocol
				// Second half of the Start Bit 1
				IR_protocolType = IR_PROTOCOL_RC5;
				IR_stageRX = IR_STAGE_DATA_BITS;
				IR_RXexpectedBits = 13;
			
				if(pulse_length > IR_RC5_JOINED_BITS_TIME){
					IR_waitNextPulse++;
				}
			}
		break;
			
		case IR_STAGE_PULSE_2:	
			if(IR_protocolType == IR_PROTOCOL_NEC){
				if(pulse_length < IR_NEC_LONG_PAUSE_MAX_TIME && pulse_length > IR_NEC_LONG_PAUSE_MIN_TIME){
					// Check for long pause
					IR_stageRX = IR_STAGE_DATA_BITS;

				}else if(pulse_length < IR_NEC_SHORT_PAUSE_MAX_TIME && pulse_length > IR_NEC_SHORT_PAUSE_MIN_TIME){
					// Check for short pause
					IR_RepeatCode = true;
					IR_stageRX = IR_STAGE_END;
				}
		
			}
		break;
	
		case IR_STAGE_DATA_BITS:
			// Collect NEC data bits - LSB is sent first
			if(IR_protocolType == IR_PROTOCOL_NEC){
				// Wait for two pulses to form a bit
				if(pulse_level){
					if(pulse_level && pulse_length > 1500){
						IR_dataRX |= ((uint32_t)1 << IR_RXexpectedBits);
					}

					IR_RXexpectedBits++;

					// All bits received. Wait for STOP bit.
					if(IR_RXexpectedBits > 31){
						IR_stageRX = IR_STAGE_STOP_BIT;
					}
				}
			
			// Collect RC-5 data bits - MSB is sent first
			}else if(IR_protocolType == IR_PROTOCOL_RC5){
				IR_waitNextPulse++;
			
				// Wait for two pulses to form a bit
				if(IR_waitNextPulse > 1){
					IR_waitNextPulse = 0;
					IR_RXexpectedBits--;
				
					if(pulse_level == 0){
						IR_dataRX |= ((uint32_t)1 << IR_RXexpectedBits);
					}
				}
			
				// Received half of the next bit
				if(pulse_length > IR_RC5_JOINED_BITS_TIME){
					IR_waitNextPulse++;
				}
			
				// All bits received
				if(IR_RXexpectedBits == 1 && IR_waitNextPulse == 1 && pulse_level == 0){
					// The case when bit 0 is at the end
					IR_stageRX = IR_STAGE_END;
				}

				if(IR_RXexpectedBits == 0){
					// The case when bit 1 is at the end
					IR_stageRX = IR_STAGE_END;
				}
			}
		break;
		
		case IR_STAGE_STOP_BIT:
			IR_stageRX = IR_STAGE_END;
	}
}


// Timer/Counter Overflow
ISR(TIMERn_OVF_vect){
	IR_TimerOverflows++;
}

#endif /* IRREMOTE_H_ */