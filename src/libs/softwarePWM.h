/*___________________________________________________________________________________________________

Title:
	softwarePWM.h v1.0

Description:
	Multichannel software PWM based on "AVR136: Low-Jitter Multi-Channel Software PWM"
	application note.
	
	For complete details visit:
	https://www.programming-electronics-diy.xyz/2021/02/multi-channel-software-pwm-library-for.html

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

* Copyright (C) 2021 Liviu Istrate, www.programming-electronics-diy.xyz (istrateliviu24@yahoo.com)

* Project URL: https://www.programming-electronics-diy.xyz/2021/02/multi-channel-software-pwm-library-for.html

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

#ifndef SOFTWARE_PWM_H
#define SOFTWARE_PWM_H

/*************************************************************
	INCLUDES
**************************************************************/
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/power.h>
#include <avr/pgmspace.h>

/*************************************************************
	SYSTEM SETTINGS
**************************************************************/
#ifndef PORTE
   #define PINE 	_SFR_IO8(0x0C)
   #define DDRE 	_SFR_IO8(0x0D)
   #define PORTE 	_SFR_IO8(0x0E)
#endif

#define TRUE		1
#define FALSE		0

#define SOFTPWM_TIMER0 			0 // Use timer0
#define SOFTPWM_TIMER1 			1 // Use timer1
#define SOFTPWM_TIMER2 			2 // Use timer2

#define SOFTPWM_PWMDEFAULT		0x00  // default PWM value at start up for all channels


/*************************************************************
	USER SETUP SECTION
**************************************************************/
#define SOFTPWM_TIMER 					SOFTPWM_TIMER0 	// Which timer to use
#define SOFTPWM_TIMER1_OCR				255
#define SOFTPWM_CHMAX					7    			// maximum number of PWM channels
#define USE_LOGARITHMIC_ARRAY			FALSE

// Mapping channels to pin and ports
#define SOFTPWM_CHANNEL0_DDR			DDRD
#define SOFTPWM_CHANNEL0_PORT			PORTD
#define SOFTPWM_CHANNEL0_PIN			4
#define SOFTPWM_CHANNEL0_PIN_STATES		pinlevelD

#define SOFTPWM_CHANNEL1_DDR			DDRD
#define SOFTPWM_CHANNEL1_PORT			PORTD
#define SOFTPWM_CHANNEL1_PIN			5
#define SOFTPWM_CHANNEL1_PIN_STATES		pinlevelD

#define SOFTPWM_CHANNEL2_DDR			DDRD
#define SOFTPWM_CHANNEL2_PORT			PORTD
#define SOFTPWM_CHANNEL2_PIN			6
#define SOFTPWM_CHANNEL2_PIN_STATES		pinlevelD

#define SOFTPWM_CHANNEL3_DDR			DDRD
#define SOFTPWM_CHANNEL3_PORT			PORTD
#define SOFTPWM_CHANNEL3_PIN			7
#define SOFTPWM_CHANNEL3_PIN_STATES		pinlevelD

#define SOFTPWM_CHANNEL4_DDR			DDRB
#define SOFTPWM_CHANNEL4_PORT			PORTB
#define SOFTPWM_CHANNEL4_PIN			2
#define SOFTPWM_CHANNEL4_PIN_STATES		pinlevelB

#define SOFTPWM_CHANNEL5_DDR			DDRB
#define SOFTPWM_CHANNEL5_PORT			PORTB
#define SOFTPWM_CHANNEL5_PIN			3
#define SOFTPWM_CHANNEL5_PIN_STATES		pinlevelB

#define SOFTPWM_CHANNEL6_DDR			DDRB
#define SOFTPWM_CHANNEL6_PORT			PORTB
#define SOFTPWM_CHANNEL6_PIN			4
#define SOFTPWM_CHANNEL6_PIN_STATES		pinlevelB

#define SOFTPWM_CHANNEL7_DDR			0
#define SOFTPWM_CHANNEL7_PORT			0
#define SOFTPWM_CHANNEL7_PIN			0
#define SOFTPWM_CHANNEL7_PIN_STATES		0

#define SOFTPWM_CHANNEL8_DDR			0
#define SOFTPWM_CHANNEL8_PORT			0
#define SOFTPWM_CHANNEL8_PIN			0
#define SOFTPWM_CHANNEL8_PIN_STATES		0

#define SOFTPWM_CHANNEL9_DDR			0
#define SOFTPWM_CHANNEL9_PORT			0
#define SOFTPWM_CHANNEL9_PIN			0
#define SOFTPWM_CHANNEL9_PIN_STATES		0

// Which ports are used
#define SOFTPWM_ON_PORT_B				TRUE
#define SOFTPWM_ON_PORT_C				FALSE
#define SOFTPWM_ON_PORT_D				TRUE
#define SOFTPWM_ON_PORT_E				FALSE

// What pins are on this ports
#define SOFTPWM_PINS_PORTB				_BV(2) | _BV(3) | _BV(4)
#define SOFTPWM_PINS_PORTC				FALSE
#define SOFTPWM_PINS_PORTD				_BV(4) | _BV(5) | _BV(6) | _BV(7)
#define SOFTPWM_PINS_PORTE				FALSE


/*************************************************************
	SYSTEM SETTINGS
**************************************************************/

#if SOFTPWM_CHMAX > 0
	#define CH0_CLEAR 						(SOFTPWM_CHANNEL0_PIN_STATES &= ~(1 << SOFTPWM_CHANNEL0_PIN))
#endif

#if SOFTPWM_CHMAX > 1
	#define CH1_CLEAR 						(SOFTPWM_CHANNEL1_PIN_STATES &= ~(1 << SOFTPWM_CHANNEL1_PIN))
#endif

#if SOFTPWM_CHMAX > 2
	#define CH2_CLEAR 						(SOFTPWM_CHANNEL2_PIN_STATES &= ~(1 << SOFTPWM_CHANNEL2_PIN))
#endif

#if SOFTPWM_CHMAX > 3
	#define CH3_CLEAR 						(SOFTPWM_CHANNEL3_PIN_STATES &= ~(1 << SOFTPWM_CHANNEL3_PIN))
#endif

#if SOFTPWM_CHMAX > 4
	#define CH4_CLEAR 						(SOFTPWM_CHANNEL4_PIN_STATES &= ~(1 << SOFTPWM_CHANNEL4_PIN))
#endif

#if SOFTPWM_CHMAX > 5
	#define CH5_CLEAR 						(SOFTPWM_CHANNEL5_PIN_STATES &= ~(1 << SOFTPWM_CHANNEL5_PIN))
#endif

#if SOFTPWM_CHMAX > 6
	#define CH6_CLEAR 						(SOFTPWM_CHANNEL6_PIN_STATES &= ~(1 << SOFTPWM_CHANNEL6_PIN))
#endif

#if SOFTPWM_CHMAX > 7
	#define CH7_CLEAR 						(SOFTPWM_CHANNEL7_PIN_STATES &= ~(1 << SOFTPWM_CHANNEL7_PIN))
#endif

#if SOFTPWM_CHMAX > 8
	#define CH8_CLEAR 						(SOFTPWM_CHANNEL8_PIN_STATES &= ~(1 << SOFTPWM_CHANNEL8_PIN))
#endif

#if SOFTPWM_CHMAX > 9
	#define CH9_CLEAR 						(SOFTPWM_CHANNEL9_PIN_STATES &= ~(1 << SOFTPWM_CHANNEL9_PIN))
#endif


/*************************************************************
	GLOBAL VARIABLES
**************************************************************/
static unsigned char compare[SOFTPWM_CHMAX];
static volatile unsigned char compbuff[SOFTPWM_CHMAX];

#if USE_LOGARITHMIC_ARRAY == TRUE
static const uint8_t LogBrightness[] PROGMEM = {
	8,8,8,8,9,9,9,9,9,10,10,10,10,11,11,11,12,12,12,12,13,13,13,14,14,14,
	15,15,15,16,16,17,17,17,18,18,19,19,19,20,20,21,21,22,22,
	23,23,24,25,25,26,26,27,28,28,29,30,30,31,32,32,33,34,35,
	35,36,37,38,39,40,41,42,42,43,44,45,46,48,49,50,51,52,53,
	54,56,57,58,59,61,62,64,65,66,68,69,71,73,74,76,78,79,81,
	83,85,87,89,91,93,95,97,99,101,103,106,108,110,113,115,118,
	121,123,126,129,132,135,138,141,144,147,150,154,157,161,164,
	168,171,175,179,183,187,191,196,200,204,209,213,218,223,228,
	233,238,243,249,255
};
#endif


/*************************************************************
	FUNCTION PROTOTYPES
**************************************************************/
void softwarePWM_Init(void);
void softwarePWM_Set(unsigned char channel, unsigned char value);
void softwarePWM_Resume(void);
void softwarePWM_Pause(void);

#if USE_LOGARITHMIC_ARRAY == TRUE
uint8_t softwarePWM_LineartoLog(uint8_t dutyCycle);
#endif


/*************************************************************
	FUNCTIONS
**************************************************************/
void softwarePWM_Init(void){
	unsigned char i; 
	unsigned char pwm = SOFTPWM_PWMDEFAULT;

	// Set port pins to output
	#if SOFTPWM_ON_PORT_B == TRUE
		DDRB |= SOFTPWM_PINS_PORTB;
	#endif
	
	#if SOFTPWM_ON_PORT_C == TRUE
		DDRC |= SOFTPWM_PINS_PORTC;
	#endif
	
	#if SOFTPWM_ON_PORT_D == TRUE
		DDRD |= SOFTPWM_PINS_PORTD;
	#endif
	
	#if SOFTPWM_ON_PORT_E == TRUE
		DDRE |= SOFTPWM_PINS_PORTE;
	#endif

	// Initialise all channels
	for(i=0; i < SOFTPWM_CHMAX; i++){
		compare[i] = pwm;
		compbuff[i] = pwm;
	}

	#if SOFTPWM_TIMER == SOFTPWM_TIMER0
		TIFR0 = (1 << TOV0);           // clear interrupt flag
		TIMSK0 = (1 << TOIE0);         // enable overflow interrupt
		TCCR0B = (1 << CS00);         	// start timer, no prescale
		#define SOFTPWM_ISR_VECT			TIMER0_OVF_vect
		
	#elif SOFTPWM_TIMER == SOFTPWM_TIMER1
		TIFR1 = (1 << OCF1A);           // clear interrupt flag
		TIMSK1 = (1 << OCIE1A);         // enable Output Compare A Match interrupt
		TCCR1B = (1 << CS10 | 1 << WGM12);	// start timer, no prescale
		OCR1A = SOFTPWM_TIMER1_OCR;
		#define SOFTPWM_ISR_VECT			TIMER1_COMPA_vect
		
	#elif SOFTPWM_TIMER == SOFTPWM_TIMER2
		TIFR2 = (1 << TOV2);           // clear interrupt flag
		TIMSK2 = (1 << TOIE2);         // enable overflow interrupt
		TCCR2B = (1 << CS20);         	// start timer, no prescale
		#define SOFTPWM_ISR_VECT			TIMER2_OVF_vect
	#endif

	sei(); // enable global interrupts
}



void softwarePWM_Set(unsigned char channel, unsigned char value){
	if(channel < SOFTPWM_CHMAX)	compbuff[channel] = value;
}



void softwarePWM_Resume(void){
	#if SOFTPWM_TIMER == SOFTPWM_TIMER0
		power_timer0_enable();
		TIMSK0 = (1 << TOIE0);         // enable overflow interrupt
		TCCR0B = (1 << CS00);         	// start timer, no prescale
		
	#elif SOFTPWM_TIMER == SOFTPWM_TIMER1
		power_timer1_enable();
		TIFR1 = (1 << OCF1A);           // clear interrupt flag
		TIMSK1 = (1 << OCIE1A);         // enable Output Compare A Match interrupt
		TCCR1B = (1 << CS10 | 1 << WGM12);	// start timer, no prescale
		OCR1A = SOFTPWM_TIMER1_OCR;
		
	#elif SOFTPWM_TIMER == SOFTPWM_TIMER2
		power_timer2_enable();
		TIMSK2 = (1 << TOIE2);         // enable overflow interrupt
		TCCR2B = (1 << CS20);         	// start timer, no prescale
	#endif
}



void softwarePWM_Pause(void){
	#if SOFTPWM_TIMER == SOFTPWM_TIMER0
		TIMSK0 &= ~(1 << TOIE0);       // disable overflow interrupt
		TCCR0B = 0;         			// No clock source (Timer/Counter stopped)
		power_timer0_disable();
		
	#elif SOFTPWM_TIMER == SOFTPWM_TIMER1
		TIMSK1 = (1 << OCIE1A);         // disable Output Compare A Match interrupt
		TCCR1B = 0;         			// No clock source (Timer/Counter stopped)
		power_timer1_disable();
		
	#elif SOFTPWM_TIMER == SOFTPWM_TIMER2
		TIMSK2 &= ~(1 << TOIE2);       // disable overflow interrupt
		TCCR2B = 0;         			// No clock source (Timer/Counter stopped)
		power_timer2_disable();
	#endif
}


#if USE_LOGARITHMIC_ARRAY == TRUE
uint8_t softwarePWM_LineartoLog(uint8_t dutyCycle){

	if(dutyCycle < 32){
		return 0;
		
	}else if(dutyCycle < 51){
		return 1;
		
	}else if(dutyCycle < 64){
		return 2;
		
	}else if(dutyCycle < 75){
		return 3;
		
	}else if(dutyCycle < 83){
		return 4;
		
	}else if(dutyCycle < 90){
		return 5;
		
	}else if(dutyCycle < 96){
		return 6;
		
	}else if(dutyCycle < 102){
		return 7;
		
	}
	
	if(dutyCycle > 255) dutyCycle = 0;
	dutyCycle -= 102;
	
	return pgm_read_byte(&LogBrightness[dutyCycle]);
	
}
#endif


/*************************************************************
	ISR Handlers
**************************************************************/

ISR(SOFTPWM_ISR_VECT){ 
	static unsigned char softcount = 0xFF;

	#if SOFTPWM_ON_PORT_B == TRUE
		static unsigned char pinlevelB = SOFTPWM_PINS_PORTB;
		// Update outputs
		PORTB = (PORTB & (~(SOFTPWM_PINS_PORTB))) | pinlevelB;
	#endif
	
	#if SOFTPWM_ON_PORT_C == TRUE
		static unsigned char pinlevelC = SOFTPWM_PINS_PORTC;
		PORTC = (PORTC & (~(SOFTPWM_PINS_PORTC))) | pinlevelC;
	#endif
	
	#if SOFTPWM_ON_PORT_D == TRUE
		static unsigned char pinlevelD = SOFTPWM_PINS_PORTD;
		PORTD = (PORTD & (~(SOFTPWM_PINS_PORTD))) | pinlevelD;
	#endif
	
	#if SOFTPWM_ON_PORT_E == TRUE
		static unsigned char pinlevelE = SOFTPWM_PINS_PORTE;
		PORTE = (PORTE & (~(SOFTPWM_PINS_PORTE))) | pinlevelE;
	#endif

	if(++softcount == 0){
		#if SOFTPWM_CHMAX > 0
			compare[0] = compbuff[0];
		#endif

		#if SOFTPWM_CHMAX > 1
			compare[1] = compbuff[1];
		#endif

		#if SOFTPWM_CHMAX > 2
			compare[2] = compbuff[2];
		#endif

		#if SOFTPWM_CHMAX > 3
			compare[3] = compbuff[3];
		#endif

		#if SOFTPWM_CHMAX > 4
			compare[4] = compbuff[4];
		#endif

		#if SOFTPWM_CHMAX > 5
			compare[5] = compbuff[5];
		#endif

		#if SOFTPWM_CHMAX > 6
			compare[6] = compbuff[6];
		#endif

		#if SOFTPWM_CHMAX > 7
			compare[7] = compbuff[7];
		#endif

		#if SOFTPWM_CHMAX > 8
			compare[8] = compbuff[8];
		#endif

		#if SOFTPWM_CHMAX > 9
			compare[9] = compbuff[9];
		#endif


		// Set all port pins high
		#if SOFTPWM_ON_PORT_B == TRUE
			pinlevelB = SOFTPWM_PINS_PORTB;
		#endif
		
		#if SOFTPWM_ON_PORT_C == TRUE
			pinlevelC = SOFTPWM_PINS_PORTC;
		#endif
		
		#if SOFTPWM_ON_PORT_D == TRUE
			pinlevelD = SOFTPWM_PINS_PORTD;
		#endif
		
		#if SOFTPWM_ON_PORT_E == TRUE
			pinlevelE = SOFTPWM_PINS_PORTE;
		#endif
	}
	
	// clear port pin on compare match (executed on next interrupt)
	#if SOFTPWM_CHMAX > 0
		if(compare[0] == softcount) CH0_CLEAR;
	#endif

	#if SOFTPWM_CHMAX > 1
		if(compare[1] == softcount) CH1_CLEAR;
	#endif

	#if SOFTPWM_CHMAX > 2
		if(compare[2] == softcount) CH2_CLEAR;
	#endif

	#if SOFTPWM_CHMAX > 3
		if(compare[3] == softcount) CH3_CLEAR;
	#endif

	#if SOFTPWM_CHMAX > 4
		if(compare[4] == softcount) CH4_CLEAR;
	#endif

	#if SOFTPWM_CHMAX > 5
		if(compare[5] == softcount) CH5_CLEAR;
	#endif

	#if SOFTPWM_CHMAX > 6
		if(compare[6] == softcount) CH6_CLEAR;
	#endif

	#if SOFTPWM_CHMAX > 7
		if(compare[7] == softcount) CH7_CLEAR;
	#endif

	#if SOFTPWM_CHMAX > 8
		if(compare[8] == softcount) CH8_CLEAR;
	#endif

	#if SOFTPWM_CHMAX > 9
		if(compare[9] == softcount) CH9_CLEAR;
	#endif

}

#endif