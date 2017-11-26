/*
* Multimeter_328p.c
*
* Created: 07-Feb-17 8:50:33 AM
* Author : Mostafa
*/

#define F_CPU 16000000UL

#define OVERFLOW 1UL<<16
#define K 1000UL
#define M K*K
#define ICR1_new ICR1

#define Rint 100000000UL
#define Rout_int 20
#define Vcc 5
#define Rs1 1E3
#define Rs2 10E3
#define Rs3 100E3

#define ADC_RESOLUTION (1U<<10)

/*DEVICE DEFINITION*/
/*
#define VOLTMETER 0
#define OHMMETER 1
#define FREQUENCY_COUNTER 2*/
typedef enum DEVICES{VOLTMETER,OHMMETER,FREQUENCY_COUNTER}DEVICES;

#define CONFIDENCE 2000
#define LCD_REFRESH_TIME_MS 300


#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <util/delay.h>
#include "hd44780.h"
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

/*ADC FUNCTIONS*/
void adc_init();
uint16_t get_adc_conversion(uint8_t adc_pin);
float get_voltage(uint8_t ADC_pin,uint16_t resolution,uint16_t maximum_value);

/*FREQUENCY COUNTER FUNCTIONS*/
void analog_comparator_init();
void timer1_init();
void frequency_counter_init();
void analog_comparator_disable();
void analog_comparator_enable();

/*VOLTMETER FUNCTIONS*/
void voltmeter_millivolt_range();
void voltmeter_volt_range();

/*OHMMETER FUNCTIONS*/
void ohmmeter_init();
void ohmmeter_enable();
void ohmmeter_disable();

/*SETUP*/
void timer0_init();
void setup();

/*FREQUENCY COUNTER VARIABLES*/
volatile uint32_t overflow_offset=0,period=0;

/*DEVICE SETTINGS*/
volatile DEVICES DEVICE=VOLTMETER;

int main(void)
{

	/*LOCAL VARIABLES*/
	uint8_t pushedFlag=0,releaseFlag=1;
	uint16_t releaseConfidence=0,pushConfidence=0;

	setup();

	while (1)
	{
		if (bit_is_clear(PINC,PINC5))
		{
			pushConfidence++;
			releaseConfidence = 0;
			if (pushConfidence > CONFIDENCE)
			{
				pushConfidence = 0;
				pushedFlag = 1;
			}
		}

		if (bit_is_set(PINC,PINC5))
		{
			releaseConfidence++;
			pushConfidence = 0;
			if (releaseConfidence > CONFIDENCE)
			{
				releaseConfidence = 0;
				pushedFlag = 0;
				releaseFlag = 1;
			}
		}

		//if(!pushedFlag&&!(BUTTONPINR&(1<<BUTTONPIN)))

		if (pushedFlag&&releaseFlag)
		{
			releaseFlag=0;
			releaseConfidence = 0;
			pushConfidence = 0;
			releaseConfidence=0;
			DEVICE=(DEVICE+1)%3;
		}

	}
}


ISR(TIMER1_CAPT_vect)
{

	static uint16_t ICR1_old=0;

	period=(overflow_offset+ICR1_new)-ICR1_old;


	ICR1_old=ICR1_new;

	/*period=(overflow_offset+ICR1_new);

	ICR1_new=TCNT1=0;*/

	/*overflow_offset=0;*/

	overflow_offset=0;

}

ISR(TIMER1_OVF_vect)
{
	overflow_offset+=OVERFLOW;
}

/*LCD REFRESH*/
ISR(TIMER0_COMPA_vect)
{
	char buffer[20];
	static uint16_t time_ms=100;
	time_ms++;

	if(time_ms>=LCD_REFRESH_TIME_MS)
	{
		time_ms=0;
		lcd_clrscr();
		switch (DEVICE)
		{
			case VOLTMETER:
			{
				analog_comparator_disable();
				ohmmeter_disable();
				voltmeter_volt_range();
				uint8_t VOLTMETER_FL=0;
				float volt=get_voltage(0,ADC_RESOLUTION,Vcc);
				if (volt<=1.1)
				{
					voltmeter_millivolt_range();
					volt=get_voltage(0,ADC_RESOLUTION,1100);
					VOLTMETER_FL=1;
				}
				else
				{
					VOLTMETER_FL=0;
				}
				dtostrf(volt,0,2,buffer);
				lcd_puts(buffer);
				if (VOLTMETER_FL)
				{
					lcd_puts(" mV");
					lcd_goto(0x40);
					lcd_puts("Vref");
				}
				else
				{
					lcd_puts(" V");
					lcd_goto(0x40);
					lcd_puts("AVCC");
				}

				break;
			}
			case OHMMETER:
			{
				analog_comparator_disable();
				voltmeter_volt_range();
				const float Rs[3]={Rs1,Rs2,Rs3};
				float Rx=-1;
				float Vrx=0;
				uint8_t range=0;
				do
				{
					switch (range)
					{
						/*RS1*/
						case 0:
						DDRB&=~(_BV(PINB2)|_BV(PINB3));
						DDRB|=_BV(PINB1);
						PORTB&=~(_BV(PINB2)|_BV(PINB3));
						PORTB|=_BV(PINB1);
						break;
						/*RS2*/
						case 1:
						DDRB&=~(_BV(PINB1)|_BV(PINB3));
						DDRB|=_BV(PINB2);
						PORTB&=~(_BV(PINB1)|_BV(PINB3));
						PORTB|=_BV(PINB2);
						break;
						/*RS3*/
						case 2:
						DDRB&=~(_BV(PINB1)|_BV(PINB2));
						DDRB|=_BV(PINB3);
						PORTB&=~(_BV(PINB1)|_BV(PINB2));
						PORTB|=_BV(PINB3);
						break;
						default:
						break;
					}
					_delay_us(5);
					Vrx=get_voltage(0,ADC_RESOLUTION,5);
					if (Vrx>(0.9*Vcc))
					{
						range++;
					}
					else if (Vrx<(0.08*Vcc))
					{
						range--;
					}
					else
					{
						Rx=(Vrx*(Rs[range]+Rout_int))/(Vcc-Vrx);
						break;
					}
				}while(range<3);

				if (Rx<0)
				{
					strcpy(buffer,"OUT OF RANGE");
				}
				else if (Rx<K)
				{
					dtostrf(Rx,0,2,buffer);
					strcat(buffer," ohm");
				}
				else if (Rx<M)
				{
					dtostrf(Rx/K,0,2,buffer);
					strcat(buffer," Kohm");
				}
				else if (Rx>=M)
				{
					dtostrf(Rx/M,0,2,buffer);
					strcat(buffer," Mohm");
				}

				lcd_puts(buffer);

				break;
			}

			case FREQUENCY_COUNTER:
			{
				analog_comparator_enable();
				lcd_puts("Frequency");
				lcd_goto(0x40);
				float freq=F_CPU/(float)period;

				if(freq<K)
				{
					dtostrf(freq,0,2,buffer);
					strcat(buffer," HZ");
				}
				else if (freq<M)
				{
					dtostrf(freq/K,0,2,buffer);
					strcat(buffer," KHZ");
				}
				else
				{
					dtostrf(freq/M,0,2,buffer);
					strcat(buffer," MHZ");
				}

				lcd_puts(buffer);
				break;
			}
			default:
			break;
		}

	}
}

ISR(ADC_vect)
{
}

void analog_comparator_init()
{
	ACSR=(0<<ACIE)|_BV(ACIS0)|_BV(ACIS1)|_BV(ACIC);
}

void timer1_init()
{
	TIMSK1=_BV(ICIE1)|_BV(TOIE1);
	TCCR1B=_BV(CS10)|(0<<CS11)|(0<<CS12)|(0<<WGM12);
}

void timer0_init()
{
	TCCR0A=_BV(WGM01);
	TCCR0B=_BV(CS00)|_BV(CS01);
	OCR0A=250;
	TIMSK0=_BV(OCIE0A);
}

void frequency_counter_init()
{

	/*INITIATE ANALOG COMPARATOR*/
	analog_comparator_init();
	/*INITIATE TIMER1*/
	timer1_init();
}

void voltmeter_millivolt_range()
{
	ADMUX=_BV(REFS0)|_BV(REFS1);
}

void voltmeter_volt_range()
{
	ADMUX=_BV(REFS0);
}

void ohmmeter_disable()
{
	DDRB&=~(_BV(PINB2)|_BV(PINB3)|_BV(PINB1));
	PORTB&=~(_BV(PINB2)|_BV(PINB3)|_BV(PINB1));
	_delay_us(5);
}

void adc_init()
{
	/*ENABLE ADC*/
	ADCSRA=_BV(ADEN)|_BV(ADIE);
	/*ENABLE ADC NOISE REDUCTION*/
	//SMCR=SLEEP_MODE_ADC;
	set_sleep_mode(SLEEP_MODE_ADC);
}

uint16_t get_adc_conversion(uint8_t adc_pin)
{
	adc_pin&=0x07;
	ADMUX=(ADMUX&0xF8)|adc_pin;
	/*YOU HAVE TO RE-ENABLE GLOBAL INTERRUPT BECAUSE YOU'RE IN ISR */
	sei();
	sleep_mode();
	/*cli();*/
	/*ADCSRA|=_BV(ADSC);
	while(ADCSRA&_BV(ADSC));
	loop_until_bit_is_clear(ADCSRA,ADSC);*/
	return ADC;
}

float get_voltage(uint8_t ADC_pin,uint16_t resolution,uint16_t maximum_value)
{
	return ((float)get_adc_conversion(ADC_pin)/resolution)*maximum_value;
}

void setup()
{
	cli();

	/*ENABLE GLOBAL INTERRUPT*/
	sei();

	frequency_counter_init();

	/*INITIATE TIMER0*/
	timer0_init();

	/*INITIATE ADC*/
	adc_init();

	/*INITIATE LCD DISPLAY*/
	lcd_init();

	/*PULLUP BUTTON PIN*/
	PORTC|=_BV(PINC5);


}

void analog_comparator_disable()
{
	/*YOU HAVE TO DISABLE ANALOG COMPARATOR IN NOISE REDUCTION MODE*/
	ACSR|=_BV(ACD);
}

void analog_comparator_enable()
{
	ACSR&=~_BV(ACD);
}

