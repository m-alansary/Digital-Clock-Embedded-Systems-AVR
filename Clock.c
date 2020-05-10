#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#define secU PD1
#define secT PD0
#define minU PC7
#define minT PC6
#define hourU PC5
#define hourT PC4
#define delay 5

unsigned char counter = 0, seconds = 0, minutes = 0, hours = 0;

void Timer0_CMP_init(void){
	TCNT0 = 0;								// Timer initial value
	TIMSK |= (1 << OCIE0);					// Set the compare timer interrupt
	TCCR0 = (1 << FOC0)
			| (1 << WGM01)
			| (1 << CS00) | (1 << CS02);
	/* Non PWM Mode
	 * Compare Mode
	 * Normal Mode
	 * Prescaler is equal to 1024
	 */
	OCR0 = 250;								// CMP value = 250
}

ISR(TIMER0_COMP_vect){
	counter++;
}

ISR(TIMER1_COMPA_vect)
{
	counter = 4;
}

void timer1_init_CTC_mode(unsigned short tick)
{
	TCNT1 = 0; //timer initial value
	OCR1A  = tick; //compare value
	TIMSK |= (1<<OCIE1A); //enable compare interrupt for channel A
	/* Configure timer1 control registers
	 * 1. Non PWM mode FOC1A=1 and FOC1B=1
	 * 2. No need for OC1A & OC1B in this example so COM1A0=0 & COM1A1=0 & COM1B0=0 & COM1B1=0
	 * 3. CTC Mode and compare value in OCR1A WGM10=0 & WGM11=0 & WGM12=1 & WGM13=0
	 */
	TCCR1A = (1<<FOC1A) | (1<<FOC1B);
	/*
	 * 4. Clock = F_CPU/1024 CS10=1 CS11=0 CS12=1
	 */
	TCCR1B = (1<<WGM12) | (1<<CS10) | (1<<CS12);
}

int main(void){
	DDRC = 0xff;
	PORTC = 0;
	DDRD |= 0x03;
	PORTD &= 0xfC;

	SREG  |= (1<<7);

	timer1_init_CTC_mode(1000);

	while(1){
		if(counter == 4){
			seconds++;
			if(seconds == 60){
				seconds = 0;
				minutes++;
				if(minutes == 60){
					minutes = 0;
					hours++;
					if(hours == 12){
						hours = 0;
					}
				}
			}
			counter = 0;
		}
		else {
			PORTD |= 1 << secU;
			PORTC = (PORTC & 0xf0) | (seconds % 10);
			_delay_ms(delay);
			PORTD &= ~(1 << secU);

			PORTD |= 1 << secT;
			PORTC = (PORTC & 0xf0) | (seconds / 10);
			_delay_ms(delay);
			PORTD &= ~(1 << secT);

			PORTC |= 1 << minU;
			PORTC = (PORTC & 0xf0) | (minutes % 10);
			_delay_ms(delay);
			PORTC &= ~(1 << minU);

			PORTC |= 1 << minT;
			PORTC = (PORTC & 0xf0) | (minutes / 10);
			_delay_ms(delay);
			PORTC &= ~(1 << minT);

			PORTC |= 1 << hourU;
			PORTC = (PORTC & 0xf0) | (hours % 10);
			_delay_ms(delay);
			PORTC &= ~(1 << hourU);

			PORTC |= 1 << hourT;
			PORTC = (PORTC & 0xf0) | (hours / 10);
			_delay_ms(delay);
			PORTC &= ~(1 << hourT);
		}
	}
}






