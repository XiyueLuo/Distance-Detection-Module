/*
 * GccApplication1.c
 *
 * Created: 2021/10/19 18:40:49
 * Author : Xiyue Luo
 */ 

#define F_CPU 16000000UL
#define BAUD_RATE 9600
#define BAUD_PRESCALER (((F_CPU / (BAUD_RATE * 16UL))) - 1)

#include <stdlib.h>
#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "uart.h"

long edge1=0;
long edge2 = 0;
int ovf_echo=0;
int ovf_tri=0;
unsigned long ticks=0;
long dis_cm=0;
char String[25];
int high = 0;

void Initialize(){
	cli(); /*disable global interrupts*/
	
	DDRB &= ~(1<<DDB0); //set PB0 input pin-echo
	DDRB &= ~(1<<DDB2); //set PB2 input pin-button
	DDRB |= (1<<DDB1); //set PB1 output pin to sensor
	DDRD |= (1<<DDD5); //set PD5 output pin to buzzer
	
	
	//Timer1 Initialize
	TCCR1B &= ~(1<<CS10);
	TCCR1B |= (1<<CS11);
	TCCR1B &= ~(1<<CS12);/*Timer1 is 8-times prescale->2MHz*/
	
	TCCR1A &= ~(1<<WGM10);
	TCCR1A &= ~(1<<WGM11);
	TCCR1B &= ~(1<<WGM12);/*set timer1 to normal*/
	
	TIMSK1 |= (1<<ICIE1); //enable the input capture*/
	TCCR1B |= (1<<ICES1);/*looking for rising edge*/
	TIFR1 |= (1<<ICF1); /*clear input capture flag*/
	
	TIMSK1|=(1<<OCIE1A);//set the output compare interrupt request
	TIFR1 |= (1<<OCF1A);//clear output compare interrupt flag
	
	TIMSK1 |= (1 << TOIE1); // enable timer overflow
	
	OCR1A=100;/*initialize OCR1A*/
	
	//Timer0 Initialize
	//TCNT0 8 bits
	TCCR0B |= (1<<CS00);
	TCCR0B |= (1<<CS01);
	TCCR0B &= ~(1<<CS02); /*prescale 64 times,250000Hz*/
	
	TCCR0A |= (1<<WGM00);
	TCCR0A &= ~(1<<WGM01);
	TCCR0B |= (1<<WGM02); /*set Timer0 Phase Correct*/
	
	TCCR0A |= (1<<COM0B0);
	TCCR0A |= (1<<COM0B1);/*non-inverting mode*/
	OCR0A = 71;/*(1/440)/(1/250000))/2*/
	OCR0B = OCR0A*1/4;
	
	TIMSK0 |= (1<<OCIE0A); /*Enable output compare interrupt*/
	TIFR0 |= (1<<OCF0A);/*clear interrupt flag*/
	
	//ADC initialize
	PRR &= ~(1<<PRADC); //clear power reduction for ADC
	
	ADMUX |= (1<<REFS0);
	ADMUX &= ~(1<<REFS1); //select Vref=AVcc
	
	ADCSRA |= (1<<ADPS0);
	ADCSRA |= (1<<ADPS1);
	ADCSRA |= (1<<ADPS2); //set ADC clock divided by 128 times->125kHz
	
	ADMUX &= ~(1<<MUX0);
	ADMUX &= ~(1<<MUX1);
	ADMUX &= ~(1<<MUX2);
	ADMUX &= ~(1<<MUX3); //select channel 0
	
	ADCSRA |= (1<<ADATE);//set to auto trigger
	
	ADCSRB &= ~(1<<ADTS0);
	ADCSRB &= ~(1<<ADTS1);
	ADCSRB &= ~(1<<ADTS2); //set to free running
	
	DIDR0 |= (1<<ADC0D); //disable digital input buffer on ADC pin
	ADCSRA |= (1<<ADEN); //enable ADC
	ADCSRA |= (1<<ADIE); //enable ADC interrupt
	ADCSRA |= (1<<ADSC); //start conversion
	
	sei(); //enable global interrupts
	}

	ISR(TIMER0_COMPA_vect)
	{
	
	}

	ISR(TIMER1_OVF_vect){
	ovf_echo++;
	ovf_tri++;
	}

	//Output of Arduino is a 10us TTL to trigger sensor
	ISR(TIMER1_COMPA_vect)
	{
	if (!(PINB & (1<<PINB0)) )/*when PB0/Echo is low, need to send a trigger*/
	{
	PORTB|=(1<<PORTB1);/*set PB1 high*/
	_delay_us(10);
	PORTB ^= (1<<PORTB1);
	}
	}

	//Input of Arduino is looking for the rising and falling edge of Echo
	ISR(TIMER1_CAPT_vect) {
	edge2 = ICR1;/*store time*/
	ticks=(edge2+ovf_echo*65535)-edge1;/*calculate ticks*/
	edge1=edge2;
	if (high == 1)
	{
	dis_cm= ticks*0.0085;/*(period*340*100/2000000)/2;*/
	sprintf(String,"Distance is %ld cm\n", dis_cm);
	UART_putstring(String);
	if (PINB&(1<<PINB2)){/*pressing the button, continuous mode*/
	OCR0A=0.82*dis_cm+117.36;
	}
	else{/*releasing the button, discrete mode*/
	if(dis_cm>=70){
	OCR0A=239;
	}
	if((dis_cm<70)&&(dis_cm>=60)){
	OCR0A=213;
	}
	if((dis_cm<60)&&(dis_cm>=50)){
	OCR0A=190;
	}
	if((dis_cm<50)&&(dis_cm>=40)){
	OCR0A=179;
	}
	if((dis_cm<40)&&(dis_cm>=30)){
	OCR0A=159;
	}
	if((dis_cm<30)&&(dis_cm>=20)){
	OCR0A=142;
	}
	if((dis_cm<20)&&(dis_cm>=10)){
	OCR0A=126;
	}
	if((dis_cm<10)&&(dis_cm>=2)){
	OCR0A=119;
	}
	}
	}
	ovf_echo=0;/*clear the overflow*/
	TCCR1B ^= (1<<ICES1);/*toggle state*/
	high = (1 - high);
	}
	
	ISR(ADC_vect){
		if(ADC>=905){
			OCR0B=OCR0A*0.50;
		}
		if( (ADC<905) && (ADC>=810) ){
			OCR0B=OCR0A*0.55;
		}
		if( (ADC<810) && (ADC>=715) ){
			OCR0B=OCR0A*0.60;
		}
		if( (ADC<715) && (ADC>=620) ){
			OCR0B=OCR0A*0.65;
		}
		if( (ADC<620) && (ADC>=525) ){
			OCR0B=OCR0A*0.70;
		}
		if( (ADC<525) && (ADC>=430) ){
			OCR0B=OCR0A*0.75;
		}
		if( (ADC<430) && (ADC>=335) ){
			OCR0B=OCR0A*0.80;
		}
		if( (ADC<335) && (ADC>=240) ){
			OCR0B=OCR0A*0.85;
		}
		if( (ADC<240) && (ADC>=145) ){
			OCR0B=OCR0A*0.90;
		}
		if( (ADC<145) && (ADC>=50) ){
			OCR0B=OCR0A*0.95;
		}
	}

	int main(void)
	{
	UART_init(BAUD_PRESCALER);
	Initialize();
	while (1)
	{

	}
	}
