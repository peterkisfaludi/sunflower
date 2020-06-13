#include <avr/io.h>
#include <avr/interrupt.h>


#define F_CPU 7372800
#include <util/delay.h>
#include <inttypes.h>
#include "conf/global.h"

#define TIMER_PRESCALE_MASK	0x07
#define TIMER_CLK_DIV8 0x02
#define TIMER_CLK_DIV1024 0x05
#define TIMER0PRESCALE TIMER_CLK_DIV8

//Push button
//buttonmask
#define BM 0b01111000
//seek max button
#define SM 0b00111000
//turn left button
#define TL 0b01101000
//turn right button
#define TR 0b01011000
//place zero button
#define PZ 0b01110000

//Stepper motor
#define ANGSTP 20
#define CW 0
#define CCW 1
#define ST1 0b00000010
#define ST2 0b00000001
#define ST3 0b00000100
#define STATEMASK 0b00000111
#define COOLDOWNTIME 500
#define SETTINGTIME 100


//GLOBALS---------------------------------------------------------
volatile u08 adresult=0;
volatile u08 buttonstate=BM;
volatile unsigned char rxbuf=0;
volatile int angle=0;
const int maxstp = 360/ANGSTP;
volatile u08 reg_buf[360/ANGSTP];
volatile u08 motorstate = ST1;

typedef enum {
	EVT_NONE,
	EVT_ADCOMPLETE,
	EVT_SEEKMAX,
	EVT_TURNLEFT,
	EVT_TURNRIGHT,
	EVT_PLACEZERO,
} event_t;

static volatile event_t event = EVT_NONE;

//USART---------------------------------------------------------
void usart_init(void) {
	UCSRA = 0x00;
	UCSRB = (1 << RXEN) | (1 << TXEN) | (1 << RXCIE);
	UCSRC = 0x86;
	UBRRH=0x00;
	UBRRL=0x2F;
}

void usart_putc(unsigned char c) {
   // wait until UDR ready
	while(!(UCSRA & (1 << UDRE)));
	UDR = c;    // send character
}

void uart_puts (char *s) {
	//  loop until *s != NULL
	while (*s) {
		usart_putc(*s);
		s++;
	}
}

void uart_putint(int num) {
	char myBuf[20];
	sprintf(myBuf, "%i", num);
	uart_puts(myBuf);
}

void uart_nl() {
	usart_putc(10);
	usart_putc(13);
}

SIGNAL (SIG_UART_RECV) { // USART RX interrupt
	rxbuf = UDR;
}

void pollUsart(void) {
	switch(rxbuf) {
		case 'm': event = EVT_SEEKMAX; break;
		case 'l': event = EVT_TURNLEFT; break;
		case 'r': event = EVT_TURNRIGHT; break;
		case 'z': event = EVT_PLACEZERO; break;
		break;
	}
}

//A2D---------------------------------------------------------
void a2dInit(void) {
	//AD konverter bekapcsolása
	//ADEN bebillentése ADCSRA-ban
	sbi(ADCSRA,ADEN);
	//az eredmény balra igazítása
	sbi(ADMUX,ADLAR);
	//Interrupt engedélyezése
	sbi(ADCSRA,ADIE);
}

void a2dSetChannel(u08 ch) {
	ADMUX |= ch;
}
//AD konverzió indítás
void a2dStart(void) {
	//Start bit beállítása
	//ADSC bebillentése ADCSRA-ban
	sbi(ADCSRA,ADSC);
}
//AD konverter kikapcsolása
void a2dOff(void) {
	cbi(ADCSRA,ADEN);
	cbi(ADCSRA,ADIE);
}

ISR(ADC_vect) {
	//további interruptok ideiglenes tiltása
	cli();
	//globális változó beállítása
	adresult = ADCH;
	event = EVT_ADCOMPLETE;
	//AD konverzió végét jelzõ flag beállítása
	//a2dComplete = 1;
	//további interruptok engedélyezése
	sei();
}

//TIMER0------------------------------------------------------
void timer0Init(void) {
	timer0SetPrescaler(TIMER0PRESCALE);
	TCNT0=0;
	sbi(TIMSK, TOIE0);
}
void timer0SetPrescaler(u08 prescale) {
	outb(TCCR0, (inb(TCCR0) & ~TIMER_PRESCALE_MASK) | prescale);	
}
	
ISR(TIMER0_OVF_vect) {
	cli();
	u08 gomb = PINB;
	buttonstate = (gomb & BM);
	sei();
}

void pollButtons(void) {
	if (buttonstate == SM) {		
		event = EVT_SEEKMAX;
	}
	if (buttonstate == PZ) {
		event = EVT_PLACEZERO;
	}
	if (buttonstate == TL) {		
		event = EVT_TURNLEFT;
	}
	if (buttonstate == TR) {
		event = EVT_TURNRIGHT;
	}
}

//STEPPER MOTOR--------------------------------------------------------
//privát függvények
//óramutató járásával megegyezõ irányú forgatás
int rot_cw(void) {
	//nullázzuk ki a motorvezérlõ lábakat
	PORTB &= 0b01111000;
	//várjunk amég kinullázódik
	_delay_ms(COOLDOWNTIME);
	switch (motorstate) {
		case ST1: motorstate=ST2; break;
		case ST2: motorstate=ST3; break;
		case ST3: motorstate=ST1; break;
	}
	//adjuk ki a jelet
	PORTB = motorstate;
	//várjunk egy kicsit
	_delay_ms(SETTINGTIME);
	//nullázzuk ki megint
	PORTB &= 0b01111000;
	return 0;
}

//óramutató járásával megegyezõ irányú forgatás
int rot_ccw(void) {
	//nullázzuk ki a motorvezérlõ lábakat
	PORTB &= 0b01111000;
	//várjunk amég kinullázódik
	_delay_ms(COOLDOWNTIME);
	switch (motorstate) {
		case ST1: motorstate=ST3; break;
		case ST2: motorstate=ST1; break;
		case ST3: motorstate=ST2; break;
	}
	//adjuk ki a jelet
	PORTB = motorstate;
	//várjunk egy kicsit
	_delay_ms(SETTINGTIME);
	//nullázzuk ki megint
	PORTB &= 0b01111000;
	return 0;
}
//privát függvény vége--------------------------------


//egy lépéssel valamilyen irányban forgatunk
//ha szélsõ helyzetben nem megfeleõ irányban akarunk forgatni,
//nem történik semmi, a sikerességet visszajelezzük
int rot_1stp(int dir) {
	a2dStart();
	int succ = FALSE;
	switch(dir) {
		case CCW:
			if(angle < 360) {
				angle += ANGSTP;
				succ = TRUE;
				rot_ccw();				
			}
			break;
		case CW:
			if(angle > 0) {
				angle -= ANGSTP;
				succ = TRUE;
				rot_cw();				
			}
			break;
	}
	//a regisztrátum feltöltése
	reg_buf[angle/ANGSTP] = adresult;
	senda2dresult();
	return succ;
}

//forgatás adott irányban adott lépésközzel
//visszaadja, hogy hány lépést tudott sikeresen végrehajtani
//ha szélsõ helyzetbe ér, visszatér
int rot(int dir, int steps) {
	int succ = FALSE;
	int i;
	for(i=0;i<steps;i++) {
		succ=rot_1stp(dir);
		if(succ == FALSE) {
			return i;
		}
	}
	return i;
}

//forgatás szélsõ helyzetig adott irányba
//visszaadja, hogy hány lépést tett meg sikeresen
int rot_lim(dir) {
	return rot(dir,maxstp);
}

//adott pozícióba forgatás
//paraméterként egy szöghelyzetet kap
int rot_pos(int pos) {
	int dir = (pos>=angle)?CCW:CW;
	return rot(dir, abs(angle-pos)/ANGSTP);
}


//USER DEFINED--------------------------------------------------------
void handleEvents(void) {
	if(event != EVT_NONE) {
		switch(event) {
			case EVT_ADCOMPLETE:
				senda2dresult();
				break;
			case EVT_SEEKMAX:
				sei();
				seekmax();
				break;
			case EVT_TURNLEFT:
				rot_1stp(CCW);
				break;
			case EVT_TURNRIGHT:
				rot_1stp(CW);
				break;
			case EVT_PLACEZERO:
				angle = 0;
				break;
			break;
		}
	}
}

void senda2dresult(void) {
	uart_puts("POSITION: ");				
	uart_putint(angle);
	uart_nl();
	uart_puts("INTENSITY: ");			
	uart_putint(adresult);
	uart_nl();

}
				
void seekmax(void) {
	int dir =CCW;
	//1.szakasz - szélsõ helyzetbe forgatás
	if(angle<180) dir =CW;
	rot_lim(dir);
	//2.szakasz - túlsó helyzetbe forgatás
	dir = (dir==CW)?CCW:CW;
	rot_lim(dir);
	//3.szakasz - max pozíció kiválasztása
	int maxpos = 0;
	int maxval = 0;
	int i=0;
	for (i=0; i < maxstp; i++) {
		if(reg_buf[i] > maxval) {
			maxpos=i;
			maxval=reg_buf[i];
		}
	}
	//4.szakasz - maximum helyzetbe forgatás
	rot_pos(maxpos*ANGSTP);
}
		
//MAIN--------------------------------------------------------
int main (void){
	//PORT init
	DDRA = 0x00;
	PORTA = 0x00;
	DDRB=0b00000111;
	PORTB &= 0b01111000;

	//init
	usart_init();	
	sei();
	//induljon be az usart
	while(!(UCSRA & (1 << UDRE)));
	UDR = 0x43; // "C"
	while(!(UCSRA & (1 << UDRE)));
	UDR = 0x0d;
	
	a2dInit();
	a2dSetChannel(0x00);

	timer0Init();
	

	//MAIN LOOP---------------------------------------------------
	while (1) {
		event = EVT_NONE;
		rxbuf=0;
		a2dStart();
		_delay_ms(500);
		pollUsart();
		pollButtons();
		handleEvents();			
	}
	return 0;
}


