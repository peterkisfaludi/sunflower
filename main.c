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
	//AD konverter bekapcsol�sa
	//ADEN bebillent�se ADCSRA-ban
	sbi(ADCSRA,ADEN);
	//az eredm�ny balra igaz�t�sa
	sbi(ADMUX,ADLAR);
	//Interrupt enged�lyez�se
	sbi(ADCSRA,ADIE);
}

void a2dSetChannel(u08 ch) {
	ADMUX |= ch;
}
//AD konverzi� ind�t�s
void a2dStart(void) {
	//Start bit be�ll�t�sa
	//ADSC bebillent�se ADCSRA-ban
	sbi(ADCSRA,ADSC);
}
//AD konverter kikapcsol�sa
void a2dOff(void) {
	cbi(ADCSRA,ADEN);
	cbi(ADCSRA,ADIE);
}

ISR(ADC_vect) {
	//tov�bbi interruptok ideiglenes tilt�sa
	cli();
	//glob�lis v�ltoz� be�ll�t�sa
	adresult = ADCH;
	event = EVT_ADCOMPLETE;
	//AD konverzi� v�g�t jelz� flag be�ll�t�sa
	//a2dComplete = 1;
	//tov�bbi interruptok enged�lyez�se
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
//priv�t f�ggv�nyek
//�ramutat� j�r�s�val megegyez� ir�ny� forgat�s
int rot_cw(void) {
	//null�zzuk ki a motorvez�rl� l�bakat
	PORTB &= 0b01111000;
	//v�rjunk am�g kinull�z�dik
	_delay_ms(COOLDOWNTIME);
	switch (motorstate) {
		case ST1: motorstate=ST2; break;
		case ST2: motorstate=ST3; break;
		case ST3: motorstate=ST1; break;
	}
	//adjuk ki a jelet
	PORTB = motorstate;
	//v�rjunk egy kicsit
	_delay_ms(SETTINGTIME);
	//null�zzuk ki megint
	PORTB &= 0b01111000;
	return 0;
}

//�ramutat� j�r�s�val megegyez� ir�ny� forgat�s
int rot_ccw(void) {
	//null�zzuk ki a motorvez�rl� l�bakat
	PORTB &= 0b01111000;
	//v�rjunk am�g kinull�z�dik
	_delay_ms(COOLDOWNTIME);
	switch (motorstate) {
		case ST1: motorstate=ST3; break;
		case ST2: motorstate=ST1; break;
		case ST3: motorstate=ST2; break;
	}
	//adjuk ki a jelet
	PORTB = motorstate;
	//v�rjunk egy kicsit
	_delay_ms(SETTINGTIME);
	//null�zzuk ki megint
	PORTB &= 0b01111000;
	return 0;
}
//priv�t f�ggv�ny v�ge--------------------------------


//egy l�p�ssel valamilyen ir�nyban forgatunk
//ha sz�ls� helyzetben nem megfele� ir�nyban akarunk forgatni,
//nem t�rt�nik semmi, a sikeress�get visszajelezz�k
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
	//a regisztr�tum felt�lt�se
	reg_buf[angle/ANGSTP] = adresult;
	senda2dresult();
	return succ;
}

//forgat�s adott ir�nyban adott l�p�sk�zzel
//visszaadja, hogy h�ny l�p�st tudott sikeresen v�grehajtani
//ha sz�ls� helyzetbe �r, visszat�r
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

//forgat�s sz�ls� helyzetig adott ir�nyba
//visszaadja, hogy h�ny l�p�st tett meg sikeresen
int rot_lim(dir) {
	return rot(dir,maxstp);
}

//adott poz�ci�ba forgat�s
//param�terk�nt egy sz�ghelyzetet kap
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
	//1.szakasz - sz�ls� helyzetbe forgat�s
	if(angle<180) dir =CW;
	rot_lim(dir);
	//2.szakasz - t�ls� helyzetbe forgat�s
	dir = (dir==CW)?CCW:CW;
	rot_lim(dir);
	//3.szakasz - max poz�ci� kiv�laszt�sa
	int maxpos = 0;
	int maxval = 0;
	int i=0;
	for (i=0; i < maxstp; i++) {
		if(reg_buf[i] > maxval) {
			maxpos=i;
			maxval=reg_buf[i];
		}
	}
	//4.szakasz - maximum helyzetbe forgat�s
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


