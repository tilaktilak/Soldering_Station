#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/io.h>
#include <stdio.h>
#include <stdint.h>
#include <math.h>

#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#ifndef BAUD
#define BAUD 9600
#endif
#include <util/setbaud.h>
#include <util/delay.h>
#include "lcd.h"

/*
   The circuit:
 * LCD RS pin to digital pin 12
 * LCD Enable pin to digital pin 11
 * LCD D4 pin to digital pin 6
 * LCD D5 pin to digital pin 7
 * LCD D6 pin to digital pin 4
 * LCD D7 pin to digital pin 5
 * LCD R/W pin to ground
 * LCD VSS pin to ground
 * LCD VCC pin to 5V
 * 10K resistor:
 * ends to +5V and ground
 * wiper to LCD VO pin (pin 3)
 *
 * Encoder : 6,7
 */

#define temp A0
#define CLK 3
#define DT 2
#define SW 6

void adc_init(void){

    ADCSRA |= ((1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0));    //16Mhz/128 = 125Khz the ADC reference clock
    ADMUX |= (1<<REFS0); //Voltage reference from Avcc (5v)
    ADCSRA |= (1<<ADEN); //Turn on ADC
    ADCSRA |= (1<<ADSC); //Do an initial conversion because this one is the slowest and to ensure that everything is up and running
}

uint16_t adc_read(void){
    ADMUX &= 0xF0;                    //Clear the older channel that was read
    ADMUX |= 0;                //Defines the new ADC channel to be read
    ADCSRA |= (1<<ADSC);                //Starts a new conversion
    while(ADCSRA & (1<<ADSC));            //Wait until the conversion is done
    return ADCW;                    //Returns the ADC value of the chosen channel
}

void uart_init(void) {
    UBRR0H = UBRRH_VALUE;
    UBRR0L = UBRRL_VALUE;

#if USE_2X
    UCSR0A |= _BV(U2X0);
#else
    UCSR0A &= ~(_BV(U2X0));
#endif

    UCSR0C = _BV(UCSZ01) | _BV(UCSZ00); /* 8-bit data */ 
    UCSR0B = _BV(RXEN0) | _BV(TXEN0);   /* Enable RX and TX */    
}

int uart_putchar(char c, FILE *stream) {
    loop_until_bit_is_set(UCSR0A, UDRE0);
    UDR0 = c;
    return 0;
}

char uart_getchar(FILE *stream) {
    loop_until_bit_is_set(UCSR0A, RXC0);
    return UDR0;
}

FILE uart_output = FDEV_SETUP_STREAM(uart_putchar,
        NULL,
        _FDEV_SETUP_WRITE
        );

#define TIMER_FREQ_HZ   1

void timer1_init(void){
    // Init Pin PB2 Output, set to 0
    DDRB |= (1<<DDB2);
    PORTB &= ~(1<<PORTB2);

    // initialize timer1
    cli();
    TCCR1A = 0;
    TCCR1B = 0;
    TCNT1  = 0;

    // Set FastPWM with OCR1A on TOP, CLEAR on Compare + Set on BOTTOM
    // Prescaler 256
    // Here 64 !
    TCCR1A = _BV(COM1A1)
        | _BV(COM1B1)
        | _BV(WGM10)
        | _BV(WGM11);

    TCCR1B = _BV(WGM12)
        | _BV(WGM13)
        | _BV(CS11)
        | _BV(CS10);

    OCR1B = 0;
    OCR1A = 3125;
    sei();             // enable all interrupts
}

long int sec;
void timer0_init(void){
    cli();

    TCCR0A = 0;
    TCCR0B = 0;
    TCNT0  = 0;

    // Presc 1024
    TCCR0B |= _BV(CS02) | _BV(CS00);

    TIMSK0 |= _BV(TOIE0);
    sei();
}

ISR(TIMER0_OVF_vect)
{
    sec+=1;//(1024.f/16E6);
}

float seconds(void){
    float result;
    cli();
    result = sec*(255.f*1024.f/16E6);
    sei();
    return result;
}

float millis(void){// OVERFLOW in 3,4E38 ms
    float result;
    cli();
    result = sec*(255.f*1024.f/(16E6*10E-3));
    sei();
    return result;
}

#define TEMP_MAX 480
#define DEFAULT_TEMP 280
#define SLEEP_TEMP 0

float dt,new_error;
float THRESHOLD;
long new_time,old_time;

void set_temp(float temp){
    uint16_t duty_cycle;
    int duty_cycle_int = ((int)temp)*25;

    duty_cycle = (uint16_t)(duty_cycle_int);
    if(duty_cycle>=2500) duty_cycle = 2500;
    if(duty_cycle<0)    duty_cycle = 0;

    cli();
    OCR1B = duty_cycle;
    sei();
}

float get_temp(void){
    int i;
    static float lpf_temp;
    float avg_temp = 0.0;
#define AVG_SAMPLE 300
    if(adc_read()*(5.0f/1024.0f) > 4){
        return -1.0f;
    }
    for(i=0;i<AVG_SAMPLE;i++){
        avg_temp += 600*(5.0f/1024.0f)*adc_read() - 100; //0.5V => 200°C
    }
    avg_temp = avg_temp/AVG_SAMPLE;
#define alpha 0.99f
    lpf_temp = lpf_temp*alpha + (1.0-alpha)*avg_temp;
    return ((float)avg_temp);
}


enum e_state {UpdateConsigne,
    ProcessCommand,
    SleepMode,
    NoIron,
    };
enum e_state state = UpdateConsigne;

int counts = 0;
void encoder_init(void){
    // DT   2
    // CK   3
    // PUSH 6

    DDRD &= ~(1<<DDD2);
    DDRD &= ~(1<<DDD3);
    DDRD &= ~(1<<DDD6);

    PORTD |= (1<<PORTD2);
    PORTD |= (1<<PORTD3);
    PORTD |= (1<<PORTD6);
    counts = 0;

    // Set Exti
    PCMSK2 = 0;
    PCICR = 0;

    PCICR |= (1 << PCIE0);
    PCMSK2 |= (1<<PCINT18);
    PCMSK2 |= (1<<PCINT19);


    PCICR |= (1 << PCIE2);    // set PCIE2 to enable PCMSK0 scan
    PCMSK2 |= (1 << PCINT22);  // set PCINT0 to trigger an interrupt on state change 


}

// Return 1 if pressed
uint8_t enc_switch_state(void){
    return !(PIND&(1<<PIND6));
}
volatile uint8_t portdhistory = 0xFF;     // default is high because the pull-up
volatile uint8_t cycle_press = 0;
volatile float awake_time = 0.f;

void increment_counts(void){
    if((DEFAULT_TEMP+2*counts)<TEMP_MAX){
        counts++;
    }
}

void decrement_counts(void){
    if((DEFAULT_TEMP+2*counts)>0){
        counts--;
    }
}

ISR (PCINT2_vect){
    uint8_t changedbits;
    changedbits = PIND ^ portdhistory;
    portdhistory = PIND;

    // User Interation : Reset awake time
    awake_time = millis();

    // State machine for Encoder Press Unbouncing
    if(changedbits & (1<<PIND6)){
        if(!(PIND&(1<<PIND6))){// Button is pressed
            cycle_press = 1;
        }
        else{//Button is unpressed
            if(cycle_press==1){
                cycle_press = 0;
                state=(state==UpdateConsigne)?ProcessCommand:UpdateConsigne;
            }
        }
    }

    // Encoder Counting
    if(state==UpdateConsigne){
        if(changedbits & (1<<PIND2)){// Edge on D2
            if(PIND&(1<<PIND2))//Rising
            {
                if(PIND&(1<<PIND3)){
                    increment_counts();
                }
                else{
                    decrement_counts();
                }
            }
            else//Falling
            {
                if(!(PIND&(1<<PIND3))){
                    increment_counts();
                }
                else{
                    decrement_counts();
                }
            }
        }
    }

}

int get_counts(void){
    int result;
    cli();
    result = counts;
    sei();
    return result;
}


uint8_t line1[]   = "Starting up ...";
uint8_t line2[]   = "               ";
void update_screen(void){
    lcd_write_instruction_4d(lcd_Home);
    _delay_us(80);                                  // 40 uS delay (min)
    lcd_write_string_4d(line1);
    new_line(2);
    lcd_write_string_4d(line2);
    new_line(1);

}

uint16_t Consigne = DEFAULT_TEMP;

long time;

int main(void)
{
    uart_init();

    sei();

    stdout = &uart_output;

    timer1_init();
    timer0_init();
    encoder_init();
    lcd_init_4d();

    adc_init();

    float temp;
    Consigne = DEFAULT_TEMP;

    float blinky = 0.f;
    uint8_t onoff = 0;

    for (;;) {
        // Measure Temperature
        temp = get_temp();
        if(temp<0.){
            state=NoIron;
        }

        // Handle Sleep Mode Transition
        if((millis()-awake_time)>20.f*60.f*100.f){
            state = SleepMode;
            awake_time = millis();
        }

        switch(state){
            case UpdateConsigne:
                set_temp(0.f);

                Consigne = DEFAULT_TEMP + 2*get_counts();
                snprintf((char*)line2,16," %i C          ",(int)temp);
                if((millis()-blinky)>50.f){
                    blinky =millis();
                    onoff = !onoff;
                }
                if(onoff){
                    snprintf((char*)line1,16,"    : %i C         ",Consigne);
                    update_screen();
                }
                else{
                    snprintf((char*)line1,16,"Set : %i C         ",Consigne);
                    update_screen();
                }

                break;

            case NoIron:
                // Update display content
                snprintf((char*)line2,16,"No Iron               ");
                update_screen();
                if(temp>=0.0){
                    state=ProcessCommand;
                }
                set_temp(0.f);
                break;
            case SleepMode:
                // Update display content
                snprintf((char*)line2,16," %i C   %s       ",(int)temp,
                        "SLEEP");
                update_screen();
                set_temp(SLEEP_TEMP);
                // Out of sleep mode on ISR Encoder Pressed
                break;

            case ProcessCommand:

                // Update display content
                snprintf((char*)line2,16," %i C          ",(int)temp);
                update_screen();
                new_error = Consigne - temp; // Process new error
                THRESHOLD = 0.f;

                if(new_error<THRESHOLD){// Cool down
                    set_temp(0.0f);
                }
                else{
                    set_temp(TEMP_MAX);// Full noise
                }
                break;
        }
    }
    return 0; /* never reached */
}
