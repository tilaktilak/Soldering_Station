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
    // Init Pin PB0 Output, set to 0
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
#define DEFAULT_TEMP 240
float old_error,dt,new_error,derivative,integral,KP,KI,KD,command;
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


enum e_state {Update_Consigne,
    Process_Commmand,
    Send_Log};
enum e_state state = Update_Consigne;
uint8_t flag_change_consigne = 0;

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
    // Trigger INT1 falling edge
    EICRA |= (1<<ISC11);
    // Enable INT1
    EIMSK |= (1<<INT1);
}

// Return 1 if pressed
uint8_t enc_switch_state(void){
    return !(PIND&(1<<PIND6)); 
}
ISR (INT1_vect){
    // A
    if(flag_change_consigne){
        if(PIND&(1<<PIND2)){
            counts ++;
        }
        else{
            counts --;

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
    lcd_init_4d();

    encoder_init();
    adc_init();

    float tempp;
    Consigne = DEFAULT_TEMP;
    uint8_t flag_no_iron = 0;
    float time_no_sleep = 0.f;

    uint8_t sleep_mode = 0;

    for (;;) {
        update_screen();

        switch(state){
            case Update_Consigne:
                Consigne = DEFAULT_TEMP + (get_counts()>>2);

                snprintf((char*)line1,16,"Set : %i C         ",Consigne);
                if(enc_switch_state()){


                    while(enc_switch_state());
                    _delay_ms(500);
                    while(!enc_switch_state()){
                        if(sleep_mode){sleep_mode = 0;}
                        set_temp(0.f);
                        flag_change_consigne = 1;
                        update_screen();
                        Consigne = DEFAULT_TEMP + (get_counts()>>2);
                        snprintf((char*)line1,16,"Set : %i C         ",Consigne);
                        printf("[%s]\n\r",line1);
                    }
                    _delay_ms(500);
                }
                flag_change_consigne = 0;
                state = Process_Commmand;
                break;
            case Process_Commmand:
                snprintf((char*)line1,16,"Temp %i C         ",Consigne);
                update_screen();
                new_time = millis();
                dt       = new_time - old_time; 
                old_time = new_time;
                time_no_sleep += dt;
                printf("timenosleep %f\r\n",(double)time_no_sleep);
                if(time_no_sleep > 10.f*60.f*100.f){
                    sleep_mode = 1;
                    time_no_sleep = 0;
                    printf("Sleep mode\n");
                }
                // Update display content
                tempp = get_temp();
                snprintf((char*)line2,16," %i C   %s       ",(int)tempp,
                        (sleep_mode)?"SLEEP":"");
                if(tempp< 0.0){
                    snprintf((char*)line2,16,"No Iron               ");
                    update_screen();
                    flag_no_iron = 1;
                }
                else{
                    flag_no_iron = 0;
                }
                new_error = Consigne - tempp; // Process new error
                derivative = (new_error-old_error)/(dt/1000);
                integral += new_error*(dt/1000);

                old_error = new_error;

                //KP = 10.f;
                //KD = 0.0f; // Rend instable le système mm avec petites valeurs
                //KI = 1.f;
                KP = 10.f;
                KI = 1.f;
                // SIMU : en 5sec à 90%, 10sec à 100%, pas de dépassement
                printf("%d ; %d\r\n",(int)millis(),(int)get_temp());
                command = KP*new_error + KD*derivative + KI*integral;

                if((command >= 0.0f) && !flag_no_iron &&!sleep_mode){
                    set_temp(command);
                }
                else if(command < 0.f){
                    set_temp(0.0f);
                }
                else{
                    set_temp(0.f);
                }

                state = Send_Log;
                break;
            case Send_Log:
                state = Update_Consigne;
                break;
        }
    }	
    return 0; /* never reached */
}
