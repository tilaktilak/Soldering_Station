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

#define MOS 10

// include the library code:
#include <LiquidCrystal.h>
#include <Encoder.h>
#include <avr/io.h>
// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(12, 11, 7, 8, 4, 5);

Encoder myEnc(DT,CLK);


void setup() {
    pinMode(MOS,OUTPUT);
    // initialize timer1
    noInterrupts();           // disable all interrupts
    TCCR1A = 0;
    TCCR1B = 0;
    TCNT1  = 0;

    // Set FastPWM with OCR1A on TOP, CLEAR on Compare + Set on BOTTOM
    // Prescaler 256
    TCCR1A = _BV(COM1A1) | _BV(COM1B1) | _BV(WGM10) | _BV(WGM11);
    TCCR1B = _BV(WGM12) | _BV(WGM13) | _BV(CS12);

    OCR1B = 300;
    OCR1A = 3125;
    interrupts();             // enable all interrupts

    // set up the LCD's number of columns and rows:
    lcd.begin(16, 2);
    // Print a message to the LCD.
    lcd.print("Starting up ...");
    pinMode(temp,INPUT);
    pinMode(CLK,INPUT);
    pinMode(DT,INPUT);
    pinMode(SW,INPUT);

    // Initiate Pull-up on switch input
    digitalWrite(SW,HIGH);

}

//ISR(TIMER1_COMPA_vect)          // timer compare interrupt service routine
//{
//      digitalWrite(MOS, digitalRead(MOS) ^ 1);   // toggle LED pin
//}

float old_error,old_time,new_time,dt,new_error,derivative,integral,KP,KI,KD,command;

float get_temp(){
    int temp = analogRead(temp);
    return ((float)temp * (5.0f / 1023.0f));
}

#define DEFAULT_TEMP 280

uint16_t Consigne = DEFAULT_TEMP;
uint16_t last_display_update_ms = 0;


/**
 * @brief MAIN LOOP
 *
 */
void loop() {
    long newPosition;
    long oldPosition = -999;
    // Pull-up do not stay setted
    digitalWrite(SW,HIGH);

    // Initialize encoder value to default
    myEnc.write(DEFAULT_TEMP<<2);

    // Setting Mode loop
    if(digitalRead(SW)==0){ // If we press button
        while(digitalRead(SW)==0); // Anti-bounce
        lcd.setCursor(0,0);
        lcd.print("Set value : ");
        while(digitalRead(SW)==1){
            newPosition = myEnc.read();
            Consigne = newPosition>>2; // Set new consigne
            lcd.setCursor(11,0);
            lcd.print(Consigne);
        }
        while(digitalRead(SW)==0); // Anti-bounce
    }

    new_time = millis();
    dt = new_time - old_time;

    // Update display content
    if(last_display_update_ms + 500 <= new_time){
        lcd.setCursor(0, 0);
        lcd.print("Temp : ");
        lcd.print(Consigne);
        lcd.print(" C     ");
        lcd.setCursor(0,1);
        lcd.print("                   ");
        lcd.setCursor(0,1);
        lcd.print(get_temp());
        lcd.print(" - ");
        lcd.print(command);
        old_error = 0;
        old_time = 0;
        last_display_update_ms = new_time;
    }

    new_error = Consigne - get_temp(); // Process new error

    derivative = (new_error-old_error)/dt;
    integral += new_error*dt;

    old_error = new_error;

    KP = 10;
    KD = 1;
    KI = 0.1;
    //command = KP*new_error + KD*derivative + KI*integral;
    command = Consigne;
    //set_temp(command);

}

#define TEMP_MAX 480
void set_temp(float temp){
    int tension = (int)(temp);// * (255/TEMP_MAX);
    analogWrite(MOS,tension);
}


