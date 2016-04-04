/*
   The circuit:
 * LCD RS pin to digital pin 12
 * LCD Enable pin to digital pin 11
 * LCD D4 pin to digital pin 5
 * LCD D5 pin to digital pin 4
 * LCD D6 pin to digital pin 3
 * LCD D7 pin to digital pin 2
 * LCD R/W pin to ground
 * LCD VSS pin to ground
 * LCD VCC pin to 5V
 * 10K resistor:
 * ends to +5V and ground
 * wiper to LCD VO pin (pin 3)
 */

#define temp A0
#define CLK 8
#define DT 7
#define SW 6

#define MOS 6
// include the library code:
#include <LiquidCrystal.h>
#include <Encoder.h>

// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(12, 11, 2, 3, 4, 5);

Encoder myEnc(CLK, DT);


void setup() {
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

    pinMode(MOS,OUTPUT);
}

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
        lcd.print(get_temp());
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
    command = KP*new_error + KD*derivative + KI*integral;
    set_temp(command);

}

#define TEMP_MAX 480
void set_temp(float temp){
    int command = temp * TEMP_MAX / 255;
    analogWrite(MOS,command);
}


