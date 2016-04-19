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


#define DEFAULT_TEMP 180
void setup() {
    Serial.begin(115200);

    pinMode(MOS,OUTPUT);
    // initialize timer1
    noInterrupts();           // disable all interrupts
    TCCR1A = 0;
    TCCR1B = 0;
    TCNT1  = 0;

    // Set FastPWM with OCR1A on TOP, CLEAR on Compare + Set on BOTTOM
    // Prescaler 256
    TCCR1A = _BV(COM1A1) | _BV(COM1B1) | _BV(WGM10) | _BV(WGM11);
    TCCR1B = _BV(WGM12) | _BV(WGM13) | _BV(CS11) | _BV(CS10);

    OCR1B = 0;//156;//10*3125/100;
    OCR1A = 3125;
    interrupts();             // enable all interrupts

    // set up the LCD's number of columns and rows:
    lcd.begin(16, 2);
    // Print a message to the LCD.
    lcd.print("Starting up ...");
    pinMode(temp,INPUT);
    digitalWrite(temp,LOW);
    pinMode(CLK,INPUT);
    pinMode(DT,INPUT);
    pinMode(SW,INPUT);

    // Initiate Pull-up on switch input
    digitalWrite(SW,HIGH);

    // Initialize encoder value to default
    myEnc.write(DEFAULT_TEMP<<2);
}

//ISR(TIMER1_COMPA_vect)          // timer compare interrupt service routine
//{
//      digitalWrite(MOS, digitalRead(MOS) ^ 1);   // toggle LED pin
//}

float old_error,dt,new_error,derivative,integral,KP,KI,KD,command;
long new_time,old_time;
#define TEMP_MAX 480

void set_temp(float temp){
    int duty_cycle_int = temp * (TEMP_MAX / 100.0f);
    uint16_t duty_cycle = (uint16_t)(duty_cycle_int) * 31;
    //lcd.setCursor(9,1);
    //lcd.print(duty_cycle);
    noInterrupts();           // disable all interrupts
    OCR1B = duty_cycle;
    interrupts();
}

float get_temp(){
    int i;
    static float lpf_temp;
    float avg_temp = 0.0;
#define AVG_SAMPLE 300
    if(analogRead(temp)*(5.0f/1024.0f) > 4){ 
        return -1.0f;
    }
    for(i=0;i<AVG_SAMPLE;i++){
        avg_temp += 600*(5.0f/1024.0f)*analogRead(temp) - 100; //0.5V => 200°C
    }
    avg_temp = avg_temp/AVG_SAMPLE;
#define alpha 0.99f
    lpf_temp = lpf_temp*alpha + (1.0-alpha)*avg_temp;
    return ((float)avg_temp);
}


uint16_t Consigne = DEFAULT_TEMP;
long last_display_update_ms = 0;

long time;
/**
 * @brief MAIN LOOP
 *
 */
void loop() {
    long newPosition;
    long oldPosition = -999;
    // Pull-up do not stay setted
    digitalWrite(SW,HIGH);

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
            lcd.print("  ");
        }
        while(digitalRead(SW)==0); // Anti-bounce
        derivative = 0.0;
        integral = 0.0;
    }

    new_time = millis();
    dt = new_time - old_time;
    old_time = new_time;
    // Update display content
    float tempp = get_temp();
    if(last_display_update_ms + 1000 <= new_time){
        lcd.print("                                               ");
        lcd.setCursor(0, 0);
        lcd.print("Set  : ");
        lcd.print(Consigne);
        lcd.print(" C     ");
        lcd.setCursor(0,1);
        //lcd.print("                   ");
        //lcd.setCursor(0,1);
        if(tempp< 0.0){
            lcd.print("No Iron");
        }
        else{
            //lcd.print("           ");
            lcd.setCursor(0,1);
            lcd.print("Temp : ");
            lcd.print(tempp);
            lcd.print(" C");
        }
        last_display_update_ms = new_time;
    }
        //lcd.print(command);
        new_error = Consigne - tempp; // Process new error
        derivative = (new_error-old_error)/(dt/1000);
        integral += new_error*(dt/1000);

        old_error = new_error;

        /*KP = 0.09; Réglage OK mais pas hyper stable
        KD = -0.13;
        KI = 0.0026;*/

        KP = 0.17;
        KD = 0.0; // Rend instable le système mm avec petites valeurs
        KI = 0.005;
        // SIMU : en 5sec à 90%, 10sec à 100%, pas de dépassement
        command = KP*new_error + KD*derivative + KI*integral;
    if(command >= 0.0){
            set_temp(command);
        }
        else{
            set_temp(0.0);
        }

    time += dt;
    Serial.print(time);
    Serial.print(";");
    Serial.println(get_temp());


}



