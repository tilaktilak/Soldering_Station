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
#define A A1
#define B A2
#define C A3

#define MOS 6
// include the library code:
#include <LiquidCrystal.h>
#include <Encoder.h>

// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(12, 11, 5, 4, 3, 2);

Encoder myEnc(A1, A2);


void setup() {
  // set up the LCD's number of columns and rows:
  lcd.begin(16, 2);
  // Print a message to the LCD.
  lcd.print("Starting up");

  pinMode(temp,INPUT);
  pinMode(A,INPUT);
  pinMode(B,INPUT);
  pinMode(C,INPUT);

  pinMode(MOS,OUTPUT);
}

void loop() {
  float consigne = 280.0;

  
  // set the cursor to column 0, line 1
  // (note: line 1 is the second row, since counting begins with 0):
  lcd.setCursor(0, 1);
  // print the number of seconds since reset:
  lcd.print("Temperature :");
  static long oldPosition = -999;
 
  if(digitalRead(C)==1){ // If we press button
    long newPosition = myEnc.read();
    if (newPosition != oldPosition) {
      oldPosition = newPosition;
      Consigne += 0.01 * newPosition; // Set new consigne
    }
  }
    lcd.print(Consigne);
    static float old_error = 0;
    static old_time = 0;
  
    float new_time = millis();
    float dt = new_time - old_time;
    
    float new_error = get_consigne()- get_temp(); // Process new error
    
    float derivative = (new_error-old_error)/dt;
    float integral += new_error*dt;
  
    old_error = new_error;
  
    float KP = 10;
    float KD = 1;
    float KI = 0.1;
    float command = KP*error + KD*derivative + KI*integral;
    set_temp(command); 
  
}

float get_temp(){
  int temp = analogRead(temp);
  return 3.14*temp;  
}

set_temp(float temp){
  int command = temp * TEMP_MAX / 255;
  analogWrite(MOS,command);
}


