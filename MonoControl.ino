#include <avr/io.h>
#include <avr/interrupt.h>
#include <EEPROM.h>

#define PULpin 2
#define DIRpin 5
#define ENApin 8
#define LEDPIN 13
#define STOP1PIN 10
#define STOP2PIN 11

#define DIR_UP HIGH
#define DIR_DOWN LOW
#define ENA LOW
#define DIS HIGH

#define DEV_NAME "Monochromator controller (v0.1, ID0855)"

int i, stepInc;
String inmsg;
long dst, tPos;
volatile int state, stop1, stop2;
volatile long k, acp;

// minTI = 1500 -> max speed = 200 RPM
//unsigned int minTI = 1500;
//unsigned int maxTI = 15000;
unsigned int minTI = 150;
unsigned int maxTI = 1500;
unsigned int maxASt = 1000;
float lt;
unsigned int aSteps; 
unsigned int accelTMap[500];

int eeAddress = 0;
volatile long stepperPos = 0;
unsigned long minPos = 0;
unsigned long maxPos = 1000000;

void setup() {
  // put your setup code here, to run once:
  pinMode(PULpin, OUTPUT);
  pinMode(DIRpin, OUTPUT);
  pinMode(ENApin, OUTPUT);
  pinMode(STOP1PIN, INPUT_PULLUP);
  pinMode(STOP2PIN, INPUT_PULLUP);
  pinMode(LEDPIN, OUTPUT);
  
  // set direction
  digitalWrite(DIRpin, DIR_UP);

  // disable
  digitalWrite(ENApin, DIS);

  // PUL default
  digitalWrite(PULpin, HIGH);

  // read stepper position
  EEPROM.get(eeAddress, stepperPos);

  Serial.begin(9600);

  // Init. Timer1
  cli();
  TCCR1A = 0;
  TCCR1B = 0;

  // CTC mode
  TCCR1B |= (1 << WGM12);
  // 1/8 CLK prescaler
  TCCR1B |= (1 << CS11);

  OCR1A = maxTI;

  TIMSK1 |= (1 << OCIE1A);  // включить прерывание по совпадению таймера 
  sei();
}

void accelMap(int aPts) {
  lt = 2 * 0.99 * (maxTI - minTI) / aPts;
  
  accelTMap[0] = maxTI;
  for (i = 1; i < aPts; i++) {
    accelTMap[i] = floor(accelTMap[i-1] - lt * exp(-pow((i - aPts / 2.0) / (aPts / 5.0), 2) / 2));
  }
}

void setMovingParams(long dst) {
  // calculation of the acceleration map
  if(dst > 2*maxASt) {
    aSteps = maxASt;
    accelMap(aSteps/2); // acceleration map contains the time value for every second step
    
  } else if(dst > maxASt) {
    aSteps = dst / 2;
    accelMap(aSteps/2);
    
  } else {
    aSteps = dst / 2;
    accelMap(maxASt/4);
  }
  
  acp = 0;
  OCR1A = accelTMap[0]; 
  
  // movement activation by setting k = number of steps
  k = dst;
}

void loop() {
  if(Serial.available() > 0 ){
    inmsg = Serial.readString();
    
    if(inmsg.indexOf("GP") > -1){
      // send ansver
      Serial.println(stepperPos);
      
    } else if(inmsg.indexOf("GA") > -1){
      // absolute position
      tPos = inmsg.substring(2).toInt();
      // travel distance
      dst = tPos - stepperPos;

      if((tPos > 0) and (tPos <= maxPos)) {
        // set direction
        if(dst > 0) {
          digitalWrite(DIRpin, DIR_UP);
          stepInc = 1;
        } else {
          digitalWrite(DIRpin, DIR_DOWN);
          stepInc = -1;
        }
  
        // enable
        digitalWrite(ENApin, ENA);
  
        // moving parameters
        setMovingParams(dst);
        
        // send ansver
        Serial.println("OK");
        
      } else {
        Serial.println("ERROR");
      }
      
    } else if(inmsg.indexOf("G+") > -1){
      // travel distance
      dst = inmsg.substring(2).toInt();
      // absolute position
      tPos = stepperPos + dst;
      
      if(tPos <= maxPos) {
        // set direction
        digitalWrite(DIRpin, DIR_UP);
        stepInc = 1;
  
        // enable
        digitalWrite(ENApin, ENA);
        
        // moving parameters
        setMovingParams(dst);
         
        // send ansver
        Serial.println("OK");
      
      } else {
        Serial.println("ERROR");
      }
      
    } else if(inmsg.indexOf("G-") > -1){
      // distance
      dst = inmsg.substring(2).toInt();
      // absolute position
      tPos = stepperPos - dst;
      
      if(tPos >= 0) {
        // set direction
        digitalWrite(DIRpin, DIR_DOWN);
        stepInc = -1;
  
        // enable
        digitalWrite(ENApin, ENA);
        
        // moving parameters
        setMovingParams(dst);
        
        // send ansver
        Serial.println("OK");
      
      } else {
        Serial.println("ERROR");
      }
      
    } else if(inmsg.indexOf("G0") > -1){
      // set direction
      digitalWrite(DIRpin, DIR_DOWN);
      stepInc = -1;

      // enable
      digitalWrite(ENApin, ENA);
      
      aSteps = 0;
      OCR1A = maxTI;
      k = maxPos;
      
      Serial.println("OK");
      
    } else if(inmsg.indexOf("DS") > -1){
      if(k > 0){
        Serial.println("BUSY");
      } else {
        Serial.println("OK");
      }
      
    } else if(inmsg.indexOf("DM") > -1){
      Serial.println(DEV_NAME);
    }
  }
}

ISR(TIMER1_COMPA_vect)
{
  if (k > 0) {
    // motor step
    state = digitalRead(LEDPIN);
    digitalWrite(LEDPIN, !state);
//    state = digitalRead(PULpin);
//    digitalWrite(PULpin, !state);
    
    if (state == HIGH) {
      k--;
      stepperPos += stepInc;

      stop1 = digitalRead(STOP1PIN);
      stop2 = digitalRead(STOP2PIN);
      if((stop1 == LOW) or (stop2 == LOW)) {
        k = 0;
        stepperPos = 0;
      }
      
      if(k == 0) {
        // save position in EEPROM
        EEPROM.put(eeAddress, stepperPos);
        // disable
        digitalWrite(ENApin, DIS);
      } else {
        // acceleration map index
        if(((dst - k) < aSteps) and (k % 2 == 0)) {
          acp += 1;
          // set timer value from acceleration map
          OCR1A = accelTMap[acp];
        } else if((k < aSteps) and (k % 2 == 0)) {
          acp -= 1;
          OCR1A = accelTMap[acp];
        }   
      } 
    }
  }
}
