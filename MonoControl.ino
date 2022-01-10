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
#define ENA HIGH
#define DIS LOW

#define DEV_NAME "Monochromator controller (v1.0, ID0855)"

int i, stepInc;
String inmsg;
long dst, tPos;
volatile byte mstep = 0;
volatile int state, stop1, stop2;
volatile long k, acp;

// minTI = 750 -> max speed = 100 RPM (1/4 substep)
unsigned int minTI = 340; // minTI = 160;
unsigned int maxTI = 4000;
unsigned int maxASt = 1200;
float sigma, lt;
unsigned int aSteps; 
unsigned int accelTMap[601];

int eeAddress = 0;
unsigned long stepperPos = 0;
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
  digitalWrite(PULpin, LOW);

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
  sigma = aPts / 3.5;
  lt = 2.535 * (maxTI - minTI) / aPts;
  
  accelTMap[0] = maxTI;
  for (i = 1; i < aPts; i++) {
    accelTMap[i] = floor(accelTMap[i-1] - lt * exp(-pow(i / sigma, 2) / 2));
    // Serial.print(String(i)+" -> ");
    // Serial.println(accelTMap[i]);
  }
}

void setMovingParams(long dst) {
  // calculation of the acceleration map
  if(dst > 2*maxASt) {
    aSteps = maxASt;
    accelMap(aSteps/2 + 1); // acceleration map contains the time value for every second step
    
  } else if(dst > maxASt) {
    aSteps = dst / 2;
    accelMap(aSteps/2 + 1);
    
  } else {
    aSteps = dst / 2;
    accelMap(maxASt/4 + 1);
  }
  
  mstep = 0;
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
          dst = abs(dst);
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
      OCR1A = minTI * 2;
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
    state = digitalRead(PULpin);
    digitalWrite(PULpin, !state);
    mstep += 1;
    
    // 1/4 substep, 2 states. 1 step = 8 microsteps 
    if (mstep > 7) {
      mstep = 0;
      k--;
      stepperPos = stepperPos + stepInc;

      if(stepInc < 0) {
        stop1 = digitalRead(STOP1PIN);
        if (stop1 == LOW) {
          k = 0;
          stepperPos = 0;
        }
      } 
      else {
        stop2 = digitalRead(STOP2PIN);
        if (stop2 == LOW) {
          k = 0;
          maxPos = stepperPos;
        }
      }
      
      if(k == 0) {
        // save position in EEPROM
        EEPROM.put(eeAddress, stepperPos);
        // disable
        digitalWrite(ENApin, DIS);
        // set dafault direction
        digitalWrite(DIRpin, DIR_UP);
      } else {
        // acceleration map
        if(((dst - k) <= aSteps) and ((dst - k) % 2 == 0)) {
          // set timer value from acceleration map
          acp += 1;
          OCR1A = accelTMap[acp];
        } else if((k <= aSteps) and (k % 2 == 0)) {
          acp -= 1;
          OCR1A = accelTMap[acp];
        }   
      } 
    }
  }
}
