#include <SoftwareSerial.h>

//h-bridge pwm pins
const int roller_enable_pin = 5;
const int hydration_enable_pin = 6;
const int extraction_enable_pin = 9; //BECAUSE TIMER 1 FREQUENCY HAS BEEN RECONFIGURED. PIN 9 AND 10 PWM NO LONGER WORKING PROPERLY. ASSIGN ANOTHER PIN FOR EXTRACTION or use digitalwrite for pin 9.

//shift register pin
const int dataPin = 18;
const int latchPin = 19;
const int clockPin = 20;

//rotary encoder pins - one quadrature output (CLK-channel A) is assigned to hardware interrupt
const int encoder_hydration_CLK_pin = 2;
const int encoder_roller_CLK_pin = 3;
//rotary encoder pins - DT-channel B
const int encoder_hydration_DT_pin = 4;
const int encoder_roller_DT_pin = 8;

//encoder pins state
volatile uint8_t currentState_hydration_CLK;
volatile uint8_t lastState_hydration_CLK;

volatile uint8_t currentState_roller_CLK;
volatile uint8_t lastState_roller_CLK;

//payload
uint8_t payload = 0b00000000;

//bit mask for each motor
#define ROLLER_MASK 0b00000011
#define HYDRATION_MASK 0b00001100
#define EXTRACTION_MASK 0b00110000

//variables for controlling speed
int current_roller_speed = 150;
int current_hydration_speed = 150;

//variable for encoder ISR
volatile int counter_roller = 0;
volatile int counter_hydration = 0;

//variable for timer ISR
volatile bool hydrationOn = false;

//ISR for hydration encoder
void readEncoderHydration(){
  //read current state of CLK
  currentState_hydration_CLK = digitalRead(encoder_hydration_CLK_pin);
  
  //if current state and last state of CLK are different, means pulse occured
  if (currentState_hydration_CLK != lastState_hydration_CLK && currentState_hydration_CLK == HIGH){

    //if DT state is different than the CLK then encoder is rotating CCW
    if (digitalRead(encoder_hydration_DT_pin) != currentState_hydration_CLK){
      --counter_hydration;
    }else{
      ++counter_hydration;
    }
  }
  lastState_hydration_CLK = currentState_hydration_CLK;
}

//ISR for extraction encoder
void readEncoderRoller(){
  //read current state of CLK
  currentState_roller_CLK = digitalRead(encoder_roller_CLK_pin);

  //if current state and last state of CLK are different, means pulse occured
  if (currentState_roller_CLK != lastState_roller_CLK && currentState_roller_CLK == HIGH){

    //if DT state is different than the CLK then encoder is rotating CCW
    if (digitalRead(encoder_roller_DT_pin) != currentState_roller_CLK){
      --counter_roller;
    }else{
      ++counter_roller;
    }

  }
  lastState_roller_CLK = currentState_roller_CLK;
}

ISR(TIMER1_COMPA_vect){
  if (hydrationOn){
    //if hydration motor is ON, now turn OFF. Advance the COMPA register; next interval is 1s
    OCR1A += 62499;
  }else{
    //if hydration motor is OFF, now turn ON. Advance the COMPA register; next interval is 2s
    OCR1A += 2 * 62499;
  }

  hydrationOn = !hydrationOn; 
}

void setup() {
  //disable all global interrupt
  cli();
  //initialise timer1
  TCCR1A = 0;
  TCCR1B = 0;
  //set prescaler to 256
  TCCR1B |= (1 << CS12 );
  //first interval is 1s
  OCR1A = 62499;
  //enable compare match A interrupt
  TIMSK1 |= (1 << OCIE1A); 
  //enable global interrupt
  sei();

  pinMode(hydration_enable_pin, OUTPUT);
  pinMode(extraction_enable_pin, OUTPUT);
  pinMode(roller_enable_pin, OUTPUT);

  pinMode(dataPin, OUTPUT);
  pinMode(latchPin, OUTPUT);
  pinMode(clockPin, OUTPUT);

  pinMode(encoder_roller_CLK_pin, INPUT);
  pinMode(encoder_hydration_CLK_pin, INPUT);

  lastState_hydration_CLK = digitalRead(encoder_hydration_CLK_pin);
  lastState_roller_CLK = digitalRead(encoder_roller_CLK_pin);

  //setup hardware interrupt pin and ISR to call
  attachInterrupt(digitalPinToInterrupt(encoder_hydration_CLK_pin), readEncoderHydration, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder_roller_CLK_pin), readEncoderRoller, CHANGE);

  Serial.begin(9600);

  /*while (!Serial){
    ;
  }
  delay(1000);*/
  startRoller();
  startHydration();
  startExtraction();

  /*for (int i = 7; i >= 0; i--) {
    Serial.print(bitRead(payload, i));
  }
  Serial.println();*/
  
}

void loop() {
  updateShiftRegister();

  updateRollerSpeed(counter_roller);
  //reset counter after passed to updateSpeed()
  counter_roller = 0;

  updateHydrationSpeed(counter_hydration);
  //reset counter after pass to updateSpeed()
  counter_hydration = 0;

  Serial.print("roller pwm: ");
  Serial.print(current_roller_speed);
  Serial.print("\thydration pwm: ");
  Serial.print(current_hydration_speed);
  Serial.print("hydration state: ");
  Serial.println(hydrationOn);
  
}

void startRoller(){
  //isolate and clear roller bits
  payload &= ~ROLLER_MASK;

  //roller rotate forward
  payload |= 0b00000010;

  //
  analogWrite(roller_enable_pin, current_roller_speed);

}

void startHydration(){
  //isolate and clear hydration pump bits
  payload &= ~HYDRATION_MASK;

  //pump rotate direction
  payload |= 0b00001000;

  //
  if (hydrationOn){
    analogWrite(hydration_enable_pin, current_hydration_speed);
  }else{
    analogWrite(hydration_enable_pin, 0);
  }
  

}

void startExtraction(){
  //isolate and clear extraction pump bits
  payload &= ~EXTRACTION_MASK;

  //pump rotate direction
  payload |= 0b00100000;

  //make the extraction pump run at fixed 80% speed
  //analogWrite(extraction_enable_pin, 204);
  //because we use timer 1 register and reconfigure the frequency, pwm on pin 9 no longer working. hence, use digitalWrite instead.
  digitalWrite(extraction_enable_pin, HIGH);

}

void updateRollerSpeed(int delta){
  //update motor speed based on encoder input
  current_roller_speed += delta;
  //clamp speed between 0-255
  current_roller_speed = max(0, min(255, current_roller_speed));
  analogWrite(roller_enable_pin, current_roller_speed);
}

void updateHydrationSpeed(int delta){
  current_hydration_speed += delta;
  //clamp speed between 0-255
  current_hydration_speed = max(0, min(255, current_hydration_speed));
  if (hydrationOn){
    analogWrite(hydration_enable_pin, current_hydration_speed);
  }else{
    analogWrite(hydration_enable_pin, 0);
  }
}
  

void updateShiftRegister(){
  //set latch pin low before shifting for smooth update
  digitalWrite(latchPin, LOW);
  shiftOut(dataPin, clockPin, MSBFIRST, payload);
  //LOW->HIGH move payload from shift register to storage register
  digitalWrite(latchPin, HIGH);
}