#include <math.h> // use for sine wave function

//STATES
#define STOPPED 0
#define RUNNING 1
#define STOPPING 2

// PINS
const int stepPin = 2;
const int dirPin = 3;
const int enablePin = 4;
const int startButtonPin = 13;

// CONSTANTS
const int step_divider = 8; //1/8 step mode
const float steps_per_mm = 50 * step_divider;
const int button_millis = 50;
// const float refresh_rate = 100; // Hz

// USER PARAMETERS
const float amplitude = 5.0; // mm
const int iterations = -1; // number of times to perform operation after power on, -1 is endless
const float test_freq = 1.0/1.0; // frequency of oscillation in Hz
const int motor_dir = -1; // 1 for forward, -1 for reverse motor direction

// VARIABLES
int dir = 0;
float motor_position = 0.0; 
int motor_steps = 0;
double t = 0.0;
unsigned long lastMillis = 0;
static int state = STOPPED; // 0
unsigned long buttonLastMillis = 0;
bool buttonPressed  = false;
unsigned long startLastMillis = 0;
float last_x = 0;
int slope = 0;
int last_slope = 0;
long cycleCount = 0;


void setDir(int d) {
  if(d == 0){
    digitalWrite(dirPin, LOW);
  }
  else if(d == 1){
    digitalWrite(dirPin, HIGH);
  }
}

void step_motor(int s) {
  s *= motor_dir;
  if(s<0){
    setDir(1);
  }
  else if(s>0){
    setDir(0);
  }
  s = abs(s);
  for(int i=0; i<s; i++){
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(50);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(50);
  }
}

void update_time(){
  unsigned long currentMillis = millis();
  unsigned long elapsedMillis = (unsigned long)(currentMillis - lastMillis);
  t += (double)(elapsedMillis)/1000;
  lastMillis = currentMillis;
//  Serial.println(t);
}



float get_motor_x(double t){
  static bool get_motor_x_flag = false;
  const static float half_amplitude = amplitude * 0.5;
  static float phase_offset = 0;
  float x = 0;
  if(get_motor_x_flag == false){
    phase_offset = (2 * PI) * fmod((test_freq * t), 1.0);
    get_motor_x_flag = true;
  }
  x =  half_amplitude * cos(2* PI * test_freq * t - phase_offset) - half_amplitude;
//  Serial.println(TWO_PI);
  if(state == STOPPING && abs(motor_position) < 0.05 && slope == 1){
    state = STOPPED;
    Serial.println("STOPPED");
    get_motor_x_flag = false;
    x = 0;
    return x;
  }
  if(x - last_x < 0){
    slope = -1; 
  } else if(x - last_x > 0){
    slope = 1;
  }
  last_x = x;
  return x;
}

void setMotorPosition(float x){
  int new_steps = (int)(x * steps_per_mm);
  step_motor(new_steps - motor_steps);
  motor_steps = new_steps;
  motor_position = x;

  //TODO
  // implement reverse motor direction here
}

void motor_enable(bool enable){
  if(enable == false){
    digitalWrite(enablePin, HIGH);
  }
  else if(enable == true){
    digitalWrite(enablePin, LOW);
  }
}

void update_count(){
  if(slope == -1 && last_slope != -1){
    cycleCount++;
    Serial.println(cycleCount);
  }
  last_slope = slope;
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(enablePin, OUTPUT);
  pinMode(startButtonPin, INPUT_PULLUP);
  setDir(dir);

  //wait for start button to be pressed
  
  
}

void updateState(){
  switch(state){
    
    case STOPPED:
      digitalWrite(enablePin, HIGH);
      while(state == STOPPED)
        {
          if(digitalRead(startButtonPin) == LOW){
            if(buttonPressed == false){
            buttonLastMillis = millis();
            buttonPressed = true;
            }
            if( iterations!= 0 && (digitalRead(startButtonPin) == LOW) && ((millis() - buttonLastMillis) >= button_millis) && (buttonPressed == true)){
              while(digitalRead(startButtonPin) == LOW){}
              buttonPressed = false;
              state = RUNNING;
              Serial.println("RUNNING");
              delay(50);
              startLastMillis = millis();
            }
          }
        }
        digitalWrite(enablePin, LOW);
      break;

    case RUNNING:
        if(millis() - startLastMillis >= button_millis){
          if(digitalRead(startButtonPin) == LOW){
            if(buttonPressed == false){
              buttonLastMillis = millis();
              buttonPressed = true;
            }
            if( (digitalRead(startButtonPin) == LOW) && ((millis() - buttonLastMillis) >= button_millis) && (buttonPressed == true)){
              state = STOPPING;
              Serial.println("STOPPING");
              buttonPressed = false;
            }
          }
          if(cycleCount % iterations == 0 && cycleCount != 0 && digitalRead(startButtonPin)==HIGH && iterations != -1){
            state = STOPPING;
            Serial.println("STOPPING");
          }
        }
      break;

    case STOPPING:
        // do nothing here, changed to stopped only by get_motor_x function.
      break;
  }
  while(state == STOPPED)
  {
    if(digitalRead(startButtonPin) == LOW){
      state = RUNNING;
      Serial.println("RUNNING");
    }
  }
}

void loop() {
  updateState();
  update_time();
  setMotorPosition(get_motor_x(t));
  update_count();
}
