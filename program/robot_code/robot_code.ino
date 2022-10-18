#include <util/atomic.h>

volatile int posi[] = {0, 0, 0, 0}; // add more for more motors

#define motor_number 4

// Motor driver and encoder pin definitions as an array
// To add more motors keep adding to the array
// my_array[] = {value_M1, value_M2, value_M3, .....};
// Try changing the encoder pins, in the arduino nano pin11 for enb did not work
const int ENABLE[] = {13, 44, 7, 45};
const int IN1[] = {12, 48, 6, 47};
const int IN2[] = {11, 46, 5, 49};
const int ENCA[] = {2, 3, 18, 19}; // Pins for interrupt signal
const int ENCB[] = {53, 52, 51, 50};

const int pwm_resolution = 255; // The max duty cycle value for the pwm signal

// PID values for each motor as an array
// control_array[] = {kp_m1, kp_m2, kp_m3...};
// start values: 3.5, 0.02, 0.25
// safe values: 1, 0, 0
const int KP[] = {3.5, 3.5, 3.5, 3.5}; // decreases rise time
const int KI[] = {0.02, 0.02, 0.02, 0.02}; // eliminates steady-state error
const int KD[] = {0.25, 0.25, 0.25, 0.25}; // decreases overshoot

// const int KP[] = {1, 1, 1, 1}; // decreases rise time
// const int KI[] = {0, 0, 0, 0}; // eliminates steady-state error
// const int KD[] = {0, 0, 0, 0}; // decreases overshoot


// PID variables used in function
long prevT = 0;
float eprev = 0;
float eintegral = 0;

// User input and communication variables
String readString;
int user_input, user_input2;
bool k = true;
int counts = 0;
String split_1;
String split_2;
int comma_index;

// Set up for stepper motor and limit switch
const int stepper_step = 24;
const int stepper_dir = 22;
const int stepper_enable = 27;

const float step_angle = 1.8; // from stepper data sheet
const int steps_per_rev = 360 / step_angle;

const int limit_switch = 39;
int limit_state = 0;

const int magnets = 31;
int magnets_state = 0;

void setup() 
{
  Serial.begin(9600);

  for(int i = 0; i < motor_number; i++)
  {
    pinMode(ENCA[i],INPUT);
    pinMode(ENCB[i],INPUT);

    pinMode(ENABLE[i],OUTPUT);
    pinMode(IN1[i],OUTPUT);
    pinMode(IN2[i],OUTPUT);
  }
  
  // Set the interruption pins for the encoders
  attachInterrupt(digitalPinToInterrupt(ENCA[0]), readEncoder<0>, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCA[1]), readEncoder<1>, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCA[2]), readEncoder<2>, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCA[3]), readEncoder<3>, RISING);

  pinMode(stepper_step, OUTPUT);
  pinMode(stepper_dir, OUTPUT);
  pinMode(stepper_enable, OUTPUT);
  pinMode(limit_switch, INPUT);
  
}

void loop() 
{
  // Initial stepper configuration
  digitalWrite(stepper_enable, HIGH);

  read_serial_port();

  limit_state = digitalRead(limit_switch);

  if (split_1 == "PZ")
  {    
    digitalWrite(stepper_enable, LOW);
    digitalWrite(stepper_dir, HIGH);

    while (limit_state != HIGH)
    {
      limit_state = digitalRead(limit_switch);

      digitalWrite(stepper_step, HIGH);
      delayMicroseconds(1000);
      digitalWrite(stepper_step, LOW);
      delayMicroseconds(1000);
    }
  }

  else if (split_1 == "F")
  {
    user_input = split_2.toInt();
    k = true;
    counts = 0; 
   
    while (k==true)
    {
      for ( int i = 0; i < motor_number; i++)
      {
        PID_control(user_input, KP[i], KD[i], KI[i], ENABLE[i], IN1[i], IN2[i], i);
      }
    }
  }

  else if (split_1 == "B")
  {
    user_input = split_2.toInt() * -1;
    k = true;
    counts = 0; 
   
    while ( k==true)
    {
      for ( int i = 0; i < motor_number; i++)
      {
        PID_control(user_input, KP[i], KD[i], KI[i], ENABLE[i], IN1[i], IN2[i], i);
      }
    }
  }
  
  else if (split_1 == "R")
  {
    user_input = split_2.toInt();
    user_input2 = split_2.toInt() * -1;
    k = true;
    counts = 0; 
   
    while ( k==true)
    {
      PID_control(user_input, KP[0], KD[0], KI[0], ENABLE[0], IN1[0], IN2[0], 0);
      PID_control(user_input2, KP[1], KD[1], KI[1], ENABLE[1], IN1[1], IN2[1], 1);
      PID_control(user_input, KP[2], KD[2], KI[2], ENABLE[2], IN1[2], IN2[2], 2);
      PID_control(user_input2, KP[3], KD[3], KI[3], ENABLE[3], IN1[3], IN2[3], 3);
    }
  }
  
  else if (split_1 == "L")
  {
    user_input = split_2.toInt();
    user_input2 = split_2.toInt() * -1;
    k = true;
    counts = 0; 
   
    while ( k==true)
    {
      PID_control(user_input2, KP[0], KD[0], KI[0], ENABLE[0], IN1[0], IN2[0], 0);
      PID_control(user_input, KP[1], KD[1], KI[1], ENABLE[1], IN1[1], IN2[1], 1);
      PID_control(user_input2, KP[2], KD[2], KI[2], ENABLE[2], IN1[2], IN2[2], 2);
      PID_control(user_input, KP[3], KD[3], KI[3], ENABLE[3], IN1[3], IN2[3], 3);
    }
  }

  else if (split_1 == "TR")
  {
    user_input = split_2.toInt();
    user_input2 = split_2.toInt() * -1;
    k = true;
    counts = 0; 
   
    while ( k==true)
    {
      PID_control(user_input, KP[0], KD[0], KI[0], ENABLE[0], IN1[0], IN2[0], 0);
      PID_control(user_input2, KP[1], KD[1], KI[1], ENABLE[1], IN1[1], IN2[1], 1);
      PID_control(user_input2, KP[2], KD[2], KI[2], ENABLE[2], IN1[2], IN2[2], 2);
      PID_control(user_input, KP[3], KD[3], KI[3], ENABLE[3], IN1[3], IN2[3], 3);
    }
  }

  else if (split_1 == "TL")
  {
    user_input = split_2.toInt();
    user_input2 = split_2.toInt() * -1;
    k = true;
    counts = 0; 
   
    while ( k==true)
    {
      PID_control(user_input2, KP[0], KD[0], KI[0], ENABLE[0], IN1[0], IN2[0], 0);
      PID_control(user_input, KP[1], KD[1], KI[1], ENABLE[1], IN1[1], IN2[1], 1);
      PID_control(user_input, KP[2], KD[2], KI[2], ENABLE[2], IN1[2], IN2[2], 2);
      PID_control(user_input2, KP[3], KD[3], KI[3], ENABLE[3], IN1[3], IN2[3], 3);
    }
  }

  else if (split_1 == "PU")
  {
    digitalWrite(stepper_enable, LOW);
    digitalWrite(stepper_dir, HIGH);

    user_input = split_2.toInt();

    for (int j = 0; j < user_input; j++)
    {
      stepper_one_turn();
    }

    digitalWrite(stepper_enable, HIGH);
    split_1 = "";
    split_2 = "";
  }

  else if (split_1 == "PD")
  {
    digitalWrite(stepper_enable, LOW);
    digitalWrite(stepper_dir, LOW);

    user_input = split_2.toInt();

    for (int j = 0; j < user_input; j++)
    {
      stepper_one_turn();
    }

    digitalWrite(stepper_enable, HIGH);
    split_1 = "";
    split_2 = "";
  }

  else if (split_1 == "EM")
  {
    magnets_state = split_2.toInt();

    if (magnets_state == 0)
    {
      digitalWrite(magnets, LOW);
    }

    else if (magnets_state == 1)
    {
      digitalWrite(magnets, HIGH);
    }
  }
  
}

void read_serial_port() 
{
  readString = "";

  if (Serial.available()) 
  {
    delay(10);

    while (Serial.available() > 0) 
    {
      readString += (char)Serial.read();
    }

    comma_index = readString.indexOf(",");
    split_1 = readString.substring(0, comma_index); // "F"
    split_2 = readString.substring(comma_index + 1);  // "360"

    Serial.flush();

  }
}

void stepper_one_turn()
{
  for (int i = 0; i < steps_per_rev; i++) 
  {
    // These four lines result in 1 step:
    digitalWrite(stepper_step, HIGH);
    delayMicroseconds(1000);
    digitalWrite(stepper_step, LOW);
    delayMicroseconds(1000);
  }
}

void PID_control(int user_input, int kp_in, int ki_in , int kd_in, int enable_in, int in1_in, int in2_in, int motor)
{
  int target = map(user_input, 0, 360, 0, 495);

  float kp = kp_in; // decreases rise time
  float ki = ki_in; // eliminates steady-state error
  float kd = kd_in; // decreases overshoot

  long currT = micros();
  float deltaT = ((float) (currT - prevT))/( 1.0e6 );
  prevT = currT;

  int pos = 0;

  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) 
  {
    pos = posi[motor];
  }

  // error
  int e = pos - target;

  // derivative
  float dedt = (e-eprev)/(deltaT);

  // integral
  eintegral = eintegral + e*deltaT;

  // control signal
  float u = kp*e + kd*dedt + ki*eintegral;

  // motor power
  float pwr = fabs(u);

  if ( pwr > pwm_resolution )
  {
    pwr = pwm_resolution;
  }

  // motor direction
  int dir = 1;

  if ( u < 0 )
  {
    dir = -1;
  }

  setMotor(dir, pwr, enable_in, in1_in, in2_in);

  // store previous error
  eprev = e;

  // "Motor: x , Target: x , Pos: x "
  Serial.println(" ");
  Serial.print("Motor: ");
  Serial.print(motor);
  Serial.print(" , ");
  Serial.print("Target: ");
  Serial.print(target);
  Serial.print(" , ");
  Serial.print("Pos: ");
  Serial.print(pos);
  Serial.print(" , ");
  Serial.print("Counts: ");
  Serial.print(counts);
  Serial.print(" , ");
  Serial.print("Error: ");
  Serial.print(eprev);
  Serial.println(" ");

  counts = counts + 1;

  if (counts > 100 )
  {
    for (int i = 0; i < motor_number; i++)
    {
      analogWrite(ENABLE[i], 0);
      posi[i] = 0;
    }
    split_1 = "";
    split_2 = "";
    readString = "";
    k = false;
  }

}

void setMotor(int dir, int pwmVal, int ena, int in1, int in2)
{

  analogWrite(ena, pwmVal); // equivalent for esp32 ledcWrite(ena, pwmVal)

  if(dir == 1)
  {
    digitalWrite(in1,HIGH);
    digitalWrite(in2,LOW);
  }

  else if(dir == -1)
  {
    digitalWrite(in1,LOW);
    digitalWrite(in2,HIGH);
  }

  else
  {
    digitalWrite(in1,LOW);
    digitalWrite(in2,LOW);
  }
}

// Creating a template generates a different version of the function
// depending on the index that is called.
template <int j>
void readEncoder()
{
  int b = digitalRead(ENCB[j]);

  if (j == 1 || j == 2)
  {
    if (b > 0) { posi[j]--; }
    else { posi[j]++; } 
  }

  else
  {
    if ( b > 0) { posi[j]++; }
    else { posi[j]--; }
  }
}