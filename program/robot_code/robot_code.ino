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

const int pwm_resolution_M1 = 255; // The max duty cycle value for the pwm signal
const int pwm_resolution_M2 = 255;
const int pwm_resolution_M3 = 255;
const int pwm_resolution_M4 = 255;

// PID values for each motor as an array
// control_array[] = {kp_m1, kp_m2, kp_m3...};
// start values: 3.5, 0.02, 0.25
// safe values: 1, 0, 0
const int KP[] = {10, 10, 10, 10}; // decreases rise time
const int KI[] = {0.01, 0.01, 0.01, 0.01}; // eliminates steady-state error
const int KD[] = {1, 1, 1, 1}; // decreases overshoot

const int permissible_error = 5;

// PID variables used in function
long prevT_M1 = 0;
float eprev_M1 = 0;
float eintegral_M1 = 0;
bool ignore_M1 = false;

long prevT_M2 = 0;
float eprev_M2 = 0;
float eintegral_M2 = 0;
bool ignore_M2 = false;


long prevT_M3 = 0;
float eprev_M3 = 0;
float eintegral_M3 = 0;
bool ignore_M3 = false;


long prevT_M4 = 0;
float eprev_M4 = 0;
float eintegral_M4 = 0;
bool ignore_M4 = false;


bool k = false, k1 = false, k2 = false, k3 = false, k4 = false;

int counts = 0;

long prevT = 0;
volatile int posi[] = {0, 0, 0, 0}; // add more for more motors
float eprev[] = {0,0,0,0};
float eintegral[] = {0,0,0,0};

// User input and communication variables
String readString;
int user_input, user_input2;
String split_1;
String split_2;
int comma_index;

// Set up for stepper motor and limit switch
const int stepper_step = 24;
const int stepper_dir = 22;
const int stepper_enable = 27;

const float step_angle = 1.8; // from stepper data sheet
const int steps_per_rev = 360; // step_angle

const int limit_switch = 39;
int limit_state = 0;

const int magnets = 31;
int magnets_state = 0;

int index1, index2, index3;
String split_2_1, split_2_2, split_2_3, split_2_4;
int M;
float kp, ki, kd; 

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
  k = false;

  // Initial stepper configuration
  digitalWrite(stepper_enable, HIGH);

  read_serial_port();

  limit_state = digitalRead(limit_switch);

  if (split_1 == "PZ")
  {    
    digitalWrite(stepper_enable, LOW);
    digitalWrite(stepper_dir, LOW);

    while (limit_state != HIGH)
    {
      limit_state = digitalRead(limit_switch);

      digitalWrite(stepper_step, HIGH);
      delayMicroseconds(1000);
      digitalWrite(stepper_step, LOW);
      delayMicroseconds(1000);
    }

    split_1 = "";
    split_2 = "";
    readString = "";
    ready_notification();
  }

  else if (split_1 == "F")
  {
    user_input = split_2.toInt();

    for (int i = 0; i < motor_number; i++)
    {
      posi[i] = 0;
    }
    
    k1 = true, k2 = true, k3 = true, k4 = true;
    ignore_M1 = false, ignore_M2 = false, ignore_M3 = false, ignore_M4 = false;
   
    while ( k1==true || k2==true || k3==true || k4==true)
    {
      if (ignore_M1 == false)
      {
        PID_M1(user_input, KP[0], KI[0], KD[0], ENABLE[0], IN1[0], IN2[0]);
        delayMicroseconds(500);
      }
      if (ignore_M2 == false)
      {
        PID_M2(user_input, KP[1], KI[1], KD[1], ENABLE[1], IN1[1], IN2[1]);
        delayMicroseconds(500);
      }
      if (ignore_M3 == false)
      {
        PID_M3(user_input, KP[2], KI[2], KD[2], ENABLE[2], IN1[2], IN2[2]);
        delayMicroseconds(500);
      }
      if (ignore_M4 == false)
      {
        PID_M4(user_input, KP[3], KI[3], KD[3], ENABLE[3], IN1[3], IN2[3]);
        delayMicroseconds(500);
      }
    }

    split_1 = "";
    split_2 = "";
    readString = "";

    ready_notification();
  }

  else if (split_1 == "B")
  {
    user_input = split_2.toInt() * -1;

    for (int i = 0; i < motor_number; i++)
    {
      posi[i] = 0;
    }

    k1 = true, k2 = true, k3 = true, k4 = true;
    ignore_M1 = false, ignore_M2 = false, ignore_M3 = false, ignore_M4 = false;
   
    while ( k1==true || k2==true || k3==true || k4==true)
    {
      if (ignore_M1 == false)
      {
        PID_M1(user_input, KP[0], KI[0], KD[0], ENABLE[0], IN1[0], IN2[0]);
        delayMicroseconds(500);
      }
      if (ignore_M2 == false)
      {
        PID_M2(user_input, KP[1], KI[1], KD[1], ENABLE[1], IN1[1], IN2[1]);
        delayMicroseconds(500);
      }
      if (ignore_M3 == false)
      {
        PID_M3(user_input, KP[2], KI[2], KD[2], ENABLE[2], IN1[2], IN2[2]);
        delayMicroseconds(500);
      }
      if (ignore_M4 == false)
      {
        PID_M4(user_input, KP[3], KI[3], KD[3], ENABLE[3], IN1[3], IN2[3]);
        delayMicroseconds(500);
      }
    }
    split_1 = "";
    split_2 = "";
    readString = "";
    ready_notification();
  }
  
  else if (split_1 == "R") // 1 and 2 fail
  {
    user_input = split_2.toInt();
    user_input2 = split_2.toInt() * -1;

    for (int i = 0; i < motor_number; i++)
    {
      posi[i] = 0;
    }

    k1 = true, k2 = true, k3 = true, k4 = true;
    ignore_M1 = false, ignore_M2 = false, ignore_M3 = false, ignore_M4 = false;
   
    while ( k1==true || k2==true || k3==true || k4==true)
    {
      if (ignore_M1 == false)
      {
        PID_M1(user_input, KP[0], KI[0], KD[0], ENABLE[0], IN1[0], IN2[0]);
      }
      if (ignore_M2 == false)
      {
        PID_M2(user_input2, KP[1], KI[1], KD[1], ENABLE[1], IN1[1], IN2[1]);
      }
      if (ignore_M3 == false)
      {
        PID_M3(user_input, KP[2], KI[2], KD[2], ENABLE[2], IN1[2], IN2[2]);
      }
      if (ignore_M4 == false)
      {
        PID_M4(user_input2, KP[3], KI[3], KD[3], ENABLE[3], IN1[3], IN2[3]);
      }
    }
    Serial.print("Final Errors: ");
    Serial.print(eprev_M1);
    Serial.print(", ");
    Serial.print(eprev_M2);
    Serial.print(", ");
    Serial.print(eprev_M3);
    Serial.print(", ");
    Serial.println(eprev_M4);
  }
  
  else if (split_1 == "L") // 3 and 4 fail
  {
    user_input = split_2.toInt();
    user_input2 = split_2.toInt() * -1;

    for (int i = 0; i < motor_number; i++)
    {
      posi[i] = 0;
    }
    
    k1 = true, k2 = true, k3 = true, k4 = true;
    ignore_M1 = false, ignore_M2 = false, ignore_M3 = false, ignore_M4 = false;
   
    while ( k1==true || k2==true || k3==true || k4==true)
    {
      if (ignore_M1 == false)
      {
        PID_M1(user_input2, KP[0], KI[0], KD[0], ENABLE[0], IN1[0], IN2[0]);
      }
      if (ignore_M2 == false)
      {
        PID_M2(user_input, KP[1], KI[1], KD[1], ENABLE[1], IN1[1], IN2[1]);
      }
      if (ignore_M3 == false)
      {
        PID_M3(user_input2, KP[2], KI[2], KD[2], ENABLE[2], IN1[2], IN2[2]);
      }
      if (ignore_M4 == false)
      {
        PID_M4(user_input, KP[3], KI[3], KD[3], ENABLE[3], IN1[3], IN2[3]);
      }
    }
    Serial.print("\nFinal Errors: ");
    Serial.print(eprev_M1);
    Serial.print(", ");
    Serial.print(eprev_M2);
    Serial.print(", ");
    Serial.print(eprev_M3);
    Serial.print(", ");
    Serial.println(eprev_M4);
  }

  else if (split_1 == "TL")
  {
    user_input = split_2.toInt();
    user_input2 = split_2.toInt() * -1;

    for (int i = 0; i < motor_number; i++)
    {
      posi[i] = 0;
    }

    k1 = true, k2 = true, k3 = true, k4 = true;
    ignore_M1 = false, ignore_M2 = false, ignore_M3 = false, ignore_M4 = false;
    
    while ( k1==true || k2==true || k3==true || k4==true)
    {
      if (ignore_M1 == false)
      {
        PID_M1(user_input, KP[0], KI[0], KD[0], ENABLE[0], IN1[0], IN2[0]);
      }
      if (ignore_M2 == false)
      {
        PID_M2(user_input2, KP[1], KI[1], KD[1], ENABLE[1], IN1[1], IN2[1]);
      }
      if (ignore_M3 == false)
      {
        PID_M3(user_input2, KP[2], KI[2], KD[2], ENABLE[2], IN1[2], IN2[2]);
      }
      if (ignore_M4 == false)
      {
        PID_M4(user_input, KP[3], KI[3], KD[3], ENABLE[3], IN1[3], IN2[3]);
      }
    }
  }

  else if (split_1 == "TR")
  {
    user_input = split_2.toInt();
    user_input2 = split_2.toInt() * -1;

    for (int i = 0; i < motor_number; i++)
    {
      posi[i] = 0;
    }
    
    k1 = true, k2 = true, k3 = true, k4 = true;
    ignore_M1 = false, ignore_M2 = false, ignore_M3 = false, ignore_M4 = false;
   
    while ( k1==true || k2==true || k3==true || k4==true)
    {
      if (ignore_M1 == false)
      {
        PID_M1(user_input2, KP[0], KI[0], KD[0], ENABLE[0], IN1[0], IN2[0]);
      }
      if (ignore_M2 == false)
      {
        PID_M2(user_input, KP[1], KI[1], KD[1], ENABLE[1], IN1[1], IN2[1]);
      }
      if (ignore_M3 == false)
      {
        PID_M3(user_input, KP[2], KI[2], KD[2], ENABLE[2], IN1[2], IN2[2]);
      }
      if (ignore_M4 == false)
      {
        PID_M4(user_input2, KP[3], KI[3], KD[3], ENABLE[3], IN1[3], IN2[3]);
      }
    }
  }

  else if (split_1 == "PU")
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
    readString = "";
    ready_notification();
  }

  else if (split_1 == "PD")
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
    readString = "";
    ready_notification();
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

    split_1 = "";
    split_2 = "";
    readString = "";
    ready_notification();
  }

  else if (split_1 == "T1")
  {
    user_input = split_2.toInt();
    k1 = true;
    ignore_M1 = false;
   
    while (k1==true)
    {
      if (ignore_M1 == false)
      {
        PID_M1(user_input, KP[0], KI[0], KD[0], ENABLE[0], IN1[0], IN2[0]);
      }
    }
  }

  else if (split_1 == "T2")
  {
    user_input = split_2.toInt();
    k2 = true;
   
    while (k2==true)
    {
      PID_M2(user_input, KP[1], KI[1], KD[1], ENABLE[1], IN1[1], IN2[1]);
    }
  }

  else if (split_1 == "T3")
  {
    user_input = split_2.toInt();
    k3 = true;
   
    while (k3==true)
    {
      PID_M3(user_input, KP[2], KI[2], KD[2], ENABLE[2], IN1[2], IN2[2]);
    }
  }

  else if (split_1 == "T4")
  {
    user_input = split_2.toInt();
    k4 = true;
   
    while (k4==true)
    {
      PID_M4(user_input, KP[3], KI[3], KD[3], ENABLE[3], IN1[3], IN2[3]);
    }
  }

  else if (split_1 == "M")
  {
    // M,F#5
    index1 = split_2.indexOf("#");
    index2 = split_2.indexOf("#", index1 + 1);

    split_2_1 = split_2.substring(0, index1);
    split_2_2 = split_2.substring(index1 + 1, index2);

    if (split_2_1 == "F")
    {
      int direction = 1;
      int pwm_value = 255;
      int stop_pwm = 0;
      int stop_time = split_2_2.toInt();

      for (int i = 0; i < 4; i++)
      {
        setMotor(direction, pwm_value, ENABLE[i], IN1[i], IN2[i]);
      }

      delay(stop_time);

      for (int i = 0; i < 4; i++)
      {
        setMotor(direction, stop_pwm, ENABLE[i], IN1[i], IN2[i]);
      }

      split_1 = "";
      ready_notification();
    }

    else if (split_2_1 == "B")
    {
      int direction = -1;
      int pwm_value = 255;
      int stop_pwm = 0;
      int stop_time = split_2_2.toInt();

      for (int i = 0; i < 4; i++)
      {
        setMotor(direction, pwm_value, ENABLE[i], IN1[i], IN2[i]);
      }

      delay(stop_time);

      for (int i = 0; i < 4; i++)
      {
        setMotor(direction, stop_pwm, ENABLE[i], IN1[i], IN2[i]);
      }

      split_1 = "";
      ready_notification();
    }

    else if (split_2_1 == "R")
    {
      int direction_1 = 1;
      int direction_2 = -1;
      int pwm_value = 255;
      int stop_pwm = 0;
      int stop_time = split_2_2.toInt();

      setMotor(direction_1, pwm_value, ENABLE[0], IN1[0], IN2[0]);
      setMotor(direction_2, pwm_value, ENABLE[1], IN1[1], IN2[1]);
      setMotor(direction_1, pwm_value, ENABLE[2], IN1[2], IN2[2]);
      setMotor(direction_2, pwm_value, ENABLE[3], IN1[3], IN2[3]);

      delay(stop_time);

      for (int i = 0; i < 4; i++)
      {
        setMotor(direction_1, stop_pwm, ENABLE[i], IN1[i], IN2[i]);
      }

      split_1 = "";
      ready_notification();
    }

    else if (split_2_1 == "L")
    {
      int direction_1 = 1;
      int direction_2 = -1;
      int pwm_value = 255;
      int stop_pwm = 0;
      int stop_time = split_2_2.toInt();

      setMotor(direction_2, pwm_value, ENABLE[0], IN1[0], IN2[0]);
      setMotor(direction_1, pwm_value, ENABLE[1], IN1[1], IN2[1]);
      setMotor(direction_2, pwm_value, ENABLE[2], IN1[2], IN2[2]);
      setMotor(direction_1, pwm_value, ENABLE[3], IN1[3], IN2[3]);

      delay(stop_time);

      for (int i = 0; i < 4; i++)
      {
        setMotor(direction_1, stop_pwm, ENABLE[i], IN1[i], IN2[i]);
      }

      split_1 = "";
      ready_notification();
    }

    else if (split_2_1 == "CW")
    {
      int direction_1 = 1;
      int direction_2 = -1;
      int pwm_value = 255;
      int stop_pwm = 0;
      int stop_time = split_2_2.toInt();

      setMotor(direction_2, pwm_value, ENABLE[0], IN1[0], IN2[0]);
      setMotor(direction_1, pwm_value, ENABLE[1], IN1[1], IN2[1]);
      setMotor(direction_1, pwm_value, ENABLE[2], IN1[2], IN2[2]);
      setMotor(direction_2, pwm_value, ENABLE[3], IN1[3], IN2[3]);

      delay(stop_time);

      for (int i = 0; i < 4; i++)
      {
        setMotor(direction_1, stop_pwm, ENABLE[i], IN1[i], IN2[i]);
      }

      split_1 = "";
      ready_notification();
    }

    else if (split_2_1 == "CCW")
    {
      int direction_1 = 1;
      int direction_2 = -1;
      int pwm_value = 255;
      int stop_pwm = 0;
      int stop_time = split_2_2.toInt();

      setMotor(direction_1, pwm_value, ENABLE[0], IN1[0], IN2[0]);
      setMotor(direction_2, pwm_value, ENABLE[1], IN1[1], IN2[1]);
      setMotor(direction_2, pwm_value, ENABLE[2], IN1[2], IN2[2]);
      setMotor(direction_1, pwm_value, ENABLE[3], IN1[3], IN2[3]);

      delay(stop_time);

      for (int i = 0; i < 4; i++)
      {
        setMotor(direction_1, stop_pwm, ENABLE[i], IN1[i], IN2[i]);
      }

      split_1 = "";
      ready_notification();
    }

    split_1 = "";
    split_2 = "";
    readString = "";
  }

  else if (split_1 == "" && split_2 == "" && readString == "")
  {
    Serial.println("Waiting");
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
      readString += Serial.readStringUntil('\n');
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

void PID_M1(int user_input, int kp_in, int ki_in , int kd_in, int enable_in, int in1_in, int in2_in)
{
  if (ignore_M1 == true)
  {
    analogWrite(ENABLE[0], 0);
    k1 = false;
  }

  else if (ignore_M1 == false)
  {
    int target = map(user_input, 0, 360, 0, 495);

    float kp = kp_in; // decreases rise time
    float ki = ki_in; // eliminates steady-state error
    float kd = kd_in; // decreases overshoot

    long currT = micros();
    float deltaT = ((float) (currT - prevT_M1))/( 1.0e6 );

    prevT_M1 = currT;

    int pos = 0;

    noInterrupts();

    pos = posi[0];

    interrupts(); 

    // error
    int e = pos - target;

    // derivative
    float dedt = (e-eprev_M1)/(deltaT);

    // integral
    eintegral_M1 = eintegral_M1 + e*deltaT;

    // control signal
    float u = kp*e + kd*dedt + ki*eintegral_M1;

    // motor power
    float pwr = fabs(u);

    if ( pwr > pwm_resolution_M1 )
    {
      pwr = pwm_resolution_M1;
    }

    // motor direction
    int dir = -1;

    if ( u < 0 )
    {
      dir = 1;
    }

    setMotor(dir, pwr, enable_in, in1_in, in2_in);

    // store previous error
    eprev_M1 = e;

    /* Serial.print("M1: ");
    Serial.println(eprev_M1); */

    if (abs(e) <= permissible_error)
    {  
      analogWrite(ENABLE[0], 0);
      posi[0] = 0;

      split_1 = "";
      split_2 = "";
      readString = "";

      k1 = false;
      //k = false;
      ignore_M1 = true;

      prevT_M1 = 0;
      eprev_M1 = 0;
      eintegral_M1 = 0;
    }

  }

}

void PID_M2(int user_input, int kp_in, int ki_in , int kd_in, int enable_in, int in1_in, int in2_in)
{
  if (ignore_M2 == true)
  {
    analogWrite(ENABLE[1], 0);
    k2 = false;
  }

  else if (ignore_M2 == false)
  {
    int target = map(user_input, 0, 360, 0, 495);

    float kp = kp_in; // decreases rise time
    float ki = ki_in; // eliminates steady-state error
    float kd = kd_in; // decreases overshoot

    long currT = micros();
    float deltaT = ((float) (currT - prevT_M2))/( 1.0e6 );
    prevT_M2 = currT;

    int pos = 0;

    noInterrupts();

    pos = posi[1];
    
    interrupts(); 

    // error
    int e = pos - target;

    // derivative
    float dedt = (e-eprev_M2)/(deltaT);

    // integral
    eintegral_M2 = eintegral_M2 + e*deltaT;

    // control signal
    float u = kp*e + kd*dedt + ki*eintegral_M2;

    // motor power
    float pwr = fabs(u);

    if ( pwr > pwm_resolution_M2 )
    {
      pwr = pwm_resolution_M2;
    }

    // motor direction
    int dir = -1;

    if ( u < 0 )
    {
      dir = 1;
    }

    setMotor(dir, pwr, enable_in, in1_in, in2_in);

    // store previous error
    eprev_M2 = e;

    /* Serial.print("M2: ");
    Serial.println(eprev_M2); */

    if (abs(e) <= permissible_error)
    {  
      analogWrite(ENABLE[1], 0);
      posi[1] = 0;

      split_1 = "";
      split_2 = "";
      readString = "";

      k2 = false;
      //k = false;
      ignore_M2 = true;

      prevT_M2 = 0;
      eprev_M2 = 0;
      eintegral_M2 = 0;
    }
  }

}

void PID_M3(int user_input, int kp_in, int ki_in , int kd_in, int enable_in, int in1_in, int in2_in)
{
  if (ignore_M3 == true)
  {
    analogWrite(ENABLE[2], 0);
    k3 = false;
  }

  else if (ignore_M3 == false)
  {
    int target = map(user_input, 0, 360, 0, 495);

    float kp = kp_in; // decreases rise time
    float ki = ki_in; // eliminates steady-state error
    float kd = kd_in; // decreases overshoot

    long currT = micros();
    float deltaT = ((float) (currT - prevT_M3))/( 1.0e6 );
    prevT_M3 = currT;

    int pos = 0;

    noInterrupts();

    pos = posi[2];
    
    interrupts(); 
    // error
    int e = pos - target;

    // derivative
    float dedt = (e-eprev_M3)/(deltaT);

    // integral
    eintegral_M3 = eintegral_M3 + e*deltaT;

    // control signal
    float u = kp*e + kd*dedt + ki*eintegral_M3;

    // motor power
    float pwr = fabs(u);

    if ( pwr > pwm_resolution_M3 )
    {
      pwr = pwm_resolution_M3;
    }

    // motor direction
    int dir = -1;

    if ( u < 0 )
    {
      dir = 1;
    }

    setMotor(dir, pwr, enable_in, in1_in, in2_in);

    // store previous error
    eprev_M3 = e;

    /* Serial.print("M3: ");
    Serial.println(eprev_M3); */

    if (abs(e) <= permissible_error)
    {  
      analogWrite(ENABLE[2], 0);
      posi[2] = 0;

      split_1 = "";
      split_2 = "";
      readString = "";

      k3 = false;
      //k = false;
      ignore_M3 = true;

      prevT_M3 = 0;
      eprev_M3 = 0;
      eintegral_M3 = 0;
    }
  }
}

void PID_M4(int user_input, int kp_in, int ki_in , int kd_in, int enable_in, int in1_in, int in2_in)
{
  if (ignore_M4 == true)
  {
    analogWrite(ENABLE[3], 0);
    k4 = false;
  }

  else if (ignore_M4 == false)
  {
    int target = map(user_input, 0, 360, 0, 495);

    float kp = kp_in; // decreases rise time
    float ki = ki_in; // eliminates steady-state error
    float kd = kd_in; // decreases overshoot

    long currT = micros();
    float deltaT = ((float) (currT - prevT_M4))/( 1.0e6 );
    prevT_M4 = currT;

    int pos = 0;

    noInterrupts();

    pos = posi[3];
    
    interrupts(); 

    // error
    int e = pos - target;

    // derivative
    float dedt = (e-eprev_M4)/(deltaT);

    // integral
    eintegral_M4 = eintegral_M4 + e*deltaT;

    // control signal
    float u = kp*e + kd*dedt + ki*eintegral_M4;

    // motor power
    float pwr = fabs(u);

    if ( pwr > pwm_resolution_M4 )
    {
      pwr = pwm_resolution_M4;
    }

    // motor direction
    int dir = -1;

    if ( u < 0 )
    {
      dir = 1;
    }

    setMotor(dir, pwr, enable_in, in1_in, in2_in);

    // store previous error
    eprev_M4 = e;

    /* Serial.print("M4: ");
    Serial.println(eprev_M4); */

    if ( abs(e) <= permissible_error)
    {  
      analogWrite(ENABLE[3], 0);
      posi[3] = 0;

      split_1 = "";
      split_2 = "";
      readString = "";

      k4 = false;
      //k = false;
      ignore_M4 = true;

      prevT_M4 = 0;
      eprev_M4 = 0;
      eintegral_M4 = 0;
    }
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

  if (j == 0 || j == 3)
  {
    if (b > 0) { posi[j]--; }
    else { posi[j]++; } 
  }

  else if(j == 1 || j == 2)
  {
    if ( b > 0) { posi[j]++; }
    else { posi[j]--; }
  }
}

void ready_notification()
{
  Serial.println("Ready");
  delay(1500);
}
