// This alternate version of the code does not require
// atomic.h. Instead, interrupts() and noInterrupts() 
// are used. Please use this code if your 
// platform does not support ATOMIC_BLOCK.

// A class to compute the control signal
class SimplePID
{
  private:
    float kp, kd, ki, umax; // Parameters
    float eintegral; // Storage

  public:
    float eprev;

  // Constructor
  SimplePID() : kp(1), ki(0), kd(0), umax(255), eprev(0.0), eintegral(0.0){}

  // A function to set the parameters
  void setParams(float kpIn, float kdIn, float kiIn, float umaxIn)
  {
    kp = kpIn; kd = kdIn; ki = kiIn; umax = umaxIn;
  }

  // A function to compute the control signal
  int evalu(int value, int target, float deltaT, int &pwr, int &dir)
  {
    // error
    int e = target - value;
  
    // derivative
    float dedt = (e-eprev)/(deltaT);
  
    // integral
    eintegral = eintegral + e*deltaT;
  
    // control signal
    float u = kp*e + kd*dedt + ki*eintegral;
  
    // motor power
    pwr = (int) fabs(u);

    if( pwr > umax )
    {
      pwr = umax;
    }
  
    // motor direction
    dir = 1;

    if(u<0)
    {
      dir = -1;
    }
  
    // store previous error
    eprev = e;

    return eprev;

  }
  
};

// How many motors
#define NMOTORS 4

// Pins
const int enca[] = {2, 3, 18, 19};
const int encb[] = {53, 52, 51, 50};
const int pwm[] = {13, 44, 7, 45};
const int in1[] = {12, 48, 6, 47};
const int in2[] = {11, 46, 5, 49};

// Globals
long prevT = 0;
volatile int posi[] = {0,0,0,0};

String readString;
String split_1;
String split_2;
int comma_index;

bool start = false;

// PID class instances
SimplePID pid[NMOTORS];

void setup() 
{
  Serial.begin(9600);

  for(int k = 0; k < NMOTORS; k++)
  {
    pinMode(enca[k],INPUT);
    pinMode(encb[k],INPUT);
    pinMode(pwm[k],OUTPUT);
    pinMode(in1[k],OUTPUT);
    pinMode(in2[k],OUTPUT);

    pid[k].setParams(10,0.01,1,255);
  }
  
  // Set the interruption pins for the encoders
  attachInterrupt(digitalPinToInterrupt(enca[0]), readEncoder<0>, RISING);
  attachInterrupt(digitalPinToInterrupt(enca[1]), readEncoder<1>, RISING);
  attachInterrupt(digitalPinToInterrupt(enca[2]), readEncoder<2>, RISING);
  attachInterrupt(digitalPinToInterrupt(enca[3]), readEncoder<3>, RISING);
  
}

void loop() 
{

  read_serial_port();

  if (split_1 == "S")
  {
    start = true;
    
    while(start == true)
    {
      // set target position
      int user_input = split_2.toInt();
      user_input = map(user_input, 0, 360, 0, 495);
      int target[NMOTORS];
      for (int i = 0; i < NMOTORS; i++)
      {
        target[i] = user_input;
      }

      // time difference
      long currT = micros();
      float deltaT = ((float) (currT - prevT))/( 1.0e6 );
      prevT = currT;

      // Read the position
      int pos[NMOTORS];
      noInterrupts(); // disable interrupts temporarily while reading
      for(int k = 0; k < NMOTORS; k++)
      {
        pos[k] = posi[k];
      }
      interrupts(); // turn interrupts back on

      // loop through the motors
      for(int k = 0; k < NMOTORS; k++)
      {
        int pwr, dir, e;
        // evaluate the control signal
        e = pid[k].evalu(pos[k],target[k],deltaT,pwr,dir);
        // signal the motor
        setMotor(dir,pwr,pwm[k],in1[k],in2[k]);

        Serial.println(e);

        if ( abs(e) < 5)
        { 
          for (int i = 0; i < 4; i++)
          {
            analogWrite(13, 0);
            analogWrite(44, 0);
            analogWrite(7, 0);
            analogWrite(45, 0);
          }

          noInterrupts(); // disable interrupts temporarily while reading
          for(int k = 0; k < NMOTORS; k++)
          {
            posi[k] = 0;
          }
          interrupts();

          start = false;
        }
      }

      for(int k = 0; k < NMOTORS; k++)
      {
        Serial.print(target[k]);
        Serial.print(" ");
        Serial.print(pos[k]);
        Serial.print(" ");
      }
      Serial.println();
    }

  }

}

void setMotor(int dir, int pwmVal, int pwm, int in1, int in2)
{
  analogWrite(pwm,pwmVal);

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

template <int j>
void readEncoder()
{
  int b = digitalRead(encb[j]);

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