#include <util/atomic.h>

#define motor_number 4

const int ENABLE[] = {13, 44, 7, 45};
const int IN1[] = {12, 48, 6, 47};
const int IN2[] = {11, 46, 5, 49};
const int ENCA[] = {2, 3, 18, 19}; // Pins for interrupt signal
const int ENCB[] = {53, 52, 51, 50};

long prevT = 0;
int posPrev = 0;
volatile int pos_i = 0;
volatile float velocity_i = 0;
volatile long prevT_i = 0;

void setup()
{
    Serial.begin(9600);

    for (int i = 0; i < motor_number; i++)
    {
        pinMode(ENCA[i], INPUT);
        pinMode(ENCB[i], INPUT);
        pinMode(ENABLE[i], OUTPUT);
        pinMode(IN1[i], OUTPUT);
        pinMode(IN2[i], OUTPUT);
    }

    attachInterrupt(digitalPinToInterrupt(ENCA[0]), readEncoder, RISING);

}

void loop()
{
    //int pwr = 255;
    for (int i = 0; i < 256; i++)
    {
        int dir = -1;
        setMotor(dir, i, ENABLE[0], IN1[0], IN2[0]);

        int pos = 0;
        float velocity2 = 0;
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
        {
            pos = pos_i;
            velocity2 = velocity_i;
        }

        long currT = micros();
        float deltaT = ((float) (currT-prevT)/1.0e6);
        float velocity = (pos - posPrev)/deltaT;
        posPrev = pos;
        prevT = currT;

        Serial.print(velocity);
        Serial.println();

        if (i == 255)
        {
            delay(3000);
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

void readEncoder()
{
    int b = digitalRead(ENCB[0]);
    int increment = 0;

    if (b > 0)
    {
        increment++;
    }

    else
    {
        increment--;
    }

    pos_i += increment;

    long currT = micros();
    float deltaT = ((float) (currT-prevT_i)/1.0e6);
    velocity_i = increment/deltaT;
    prevT_i = currT;
    
}