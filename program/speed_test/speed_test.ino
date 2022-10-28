/*
  Gearmotor Rotary Encoder Test
  motor-encoder-rpm.ino
  Read pulses from motor encoder to calculate speed
  Control speed with potentiometer
  Displays results on Serial Monitor
  Use Cytron MD10C PWM motor controller
  DroneBot Workshop 2019
  https://dronebotworkshop.com
*/
 
// Motor encoder output pulse per rotation (change as required)
#define ENC_COUNT_REV 495
#define PI 3.1415926535897932384626433832795

#define motor_number 4
 
// Encoder output to Arduino Interrupt pin
#define ENCA 2 //3,18,19 

const int ENABLE[] = {13, 44, 7, 45};
const int IN1[] = {12, 48, 6, 47};
const int IN2[] = {11, 46, 5, 49};
 
// Analog pin for potentiometer
int speedcontrol = 0;
 
// Pulse count from encoder
volatile long encoderValue = 0;
 
// One-second interval for measurements
int interval = 1000;
 
// Counters for milliseconds during interval
long previousMillis = 0;
long currentMillis = 0;
 
// Variable for RPM measuerment
int rpm = 0, w = 0, v = 0;
 
// Variable for PWM motor speed output
int motorPwm = 0;
 
void setup()
{
  // Setup Serial Monitor
  Serial.begin(9600); 
  
  // Set encoder as input with internal pullup  
  pinMode(ENCA,INPUT);
 
  // Set PWM and DIR connections as outputs
  for(int i = 0; i < motor_number; i++)
  {
    pinMode(ENABLE[i],OUTPUT);
    pinMode(IN1[i],OUTPUT);
    pinMode(IN2[i],OUTPUT);
  }
  
  // Attach interrupt 
  attachInterrupt(digitalPinToInterrupt(ENCA), updateEncoder, RISING);
  
  // Setup initial values for timer
  previousMillis = millis();
}
 
void loop()
{
    for (int j = 0; j <= 255; j++)
    {
        // Control motor with loop
        motorPwm = j;
        
        // Write PWM to controller
        setMotor(1, motorPwm, ENABLE[0], IN1[0], IN2[0]);
        
        // Update RPM value every second
        currentMillis = millis();

        if (currentMillis - previousMillis > interval) 
        {
            previousMillis = currentMillis;
        }
    
        // Calculate RPM, angular velocity and speed
        rpm = (float)(encoderValue * 60 / ENC_COUNT_REV);

        w = (rpm/60) * 2 * PI;

        v = w * 0.06; // wheel of 60mm diamater in meters 
    
        // Only update display when there is a reading
        if (motorPwm > 0 || rpm > 0) 
        {
        Serial.print(motorPwm); // PWM for motor
        Serial.print(" , ");
        Serial.println(v); // Calculated rpm
        }

        if (j == 255)
        {
            delay(2000);
            setMotor(0, 0, ENABLE[0], IN1[0], IN2[0]);
            Serial.end();
        }
        
        
    }

    encoderValue = 0;
}
 
void updateEncoder()
{
  // Increment value for each pulse from encoder
  encoderValue++;
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