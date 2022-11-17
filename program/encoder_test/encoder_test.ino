#define motor_number 4
volatile int posi[] = {0, 0, 0, 0}; // add more for more motors

const int ENABLE[] = {13, 44, 7, 45};
const int IN1[] = {12, 48, 6, 47};
const int IN2[] = {11, 46, 5, 49};
const int ENCA[] = {2, 3, 18, 19}; // Pins for interrupt signal
const int ENCB[] = {53, 52, 51, 50};

// User input and communication variables
String readString;
String split_1;
String split_2;
int comma_index;
int dir;
int pwm = 100;

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
  
}

void loop()
{
    read_serial_port();

    if (split_1 == "0") 
    {
        dir = split_2.toInt();
        setMotor(dir,pwm,ENABLE[0], IN1[0], IN2[0]);
    }

    else if (split_1 == "1") 
    {
        dir = split_2.toInt();
        setMotor(dir,pwm,ENABLE[1], IN1[1], IN2[1]);
    }

    else if (split_1 == "2") 
    {
        dir = split_2.toInt();
        setMotor(dir,pwm,ENABLE[2], IN1[2], IN2[2]);
    }

    else if (split_1 == "3") 
    {
        dir = split_2.toInt();
        setMotor(dir,pwm,ENABLE[3], IN1[3], IN2[3]);
    }

    else if (split_1 == "S") 
    {
        dir = split_2.toInt();

        for (int i = 0; i < 4; i++)
        {
            setMotor(dir,0,ENABLE[i], IN1[i], IN2[i]);
        }
    }

    Serial.print(posi[0]);
    Serial.print(", ");
    Serial.print(posi[1]);
    Serial.print(", ");
    Serial.print(posi[2]);
    Serial.print(", ");
    Serial.println(posi[3]);
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