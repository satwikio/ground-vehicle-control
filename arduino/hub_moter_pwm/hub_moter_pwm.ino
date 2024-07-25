const int pwmPin = 3;
int pwm = 90;
void setup()
{
  Serial.begin(9600);
}

void loop()
{

    analogWrite(pwmPin, pwm);     // Write Analog output in pin 3 
    }
