void setup()
{
  Serial.begin(9600);
}

void loop()
{
  // for loop generate ramp from 0 to 255
  for(int digitalInput=0; digitalInput<255 ; digitalInput++)
  {
    
    Serial.print("Digital input: ");  // Text to be printed in Serial monitor
    Serial.println(digitalInput);     // Print Digital input in Serial monitor
    
    analogWrite(11, digitalInput);    // Write Analog output in pin 11
    analogWrite(9, digitalInput);     // Write Analog output in pin 9
    analogWrite(6, digitalInput);     // Write Analog output in pin 6
    
  }
}