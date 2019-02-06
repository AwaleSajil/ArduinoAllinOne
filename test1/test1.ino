#include <SoftwareSerial.h>
SoftwareSerial mySerial(12, 13); 

void setup()
{
  Serial.begin(9600);
  mySerial.begin(9600);

    mySerial.print("Shrey");
}

void loop()
{

 
  
}
