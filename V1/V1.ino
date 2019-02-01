#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#include <LiquidCrystal.h>



#define DHTPIN  2   // Pin which is connected to the DHT sensor
#define LDRPIN A0   //CONcider it for now
#define AMMONIAPIN A1 //concider for now

const int rs = 3, en = 4, d4 = 5, d5 = 6, d6 = 7, d7 = 8;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);






#define DHTTYPE           DHT11     // DHT 11 

DHT_Unified dht(DHTPIN, DHTTYPE);

//struture defination area
struct sensorData
{
  int Temperature = -1;
  int Humidity = -1;
  int LDR = -1;
  int Ammonia = -1;
};


int dataReceived[4];
//dataReceived is an array containing
//dataReceived[0]   --->  Heater
//dataReceived[1]   --->  Fan
//dataReceived[2]   --->  Lights
//dataReceived[3]   --->  Servo



//structure declaration area
sensorData sensorVar;







void rData()
{
  //reads data from python and store it
  if(Serial.available() > 0){
    if(Serial.read() == 'S')
    {
      for (int i = 0; i< 4 ; i++)
      {
        dataReceived[i] = Serial.read();
      }
      if(Serial.read() == 'E'){
        return;
      }
    }
  }
}


void getDHTdata()
{
  // Get temperature event and print its value.
  sensors_event_t event;  
  dht.temperature().getEvent(&event);
  if (isnan(event.temperature)) {
    //cannot get the temperature reading
    sensorVar.Temperature = -1;
  }
  else {
    sensorVar.Temperature = int(event.temperature);
  }

  // Get humidity event and print its value.
  dht.humidity().getEvent(&event);
  if (isnan(event.relative_humidity)) {
    //Error reading humidity
    sensorVar.Humidity = -1;
  }
  else {
    sensorVar.Humidity = int(event.relative_humidity);
  }
}

void getLDRdata()
{
  int data = analogRead(LDRPIN);
  //data manupulation here if any

  //update sensorVal
  sensorVar.LDR = data;
}


void getAmmoniadata()
{
  int data = analogRead(AMMONIAPIN);
  //data manupulation here if any

  //update sensorVal
  sensorVar.Ammonia = data;
}


void showSensorValue()
{
  Serial.print("Temperature: ");
  Serial.println(sensorVar.Temperature);
  Serial.print("Humidity: ");
  Serial.println(sensorVar.Humidity);
  Serial.print("LDR: ");
  Serial.println(sensorVar.LDR);
  Serial.print("Ammonia: ");
  Serial.println(sensorVar.Ammonia);
  Serial.println("--------------------------");
  
}

void showdataReceived()
{
  lcd.setCursor(0, 0);
  lcd.print("H:");
  lcd.print(dataReceived[0]);
  lcd.print(", ");
  lcd.print("F:");
  lcd.print(dataReceived[1]);

  lcd.setCursor(0, 1);
  lcd.print("L:");
  lcd.print(dataReceived[2]);
  lcd.print(", ");
  lcd.print("S:");
  lcd.print(dataReceived[3]);
}


void sendSensorData() {
  Serial.write('S');
  Serial.write((uint8_t *)&sensorVar, sizeof(sensorVar));
  Serial.write('E');
  return;
}


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600); 

  // Initialize DHT device
  dht.begin();

  //initilize lcd display
  lcd.begin(16, 2);
}

void loop() {
  // put your main code here, to run repeatedly:
  delay(1000);
  //get sensor data

  //get humidity and temperature
  getDHTdata();

  //get LDR value pot
  getLDRdata();

  //get ammonia sensor value pot
  getAmmoniadata();

  //display sensor datas in Serial Monitor
//  showSensorValue();


  //send the sensorVar structure to the python / pi
  sendSensorData();
  

  //check for any Serialdata from the python/pi/APP... if yes update the dataReceived
  rData();


  //display the dataReceived in the LCD for debugging
  showdataReceived();

  //use the dataReceived to (take actions) control the appliances // heater, fan, lights

  

}
