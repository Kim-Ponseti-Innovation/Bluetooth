/*
  HS300x - Read Sensors

  This example reads data from the on-board HS300x sensor of the
  Nano 33 BLE Sense Rev2 and prints the temperature and humidity sensor
  values to the Serial Monitor once a second.

  The circuit:
  - Arduino Nano 33 BLE Sense Rev2

  This example code is in the public domain.
*/

#include <Arduino_HS300x.h>
#include <ArduinoBLE.h>

BLEService tempSense("181A");
BLEShortCharacteristic tempCharacteristic = BLEShortCharacteristic("2A6E", BLERead | BLENotify);


//BLEUnsignedCharCharacteristic tempSenseChar("2A19", BLERead | BLENotify);



float old_temp = 0;
float old_hum = 0;
long previousTime = 0;


void setup() {
  Serial.begin(9600);
  while (!Serial);


  pinMode(LED_BUILTIN, OUTPUT);


  if (!BLE.begin()) {
    Serial.println("starting BLE failed!");

    while (1);
  }

  if (!HS300x.begin()) {
    Serial.println("Failed to initialize humidity temperature sensor!");
    while (1);
  }


  BLE.setLocalName("TempertureSensor");
  BLE.setAdvertisedService(tempSense); 
  BLE.addService(tempSense); 
  
  
  BLE.advertise();

  Serial.println("Bluetooth® device active, waiting for connections...");
}

void loop() {

  BLEDevice central = BLE.central();
  // read all the sensor values


  if (central) {
    Serial.print("Connected to central: ");
    
    Serial.println(central.address());
    
    digitalWrite(LED_BUILTIN, HIGH);

    while (central.connected()) {
      long currentTime = (millis());

      if(currentTime - previousTime >= 2000){
        previousTime = currentTime;
        updateTemp();
      }
    }
    digitalWrite(LED_BUILTIN, LOW);
    Serial.print("Disconnected from central: ");
    Serial.println(central.address());
    } 
}

void updateTemp(){

  float temperature = HS300x.readTemperature();
  float humidity    = HS300x.readHumidity();

  if (abs(old_temp - temperature) >= 0.5 || abs(old_hum - humidity) >= 1 ){
  
      Serial.print("Temperature = ");
      Serial.print(temperature);
      Serial.println(" °C");

      Serial.print("Humidity    = ");
      Serial.print(humidity);
      Serial.println(" %");

      Serial.println();
      }

}