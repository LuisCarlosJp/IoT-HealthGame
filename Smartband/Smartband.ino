
//Carrega a biblioteca Wire
#include<Wire.h>
#include "MAX30105.h"
#include "heartRate.h"
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>

// See the following for generating UUIDs:
// https://www.uuidgenerator.net/

#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"


MAX30105 particleSensor;
const byte RATE_SIZE = 4; //Increase this for more averaging. 4 is good.
byte rates[RATE_SIZE]; //Array of heart rates
byte rateSpot = 0;
long lastBeat = 0; //Time at which the last beat occurred
float beatsPerMinute;
int beatAvg;
long irValue = 0;

//Endereco I2C do MPU6050
const int MPU=0x68;  
int16_t accelX, accelY, accelZ;

const float threshold = 1.0; // Adjust this threshold for step detection sensitivity
const int bufferLength = 15; // Number of accelerometer readings in the buffer
float buffer[bufferLength];
int bufferIndex = 0;

int stepCount = 0;
bool stepDetected = false;
unsigned long lastStepTime = 0;
const unsigned long debounceDelay = 300; // Debounce delay in milliseconds

float accMagnitudePrev = 0;

BLECharacteristic *pStepDataCharacteristic;

void setup() {
  Wire.begin();
  Serial.begin(115200);
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);   //PWR_MGMT_1 register
   
  //Inicializa o MPU-6050
  Wire.write(0); 
  Wire.endTransmission(true);

  // Initialize sensor
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
  {
    Serial.println("MAX30105 was not found. Please check wiring/power. ");
    while (1);
  }
  Serial.println("Place your index finger on the sensor with steady pressure.");

  particleSensor.setup(); //Configure sensor with default settings
  particleSensor.setPulseAmplitudeRed(0x0A); //Turn Red LED to low to indicate sensor is running
  particleSensor.setPulseAmplitudeGreen(0); //Turn off Green LED

  BLEDevice::init("Smartband ESP32");
  BLEServer *pServer = BLEDevice::createServer();
  
  BLEService *pService = pServer->createService(SERVICE_UUID);
  
  // Criando uma característica para os dados de passos
  pStepDataCharacteristic = pService->createCharacteristic(
      CHARACTERISTIC_UUID,
      BLECharacteristic::PROPERTY_READ | 
      BLECharacteristic::PROPERTY_NOTIFY  // Permitindo leitura e notificação
  );
  
  // Adicionando o descritor BLE2902 para permitir notificações
  pStepDataCharacteristic->addDescriptor(new BLE2902());

  pService->start();
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();
  Serial.println("BLE device is ready to be connected");
}

void loop() {
  // put your main code here, to run repeatedly:
  HRSensor();
  readAccelerometerData();
  detectStep();
}


void readAccelerometerData(){
  Wire.beginTransmission(MPU);
  Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
    //Solicita os dados do sensor
  Wire.requestFrom(MPU, 6, true);

  accelX=Wire.read()<<8|Wire.read();  //0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)     
  accelY=Wire.read()<<8|Wire.read();  //0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  accelZ=Wire.read()<<8|Wire.read();  //0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
}

void display(){
  Serial.println("Step detected!");
  Serial.print("Step count: ");
  Serial.println(stepCount);
  Serial.print("IR=");
  Serial.print(irValue);
  Serial.print(", BPM=");
  Serial.print(beatsPerMinute);
  Serial.print(", Avg BPM=");
  Serial.println(beatAvg);
  // Create a string containing the step count data
  String dados = "Passos: " + String(stepCount) + ", FC: " + String(beatAvg);
   // Update the characteristic value
  pStepDataCharacteristic->setValue(dados.c_str());
  pStepDataCharacteristic->notify();
}


void detectStep(){
  float accX = accelX / 16384.8;
  float accY = accelY / 16384.8;
  float accZ = accelZ / 16384.8;

  float accelerationMagnitude = sqrt(accX * accX +
                                     accY * accY +
                                     accZ * accZ);

  buffer[bufferIndex] = accelerationMagnitude;
  bufferIndex = (bufferIndex + 1) % bufferLength;

  // Detect a step if the current magnitude is greater than the average of the buffer by the threshold
  float avgMagnitude = 0;
  for (int i = 0; i < bufferLength; i++) {
    avgMagnitude += buffer[i];
  }
  avgMagnitude /= bufferLength;

  unsigned long currentMillis = millis();

  if (accelerationMagnitude > (avgMagnitude + threshold)) {
    if (!stepDetected && (currentMillis - lastStepTime) > debounceDelay) {
      stepCount++;
      stepDetected = true;
      lastStepTime = currentMillis;
      display();
      
    }
  } else {
    stepDetected = false;
  }
}

void HRSensor(){
   long irValue = particleSensor.getIR();

  if (checkForBeat(irValue) == true)
  {
    //We sensed a beat!
    long delta = millis() - lastBeat;
    lastBeat = millis();

    beatsPerMinute = 60 / (delta / 1000.0);

    if (beatsPerMinute < 255 && beatsPerMinute > 20)
    {
      rates[rateSpot++] = (byte)beatsPerMinute; //Store this reading in the array
      rateSpot %= RATE_SIZE; //Wrap variable

      //Take average of readings
      beatAvg = 0;
      for (byte x = 0 ; x < RATE_SIZE ; x++)
        beatAvg += rates[x];
      beatAvg /= RATE_SIZE;
    }
  }
}
