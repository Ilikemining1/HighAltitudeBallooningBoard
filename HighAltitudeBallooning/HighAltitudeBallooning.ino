#include <Wire.h> // Library Dependencies
#include <SPI.h>

#include <SparkFun_u-blox_GNSS_Arduino_Library.h> // Sensor Libraries
#include <Adafruit_Sensor.h>
#include <Adafruit_BME680.h>
#include <INA219.h>
#include <ICM20948_WE.h>

#include <SD.h> // Telemetry and Datalogging
#include <RadioLib.h>

INA219 sysBus;
Adafruit_BME680 bme;
SFE_UBLOX_GNSS gnss;
//SX1276 vhf = new Module(8, 14, 6, 15);
//SX1276 uhf = new Module(9, 20, 7, 21);


void setup() {
  Serial.begin(115200);  // Initialize USB UART for debug
  Serial.printf("HAB PCB rev 1\nInitializing system!\n");

  SPI1.setRX(12); // Configure SPI1 peripheral for SD card
  SPI1.setCS(13);
  SPI1.setSCK(10);
  SPI1.setTX(11);

  Wire.begin(); // Start I2C bus at 400kHz
  Wire.setClock(400000);

  SD.begin(13, SPI1);

  if (!gnss.begin() { // GNSS Initialization
    Serial.println("GNSS initialization failed!");
    while (1);
  }

  if (sysBus.begin()) { // INA219 Initialization
    Serial.println("INA219 initialization failed!");
    while (1);
  }
  
  sysBus.configure(INA219::RANGE_16V, INA219::GAIN_1_40MV, INA219::ADC_64SAMP, INA219::ADC_64SAMP, INA219::CONT_SH_BUS); // Configure INA219 range settings
  sysBus.calibrate(0.027, 0.04, 6, 2); // 27mOhm shunt, 40mV max shunt voltage, 6V max bus voltage, 2A max bus current

  if (!bme.begin() { // BME688 Initialization
    Serial.println("BME680 initialization failed!");
    while (1);
  }

  // Set up oversampling and filter initialization for BME688
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150); // 320*C for 150 ms

}

void loop() {

  if (!bme.performReading()) {
    Serial.println("Failed to get current data from BME688!");
    return;
  }

  Serial.printf("Current Temperature: %lf degrees C\n", bme.temperature); // Print BME Data
  Serial.printf("Current Pressure: %lf hPa\n", bme.pressure / 100);
  Serial.printf("Current Humidity: %lf %\n", bme.humidity);

  Serial.printf("Current Bus Voltage: %lf V\n", sysBus.busVoltage());
  Serial.printf("Current Bus Current: %lf mA\n", sysBus.shuntCurrent());

  

}
