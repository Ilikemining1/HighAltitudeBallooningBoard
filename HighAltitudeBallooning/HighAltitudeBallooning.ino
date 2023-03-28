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
ICM20948_WE imu = ICM20948_WE(0x68);

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

  if (!gnss.begin()) { // GNSS Initialization
    Serial.println("GNSS initialization failed!");
    while (1);
  }

  if (sysBus.begin()) { // INA219 Initialization
    Serial.println("INA219 initialization failed!");
    while (1);
  }

if (!imu.init()) { // ICM20948 Initialization
    Serial.println("ICM20948 initialization failed!");
    while (1);
  }
  
  sysBus.configure(INA219::RANGE_16V, INA219::GAIN_1_40MV, INA219::ADC_64SAMP, INA219::ADC_64SAMP, INA219::CONT_SH_BUS); // Configure INA219 range settings
  sysBus.calibrate(0.027, 0.04, 6, 2); // 27mOhm shunt, 40mV max shunt voltage, 6V max bus voltage, 2A max bus current

  if (!bme.begin()) { // BME688 Initialization
    Serial.println("BME680 initialization failed!");
    while (1);
  }

  // Set up oversampling and filter initialization for BME688
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);

  Serial.println("IMU Auto-Level, do not disturb!");
  delay(1000);
  imu.autoOffsets();
  Serial.println("Done!");
  imu.setAccRange(ICM20948_ACC_RANGE_16G);
  imu.setAccDLPF(ICM20948_DLPF_6);
  imu.setAccSampleRateDivider(10);
  imu.initMagnetometer();
  imu.setMagOpMode(AK09916_CONT_MODE_20HZ);
  imu.setGyrRange(ICM20948_GYRO_RANGE_250);
  imu.setGyrDLPF(ICM20948_DLPF_6);

  pinMode(28, OUTPUT);

}

void loop() {

  if (!bme.performReading()) {
    Serial.println("Failed to get current data from BME688!");
    return;
  }

  imu.readSensor();
  xyzFloat gVal = imu.getGValues();
  xyzFloat magVal = imu.getMagValues();
  xyzFloat gyroVal = imu.getGyrValues();

  Serial.printf("Current Temperature: %lf degrees C\n", bme.temperature); // Print BME Data
  Serial.printf("Current Pressure: %lf hPa\n", bme.pressure / 100.0);
  Serial.printf("Current Humidity: %lf %\n", bme.humidity);

  Serial.printf("Current Bus Voltage: %lf V\n", sysBus.busVoltage());
  Serial.printf("Current Bus Current: %lf mA\n", sysBus.shuntCurrent() * 1000.0);

  Serial.printf("Current GNSS Fix Type: %d Current SIV: %d\n", gnss.getFixType(), gnss.getSIV());
  Serial.printf("Current Latitude: %lf degrees\n", (double) gnss.getLatitude() / 10000000.0);
  Serial.printf("Current Longitude: %lf degrees\n", (double) gnss.getLongitude() / 10000000.0);
  Serial.printf("Current GNSS Altitude: %lf m\n", (double) gnss.getAltitude() / 1000.0);

  Serial.printf("Current Acceleration x:%lf, y:%lf, z:%lf g\n", gVal.x, gVal.y, gVal.z);
  Serial.printf("Current Mag Values x:%lf, y:%lf, z:%lf\n", magVal.x, magVal.y, magVal.z);
  Serial.printf("Current Gyro Values x:%lf, y:%lf, z:%lf\n", gyroVal.x, gyroVal.y, gyroVal.z);

  Serial.printf("\n\n\n\n\n");

  digitalWrite(28, HIGH);
  delay(500);
  digitalWrite(28, LOW);
  delay(500);

}
