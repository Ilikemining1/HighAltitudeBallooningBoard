#include <Wire.h>  // Library Dependencies
#include <SPI.h>

#include <SparkFun_u-blox_GNSS_Arduino_Library.h>  // Sensor Libraries
#include <Adafruit_Sensor.h>
#include <Adafruit_BME680.h>
#include <INA219.h>
#include <ICM20948_WE.h>

#include <SD.h>  // Telemetry and Datalogging
#include <RadioLib.h>

INA219 sysBus;
Adafruit_BME680 bme;
SFE_UBLOX_GNSS gnss;
ICM20948_WE imu = ICM20948_WE(0x68);

int fileNumber = 1;
char fileName[25];

//SX1276 vhf = new Module(8, 14, 6, 15);
//SX1276 uhf = new Module(9, 20, 7, 21);


void setup() {
  Serial.begin(115200);  // Initialize USB UART for debug
  Serial.printf("HAB PCB rev 1\nInitializing system!\n");

  SPI1.setRX(12);  // Configure SPI1 peripheral for SD card
  SPI1.setCS(13);
  SPI1.setSCK(10);
  SPI1.setTX(11);

  Wire.begin();  // Start I2C bus at 100kHz
  Wire.setClock(100000);

  SD.begin(13, SPI1);

  if (!gnss.begin()) {  // GNSS Initialization
    Serial.println("GNSS initialization failed!");
    while (1)
      ;
  }
  gnss.setI2COutput(COM_TYPE_UBX);
  gnss.setNavigationFrequency(5);
  gnss.setAutoPVT(true);

  if (sysBus.begin()) {  // INA219 Initialization
    Serial.println("INA219 initialization failed!");
    while (1)
      ;
  }

  if (!imu.init()) {  // ICM20948 Initialization
    Serial.println("ICM20948 initialization failed!");
    while (1)
      ;
  }

  sysBus.configure(INA219::RANGE_16V, INA219::GAIN_1_40MV, INA219::ADC_64SAMP, INA219::ADC_64SAMP, INA219::CONT_SH_BUS);  // Configure INA219 range settings
  sysBus.calibrate(0.027, 0.04, 6, 2);                                                                                    // 27mOhm shunt, 40mV max shunt voltage, 6V max bus voltage, 2A max bus current

  if (!bme.begin()) {  // BME688 Initialization
    Serial.println("BME680 initialization failed!");
    while (1)
      ;
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
  pinMode(0, OUTPUT);
  pinMode(1, OUTPUT);
  pinMode(2, OUTPUT);

  while (1) {
    snprintf(fileName, 25, "data%u.csv", fileNumber);
    if (!(SD.exists(fileName))) break;
    Serial.printf("File data%u.csv exists!\n", fileNumber);
    fileNumber++; 
  }

  File data = SD.open(fileName, "a+");

  if (!data) {  // Check if file was opened
    Serial.printf("Unable to open %s for append!", fileName);
    while (1)
      ;
  }

  data.write("Date,Time,Temperature,Pressure,Humidity,Latitude,Longitude,GNSS Altitude,Bus Voltage,Bus Current\n");

  data.close();
}

void loop() {

  char line[200];
  static unsigned int gpsDateTime[6];
  static unsigned int gpsNav[3];

  if (!bme.performReading()) {
    Serial.println("Failed to get current data from BME688!");
    watchdog_reboot(0,0,0);
    return;
  }

  if (gnss.getPVT() && (gnss.getInvalidLlh() == false)) {

    gpsDateTime[0] = gnss.getYear();
    gpsDateTime[1] = gnss.getMonth();
    gpsDateTime[2] = gnss.getDay();
    gpsDateTime[3] = gnss.getHour();
    gpsDateTime[4] = gnss.getMinute();
    gpsDateTime[5] = gnss.getSecond();

    gpsNav[0] = gnss.getLatitude();
    gpsNav[1] = gnss.getLongitude();
    gpsNav[2] = gnss.getAltitude();

    if (gnss.getFixType() == 0) {
      digitalWrite(1, HIGH);
      digitalWrite(0, LOW);
    } else if (gnss.getFixType() == 2) {
      digitalWrite(0, HIGH);
      digitalWrite(1, HIGH);
    } else if (gnss.getFixType() == 3) {
      digitalWrite(0, HIGH);
      digitalWrite(1, LOW);
    }
    digitalWrite(2, !digitalRead(2));

  } else {
    Serial.println("Unable to get current NAV data!");
  }

  
  imu.readSensor();
  xyzFloat gVal = imu.getGValues();
  xyzFloat magVal = imu.getMagValues();
  xyzFloat gyroVal = imu.getGyrValues(); 

/*
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

  Serial.printf("\n\n\n\n\n"); */

  snprintf(line, 200, "%u/%u/%u,%u:%u:%u,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf \n", gpsDateTime[0], gpsDateTime[1], gpsDateTime[2], gpsDateTime[3], gpsDateTime[4], gpsDateTime[5], bme.temperature, bme.pressure / 100.0, bme.humidity, gpsNav[0] / 10000000.0, gpsNav[1] / 10000000.0, gpsNav[2] / 1000.0, sysBus.busVoltage(), sysBus.shuntCurrent() * 1000.0);

  File data = SD.open(fileName, "a+");
  Serial.println(fileName);

  if (!data) {  // Check if file was opened
    Serial.printf("Unable to open %s for append!", fileName);
    while (1);
  }
    
  data.print(line);

  data.close();

  digitalWrite(28, !digitalRead(28));
  delay(200);
}
