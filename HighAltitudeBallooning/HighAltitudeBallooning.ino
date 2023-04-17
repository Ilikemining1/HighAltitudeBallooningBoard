#include <Wire.h>  // Library Dependencies
#include <SPI.h>

#include <SparkFun_u-blox_GNSS_Arduino_Library.h>  // Sensor Libraries
#include <Adafruit_Sensor.h>
#include <Adafruit_BME680.h>
#include <INA219.h>
#include <ICM20948_WE.h>

#include <SD.h>

INA219 sysBus;
Adafruit_BME680 bme;
SFE_UBLOX_GNSS gnss;
ICM20948_WE imu = ICM20948_WE(0x68);

int fileNumber = 1;
char fileName[25];

void setup() {
  Serial.begin(115200);  // Initialize USB UART for debug
  Serial.printf("HAB PCB rev 1\nInitializing system!\n");

  SPI1.setRX(12);  // Configure SPI1 peripheral for SD card
  SPI1.setCS(13);
  SPI1.setSCK(10);
  SPI1.setTX(11);
  
  Serial1.setRX(1);
  Serial1.setTX(0);
  Serial1.begin(115200);

  Wire.begin();  // Start I2C bus at 100kHz
  Wire.setClock(100000);

  SD.begin(13, SPI1);

  if (!gnss.begin(Serial1)) {  // GNSS Initialization
    Serial1.begin(38400);
    gnss.begin(Serial1);
    gnss.setSerialRate(115200);
    watchdog_reboot(0, 0, 0);  // Reset to apply baud rate.
  }

  gnss.setUART1Output(COM_TYPE_UBX);
  gnss.setDynamicModel(DYN_MODEL_AIRBORNE2g);
  if (gnss.getDynamicModel() != DYN_MODEL_AIRBORNE2g) {
    Serial.println("Dynamics model could not be updated!");
    while (1);
  }

  if (sysBus.begin()) {  // INA219 Initialization
    Serial.println("INA219 initialization failed!");
    while (1);
  }

  sysBus.configure(INA219::RANGE_16V, INA219::GAIN_1_40MV, INA219::ADC_64SAMP, INA219::ADC_64SAMP, INA219::CONT_SH_BUS);  // Configure INA219 range settings
  sysBus.calibrate(0.027, 0.04, 6, 2);                                                                                    // 27mOhm shunt, 40mV max shunt voltage, 6V max bus voltage, 2A max bus current

  if (!bme.begin()) {  // BME688 Initialization
    Serial.println("BME688 initialization failed!");
    while (1);
  }

  // Set up oversampling and filter initialization for BME688
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);

  if (!imu.init()) {  // ICM20948 Initialization
    Serial.println("ICM20948 initialization failed!");
    while (1);
  }

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
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);

  pinMode(29, INPUT_PULLUP);

  while (1) {
    snprintf(fileName, 25, "data%u.csv", fileNumber);
    if (!(SD.exists(fileName))) break;
    Serial.printf("File data%u.csv exists!\n", fileNumber);
    fileNumber++; 
  }

  Serial.printf("Using file: %s\n", fileName);
  File data = SD.open(fileName, "a+");

  if (!data) {  // Check if file was opened
    Serial.printf("Unable to open %s for append!", fileName);
    while (1);
  }

  data.write("Date,Time,Temperature,Pressure,Humidity,Latitude,Longitude,GNSS Altitude,GNSS Fix Type,GNSS SIV,Accell X,Accell Y,Accell Z,Mag X,Mag Y,Mag Z,Gyro X,Gyro Y,Gyro Z,Bus Voltage,Bus Current\n");

  data.close();
}

void loop() {

  char line[300];

  if (!bme.performReading()) {
    Serial.println("Failed to get current data from BME688!");
    return;
  }

  if (gnss.getFixType() == 0) {
    digitalWrite(3, HIGH);
    digitalWrite(2, LOW);
  } else if (gnss.getFixType() == 2) {
    digitalWrite(2, HIGH);
    digitalWrite(3, HIGH);
  } else if (gnss.getFixType() == 3) {
    digitalWrite(2, HIGH);
    digitalWrite(3, LOW);
  }
  
  imu.readSensor();
  xyzFloat gVal = imu.getGValues();
  xyzFloat magVal = imu.getMagValues();
  xyzFloat gyroVal = imu.getGyrValues(); 

  snprintf(line, 300, "%u/%u/%u,%u:%u:%u,%lf,%lf,%lf,%lf,%lf,%lf,%u,%u,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf \n", gnss.getYear(), gnss.getMonth(), gnss.getDay(), gnss.getHour(), gnss.getMinute(), gnss.getSecond(), bme.temperature, bme.pressure / 100.0, bme.humidity, gnss.getLatitude() / 10000000.0, gnss.getLongitude() / 10000000.0, gnss.getAltitude() / 1000.0, gnss.getFixType(), gnss.getSIV(), gVal.x, gVal.y, gVal.z, magVal.x, magVal.y, magVal.z, gyroVal.x, gyroVal.y, gyroVal.z, sysBus.busVoltage(), sysBus.shuntCurrent() * 1000.0);

  File data = SD.open(fileName, "a+");

  if (!data) {  // Check if file was opened
    Serial.printf("Unable to open %s for append!", fileName);
    while (1);
  }
    
  data.print(line);

  data.close();

  if (digitalRead(29) == LOW) {
    data.close();
    digitalWrite(2, LOW);
    digitalWrite(3, LOW);
    Serial.println("File has been closed out!");

    while(true) {
      digitalWrite(28, !digitalRead(28));
      delay(100); 
    }

  }

  digitalWrite(28, !digitalRead(28));
  delay(50);
}
