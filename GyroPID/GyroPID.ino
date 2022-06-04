/*
    MPU6050 Triple Axis Gyroscope & Accelerometer. Simple Gyroscope Example.
    Read more: http://www.jarzebski.pl/arduino/czujniki-i-sensory/3-osiowy-zyroskop-i-akcelerometr-mpu6050.html
    GIT: https://github.com/jarzebski/Arduino-MPU6050
    Web: http://www.jarzebski.pl
    (c) 2014 by Korneliusz Jarzebski
*/

#include <Wire.h>
#include <MPU6050.h>
#include <PID_v1.h>

MPU6050 mpu;
int16_t ax, ay, az, Nax, Nay, Naz;
int16_t gx, gy, gz, Ngx, Ngy, Ngz;

//Define Variables we'll be connecting to
double Setpoint, Input, Output;
double Kp=0, Ki=30, Kd=0;
PID myPID(&Input, &Output, &Setpoint,Kp,Ki,Kd, REVERSE);

void setup() 
{
  Serial.begin(115200);

  Setpoint=90;
  myPID.SetMode(AUTOMATIC);
  myPID.SetTunings(Kp,Ki,Kd);
  myPID.SetOutputLimits(0, 180);

  // Initialize MPU6050
  Serial.println("Initialize MPU6050");
  while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
  {
    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    delay(500);
  }
  
  // If you want, you can set gyroscope offsets
  // mpu.setGyroOffsetX(155);
  // mpu.setGyroOffsetY(15);
  // mpu.setGyroOffsetZ(15);
  
  // Calibrate gyroscope. The calibration must be at rest.
  // If you don't want calibrate, comment this line.
  mpu.calibrateGyro();

  // Set threshold sensivty. Default 3.
  // If you don't want use threshold, comment this line or set 0.
  mpu.setThreshold(3);
  
  // Check settings
  checkSettings();
}

void checkSettings()
{
  Serial.println();
  
  Serial.print(" * Sleep Mode:        ");
  Serial.println(mpu.getSleepEnabled() ? "Enabled" : "Disabled");
  
  Serial.print(" * Clock Source:      ");
  switch(mpu.getClockSource())
  {
    case MPU6050_CLOCK_KEEP_RESET:     Serial.println("Stops the clock and keeps the timing generator in reset"); break;
    case MPU6050_CLOCK_EXTERNAL_19MHZ: Serial.println("PLL with external 19.2MHz reference"); break;
    case MPU6050_CLOCK_EXTERNAL_32KHZ: Serial.println("PLL with external 32.768kHz reference"); break;
    case MPU6050_CLOCK_PLL_ZGYRO:      Serial.println("PLL with Z axis gyroscope reference"); break;
    case MPU6050_CLOCK_PLL_YGYRO:      Serial.println("PLL with Y axis gyroscope reference"); break;
    case MPU6050_CLOCK_PLL_XGYRO:      Serial.println("PLL with X axis gyroscope reference"); break;
    case MPU6050_CLOCK_INTERNAL_8MHZ:  Serial.println("Internal 8MHz oscillator"); break;
  }
  
  Serial.print(" * Gyroscope:         ");
  switch(mpu.getScale())
  {
    case MPU6050_SCALE_2000DPS:        Serial.println("2000 dps"); break;
    case MPU6050_SCALE_1000DPS:        Serial.println("1000 dps"); break;
    case MPU6050_SCALE_500DPS:         Serial.println("500 dps"); break;
    case MPU6050_SCALE_250DPS:         Serial.println("250 dps"); break;
  } 
  
  Serial.print(" * Gyroscope offsets: ");
  Serial.print(mpu.getGyroOffsetX());
  Serial.print(" / ");
  Serial.print(mpu.getGyroOffsetY());
  Serial.print(" / ");
  Serial.println(mpu.getGyroOffsetZ());
  
  Serial.println();
}

void serialPrintReadings(){
  
  Serial.print("ACC Xraw = ");
  Serial.print(ax);
  Serial.print("ACC Yraw = ");
  Serial.print(ay);
  Serial.print("ACC Zraw = ");
  Serial.println(az);
  
  Serial.print("ACC Xnorm = ");
  Serial.print(Nax);
  Serial.print("ACC Ynorm = ");
  Serial.print(Nay);
  Serial.print("ACC Znorm = ");
  Serial.println(Naz);
  
  Serial.print("GYRO Xraw = ");
  Serial.print(gx);
  Serial.print("GYRO Yraw = ");
  Serial.print(gy);
  Serial.print("GYRO Zraw = ");
  Serial.println(gz);
  
  Serial.print("GYRO Xnorm = ");
  Serial.print(Ngx);
  Serial.print("GYRO Ynorm = ");
  Serial.print(Ngy);
  Serial.print("GYRO Znorm = ");
  Serial.println(Ngz);
  
  
  }

void loop()
{
  // GYRO READINGS
  Vector rawGyro = mpu.readRawGyro();
  Vector normGyro = mpu.readNormalizeGyro();
  // ACCELEROMETER READINGS 
  Vector rawAccel = mpu.readRawAccel();
  Vector normAccel = mpu.readNormalizeAccel();

  gx = rawGyro.XAxis; 
    gy = rawGyro.YAxis;
      gz = rawGyro.ZAxis;
  Ngx = normGyro.XAxis;
    Ngy = normGyro.YAxis;
      Ngz = normGyro.ZAxis;

  ax = rawAccel.XAxis;
    ay = rawAccel.YAxis;
      az = rawAccel.ZAxis;  
  Nax = normAccel.XAxis;
    Nay = normAccel.YAxis;
      Naz = normAccel.ZAxis;

  //serialPrintReadings();

  Input=map (gy, 17000, -17000, 0, 180) ;
  myPID.Compute();
  Serial.print("gy = ");
  Serial.println(gy);
  Serial.println(Output);
  Serial.print(Output);
  
  delay(100);
}
