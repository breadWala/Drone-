#include <Wire.h>
#include <ESP32Servo.h>

#define MPU_ADDR 0x68
float currentAngle=0.0;
unsigned long lastTime=0;









// =========================
// ESCs
// =========================

Servo esc1;
Servo esc2;

int BASE_MOTOR1 = 1310;  //assuming it to be the base thrust
int BASE_MOTOR2 = 1285;


const int ESC1_PIN = 19;
const int ESC2_PIN = 17;

int BASE_THROTTLE = 1200;


// =========================
// PD PARAMETERS
// =========================

float Kp = 4.8;
float Kd = 2.4;

const int MAX_CORRECTION = 200;



// =========================
// MPU CALIBRATION
// =========================

float gyroBiasY = 0;
float targetAngle = 0;


// =========================
// READ MPU6500
// =========================

void readMPU(float &ay, float &az, float &gyroY)
{
  // Accelerometer

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);

  Wire.requestFrom(MPU_ADDR, 6, true);

  int16_t rawAx = (Wire.read() << 8) | Wire.read();
  int16_t rawAy = (Wire.read() << 8) | Wire.read();
  int16_t rawAz = (Wire.read() << 8) | Wire.read();

  ay = rawAy / 16384.0;
  az = rawAz / 16384.0;


  // Gyroscope

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x43);
  Wire.endTransmission(false);

  Wire.requestFrom(MPU_ADDR, 6, true);

  int16_t rawGx = (Wire.read() << 8) | Wire.read();
  int16_t rawGy = (Wire.read() << 8) | Wire.read();
  int16_t rawGz = (Wire.read() << 8) | Wire.read();

  gyroY = rawGy / 131.0;
}


// =========================
// SETUP
// =========================

void setup()
{
  Serial.begin(115200);

  Wire.begin(21, 22);


  // =========================
  // WAKE MPU6500
  // =========================

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();

  delay(1000);


  // =========================
  // GYRO CALIBRATION
  // =========================

  Serial.println();
  Serial.println("Keep frame completely still.");
  Serial.println("Calibrating gyro...");

  float gyroSum = 0;

  for (int i = 0; i < 500; i++)
  {
    float ay, az, gyroY;

    readMPU(ay, az, gyroY);

    gyroSum += gyroY;

    delay(5);
  }

  gyroBiasY = gyroSum / 500.0;


  // =========================
  // TARGET ANGLE CALIBRATION
  // =========================

  Serial.println("Keep frame at BALANCED position.");
  Serial.println("Calibrating target angle...");

  float angleSum = 0;

  for (int i = 0; i < 500; i++)
  {
    float ay, az, gyroY;

    readMPU(ay, az, gyroY);

    float angle =
        atan2(ay, -az) * 180.0 / PI;

    angleSum += angle;

    delay(5);
  }

  targetAngle = angleSum / 500.0;

  currentAngle=targetAngle;
  lastTime=millis();


  // =========================
  // PRINT CALIBRATION
  // =========================

  Serial.println();

  Serial.print("Gyro Y bias: ");
  Serial.print(gyroBiasY, 3);
  Serial.println(" deg/s");

  Serial.print("Target angle: ");
  Serial.print(targetAngle, 2);
  Serial.println(" deg");


  // =========================
  // ESC SETUP
  // =========================

  esc1.attach(ESC1_PIN, 1000, 2000);
  esc2.attach(ESC2_PIN, 1000, 2000);

  esc1.writeMicroseconds(1000);
  esc2.writeMicroseconds(1000);

  delay(3000);

  Serial.println();
  Serial.println("Two-motor PD controller ready.");
}


// =========================
// LOOP
// =========================

void loop()
{
  float ay;
  float az;
  float gyroY;


  // =========================
  // READ MPU
  // =========================

  readMPU(ay, az, gyroY);


  // =========================
  // calculate time delta ( dt)
 
 unsigned long currentTime=millis();
 float dt=(currentTime-lastTime)/1000.0; //converting from ms to seconds
 lastTime=currentTime;








  // =========================
  // CORRECT GYRO BIAS
  // =========================
 float correctedGyro=gyroY-gyroBiasY;
  


  // =========================
  // ANGLE 
  // =========================

  //1.Raw acceleration angle(noisy)
  float accelAngle=atan2(ay,-az)*180.0/PI;
  //2.complementary filter(98 percent gyro, 2 percent acclerometer)
  currentAngle=0.98*(currentAngle+(correctedGyro*dt)) + 0.02*accelAngle;



  //================================
  //angle error
 float error=targetAngle-currentAngle;
 //fixing atan2 wraparound
 if(error>180.0)
 {
    error=error-360.0;

 }
 else if(error <-180.0)
 {
    error=error+360.0;
 }

  // =========================
  // P TERM
  // =========================

  float P =
      Kp * error;


  // =========================
  // D TERM
  // =========================

  float D =
      -Kd * correctedGyro;


  // =========================
  // PD CORRECTION
  // =========================

  float correction =
      P + D;


  // =========================
  // LIMIT CORRECTION
  // =========================

  correction =
      constrain(
          correction,
          -MAX_CORRECTION,
          MAX_CORRECTION
      );


  // =========================
  // TWO-MOTOR MIXING
  // =========================

  int motor1Command =
      BASE_MOTOR1 + correction;

  int motor2Command =
      BASE_MOTOR2 - correction;


  // =========================
  // LIMIT ESC COMMANDS
  // =========================

  motor1Command =
      constrain(
          motor1Command,
          1050,
          2000
      );

  motor2Command =
      constrain(
          motor2Command,
          1050,
          2000
      );
        
          
esc1.writeMicroseconds(motor1Command);
esc2.writeMicroseconds(motor2Command);

  // =========================
  // DEBUG
  // =========================


  delay(5);
}


//version 2
 //will be implementing a complementary filter on the accelerometer -- 
  // Also we will be doing error normalization to fix the wrap around of the angle problem 