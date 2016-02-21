#include <PID_v1.h>

#include "MPU6050.h"
#include "Wire.h"
#include "I2Cdev.h"
#include <Servo.h>

#define MOTOR_FRONT 1
#define MOTOR_RIGHT 0
#define MOTOR_LEFT 3
#define MOTOR_REAR 2
#define RAD_TO_DEG   57.29578
#define M_PI         3.14159265358979323846
 boolean armed = false;
boolean powerOn = false;
int baseSpeed = 0;
Servo motors[4];
 float accelAngle[2];
    float gyroRotation[3];
int buffersize=1000;     //Amount of readings used to average, make it higher to get more precision but sketch will be slower  (default:1000)
int acel_deadzone=8;     //Acelerometer error allowed, make it lower to get more precision, but sketch may not converge  (default:8)
int giro_deadzone=1;
MPU6050 mpu(0x68); // <-- use for AD0 high

int16_t ax, ay, az,gx, gy, gz;

int mean_ax,mean_ay,mean_az,mean_gx,mean_gy,mean_gz,state=0;
int ax_offset,ay_offset,az_offset,gx_offset,gy_offset,gz_offset;
float gxAngle = 0, gyAngle = 0, gzAngle = 0;
float axAngle = 0, ayAngle = 0, azAngle = 0;
float angle_x_Prediction,angle_y_Prediction=0;
int desired_angle[2] = {0, 0};
unsigned long lastReadingTime = 0;
const int motorPins[] = {3, 4, 5, 6};
double yaw = 0, pitch = 0, roll = 0;
//Variables for kalman filter
float Pxx = 0.1; // angle variance
float Pvv = 0.1; // angle change rate variance
float Pxv = 0.1; // angle and angle change rate covariance
float gyroVar = 0.1;
float deltaGyroVar = 0.1;
float accelVar = 5.0;
/*
 * Integral of angle for "integral" component of PID controller
 */
float error_integral[2] = {0, 0};

void setup()
{
   
 
  Serial.begin(115200);
armESC();
initializeMPU();

}
void loop() {
 if(powerOn)
  {
float timeDelta = (float)(micros() - lastReadingTime) / 1000000.0;
    if(lastReadingTime == 0) {
      timeDelta = 0;
    }
    
    lastReadingTime = micros();
    
updateGyroAcc();
updateAngles();
gxAngle+=gx*timeDelta;
gyAngle+=gy*timeDelta;
gzAngle+=gz*timeDelta;

angle_x_Prediction+=gx*timeDelta;
angle_y_Prediction+=gy*timeDelta;

   
    /*
     * Kalman filter algorithm from http://www.den-uijl.nl/electronics/gyro.html
     */
    Pxx += timeDelta * (2 * Pxv + timeDelta * Pvv);
    Pxv += timeDelta * Pvv;
    Pxx += timeDelta * gyroVar;
    Pvv += timeDelta * deltaGyroVar;
    float kx = Pxx * (1.0 / (Pxx + accelVar));
    float kv = Pxv * (1.0 / (Pxx + accelVar));
    
    angle_x_Prediction += (ax - angle_x_Prediction) * kx;
    angle_y_Prediction += (ay - angle_y_Prediction) * kx;
    
    Pxx *= (1 - kx);
    Pxv *= (1 - kx);
    Pvv -= kv * Pxv;
  /*
     * The sensors on the quadcopter are rotated 45 degrees from the axes of the frame,
     * so the frame's rotation needs to be calculated from the sensors' rotation
     */
    float rotation[2];
    rotation[0] = angle_x_Prediction * cos(PI / 4.0) + angle_y_Prediction * sin(PI / 4.0);
    rotation[1] = angle_x_Prediction * sin(PI / 4.0) - angle_y_Prediction * cos(PI / 4.0);
    
    float rotation_derivative[2];
    rotation_derivative[0] = gx * cos(PI / 4.0) +gy * sin(PI / 4.0);
    rotation_derivative[1] = gx * sin(PI / 4.0) - gy * cos(PI / 4.0);
     /*
     * PID controller
     */
    float error[2];
    error[0] = rotation[0] - desired_angle[0];
    error[1] = rotation[1] - desired_angle[1];
    
    float proportional[2], integral[2], derivative[2];
    int output[2];
    
    #define GAIN_PROPORTIONAL 0.08
    proportional[0] = error[0] * GAIN_PROPORTIONAL;
    proportional[1] = error[1] * GAIN_PROPORTIONAL;
    
    error_integral[0] += error[0] * timeDelta;
    error_integral[1] += error[1] * timeDelta;
    #define GAIN_INTEGRAL 0.02
    integral[0] = error_integral[0] * GAIN_INTEGRAL;
    integral[1] = error_integral[1] * GAIN_INTEGRAL;
    
    #define GAIN_DERIVATIVE 0.0
    derivative[0] = rotation_derivative[0] * GAIN_DERIVATIVE;
    derivative[1] = rotation_derivative[1] * GAIN_DERIVATIVE;
    
    output[0] = proportional[0] + integral[0] + derivative[0];
    output[1] = proportional[1] + integral[1] + derivative[1];
    setSpeed(MOTOR_FRONT,baseSpeed + output[1]); //baseSpeed + output[1]);
    setSpeed(MOTOR_REAR, baseSpeed - output[1]); //baseSpeed - output[1]);
    setSpeed(MOTOR_LEFT, baseSpeed + output[0]);
    setSpeed(MOTOR_RIGHT, baseSpeed - output[0]);
  }

  else   {
    setSpeed(MOTOR_FRONT, 0);
    setSpeed(MOTOR_REAR, 0);
    setSpeed(MOTOR_LEFT, 0);
    setSpeed(MOTOR_RIGHT, 0);
    
    error_integral[0] = 0;
    error_integral[1] = 0;
  }
/* Serial.print(gxAngle); 
    Serial.print("\t");
    
 Serial.print(gyAngle); 
    Serial.print("\t");
    
 Serial.print(gzAngle); 
    Serial.print("\t");

 Serial.print( output[0]); 
    Serial.print("\t");
    
 Serial.print( output[1]); 
    Serial.print("\t");
   
    Serial.print("\n");
  */  
}
void serialEvent()
{
 readCommand();
}
void readCommand(){
   char val;
   if( Serial.available())
  {
    val = Serial.read();
  }
  //Input key switch
    switch (val) {
    case '0':    
      //Code when no key is pressed
      break;
    case '1':  
      desired_angle[0]+=1; 
      break;
    case '2':    
      desired_angle[0]-=1; 
      break;
    case '3':    
      desired_angle[1]+=1; 
      break;   
    case '4':    
      desired_angle[0]-=1; 
      break;
    case '5':    
    break;
    case '6':    
      powerOn =!powerOn;
      //basespeed-=1; 
      break;
    case '7':    
       baseSpeed+=1;
      break;
    case '8':    
      baseSpeed-=1;
      //Code when TRIANGLE key is pressed 
      break;
    case '9':    
      //Code when SELECT key is pressed 
      break;
    case 'A':    
      //Code when START key is pressed 
      break;
    default:
      break;
      // default code (should never run)
    } 
  }
  
  
  
void setSpeed(int motor, int speed)
{
  speed = constrain(speed, 0, 100);
  int fakeAngle = map(speed, 0, 100, 0, 180);
  motors[motor].write(fakeAngle);
}
void armESC(){
   motors[MOTOR_FRONT].attach(6);
  motors[MOTOR_RIGHT].attach(3);
  motors[MOTOR_LEFT].attach(5);
  motors[MOTOR_REAR].attach(4);
   setSpeed(MOTOR_FRONT, 100);
    setSpeed(MOTOR_REAR, 100);
    setSpeed(MOTOR_LEFT, 100);
    setSpeed(MOTOR_RIGHT, 100);

  
  
  }
