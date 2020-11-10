#include <Servo.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69

MPU6050 accelgyro;
//MPU6050 accelgyro(0x69); // <-- use for AD0 high

Servo myservo;             // creating a servo object

int16_t Acc_rawX, Acc_rawY, Acc_rawZ,Gyr_rawX, Gyr_rawY, Gyr_rawZ;

float Acceleration_angle[2];
float Gyro_angle[2];
float Total_angle[2];

float elapsedTime, time, timePrev;
int i;
float rad_to_deg = 180/3.141592654;

float PID, pwmLeft, pwmRight, error, previous_error;
float pid_p=0;
float pid_i=0;
float pid_d=0;

/////////////////PID CONSTANTS/////////////////
                                                
double kp=2.55; 
double ki=0.003;
double kd=1.05; 

/*
 * History
 * kp=3.55
 * ki=0.003
 * kd=2.05
*/
///////////////////////////////////////////////

float desired_angle = 0; //This is the angle at which we want the
                         //balance to stay steady

void setup() {
  myservo.attach(9);     // attaches the servo on pin 9 to the servo object

  Wire.begin(); //begin the wire comunication
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

  time = millis(); //Start counting time in milliseconds
}

void loop() {

/////////////////////////////MPU 6050///////////////////////////////////////////
  timePrev = time;  // the previous time is stored before the actual time read
  time = millis();  // actual time read
  elapsedTime = (time - timePrev) / 1000; 
  
  /*The time step is the time that is elapsed since the previous loop. 
   * This is the value that we will use in the formulas as "elapsedTime" 
   * in seconds. It is in "ms" so we have to divide the value by 1000 
   to obtain it in seconds*/

  /*Reading the values from the accelerometer.
   * We know that the slave adress for this IMPU6050 is 0x68 in
   * hexadecimal*/
   
   Wire.beginTransmission(0x68);
   Wire.write(0x3B);  //Ask for the 0x3B register- correspond to AcX
   Wire.endTransmission(false);
   Wire.requestFrom(0x68,6,true); 
   
   /*We have asked for the 0x3B register. The MPU6050 will send a brust of register.
    * The amount of register to read is specify in the requestFrom function.
    * In this case we request 6 registers. Each value of acceleration is made out of
    * two 8-bits registers, low values and high values. For that we request the 6 of them  
    * and just make then sum of each pair. For that we shift to the left the high values 
    * register (<<) and make an or (|) operation to add the low values.*/
    
   Acc_rawX=Wire.read()<<8|Wire.read(); //each value needs two registres
   Acc_rawY=Wire.read()<<8|Wire.read();
   Acc_rawZ=Wire.read()<<8|Wire.read();

 
   /*///This is the part where you need to calculate the angles using Euler equations///*/
    
   /* - Now, to obtain the values of acceleration in "g" units we first have to divide the raw   
    * values that we have just read by 16384.0 because that is the value that was included in the MPU6050 
    * datasheet.*/
   /* - Next we have to calculate the radian to degree value by dividing 180º by the PI number
    * which is 3.141592654 and store this value in the rad_to_deg variable. In order to not have
    * to calculate this value in each loop we have done that just once before the setup void.
    */

   /* Now we can apply the Euler formula. The atan will calculate the arctangent. The
    *  pow(a,b) will elevate the a value to the b power. And sqrt function
    *  will calculate the root square.*/
   /*---X---*/
   Acceleration_angle[0] = atan((Acc_rawY/16384.0)/sqrt(pow((Acc_rawX/16384.0),2) + pow((Acc_rawZ/16384.0),2)))*rad_to_deg;
   /*---Y---*/
   Acceleration_angle[1] = atan(-1*(Acc_rawX/16384.0)/sqrt(pow((Acc_rawY/16384.0),2) + pow((Acc_rawZ/16384.0),2)))*rad_to_deg;
 
   /*Now we read the Gyro data in the same way as the Acc data. The adress for the
    * gyro data starts at 0x43. We can see this adresses if we look at the register map
    * of the MPU6050. In this case we request just 4 values. W don¡t want the gyro for 
    * the Z axis (YAW).*/
    
   Wire.beginTransmission(0x68);
   Wire.write(0x43); //Gyro data first adress
   Wire.endTransmission(false);
   Wire.requestFrom(0x68,4,true); //Just 4 registers
   
   Gyr_rawX=Wire.read()<<8|Wire.read(); //Once again we shif and sum
   Gyr_rawY=Wire.read()<<8|Wire.read();
 
   /*Now in order to obtain the gyro data in degrees/second we have to divide first
   the raw value by 131 because that's the value that the was included in the MPU6050's datasheet*/

   /*---X---*/
   Gyro_angle[0] = Gyr_rawX/131.0; 
   /*---Y---*/
   Gyro_angle[1] = Gyr_rawY/131.0;

   /*Now in order to obtain degrees we have to multiply the degree/seconds
   *value by the elapsedTime.*/
   /*Finaly we can apply the final filter where we add the acceleration
   *part that afects the angles and ofcourse multiply by 0.98 */

   /*---X axis angle---*/
   Total_angle[0] = 0.99 *(Total_angle[0] + Gyro_angle[0]*elapsedTime) + 0.02*Acceleration_angle[0];
   /*---Y axis angle---*/
   Total_angle[1] = 0.99 *(Total_angle[1] + Gyro_angle[1]*elapsedTime) + 0.02*Acceleration_angle[1];
   
   /*angle values from -10º0 to 100º "at coefficient = 0.98"*/
   //Serial.println(Total_angle[0]);
   //Serial.println(Total_angle[1]);

///////////////////////////////////////////P I D//////////////////////////////////////////////////
   /*Remember that for the balance we will use just one axis. I've choosen the X angle
   to implement the PID with. That means that the X - axis of the IMU has to be paralel to
   the balance*/
    
   /*First calculate the error between the desired angle and 
    *the real measured angle*/
   
   error = Total_angle[1] - desired_angle;
        
   /*Next the proportional value of the PID is just a proportional constant
    *multiplied by the error*/
    
   pid_p = kp*error;
    
   /*The integral part should only act if we are close to the
   desired position but we want to fine tune the error. That's
   why I've made a if operation for an error between -2 and 2 degree.
   To integrate we just sum the previous integral value with the
   error multiplied by  the integral constant. This will integrate (increase)
   the value each loop till we reach the 0 point*/
   
   if(-3 <error <3)
   {
     pid_i = pid_i+(ki*error);  
   }
    
   /*The last part is the derivate. The derivate acts upon the speed of the error.
   As we know the speed is the amount of error that produced in a certain amount of
   time divided by that time. For that we will use a variable called previous_error.
   We substract that value from the actual error and divide all by the elapsed time. 
   Finaly we multiply the result by the derivate constant*/
    
   pid_d = kd*((error - previous_error)/elapsedTime);
    
   /*The final PID values is the sum of each of this 3 parts*/
   PID = pid_p + pid_i + pid_d;

   myservo.write(Total_angle[1]);
   delay(10);
}
