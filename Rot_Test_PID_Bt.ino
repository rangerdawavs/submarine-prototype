/* Last updated April 14 2020. The following code is used by an arduino to hold a desired position 
 and attitude using gyroscope and accelerometer data from an MPU6050 merged using a Kalman filter with a
 raspberry Pi lifted above the vehicle. With that data, it uses a PID controller to follow the waypoints sent by 
 the raspberry.  
*/
#include <MPU6050_tockn.h>
#include <Wire.h>
#include <math.h>
#include <RobotKit.h>
MPU6050 mpu6050(Wire);
#include <BasicLinearAlgebra.h>
using namespace BLA;
// These libraries are used to interpret IMU data, do calculations quicker and control the motor drivers in the arduino
float zeta;// the current angle the vehicle is pointed at 
int alpha; // the desired angle
String a;// This variable holds the string recieved through the serial before it is broken down
String b,c,d,e,f,g,h;// a is broken down into 6 numbers , stored in these variables before use
int error_Rot,error_Pos;
float errorDot_Pos,errorDot_Rot;
float errorInt_Pos,errorInt_Rot;
float control_Pos,control_Rot;
float x_Current,y_Current, x_Desired, y_Desired;
int previousTime,currentTime;// used to calculate time between measurements
float delta_t;// time between measurements
int errorDeadband_Rot=8 ;// the deadband makes sure that the system is not overcompensating for negligable error
int errorDeadband_Pos=1;
int Msg =0;// this variable is used to make the vehicle wait for an input. Currentlly it only waits for calibration to end.
BLA::Matrix<4,4> Pos_StateTransition;
BLA::Matrix<4,2> Pos_ControlMatrix;
BLA::Matrix<2>   Pos_ControlVector;// the accelerometer output
BLA::Matrix<2,2> Pos_PredictionVariance;// should be a identity matrix multiplied by the accelerometer variance
BLA::Matrix<4,4> Pos_MeasurmentVariance;//should be the camera variance diagonally and 0s in the remaining spaces
BLA::Matrix<4>   Pos_MeasurementVector;
BLA::Matrix<4,4> Pos_StateCovarianceMatrix;
BLA::Matrix<4,4> Pos_H;
BLA::Matrix<4>   PositionVelocityVector;
BLA::Matrix<4,4> Pos_KalmanGain; 
BLA::Matrix<4,4> Pos_identity = {1,0,0,0,
                                 0,1,0,0,
                                 0,0,1,0,
                                 0,0,0,1};

BLA::Matrix<2,2> Rot_StateTransition;
BLA::Matrix<2>   Rot_ControlMatrix;
BLA::Matrix<1>   Rot_ControlVector;// the gyro output
BLA::Matrix<2,2> Rot_PredictionVariance;// should be a identity matrix multiplied by the gyro variance
BLA::Matrix<2,2> Rot_MeasurmentVariance;//should be the camera variance diagonally and 0s in the remaining spaces
BLA::Matrix<2>   Rot_MeasurementVector;
BLA::Matrix<2,2> Rot_StateCovarianceMatrix;
BLA::Matrix<2,2> Rot_H;
BLA::Matrix<2>   RotationVelocityVector;
BLA::Matrix<2,2> Rot_KalmanGain; 
BLA::Matrix<2,2> Rot_identity = {1,0, 
                                 0,1};
BLA::Matrix<6> Coefficients = {0,
                               0,
                               0, 
                               1.5,
                               1.8,
                              -0.8};
 // coefficients of proportional, derivative and integral for position and angle respectively
BLA::Matrix<6> Desired_State;
BLA::Matrix<6> error;
//LED pins for bugtesting
int redLED= 7;
int yellowLED= 8;
int greenLED= 9;
int blueLED= 12;
void setup() {
  setMotors();  // turn motors on
  Serial.begin(9600);
  Serial.setTimeout(10);
  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);
  //LED pins are for bugtesting
  pinMode(redLED,OUTPUT);
  pinMode(yellowLED,OUTPUT);
  pinMode(greenLED,OUTPUT);
  pinMode(blueLED,OUTPUT);
} 
void loop() {
  if( Msg<1){
    delay(300);
    Msg=1;
    PositionVelocityVector.Fill(0);
    previousTime=millis();
  }
  if (Serial.available()){ //If something is recieved.
   a = Serial.readString();
   b= getValue(a,',',0);// this first bit number holds information about what the message is about. 
   c= getValue(a,',',1);
   d= getValue(a,',',2); 
   e= getValue(a,',',3);
   f= getValue(a,',',4);
   g= getValue(a,',',5);
   h= getValue(a,',',6);
   if(b=="0"){// this is position and rotation information calculated by the camera 
    Pos_MeasurementVector(0)= c.toFloat();// x position
    Pos_MeasurementVector(1)= d.toFloat();// x dot aka x velocity
    Pos_MeasurementVector(2)= e.toFloat();// y position
    Pos_MeasurementVector(3)= f.toFloat();// y dot aka y velocity
    Rot_MeasurementVector(0)= g.toFloat();// zeta
    Rot_MeasurementVector(1)= h.toFloat();//rotational velocity
    //The following section calculates the Kalman Gain, the Vector and the covariance matrix for position and rotation
    Pos_KalmanGain+=((Pos_StateCovarianceMatrix)*(Pos_StateCovarianceMatrix+Pos_MeasurmentVariance).Inverse())-Pos_KalmanGain ;
    PositionVelocityVector+=Pos_KalmanGain*(Pos_MeasurementVector-PositionVelocityVector);
    Pos_StateCovarianceMatrix += ((Pos_identity-Pos_KalmanGain)*Pos_StateCovarianceMatrix) - Pos_StateCovarianceMatrix; 
    Rot_KalmanGain+=((Rot_StateCovarianceMatrix)*(Rot_StateCovarianceMatrix+Rot_MeasurmentVariance).Inverse())-Rot_KalmanGain ;
    RotationVelocityVector+=Rot_KalmanGain*(Rot_MeasurementVector-RotationVelocityVector);
    Rot_StateCovarianceMatrix += ((Rot_identity-Rot_KalmanGain)*Rot_StateCovarianceMatrix) - Rot_StateCovarianceMatrix;
    // this assingning of values will be elimitaed later and matrixes will be used directly
    x_Current=PositionVelocityVector(0);
    y_Current=PositionVelocityVector(2);
    x_Desired=Desired_State(0);
    y_Desired=Desired_State(2);
    // the following section calculates the angle alpha, where the vehicle must point to get to the target PositionState
    if(y_Current<y_Desired){
     alpha = atan((x_Current-x_Desired)/(y_Current-y_Desired));
    }
    if(y_Current>y_Desired){
     if(y_Current=y_Desired){
      if(x_Current>y_Desired){
       alpha = 180-atan((x_Current-x_Desired)/(y_Current-y_Desired));
      }
      if(x_Current<y_Desired){
       alpha = 180+-atan((x_Current-x_Desired)/(y_Current-y_Desired));
      }
     }  
    if(y_Current=y_Desired){
     if(x_Current>y_Desired){
      alpha = -90;
     }
     if(x_Current<y_Desired){
      alpha = 90;
     }
    }
   }
   } 
   if(b=="1"){// in the case that it is coefficient and deadband
    Coefficients(0)=c.toFloat();// Coefficient for positon proportional
    Coefficients(1)=d.toFloat();// coefficient for position derivative
    Coefficients(2)=e.toFloat();// coefficient for rotation integral
    Coefficients(3)=f.toFloat();// coefficient for rotation proportional
    Coefficients(4)=g.toFloat();// coefficient for rotation derivative
    Coefficients(5)=h.toFloat();// coefficient for rotation integral
   } 
   if(b=="2"){// in the case that it is setup information initial position in x, y and angle
    PositionVelocityVector(0) = c.toInt();// x position
    PositionVelocityVector(2) = d.toInt();// y position
    RotationVelocityVector(0) = e.toInt();// angle position
    Msg=1;
   }
   if(b=="3"){// when waypoint information is sent
    Desired_State(0)= c.toFloat(); //x position
    Desired_State(1)= d.toFloat();// x dot position
    Desired_State(2)= e.toFloat();// y  position
    Desired_State(3)= f.toFloat(); //y dot position
    Desired_State(4)= g.toFloat(); // Rot
    Desired_State(5)= h.toFloat();// rot dot
   }
   }
  if( Msg>=1){ //this condition is always met. It is just there in case another condition needs to be added later
     mpu6050.update();
     currentTime= millis();
     delta_t= (currentTime-previousTime)*0.001;
     previousTime=currentTime;
     //the following section does the predict step of the Kalman filter
     
     Pos_StateTransition << 1, delta_t,  0,  0,
                            0,    1   ,  0,  0,
                            0,    0   ,  1,delta_t,
                            0,    0   ,  0,  1;
     Pos_ControlMatrix << sq(delta_t)/2,   0,
                                delta_t,   0,
                               0,  sq(delta_t)/2,
                               0     , delta_t; 
     Pos_ControlVector << ((mpu6050.getAccX()*-1)*9.8*cos(RotationVelocityVector(0)*PI/180)+(mpu6050.getAccY()*-1)*9.8*sin(- RotationVelocityVector(0)*PI/180)),
                          ((mpu6050.getAccX()*-1)*9.8*sin(RotationVelocityVector(0)*PI/180)+ (mpu6050.getAccY()*-1)*9.8*cos(RotationVelocityVector(0)*PI/180));
     
     Rot_StateTransition << 1, 0,
                            0, 0;
     Rot_ControlMatrix << delta_t,
                             1; 
     Rot_ControlVector<< (mpu6050.getGyroZ());
     
   // the next 4 lines is following formulas found online
     PositionVelocityVector+=((Pos_StateTransition*PositionVelocityVector)+(Pos_ControlMatrix*Pos_ControlVector))-PositionVelocityVector;
     Pos_StateCovarianceMatrix +=(Pos_StateTransition*Pos_StateCovarianceMatrix*~Pos_StateTransition)-Pos_StateCovarianceMatrix;
     Pos_StateCovarianceMatrix +=(Pos_ControlMatrix*Pos_PredictionVariance*~Pos_ControlMatrix)-Pos_StateCovarianceMatrix;
     RotationVelocityVector+=((Rot_StateTransition*RotationVelocityVector)+(Rot_ControlMatrix*Rot_ControlVector))-RotationVelocityVector;
     Rot_StateCovarianceMatrix+=(Rot_StateTransition* Rot_StateCovarianceMatrix*~Rot_StateTransition)+(Rot_ControlMatrix*~Rot_ControlMatrix*Rot_PredictionVariance)-Rot_StateCovarianceMatrix;
 // the following section will correct the Rot_Velocity vector to make sure its output is always between -180 and 180 just like the output of the raspbeery
     
     zeta= RotationVelocityVector(0);
     if(abs(int(zeta))%360>180){
      zeta=(-180+((abs(int(zeta))%180)))*getSign(zeta)+(zeta-int(zeta));
     }
     if(abs(int(zeta))%360<180){
       zeta=1*(int(zeta)%180)+(zeta-int(zeta));
     }
     if(abs(int(zeta)%360==180)){
       zeta=getSign(int(zeta))*(-180+abs(zeta-int(zeta)));
     }  
     RotationVelocityVector(0)=zeta;    
     // this is the control section    
     if((zeta <= 180 && zeta >= -180)){
   //^fliters out unexpected values
   //the following calculates the error with the diference between the desired state of waypoints and current state
      for(int i= 0; i<4;i++){
       error(i)= Desired_State(i)-PositionVelocityVector(i);
      }
  //  error(5)=Desired_State(5)-RotationVelocityVector(5);
      error(4)=170-zeta;// error for angle
      error(5)=Desired_State(5)-RotationVelocityVector(1);// error for angle derivative
      
      error(1)= error(1)*(-1*getSign(error(0)));// the error derivative in x is the same as x velocity with sign adjusted 
      error(3)= error(3)*(-1*getSign(error(2)));// the error derivative in y is the same as y velocity with sign adjusted 
      
      error_Rot= error(4);
     // errorDot_Rot= error(5)*(-1*getSign(error(4)));// the error derivative in angle is the same as rotational velocity with sign adjusted 
      errorDot_Rot=( errorDot_Rot+ -1*error(5))/2;
      error_Pos = sqrt(error(0)*error(0)+ error(2)*error(2)) ;// the magnitude of the error in x and y 
      errorDot_Pos = sqrt(error(1)*error(1)+ error(3)*error(3));// the magnitude of the error derivatives in x and y
      if (error_Rot>180 or error_Rot<-180){ //interprets error angles outside of the range in module 180 (taking in count sign)
        error_Rot = error_Rot-getSign(error_Rot)*360;
      }
      
    // end of error correction
      if(abs(error_Rot)<30){
      errorInt_Rot+= delta_t*error_Rot;
      control_Rot = (control_Rot+(-Coefficients(3)*error_Rot +Coefficients(4)*errorDot_Rot)+Coefficients(5)*errorInt_Rot)/2; //creates a signal using the 
      }
      else{
        control_Rot=(-Coefficients(3)*error_Rot +Coefficients(4)*errorDot_Rot);
      }
      control_Pos = -Coefficients(0)*error_Pos +Coefficients(1)*errorDot_Pos;//creates PID signal 
     
      if (abs(control_Rot<256)){
       control_Rot= map(control_Rot,-255,255,-120,120);// maping provides better control while not providing too much voltage to propellers
      }
      if(abs(control_Rot)>=256){
      control_Rot= 120*(getSign(control_Rot));// sets a ceiling for control signal values for rotation
      }
      if (abs(control_Pos)<256){
       control_Pos= map(control_Pos,-255,255,-120,120);// maping provides better control while not providing too much voltage to propellers
      }
      if(abs(control_Pos)>=256){
      control_Pos= 120*(getSign(control_Pos));// sets a ceiling for control signal values for rotation
      }
      // a similar function must be developed to prevent the thrusters from recieving too much voltage in forward and backward
      delay(30);// not sure if necesary
      if(abs(control_Rot)>abs(errorDeadband_Rot)){ //checks if the error is outside of the deadband 
       if(control_Rot<0){
        pivotLeft(abs(control_Rot));// if signal is negative rotate counterclockwise
        digitalWrite(LED_BUILTIN, HIGH);// used for bug testing
       }
       if(control_Rot>0){
        pivotRight(abs(control_Rot));// if signal is positive rotate clockwise
        digitalWrite(LED_BUILTIN, LOW);//used for bug testing
       } 
      }
      if(abs(control_Rot)<abs(errorDeadband_Rot)){
       if(abs(control_Pos)>abs(errorDeadband_Pos)){// if the control signal is out of deadband then:
        if(control_Pos<0){
         backward(100,abs(control_Pos));// move backward if negative
        }
        if(control_Pos>0){
         forward(100,abs(control_Pos));// move forward if positive
        }
       }
       if(abs(control_Pos)<abs(errorDeadband_Pos)){
        stopMotors();// if the control_Pos signal is within deadband stop motors
       }
      }
     } 
    }
    //code for bugtestig
     Serial.print(delta_t);
     Serial.print(" ");
     //Serial.print(zeta);
     Serial.print(" ");
     Serial.print(error_Rot);
     Serial.print(" ");
     //Serial.print(errorInt_Rot);
     Serial.print(" ");
     Serial.print(errorDot_Rot);
     Serial.print(" ");
     Serial.print(control_Rot);
     Serial.println(" ");
     /*
     Serial << "" <<PositionVelocityVector ;     
     Serial << "" <<RotationVelocityVector << '\n';
     */
     if((abs(error_Rot)<20 and abs(error_Rot)>5)){
     digitalWrite(blueLED,HIGH);
     }
     else{
     digitalWrite(blueLED,LOW);
      }
      if(abs(error_Rot)<10){
     digitalWrite(greenLED,HIGH);
     }
     else{
     digitalWrite(greenLED,LOW);
      }
      if(abs(error_Rot)< 100  and abs(error_Rot)>15){
     digitalWrite(yellowLED,HIGH);
     }
     else{
     digitalWrite(yellowLED,LOW);
      }
     if(error_Rot<180 and error_Rot>80){
     digitalWrite(redLED,HIGH);
     }
     else{
     digitalWrite(redLED,LOW);
      }
      
  }


int getSign(float x){// this function finds the sign of a float or int and returns -1 or 1
  if(x!=0){
    return x/abs(x);
      }
  else{
    return 0;
  }
}
  

String getValue(String data, char separator, int index){
    int found = 0;
    int strIndex[] = { 0, -1 };
    int maxIndex = data.length() - 1;

    for (int i = 0; i <= maxIndex && found <= index; i++) {
        if (data.charAt(i) == separator || i == maxIndex) {
            found++;
            strIndex[0] = strIndex[1] + 1;
            strIndex[1] = (i == maxIndex) ? i+1 : i;
        }
    }
    return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}
