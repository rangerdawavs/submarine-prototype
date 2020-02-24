/* Originalmente de: Tomás de Camino Beck
 Editado para recibir comandos por el Serial
 Luego re editado para recibir un angulo por BlueTooth, calcular la diferencia entre el calculo deciado y el actual y usar un PD control system para mantener un angulo deseado
*/
#include <MPU6050_tockn.h>
#include <Wire.h>
#include <math.h>
#include <RobotKit.h>
MPU6050 mpu6050(Wire);
#include <BasicLinearAlgebra.h>
using namespace BLA;
// robotkit es una biblioteca de donde sacamos los functions de forward, backward y rotations usando dos motores
int zeta;//esta variable guarda el angulo donde esta apuntando el vehiculo y el offset para el gyroscopio 
int alpha;
String a;// esta variable guarda temporalmete lo que se recibe del serial
String b,c,d,e,f,g,h;// estas variable guardan un substring
int error_Rot,error_Pos;
float errorDot_Pos,errorDot_Rot;
float control_Pos,control_Rot;
float k_P_Rot=1.4;// El coefficient del proportional  del angulo
float k_D_Rot =4;// el coefficient de la derivada del angulo
float k_P_Pos=1;// El coefficient del proportional del magnitude del error en position
float k_D_Pos =0;// el coefficient de la derivada del magnitud del error en position
float x_Current,y_Current, x_Desired, y_Desired;
int previousTime,currentTime;// used to calculate time between measurements
float delta_t;// actual time between measurements
int errorDeadband_Rot=4;// the deadband makes sure that the system is not overcompensating for negligable error
int errorDeadband_Pos=1;
int Msg =0;// this variable stores the message number so classify the first and second as coefficients
BLA::Matrix<4,4> Pos_StateTransition;
BLA::Matrix<4,2> Pos_ControlMatrix;
BLA::Matrix<2>   Pos_ControlVector;// the accelerometer output
BLA::Matrix<4,4> Pos_PredictionVariance;// should be a identity matrix multiplied by the accelerometer variance
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
BLA::Matrix<4> Coefficients;
BLA::Matrix<6> Desired_State;
BLA::Matrix<6> error;
void setup() {
  setMotors();  // turn motors on
  Serial.begin(9600);
  Serial.setTimeout(10);
  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);
} 
void loop() {
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
    Pos_MeasurementVector(1)= c.toFloat();// x position
    Pos_MeasurementVector(2)= d.toFloat();// x dot aka x velocity
    Pos_MeasurementVector(3)= e.toFloat();// y position
    Pos_MeasurementVector(4)= f.toFloat();// y dot aka y velocity
    Rot_MeasurementVector(1)= g.toFloat();// zeta
    Rot_MeasurementVector(2)= h.toFloat();//rotational velocity
    //The following section calculates the Kalman Gain, the Vector and the covariance matrix for position and rotation
    Pos_KalmanGain+=((Pos_StateCovarianceMatrix)*(Pos_StateCovarianceMatrix+Pos_MeasurmentVariance).Inverse())-Pos_KalmanGain ;
    PositionVelocityVector+=Pos_KalmanGain*(Pos_MeasurementVector-PositionVelocityVector);
    Pos_StateCovarianceMatrix += ((Pos_identity-Pos_KalmanGain)*Pos_StateCovarianceMatrix) - Pos_StateCovarianceMatrix; 
    Rot_KalmanGain+=((Rot_StateCovarianceMatrix)*(Rot_StateCovarianceMatrix+Rot_MeasurmentVariance).Inverse())-Rot_KalmanGain ;
    RotationVelocityVector+=Rot_KalmanGain*(Rot_MeasurementVector-RotationVelocityVector);
    Rot_StateCovarianceMatrix += ((Rot_identity-Rot_KalmanGain)*Rot_StateCovarianceMatrix) - Rot_StateCovarianceMatrix;
    // this assingning of values will be elimitaed later and matrixes will be used directly
    x_Current=PositionVelocityVector(1);
    y_Current=PositionVelocityVector(3);
    x_Desired=Desired_State(1);
    y_Desired=Desired_State(3);
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
   if(b=="1"){// in the case that it is coefficient and deadband
    Coefficients(1)=c.toFloat();// Coefficient for positon proportional
    Coefficients(2)=d.toFloat();// coefficient for position derivative
    Coefficients(3)=e.toFloat();// coefficient for rotation proportional
    Coefficients(4)=f.toFloat();// coefficient for rotation derivative
    errorDeadband_Pos=g.toFloat();// deadband for position
    errorDeadband_Rot=h.toFloat();// deadband for rotation
   } 
   if(b=="2"){// in the case that it is setup information initial position in x, y and angle
    PositionVelocityVector(1) = c.toInt();// x position
    PositionVelocityVector(3) = d.toInt();// y position
    RotationVelocityVector(1) = e.toInt();// angle position
    Msg=1;
   }
   if(b=="3"){// when waypoint information is sent
    Desired_State(1)= c.toFloat(); //x position
    Desired_State(2)= d.toFloat();// x dot position
    Desired_State(3)= e.toFloat();// y  position
    Desired_State(4)= f.toFloat(); //y dot position
    Desired_State(5)= g.toFloat(); // Rot
    Desired_State(6)= h.toFloat();// rot dot
   }
  }
  if( Msg>=1){ //this condition is always met. It is just there in case another condition needs to be added later
     currentTime= millis();
     delta_t= (currentTime-previousTime)*0.001;
     previousTime=currentTime;
     //the following section does the predict step of the Kalman filter
     Pos_StateTransition << 1, delta_t,  0,  0,
                            0,    1   ,  0,  0,
                            0,    0   ,  1,delta_t,
                            0,    0   ,  0,  1;
     Pos_ControlMatrix << sq(delta_t)/2,   0,
                          0,         delta_t,
                          sq(delta_t)/2,    0,
                          0       , delta_t; 
     Pos_ControlVector << ((mpu6050.getAccX())*sin(RotationVelocityVector(1)*PI/180)+(mpu6050.getAccY())*cos(RotationVelocityVector(1)*PI/180)),
                          ((mpu6050.getAccX())*cos(RotationVelocityVector(1)*PI/180)+ (mpu6050.getAccY())*sin(-RotationVelocityVector(1)*PI/180));
     Rot_StateTransition << 1, 0,
                            0, 0;
     Rot_ControlMatrix << delta_t,
                             1; 
     Rot_ControlVector<< (mpu6050.getGyroZ());
   // the next 4 lines is following formulas found online
     PositionVelocityVector+=((Pos_StateTransition*PositionVelocityVector)+(Pos_ControlMatrix*Pos_ControlVector))-PositionVelocityVector;
     Pos_StateCovarianceMatrix +=(Pos_StateTransition* Pos_StateCovarianceMatrix*~Pos_StateTransition)+(Pos_ControlMatrix*~Pos_ControlMatrix*Pos_PredictionVariance)-Pos_StateCovarianceMatrix;
     RotationVelocityVector+=((Rot_StateTransition*RotationVelocityVector)+(Rot_ControlMatrix*Rot_ControlVector))-RotationVelocityVector;
     Rot_StateCovarianceMatrix+=(Rot_StateTransition* Rot_StateCovarianceMatrix*~Rot_StateTransition)+(Rot_ControlMatrix*~Rot_ControlMatrix*Rot_PredictionVariance)-Rot_StateCovarianceMatrix;
 // the following section will correct the Rot_Velocity vector to make sure its output is always between -180 and 180 just like the output of the raspbeery
     zeta= (int)PositionVelocityVector(1);
     if(abs(zeta)%360>180){
      zeta=(-180+((abs(zeta)%180)))*getSign(zeta);
     }
     if(abs(zeta)%360<180){
       zeta=1*(zeta%180);
     }
     if(abs(zeta%360==180)){
       zeta=180;
     }  
     PositionVelocityVector(1)=zeta;    
     // this is the control section    
     if((zeta <= 180 && zeta >= -180)){
   //^fliters out unexpected values
   //the following calculates the error with the diference between the desired state of waypoints and current state
      for(int i= 1; i<5;i++){
       error(i)= Desired_State(i)-PositionVelocityVector(i);
      }
  //  error(5)=Desired_State(5)-RotationVelocityVector(5);
      error(5)=alpha-RotationVelocityVector(5);// error for angle
      error(6)=Desired_State(6)-RotationVelocityVector(6);// error for angle derivative
      error(2)= error(2)*(-1*getSign(error(1)));// the error derivative in x is the same as x velocity with sign adjusted 
      error(4)= error(4)*(-1*getSign(error(3)));// the error derivative in y is the same as y velocity with sign adjusted 
      error_Rot= error(5);
      errorDot_Rot= error(6)*(-1*getSign(error(5)));// the error derivative in angle is the same as rotational velocity with sign adjusted 
      error_Pos = sqrt(error(1)*error(1)+ error(3)*error(3)) ;// the magnitude of the error in x and y 
      errorDot_Pos = sqrt(error(2)*error(2)+ error(4)*error(4));// the magnitude of the error derivatives in x and y
      if (error_Rot>180 or error_Rot<-180){ //interprets error angles outside of the range in module 180 (taking in count sign)
       if (error_Rot>0){
        error_Rot = error_Rot-360;
       }
       if (error_Rot<0){
        error_Rot = error_Rot+360;
       }
      }
    // end of error correction
      control_Rot = -Coefficients(3)*error_Rot +Coefficients(4)*errorDot_Rot; //creates a signal using the PID
      control_Pos = -Coefficients(1)*error_Pos +Coefficients(2)*errorDot_Pos;//creates PID signal 
      if (control_Rot<256&&control_Rot>-256){
       control_Rot= map(control_Rot,-255,255,-120,120);// maping provides better control while not providing too much voltage to propellers
      }
      if(control_Rot>=256||control_Rot<=-256){
      control_Rot= 120*(getSign(control_Rot));// sets a ceiling for control signal values for rotation
      }
      // a similar function must be developed to prevent the thrusters from recieving too much voltage in forward and backward
      delay(10);// not sure if necesary
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
  }
}

int getSign(float x){// this function finds the sign of a float or int and returns -1 or 1
 return x/abs(x);
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
