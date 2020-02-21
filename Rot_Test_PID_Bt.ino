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

//declarando las variables que usaremos para leer el Serial
int zeta,zetaOffset;//esta variable guarda el angulo donde esta apuntando el vehiculo y el offset para el gyroscopio 
int alpha;
float mag; //distancia al centro
String a;// esta variable guarda temporalmete lo que se recibe del serial
String b,c,d,e,f,g,h,i;// estas variable guardan un substring
int error_Rot, previousError_Rot;
float errorDot_Rot;
float control_Rot;
int error_Pos, previousError_Pos;
float errorDot_Pos;
float control_Pos;
float k_P_Rot=1;// El coefficient del proportional  del angulo
float k_D_Rot =0;// el coefficient de la derivada del angulo
float k_P_Pos=1;// El coefficient del proportional del magnitude del error en position
float k_D_Pos =0;// el coefficient de la derivada del magnitud del error en position
float x_Current,y_Current, x_Desired, y_Desired;
int previousTime;
int currentTime;// these two are needed to find the derivative of the error
int errorDeadband_Rot=1;// the deadband makes sure that the system is not overcompensating for negligable error
int errorDeadband_Pos=1;
int Msg =0;// this variable stores the message number so classify the first and second as coefficients
BLA::Matrix<4,4> Pos_StateTransition;
BLA::Matrix<4,2> Pos_ControlMatrix;
BLA::Matrix<2> Pos_ControlVector;// the accelerometer output
BLA::Matrix<4,4> Pos_PredictionVariance;// should be a identity matrix multiplied by the accelerometer variance
BLA::Matrix<4,4> Pos_MeasurmentVariance;//should be the camera variance diagonally and 0s in the remaining spaces
BLA::Matrix<4> Pos_MeasurementVector;
BLA::Matrix<4,4> Pos_StateCovarianceMatrix;
BLA::Matrix<4,4> Pos_H;
BLA::Matrix<4> PositionVelocityVector;
BLA::Matrix<4,4> Pos_KalmanGain; 
BLA::Matrix<4,4> Pos_identity = {1,0,0,0,
                                 0,1,0,0,
                                 0,0,1,0,
                                 0,0,0,1};

BLA::Matrix<2,2> Rot_StateTransition;
BLA::Matrix<2> Rot_ControlMatrix;
BLA::Matrix<1> Rot_ControlVector;// the gyro output
BLA::Matrix<2,2> Rot_PredictionVariance;// should be a identity matrix multiplied by the gyro variance
BLA::Matrix<2,2> Rot_MeasurmentVariance;//should be the camera variance diagonally and 0s in the remaining spaces
BLA::Matrix<2> Rot_MeasurementVector;
BLA::Matrix<2,2> Rot_StateCovarianceMatrix;
BLA::Matrix<2,2> Rot_H;
BLA::Matrix<2> RotationVelocityVector;
BLA::Matrix<2,2> Rot_KalmanGain; 
BLA::Matrix<2,2> Rot_identity = {1,0, 
                                 0,1};
BLA::Matrix<4> Coefficients;
BLA::Matrix<6> Desired_State;
BLA::Matrix<6> error;
float delta_t;//  the time between measurements
void setup() {
  setMotors();  // turn motors on
  Serial.begin(9600);
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
    h= getValue(a,',',5);
    i= getValue(a,',',6);
   if(b==0){// in the case that it is information
    Pos_MeasurementVector(1)= c.toFloat();
    Pos_MeasurementVector(2)= d.toFloat();
    Pos_MeasurementVector(3)= e.toFloat();
    Pos_MeasurementVector(4)= f.toFloat();
    Rot_MeasurementVector(1)= g.toFloat();
    Rot_MeasurementVector(2)= h.toFloat();
    //The following section calculates the Kalman Gain, the Vector and the covariance matrix for position and rotation
    Pos_KalmanGain+=((Pos_StateCovarianceMatrix)*(Pos_StateCovarianceMatrix+Pos_MeasurmentVariance).Inverse())-Pos_KalmanGain ;
    PositionVelocityVector+=Pos_KalmanGain*(Pos_MeasurementVector-PositionVelocityVector);
    Pos_StateCovarianceMatrix += ((Pos_identity-Pos_KalmanGain)*Pos_StateCovarianceMatrix) - Pos_StateCovarianceMatrix;
 
    Rot_KalmanGain+=((Rot_StateCovarianceMatrix)*(Rot_StateCovarianceMatrix+Rot_MeasurmentVariance).Inverse())-Rot_KalmanGain ;
    RotationVelocityVector+=Rot_KalmanGain*(Rot_MeasurementVector-RotationVelocityVector);
    Rot_StateCovarianceMatrix += ((Rot_identity-Rot_KalmanGain)*Rot_StateCovarianceMatrix) - Rot_StateCovarianceMatrix;
    
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
   if(b=1){// in the case that it is coefficient information
      Coefficients(1)=e.toFloat();// El coefficient del proportional del magnitude del error en position
      Coefficients(2)=f.toFloat();// coefficient for derivative in position
      Coefficients(3)=c.toFloat();// El coefficient del proportional  del angulo
      Coefficients(4)=d.toFloat();// el coefficient de la derivada del angulo
      
   } 
   if(b==2){// in the case that it is setup information
    RotationVelocityVector(1) = c.toInt();// this must be changed before testing
    Msg=1;
   if(b==3){// in the case that it is information
    Desired_State(1)= c.toFloat(); //x position
    Desired_State(2)= d.toFloat();// x dot position
    Desired_State(3)= e.toFloat();// y  position
    Desired_State(4)= f.toFloat(); //y dot position
    Desired_State(5)= g.toFloat(); // Rot
    Desired_State(6)= h.toFloat();// rot dot
       }
  }
 
    if( Msg>=1){ //this condition is always met. It is just there in case another condition needs to be added later
     delta_t= currentTime-previousTime;
     //the followin section does the predict step of the Kalman filter
     
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
   
  PositionVelocityVector+=((Pos_StateTransition*PositionVelocityVector)+(Pos_ControlMatrix*Pos_ControlVector))-PositionVelocityVector;
  Pos_StateCovarianceMatrix +=(Pos_StateTransition* Pos_StateCovarianceMatrix*~Pos_StateTransition)+(Pos_ControlMatrix*~Pos_ControlMatrix*Pos_PredictionVariance)-Pos_StateCovarianceMatrix;
  RotationVelocityVector+=((Rot_StateTransition*RotationVelocityVector)+(Rot_ControlMatrix*Rot_ControlVector))-RotationVelocityVector;
  Rot_StateCovarianceMatrix +=(Rot_StateTransition* Rot_StateCovarianceMatrix*~Rot_StateTransition)+(Rot_ControlMatrix*~Rot_ControlMatrix*Rot_PredictionVariance)-Rot_StateCovarianceMatrix;
 // the following section will correct the Rot_Velocity vector to make sure its output is always between -180 and 180 just like the output of the raspbeery

    zeta= (int)PositionVelocityVector(1);
      if(abs(zeta)%360>180) {
      zeta=(-180+((abs(zeta)%180)))*abs((zeta)/zeta);
      }
      if(abs(zeta)%360<180){
        zeta=1*(zeta%180);
      }
      if(abs(zeta%360==180)){
        zeta=180;
      }  
    PositionVelocityVector(1)=zeta;
     
     
   
     // this section starts the control loop
     currentTime= millis();
  if((zeta <= 180 && zeta >= -180)){
   //fliters out unexpected values
   
   
   //This calculates the error
    for(int i= 1; i<5;i++){
   error(i)= Desired_State(i)-PositionVelocityVector(i);
    }
  // error(5)=Desired_State(5)-RotationVelocityVector(5);
    error(5)=alpha-RotationVelocityVector(5);
   error(6)=Desired_State(6)-RotationVelocityVector(6);
   error(2)= error(2)*(-error(1)/abs(error(1)));
   error(4)= error(4)*(-error(3)/abs(error(3)));
   error_Rot= error(5);
   errorDot_Rot= error(6)*(-error_Rot/abs(error_Rot));
   error_Pos = sqrt(error(1)*error(1)+ error(3)*error(3)) ;
   errorDot_Pos = sqrt(error(2)*error(2)+ error(4)*error(4));
  
   
   if (error_Rot>180 or error_Rot<-180){ //interprets angles outside of the range in module 180 (taking in count sign)
    if (error_Rot>0){
     error_Rot = error_Rot-360;
    }
    if (error_Rot<0){
     error_Rot = error_Rot+360;
    }
   }
    // end of error correction
    control_Rot = -Coefficients(3)*error_Rot +Coefficients(4)*errorDot_Rot; //creates a signal using the PID
   control_Pos = -Coefficients(1)*error_Pos +Coefficients(2)*errorDot_Pos;
    
   
   if (control_Rot<256&&control_Rot>-256){
   control_Rot= map(control_Rot,-255,255,-120,120);
   
   }
   if(control_Rot>=256||control_Rot<=-256){
   control_Rot= 120*(abs(control_Rot)/control_Rot);
   }
   delay(10);// not sure if necesary
   previousTime=currentTime;
   if(abs(control_Rot)>abs(errorDeadband_Rot)){ //checks if the error is outside of the deadband 
    if (control_Rot<0){
      pivotLeft(abs(control_Rot));
      digitalWrite(LED_BUILTIN, HIGH);
    }
    if ( control_Rot> 0) {
      pivotRight(abs(control_Rot));
      digitalWrite(LED_BUILTIN, LOW);
    } 
    }
   if(abs(control_Rot)<abs(errorDeadband_Rot)){
    if(abs(control_Pos)>abs(errorDeadband_Pos)){
    if (control_Pos<0){
      backward(100,abs(control_Pos));
    }
    if ( control_Pos> 0) {
      forward(100,abs(control_Pos));
    }
    }
    if(abs(control_Pos)<abs(errorDeadband_Pos)){
  stopMotors();
    }
   }
  }
}}
}
}


String getValue(String data, char separator, int index)
{
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
