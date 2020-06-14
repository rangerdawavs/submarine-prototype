#include <BasicLinearAlgebra.h>



using namespace BLA;
// in this example the process will be done twice. Instead of just doing it once with a matrix that includes position and angle,
// two separate matrixes wil be used to make explanations easier. The example will just use one dimension for position. For the angle it
//will omit a process that changes the format of the angle to match that of the raspberrypi.

 BLA::Matrix<2,2> Pos_StateTransition;
BLA::Matrix<2> Pos_ControlMatrix;
BLA::Matrix<1> Pos_ControlVector;// the accelerometer output
BLA::Matrix<2,2> Pos_PredictionVariance;// should be a identity matrix multiplied by the accelerometer variance
BLA::Matrix<2,2> Pos_MeasurmentVariance;//should be the camera variance diagonally and 0s in the remaining spaces
BLA::Matrix<2> Pos_MeasurementVector;
BLA::Matrix<2,2> Pos_StateCovarianceMatrix;
BLA::Matrix<2,2> Pos_H;
BLA::Matrix<2> PositionVelocityVector;
BLA::Matrix<2,2> Pos_KalmanGain; 
BLA::Matrix<2,2> Pos_identity = {1,0, 
                             0,1};

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
float delta_t;//  the time between measurements
int MsgRecv ; // if a message is recieved by bluetooth
void setup(){
 Serial.begin(9600);

}


void loop(){
  // these are placeholder values to test the matrix library
  PositionVelocityVector << 2,
                            1; 
  Pos_ControlVector << 1;                   
  delta_t= 2;
// placeholder values end here
 
 
  if(true){// This is the condition to do the update step which should be every loop 
 Pos_StateTransition << 1, delta_t,
                        0,    1   ;
 Pos_ControlMatrix << sq(delta_t)/2,
                    delta_t;       
 Rot_StateTransition << 1, 0,
                        0, 0;
 Rot_ControlMatrix << delta_t,
                        1;      
   
  PositionVelocityVector+=((Pos_StateTransition*PositionVelocityVector)+(Pos_ControlMatrix*Pos_ControlVector))-PositionVelocityVector;
  Pos_StateCovarianceMatrix +=(Pos_StateTransition* Pos_StateCovarianceMatrix*~Pos_StateTransition)+(Pos_ControlMatrix*~Pos_ControlMatrix*Pos_PredictionVariance)-Pos_StateCovarianceMatrix;
  RotationVelocityVector+=((Rot_StateTransition*RotationVelocityVector)+(Rot_ControlMatrix*Rot_ControlVector))-RotationVelocityVector;
  Rot_StateCovarianceMatrix +=(Rot_StateTransition* Rot_StateCovarianceMatrix*~Rot_StateTransition)+(Rot_ControlMatrix*~Rot_ControlMatrix*Rot_PredictionVariance)-Rot_StateCovarianceMatrix;
  Serial << "Predict step new position velocity vector: " <<PositionVelocityVector << '\n';
  Serial << " Predict step new position state covariance matrix: " <<Pos_StateCovarianceMatrix << '\n';
  Serial << "Predict step new rotation velocity vector: " <<RotationVelocityVector << '\n';
  Serial << " Predict step new rotation state covariance matrix: " <<Rot_StateCovarianceMatrix << '\n';

  }
if(MsgRecv>=0){//if a camera measurement is recieved from the raspberry Pi 
Pos_KalmanGain+=((Pos_StateCovarianceMatrix)*(Pos_StateCovarianceMatrix+Pos_MeasurmentVariance).Inverse())-Pos_KalmanGain ;
PositionVelocityVector+=Pos_KalmanGain*(Pos_MeasurementVector-PositionVelocityVector);
Pos_StateCovarianceMatrix += ((Pos_identity-Pos_KalmanGain)*Pos_StateCovarianceMatrix) - Pos_StateCovarianceMatrix;
 
Rot_KalmanGain+=((Pos_StateCovarianceMatrix)*(Pos_StateCovarianceMatrix+Pos_MeasurmentVariance).Inverse())-Pos_KalmanGain ;
RotationVelocityVector+=Pos_KalmanGain*(Pos_MeasurementVector-PositionVelocityVector);
Rot_StateCovarianceMatrix += ((Rot_identity-Rot_KalmanGain)*Rot_StateCovarianceMatrix) - Rot_StateCovarianceMatrix;

  Serial << "Update step position Kalman Gain: " <<Pos_KalmanGain << '\n';
  Serial << "Update step new position velocity vector: " <<PositionVelocityVector << '\n';
  Serial << "Update step new position state covariance matrix: " <<Pos_StateCovarianceMatrix << '\n';
 
  Serial << "Update step Rotation Kalman Gain: " <<Rot_KalmanGain << '\n';
  Serial << "Update step new rotation velocity vector: " <<RotationVelocityVector << '\n';
  Serial << "Update step new Rotation state covariance matrix: " <<Rot_StateCovarianceMatrix << '\n';

}
}
