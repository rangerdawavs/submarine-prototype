#include <BasicLinearAlgebra.h>



using namespace BLA;
 BLA::Matrix<2,2> StateTransition;
BLA::Matrix<2> ControlMatrix;
BLA::Matrix<1> ControlVector;// the accelerometer output
BLA::Matrix<2,2> PredictionVariance;// should be a identity matrix multiplied by the accelerometer variance
BLA::Matrix<2,2> MeasurmentVariance;//should be the camera variance diagonally and 0s in the remaining spaces
BLA::Matrix<2> MeasurementVector;
BLA::Matrix<2,2> StateCovarianceMatrix;
BLA::Matrix<2,2> H;
BLA::Matrix<2> PositionVelocityVector;
BLA::Matrix<2,2> KalmanGain; 
BLA::Matrix<2,2> identity = {1,0, 
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
  ControlVector << 1;                   
  delta_t= 2;
// placeholder values end here
  if(true){// This is the condition to do the update step which should be every loop 
  StateTransition << 1, delta_t,
                    0, 1;
 ControlMatrix << sq(delta_t)/2,
                    delta_t;                   

 
  PositionVelocityVector+=((StateTransition*PositionVelocityVector)+(ControlMatrix*ControlVector))-PositionVelocityVector;
  StateCovarianceMatrix +=(StateTransition* StateCovarianceMatrix*~StateTransition)+(ControlMatrix*~ControlMatrix*PredictionVariance)-StateCovarianceMatrix;
  Serial << "Predict step new position velocity vector: " <<PositionVelocityVector << '\n';
  Serial << " Predict step new state covariance matrix: " <<StateCovarianceMatrix << '\n';

  }
if(MsgRecv>=0){//if a camera measurement is recieved from the raspberry Pi 
KalmanGain+=((StateCovarianceMatrix)*(StateCovarianceMatrix+MeasurmentVariance).Inverse())-KalmanGain ;
PositionVelocityVector+=KalmanGain*(MeasurementVector-PositionVelocityVector);
StateCovarianceMatrix += ((identity-KalmanGain)*StateCovarianceMatrix) - StateCovarianceMatrix;


  Serial << "Update step Kalman Gain: " <<PositionVelocityVector << '\n';
  Serial << "Update step new position velocity vector: " <<PositionVelocityVector << '\n';
  Serial << "Update step new state covariance matrix: " <<StateCovarianceMatrix << '\n';


}
}
