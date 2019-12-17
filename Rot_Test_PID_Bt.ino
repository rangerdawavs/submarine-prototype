/* Originalmente de: Tomás de Camino Beck
 Editado para recibir comandos por el Serial
 Luego re editado para recibir un angulo por BlueTooth, calcular la diferencia entre el calculo deciado y el actual y usar un PD control system para mantener un angulo deseado
*/
#include <MPU6050_tockn.h>
#include <Wire.h>
#include <RobotKit.h>
MPU6050 mpu6050(Wire);

// robotkit es una biblioteca de donde sacamos los functions de forward, backward y rotations usando dos motores

//declarando las variables que usaremos para leer el Serial
int zeta, zetaOffset;//esta variable guarda el angulo donde esta apuntando el vehiculo y el offset para el gyroscopio 
int alpha;//angulo al cual se tiene que llegar
float mag; //distancia al centro
String a;// esta variable guarda temporalmete lo que se recibe del serial
String b,c,d,e,f;// estas variable guardan un substring
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
int Msg =4;// this variable stores the message number so classify the first and second as coefficients


void setup() {
  setMotors();  // turn motors on
  Serial.begin(9600);
  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);
} 
void loop() {
  // put your main code here, to run repeatedly:

  if (Serial.available()){ //If something is detected.
    a = Serial.readString();
    b= getValue(a,',',0);// this first bit number holds information about what the message is about. 
    c= getValue(a,',',1);
    d= getValue(a,',',2); //breaks down the triples into three strings
    e= getValue(a,',',3);
    f= getValue(a,',',4);
   if(b==0){// in the case that it is just position information
    x_Current= c.toFloat();
    y_Current= d.toFloat();
    x_Desired= e.toFloat();
    y_Desired= f.toFloat();
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
    
    mag = sqrt(sq(x_Current-x_Desired)+sq(y_Current-y_Desired));
   } 
   if(b=1){// in the case that it is coefficient information
      k_P_Rot=c.toFloat();// El coefficient del proportional  del angulo
      k_D_Rot=d.toFloat();// el coefficient de la derivada del angulo
      k_P_Pos=e.toFloat();// El coefficient del proportional del magnitude del error en position
      k_D_Pos=f.toFloat();
   } 
   if(b==2){// in the case that it is the intinial angle setup information
    zetaOffset = c.toInt();
       }  
  }
    if( Msg>=4){ //this condition is always met. It is just there in case another condition needs to be added later
    zeta = mpu6050.getAngleZ()+ zetaOffset;
   
     Serial.print(zeta);
     Serial.print(',');
     Serial.print(alpha);
     Serial.print(',');
     Serial.println(mag); //prints data
  if((zeta <= 180 && zeta >= -180)){
   //fliters out unexpected values
   currentTime= millis(); //records time since the last message was received
   error_Rot= zeta-alpha; // calculates the error of the angle
   error_Pos= mag-0; //calculates the error in the position
   if (error_Rot>180 or error_Rot<-180){ //interprets angles outside of the range in module 180 (taking in count sign)
    if (error_Rot>0){
     error_Rot = error_Rot-360;
    }
    if (error_Rot<0){
     error_Rot = error_Rot+360;
    }
   }
    errorDot_Rot =(error_Rot-previousError_Rot)/(currentTime-previousTime); //calculates derivative
    errorDot_Pos =(error_Pos-previousError_Pos)/(currentTime-previousTime);
    previousError_Rot= error_Rot;
   previousError_Pos= error_Pos;
    previousTime=currentTime;    
    control_Rot = -k_P_Rot*error_Rot +k_D_Rot*errorDot_Rot; //creates a signal using the PID
   control_Pos = -k_P_Pos*error_Pos +k_D_Pos*errorDot_Pos;
    if(abs(control_Rot)>abs(errorDeadband_Rot) && -abs(errorDeadband_Rot)> -abs(control_Rot)){ //checks if the error is outside of the deadband 
    if (control_Rot<0){
      pivotLeft(100,abs(control_Rot));
    }
    if ( control_Rot> 0) {
      pivotRight(100,abs(control_Rot));
    }
  stopMotors();
    }
   if(abs(control_Rot)<abs(errorDeadband_Rot)&& -abs(errorDeadband_Rot)< -abs(control_Rot)){
    if(abs(control_Pos)>abs(errorDeadband_Pos) && -abs(errorDeadband_Pos)> -abs(control_Pos)){
    if (control_Pos<0){
      backward(100,abs(control_Pos));
      //Serial.println("left");
    }
    if ( control_Pos> 0) {
      forward(100,abs(control_Pos));
    }
  stopMotors();
   }
  }
}
  }

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
