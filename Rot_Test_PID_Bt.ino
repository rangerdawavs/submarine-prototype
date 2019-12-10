/* Originalmente de: Tomás de Camino Beck
 Editado para recibir comandos por el Serial
 Luego re editado para recibir un angulo por BlueTooth, calcular la diferencia entre el calculo deciado y el actual y usar un PD control system para mantener un angulo deseado
*/

#include <RobotKit.h>
// robotkit es una biblioteca de donde sacamos los functions de forward, backward y rotations usando dos motores

//declarando las variables que usaremos para leer el Serial
int zeta;//esta variable guarda el angulo donde esta apuntando el vehiculo
int alpha;//angulo al cual se tiene que llegar
int mag; //distancia al centro
String a;// esta variable guarda temporalmete lo que se recibe del serial
String b;// esta variable guarda un substring
String c;// esta variable guarda un substring
String d;// esta variable guarda un substring
int error_Rot;
int previousError_Rot;
float errorDot_Rot;
float control_Rot;
int error_Pos;
int previousError_Pos;
float errorDot_Pos;
float control_Pos;
float k_P_Rot=1;// El coefficient del proportional  del angulo
float k_D_Rot =0;// el coefficient de la derivada del angulo
float k_P_Pos=1;// El coefficient del proportional del magnitude del error en position
float k_D_Pos =0;// el coefficient de la derivada del magnitud del error en position
int previousTime;
int currentTime;// these two are needed to find the derivative of the error
int errorDeadband_Rot=1;// the deadband makes sure that the system is not overcompensating for negligable error
int errorDeadband_Pos=1;
int Msg ;// this variable stores the message number so classify the first and second as coefficients


void setup() {
  setMotors();  // turn motors on
  Serial.begin(9600);
  Msg=0;
} 
void loop() {
  // put your main code here, to run repeatedly:

  if (Serial.available()){ //waits to detect something 

    if( Msg<4){ //first 4 messages saved as coefficients
       if(Msg=0){
        a = Serial.readString();
        k_P_Rot = a.toFloat(); //the coefficient that determines how important the error in the rotation is
        Msg=1;  
        
        }
       if(Msg=1){
        a = Serial.readString();
        k_D_Rot = a.toFloat(); //the coefficient that determines how important the derivative in the rotation is
        Msg=2; 
        }
       if(Msg=2){
        a = Serial.readString();
        k_P_Pos = a.toFloat(); //the coefficient that determines how important the error in the position is
        Msg=3; 
        }
     if(Msg=3){
        a = Serial.readString();
        k_D_Pos = a.toFloat(); //the coefficient that determines how important the derivative in the position is
        Msg=4;
        }

  }
    if( Msg>=4){ //from here on, the other messages are taken as data. received as triples
    a = Serial.readString();
    b= getValue(a,',',0);
    c= getValue(a,',',1);
    d= getValue(a,',',2); //breaks down the triples into three strings
    zeta = b.toInt();
    alpha = c.toInt();
    mag = d.toInt(); // changes string to integers
     Serial.print(zeta);
     Serial.print(',');
     Serial.print(alpha);
     Serial.print(',');
     Serial.println(mag); //prints data
  if((zeta <= 180 && zeta >= -180)){
   //fliters out unexpected values
    currentTime= millis(); //records time since the last message was received
   error_Rot= zeta-alpha; // calculates the error of the angle
   if (error_Rot>180 or error_Rot<-180){ //interprets angles outside of the range in module 180 (taking in count sign)
    if (error_Rot>0){
     error_Rot = error_Rot-360;
    }
    if (error_Rot<0){
     error_Rot = error_Rot+360;
    }
    
   }
     error_Pos= mag-0; //calculates the error in the position
    errorDot_Rot =(error_Rot-previousError_Rot)/(currentTime-previousTime); //calculates derivative
    errorDot_Pos =(error_Pos-previousError_Pos)/(currentTime-previousTime);
    //Serial.println("detected ");
    previousError_Rot= error_Rot;
   previousError_Pos= error_Pos;
    previousTime=currentTime;    
    control_Rot = -k_P_Rot*error_Rot +k_D_Rot*errorDot_Rot; //creates a signal using the PID
   control_Pos = -k_P_Pos*error_Pos +k_D_Pos*errorDot_Pos;
    if(abs(control_Rot)>abs(errorDeadband_Rot) && -abs(errorDeadband_Rot)> -abs(control_Rot)){ //checks if the error is outside of the deadband 
    if (control_Rot<0){
      pivotLeft(100,abs(control_Rot));
      //Serial.println("left");
    }
    if ( control_Rot> 0) {
      pivotRight(100,abs(control_Rot));
      //Serial.println("right");
    }
    
 
   
  stopMotors();
   //delay(100);
    }
   if(abs(control_Rot)<abs(errorDeadband_Rot)&& -abs(errorDeadband_Rot)< -abs(control_Rot)){
    if(abs(control_Pos)>abs(errorDeadband_Pos) && -abs(errorDeadband_Pos)> -abs(control_Pos)){
    if (control_Pos<0){
      backward(100,abs(control_Pos));
      //Serial.println("left");
    }
    if ( control_Pos> 0) {
      forward(100,abs(control_Pos));
      //Serial.println("right");
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
