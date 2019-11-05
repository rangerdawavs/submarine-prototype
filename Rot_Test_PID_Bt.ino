/* Originalmente de: Tomás de Camino Beck
 Editado para recibir comandos por el Serial
 Luego re editado para recibir un angulo por BlueTooth, calcular la diferencia entre el calculo deciado y el actual y usar un PD control system para mantener un angulo deseado
*/

#include <RobotKit.h>
// robotkit es una biblioteca de donde sacamos los functions de forward, backward y rotations usando dos motores

//declarando las variables que usaremos para leer el Serial
int zeta;//esta variable guarda el angulo donde esta apuntando el vehiculo
String a;// esta variable guarda temporalmete lo que se recibe del serial
String b;
int error;
int previousError;
float errorDot;
float control;
float k_P=1;// El coefficient del proportional  del angulo
float k_D =0;// el coefficient de la derivada del angulo
int previousTime;
int currentTime;// these two are needed to find the derivative of the error
int errorDeadband=1;// the deadband makes sure that the system is not overcompensating for negligable error
int Msg ;// this variable stores the message number so classify the first and second as coefficients


void setup() {
  setMotors();  // prendiendo motores
  Serial.begin(9600);
  Msg=0;
} 
void loop() {
  // put your main code here, to run repeatedly:

  if (Serial.available()){ //espera a detectar algo. 

    if( Msg<2){
       if(Msg=0){
        a = Serial.readString();
        k_P = a.toFloat();
        Msg=1;
        
        }
       if(Msg=1){
        a = Serial.readString();
        k_D = a.toFloat();
        Msg=2;
        }

  }
    if( Msg<=2){
    a = Serial.readString();
    //b= getValue(a,';',0);
    //zeta = b.toInt();
    zeta = a.toInt();
     Serial.println(zeta);
  if((zeta<180 && zeta> -180)){
   //y ahora usamos if statements para detectar si el comando es idéntico a uno de los que  
   //pusimos al principio
    currentTime= millis();
     error= zeta-0 ;
    errorDot =(error-previousError)/(currentTime-previousTime);
    //Serial.println("detected ");
    previousError= error;
    previousTime=currentTime;    
    control = -k_P*error +k_D*errorDot;
    if(abs(control)>abs(errorDeadband) && -abs(errorDeadband)> -abs(control)){
    if (control<0){
      pivotLeft(100,abs(control));
      //Serial.println("left");
    }
    if ( control> 0) {
      pivotRight(100,abs(control));
      //Serial.println("right");
    }
    
 
   
  stopMotors();
   //delay(100);
    }
  }
}
  }
}
