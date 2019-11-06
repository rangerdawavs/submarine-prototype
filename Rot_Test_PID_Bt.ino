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
    b= getValue(a,',',0);
    c= getValue(a,',',1);
    d= getValue(a,',',2);
    zeta = b.toInt();
    alpha = c.toInt();
    mag = d.toInt();
     Serial.print(zeta);
     Serial.print(',');
     Serial.print(alpha);
     Serial.print(',');
     Serial.println(mag);
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
