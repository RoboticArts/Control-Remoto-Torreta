//Por puerto serie se verá lo que recibe 

#include "string.h"
#include <SoftwareSerial.h>

int led = 13;  // usamos un pin de salida al LED
int DATO = 0;      // Variable lectrura serial

char mydate[20];
int cont = 0;

bool llave = false;

String coordenadas;


/*Variables globales para X, Y y S */
int EjeX;
int EjeY;
int shoot;
/* Fin de variables globales*/

/*Rele para el disparo*/
const int rele = A0;


//Placa controladora de servos
// Bajo esta configuración:
// TX controlador al pin 3
// RX controlador al pin 2
SoftwareSerial servoController(3, 2); // RX, TX Pines no cambiables

//Bluetooth
//SoftwareSerial mySerial(10,11); // RX,TX para ARDUINO respectivamente.



void write_servoController(int X, int Y)
{   
    servoController.begin(9600);
    servoController.listen();
    
    int limMinY = 1200; // Calibracion para Servo 2 entre 135 y 180 grados 
    int limMaxY = 400; 

    int limMinX = 300; // Calibracion para Servo 1 entre 135 y 45 grados
    int limMaxX = 300;
    
    //X e Y deben ser valores entre el rango de 45-135 y 70-110 respectivamente
    int anguloX = map (X , 45, 135 , 550 + limMinX,2200 - limMaxX); // 1º servo: Calibrado para Servo MG996R
    int anguloY = map (Y , 70, 110, 550 + limMinY ,2450 - limMaxY ); // 2º servo: Calibrado para Servo Futaba
    
    servoController.print("#1P");
    servoController.print(anguloX);
    servoController.print("T100");
    servoController.print("#2P");
    servoController.print(anguloY);
    servoController.print("T100");
    
    /*
    servoController.print("MMM ");
    servoController.print(X);
    servoController.print(" ");
    servoController.print(Y);
    servoController.print(" ");
    servoController.print(shoot);
    */
    servoController.println("\r\n");

    servoController.end();

}


void write_shootRele(int state)
{
    digitalWrite(rele,state);

}



void setup() {
    pinMode(led, OUTPUT);
    pinMode(rele,OUTPUT);
    digitalWrite(led, LOW);
    Serial.begin(9600);
    //servoController.begin(9600);
    //mySerial.begin(9600);
}
  
void loop() {

/////////////////////////// NO TOCAR ///////////////////////

  //Escribe datos hasta encontrarse con un \n

     if(Serial.available() > 0){


        mydate[cont] = Serial.read();
        
        if(mydate[cont] == '\n')
        {
          String str(mydate);
          coordenadas = str; //Se almacena en la variable global
          llave = true; // Activa la escritura por el puerto serie o las acciones
          //Serial.println(str);
          memset(mydate, 0, sizeof(mydate)); //Limpia el array de chars
          cont = 0; // Se resetea el contador
        }         
        else 
          cont++;
        
    }

/////////////////////////// NO TOCAR ///////////////////////


///////////////////////// AQUI EMPIEZA EL CODIGO ////////////////////

  if(llave == true) //Solo se ejecuta cuando se ha recibido una cadena nueva. Aqui se meten todas las acciones
  {
    //Serial.println(coordenadas);
    llave = false; 


    
    //Hay dos formas de recibir los datos: X100Y100 y S0 separados (Qt) y X100Y100S0 juntos (STM)

    int nx = coordenadas.indexOf('X'); // Se obtiene la posicion de la X en la cadena
    int ny = coordenadas.indexOf('Y'); // Se obtiene la posicion de la Y en la cadena
    int ns = coordenadas.indexOf('S'); // Se obtiene la posicion de la S en la cadena
    int lf = coordenadas.indexOf('\n'); // Se obtiene la posicion de salto de linea, debe ser el ultimo dato del vector
    int na = coordenadas.indexOf('A'); // Boton de armado. Si se pulsa no debe ocurrir nada

    if(na != -1)
    {
      //Cuando se encuentra una A (boton de armado) no hace nada. El boton de armado se ha eliminado pero 
      // sigue implementado en Qt
    } 
    else if(ns == -1) //Si no se encuentra la S la informacion proviene de Qt
    {
      //Hacer cosas para X e Y
      EjeX = coordenadas.substring(nx+1,ny).toInt();
      EjeY = coordenadas.substring(ny+1,lf).toInt();

      //El centro son 90 grados
      EjeX = map(EjeX, 0,255, 45, 135); 
      EjeY = map(EjeY, 0,255, 70, 110); 

      /*
      Serial.print("Qt X e Y: ");
      Serial.print(EjeX);
      Serial.print(" ");
      Serial.println(EjeY);
      */

      //Se escribe al controlador de servos
       write_servoController(EjeX,EjeY);
      
    }

    else if (nx == -1 && ny == -1) //Si no se encuentra la X y la Y, se ha enviado un S0 y proviene de Qt
    {
      //Hacer cosas para S
      shoot = coordenadas.substring(ns+1,lf).toInt();

      /*
      Serial.print("Qt S: ");
      Serial.println(shoot);
      */
      
      if(shoot == 1) //Se activa el rele
        write_shootRele(1);
      else          //En caso contrario se apaga
        write_shootRele(0);
    }

    else //Si se ha encontrado una X, una Y y una S, la información proviene de la STM
    {
      //Hacer cosas para X, Y e S
      EjeX = coordenadas.substring(nx+1,ny).toInt();
      EjeY = coordenadas.substring(ny+1,ns).toInt();
      shoot = coordenadas.substring(ns+1,lf).toInt();

      /*
      Serial.print("STM X Y S: ");
      Serial.print(EjeX);
      Serial.print(" ");
      Serial.print(EjeY);
      Serial.print(" ");
      Serial.println(shoot);
      */

      //Se escribe al controlador de servos
      write_servoController(EjeX,EjeY);
      
      if(shoot == 1) //Se activa el rele
        write_shootRele(1);
      else          //En caso contrario se apaga
        write_shootRele(0);
       
    }

   
  }         
}
  


