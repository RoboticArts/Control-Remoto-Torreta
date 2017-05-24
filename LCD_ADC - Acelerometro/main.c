
/**
  ******************************************************************************
  * @file    main.c
  * @author  Robert Vasquez, Marc Abella Garcia, Emilio Boza Parada
  * @version V1.0.0
  * @date    24-Mayo-2017
  * @brief   Este programa es empleado en el control remoto de la torreta Hyperion.
		Para ello se lee el angulo de inclinacion del acelerometro, el valor de un
		potenciometro y el estado de dos pulsadores. El angulo es mostrado por una 
		pantalla LCD y la lectura del potenciometro por un display de 7 segmentos 
		mapeado. Toda esta información también es enviada mediante bluetooth
		
  ******************************************************************************
  * @attention
  *
	* El presente código es de uso libre no comercial. Todos los derechos serán atribuidos
	* a los autores de este programa. Está permitida la distribución de este archivo para
	* uso educativo u otro cualquiera sin ánimo de lucro. Ante cualquier duda de licencia
	* acudir a robert29296@gmail.com
  *
  *											      COPYRIGHT 2017 Robotic Arts
  ******************************************************************************
  */ 




#include <stdio.h>
#include <stm32f4xx.h>
#include <string.h>
#include <binario.h>
#include <stdbool.h>

#include "hyperion.h"
#include "tm_stm32f4_hd44780.h"
#include "tm_stm32f4_adc.h"
#include "tm_stm32f4_lis302dl_lis3dsh.h"

#define ToDegree 0.086956 //Convierte la lectura del acelerometro en grados 

char pot[20];
char axesX[20];
char axesY[20];
char machineAxesX[5];
char machineAxesY[5];
char data[20];
char shoot[2] = {'S','0'}; //Boton de disparo. Toma como valores 0 (no disparo) y 1 (disparo)
char stabilizer = 0; //Boton para enviar el angulo a la t torreta. Valores de 0 y 1

uint16_t count_lcd = 0; // Para muestrear el plot del display

TM_LIS302DL_LIS3DSH_t Axes_Data; //Crea una estructura


int main(void)
{
	//Inicia el sistema
	System_Init();
   
   while (1) {
		 
		
		TM_LIS302DL_LIS3DSH_ReadAxes(&Axes_Data);
		 
		uint16_t sensor  = TM_ADC_Read(ADC1, ADC_Channel_1); //Lecutra del potenciometro
		int16_t ejeX = Axes_Data.Z * ToDegree; //1035 a los 90 grados
		int16_t ejeY = Axes_Data.Y * ToDegree; //1035 a los 90 grados		 
		
		 //Filtro por si se satura a los valores mapeados
		if(Axes_Data.Z > 1035) ejeX = 90;
		if(Axes_Data.Z < 0) ejeX = 0;
		 
		if(Axes_Data.Y > 1010) ejeY = 90; //Se pone el limite un poco antes de los 90 grados
	  if(Axes_Data.Y < -1010) ejeY = -90; //Para que se vea el led encendido
		 
		 
		//printf("X: %d Y: %d \n",ejeX,ejeY);	
		


	  /* Para los leds */
		 
		 //Eje Y
		 uint8_t y =  map(ejeY, 90, -90 , 8, 0);	 
		 ShowLeds('Y', y);
		 
		 //Eje X
     uint8_t x =  map(ejeX , 90, 0 , 8, 0);	 
		// ShowLeds('X', x);
		 
		 
		/*Para el display */
		 
		uint8_t number =  map(sensor, 4095,0, 9, 0);
		ShowDisplay(number);
		 
		/* Conversion a cadena de caracteres	*/
		
	  //Grados Eje X mando -90º - 90º
		sprintf(axesX,"%d", ejeX); 
		
	  //Grados Eje X torreta 23º - 67º
		ejeX = map(ejeX,90,0,67,23);
		sprintf(machineAxesX,"%d", ejeX);
		
		//Grados Eje Y mando -90º -  90º 	
		sprintf(axesY,"%d", ejeY); 
		
		//Grados Eje Y torreta 45º - 135º
		ejeY = map(ejeY,90,-90,135,45);
		sprintf(machineAxesY,"%d", ejeY); 


		/*Para el LCD*/
	
		if(count_lcd >= 300) //Siempre que pasen 300ms se plotean la informacion
		{
		   TM_HD44780_Clear(); //Borramos lo que haya en el LCD
			
			
			//Grados Eje X mando -90º - 90º
			TM_HD44780_Puts(0, 0, "X:");
			TM_HD44780_Puts(2, 0, axesX);

			//Grados Eje X torreta 23º - 67º
			TM_HD44780_Puts(0, 1, "X:");
			TM_HD44780_Puts(2, 1, machineAxesX);

			
			
			//Grados Eje Y mando -90º -  90º 	
		  TM_HD44780_Puts(8, 0, "Y:");
			TM_HD44780_Puts(10, 0, axesY);

			//Grados Eje Y torreta 45º - 135º
			TM_HD44780_Puts(8, 1, "Y:");
			TM_HD44780_Puts(10, 1, machineAxesY);
	
		  count_lcd = 0; //Se resetea el contador

		}
		
		
		/*Para los pulsadores*/
		 
		 //La posicion shoot[0] hace referencia a la S
 		 if(shoot_button() == 1)
				shoot[1] = '1';  //Si esta pulsado, dispara
		 else
				shoot[1] = '0'; //Sino, no dispara
		 	 
		 //Si el boton "stabilizer" esta pulsado se envia la información a la torreta
		 if(stabilizer_button() == 1)
		 {	

					/*Para el bluetooth*/ 
		 
					//Se añade la primera letra al vector "data"
					//Se concatena los vectores tipo char
	 		 
					strcpy(data, "X");
					strcat(data, machineAxesX);
					strcat(data, "Y");
					strcat(data, machineAxesY);
					strcat(data, shoot); 
					strcat(data, "\n");
	
					USART_SendString(USART3, data); 
				
		 }
		 
	 }      

   return(0);
}

