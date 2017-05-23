/**
   @file main.c

   @brief Plantilla Keil 5 para la placa STM32F$Discovery
   Más detalles en el archivo leeme.txt
   
   @author Equipo ARM Power http://armpower.blogs.upv.es/
   @date 2017/03/06
*/

#include <stdio.h>
#include <stm32f4xx.h>
#include <string.h>
#include <binario.h>
#include <stdbool.h>

#include "tm_stm32f4_delay.h"
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

uint16_t PuertoE;
uint16_t display[10] = {0x0100,0x4F00,0x1200,0x0600,0x4C00,0x2400,0x2000,0x0F00,0x0000,0x0400};

uint16_t LED[9] = {0x00F0,0x0070,0x0030, 0x0010, 0x0000, 0x0008, 0x000C, 0x000E, 0x000F};

void leds_init()
	
{

	GPIO_InitTypeDef MiPuerto;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	
	MiPuerto.GPIO_Pin = 0xFFFF;
	MiPuerto.GPIO_Mode = GPIO_Mode_OUT;
	MiPuerto.GPIO_OType = GPIO_OType_PP;
	MiPuerto.GPIO_Speed = GPIO_Speed_2MHz;
	MiPuerto.GPIO_PuPd = GPIO_PuPd_NOPULL;
	
	GPIO_Init(GPIOD, &MiPuerto); 
}



void buttons_init()
{
	GPIO_InitTypeDef MiPuerto;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	
	MiPuerto.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
	MiPuerto.GPIO_Mode = GPIO_Mode_IN;
	MiPuerto.GPIO_PuPd = GPIO_PuPd_DOWN;
	
	GPIO_Init(GPIOA, &MiPuerto);
		
}

bool shoot_button(){

	return (bool)GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_3);
	
}

bool stabilizer_button()
{
	return (bool)GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_2);
}


/*
void delay()
{
	int i = 0;
	for(i = 0; i<=1000000; i++);

}

*/




void USART3_Init (uint16_t baudrate)
{
 //Para el Puerto USAR3  con los pines 10 y 11 del puerto C
	
        //-----Declarar ESTRUCTURAS-----     
        GPIO_InitTypeDef  GPIO_InitStruct;
        USART_InitTypeDef USART_InitStruct;
        NVIC_InitTypeDef NVIC_InitStruct;
   
     
     
        //-----DAR RELOJ A LOS PERIFÉRICOS-----     
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC , ENABLE);
 
     
        //-----CONFIGURACION DE LOS PINES DEL CANAL 8 Y 9 PARA LA USART-----     
        GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_10 | GPIO_Pin_11; // Pins 10 (TX) and 11 (RX) are used
        GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_AF;            // the pins are configured as alternate function so the USART peripheral has access to them
        GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;       // this defines the IO speed and has nothing to do with the baudrate!
        GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;          // this defines the output type as push pull mode (as opposed to open drain)
        GPIO_InitStruct.GPIO_PuPd  = GPIO_PuPd_UP;           // this activates the pullup resistors on the IO pins
     
        GPIO_Init(GPIOC, &GPIO_InitStruct);  
 
 
        //-----CONECTAR PD Y PD9 A LOS PUERTOS USART TX Y RX-----     
        GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_USART3); //
        GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_USART3);
 
 
        //-----CONFIGURACION DE LA USART-----     
        USART_InitStruct.USART_BaudRate            = baudrate;            // the baudrate is set to the value we passed into this init function
        USART_InitStruct.USART_WordLength          = USART_WordLength_8b; // we want the data frame size to be 8 bits (standard)
        USART_InitStruct.USART_StopBits            = USART_StopBits_1;    // we want 1 stop bit (standard)
        USART_InitStruct.USART_Parity              = USART_Parity_No;     // we don't want a parity bit (standard)
        USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // we don't want flow control (standard)
        USART_InitStruct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;      // we want to enable the transmitter and the receiver
     
        USART_Init(USART3, &USART_InitStruct);                            // again all the properties are passed to the USART_Init function which takes care of all the bit setting
            
      //Configurando interrupción para RX
      USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
      //Configurando interrupción en NVIC
      NVIC_InitStruct.NVIC_IRQChannel = USART3_IRQn;
      NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
      NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
      NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
      //Inicializando NVIC
      NVIC_Init(&NVIC_InitStruct);
 
        //-----HABILITAR USART-----    
        USART_Cmd(USART3, ENABLE);
     
}


void display_init()
{
	GPIO_InitTypeDef MiPuerto;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE,ENABLE);
	
	MiPuerto.GPIO_Pin = GPIO_Pin_14 | GPIO_Pin_13 | GPIO_Pin_12 | GPIO_Pin_11 | GPIO_Pin_10 | GPIO_Pin_9 | GPIO_Pin_8;
	MiPuerto.GPIO_Mode = GPIO_Mode_OUT;
	MiPuerto.GPIO_OType = GPIO_OType_PP;
	MiPuerto.GPIO_Speed = GPIO_Speed_2MHz;
	MiPuerto.GPIO_PuPd = GPIO_PuPd_NOPULL;
	
	GPIO_Init(GPIOE, &MiPuerto);
	
	PuertoE = GPIO_ReadOutputData(GPIOE); //Lee el estado inicial del puerto
	
}

void ShowDisplay(uint8_t n)
{

		/*    DISPLAY 7 SEGMENTOS      */
		 
	   //Se resetean los bits del display (los otros se mantienen)
		GPIO_Write(GPIOE, PuertoE & 0x00FF); 
		 
		//Guarda el estado del puerto actual
		PuertoE = GPIO_ReadOutputData(GPIOE);
		 
		//Se muestra el numero del display	 
		GPIO_Write(GPIOE, PuertoE | display[n]);

		 binario(GPIO_ReadOutputData(GPIOE));
	
}



//De: valores que se espera recibir en valor
//Hasta: mapeo deseado
int16_t map(int16_t valor, int16_t DeMax, int16_t DeMin, int16_t HastaMax, uint16_t HastaMin)
{
	return (((valor+abs(DeMin))*(HastaMax-HastaMin))/(DeMax-DeMin))+HastaMin;
}

void USART_SendString(USART_TypeDef* USARTx, char buffer[])
{
	//Se obtiene el numero de caracteres de la cadena a enviar
	uint16_t size = strlen(buffer);
	uint16_t i = 0;
	
	
	//Se convierte de char a uint16_t y se envian todos los caracteres de la cadena
	for(i = 0; i<size; i++)
	{
		uint16_t word = (uint16_t) buffer[i];
	  USART_SendData(USARTx, word);
		Delayms(10); //Delay para no saturar el puerto serie
	}
		
}



int main(void)
{

	 //Inicializa las dos tiras led
	 leds_init();

	 //Inicaliza el display de 7 segmentos
	 display_init();
	
	 //Inicializa el puerto serie
	 USART3_Init(9600);
	
	 //Inicializa el acelerometro
	 TM_LIS302DL_LIS3DSH_t Axes_Data;
   TM_LIS302DL_LIS3DSH_Init(TM_LIS3DSH_Sensitivity_2G, TM_LIS3DSH_Filter_800Hz);
    
   //Inicializa el LCD
   TM_HD44780_Init(16, 2);
        
   //Inicializa la entrada analogica
   TM_ADC_Init(ADC1, ADC_Channel_1);
	
	//Inicializa los pulsadores
   buttons_init();
   
   while (1) {
		 
		
			 
		TM_HD44780_Clear(); //Borramos lo que haya en el LCD
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
		 GPIO_Write(GPIOD, LED[y]);
		 
		 //Eje X
      //Escribir codigo Eje X
		 
		 
		/*Para el display */
		 
		uint8_t number =  map(sensor, 4095,0, 9, 0);
		ShowDisplay(number);
		 
		 
		/*Para el LCD*/
		 
		//Potenciometro 
		//sprintf(pot,"%d",sensor);
		//TM_HD44780_Puts(12, 0, pot);
    
		//Grados Eje X mando -90º - 90º
		sprintf(axesX,"%d", ejeX); 
		TM_HD44780_Puts(0, 0, "X:");
		TM_HD44780_Puts(2, 0, axesX);
		
		//Grados Eje X torreta 23º - 67º
		ejeX = map(ejeX,90,0,67,23);
		sprintf(machineAxesX,"%d", ejeX);
		TM_HD44780_Puts(0, 1, "X:");
		TM_HD44780_Puts(2, 1, machineAxesX);
		
		
		
		//Grados Eje Y mando -90º -  90º 	
		sprintf(axesY,"%d", ejeY); 
		TM_HD44780_Puts(8, 0, "Y:");
		TM_HD44780_Puts(10, 0, axesY);

    //Grados Eje Y torreta 45º - 135º
		ejeY = map(ejeY,90,-90,135,45);
		sprintf(machineAxesY,"%d", ejeY); 
		TM_HD44780_Puts(8, 1, "Y:");
		TM_HD44780_Puts(10, 1, machineAxesY);
		

			 
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
		 
		 Delayms(300);
	 }      

   return(0);
}

