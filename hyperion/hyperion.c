
#include <stdio.h>
#include <stm32f4xx.h>
#include <string.h>
#include <binario.h>
#include <stdbool.h>

#include "tm_stm32f4_hd44780.h"
#include "tm_stm32f4_adc.h"
#include "tm_stm32f4_lis302dl_lis3dsh.h"

#include "hyperion.h"

uint16_t count_system = 0; //Para la funcion delay()

uint16_t PuertoE;
uint16_t display[10] = {0x0100,0x4F00,0x1200,0x0600,0x4C00,0x2400,0x2000,0x0F00,0x0000,0x0400};
uint16_t PuertoD;
uint16_t LEDS[9] = {0x00F0,0x0070,0x0030, 0x0010, 0x0000, 0x0008, 0x000C, 0x000E, 0x000F};

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
   
   PuertoD = GPIO_ReadOutputData(GPIOD); //Lee el estado inicial del puerto D
}

void delay(uint16_t ms) //Valor máximo de tiempo 65 segundos
{

   while(count_system < ms){}
  count_system = 0;
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

bool shoot_button(){ //Boton de disparo

   return (bool)GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_3);
   
}

bool stabilizer_button() //Boton que manda la informacion por bluetooth
{
   return (bool)GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_2);
}




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
        GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_10 | GPIO_Pin_11; // Pin 10 (TX)  Pin 11(RX) 
        GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_AF;            // Se configuran como "alternate" para cambiar de IN a OUT y viceversa
        GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;       // Velocidad de los pines (no esta relacionado con la tasa de baudios)
        GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;          // Salida en modo PushPull
        GPIO_InitStruct.GPIO_PuPd  = GPIO_PuPd_UP;           // Se activa la resistencia Pull UP
     
        GPIO_Init(GPIOC, &GPIO_InitStruct);                  //Se hace efectiva la configuracion
 
 
        //-----CONECTAR PC10 Y PC11 A LOS PUERTOS USART TX Y RX-----     
        GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_USART3); //
        GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_USART3);
 
 
        //-----CONFIGURACION DE LA USART-----     
        USART_InitStruct.USART_BaudRate            = baudrate;            //  Tasa de baudios, recibe el argumento de la funcion
        USART_InitStruct.USART_WordLength          = USART_WordLength_8b; // Se envia una palabra de 8 bits (la estándar)
        USART_InitStruct.USART_StopBits            = USART_StopBits_1;    // Con 1 bit de parada
        USART_InitStruct.USART_Parity              = USART_Parity_No;     // Sin paridad (la estandard)
        USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // Sin control de flujo (estádar)
        USART_InitStruct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;      // Se habilita el TX y RX
     
        USART_Init(USART3, &USART_InitStruct);                                         //Se hace efectiva la configuracion                       
            
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
   
   PuertoE = GPIO_ReadOutputData(GPIOE); //Lee el estado inicial del puerto E
   
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

      // binario(GPIO_ReadOutputData(GPIOE));
   
}

void ShowLeds(char axis, uint8_t n)
{
   //Se resetean los bits de los led (los del otro eje se mantienen)
   if(axis == 'X')
      GPIO_Write(GPIOD, PuertoD & 0x00FF);
  if(axis == 'Y')   
      GPIO_Write(GPIOD, PuertoD & 0xFF00);
   
   //Guarda el estado del puerto actual
   PuertoD = GPIO_ReadOutputData(GPIOD);
   
   //Se muestra el numero por las tiras led 
   if(axis == 'X')
      GPIO_Write(GPIOD, PuertoD | (LEDS[n] >> 8));
   if(axis == 'Y')   
      GPIO_Write(GPIOD, PuertoD | LEDS[n]);
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
      delay(10); //Delay para no saturar el puerto serie
   }
      
}

void System_Init()
{

		//Inicializa el SysTick
	 SysTick_Config(168000); // Para 1 ms
	
	 //Inicializa las dos tiras led
	 leds_init();

	 //Inicaliza el display de 7 segmentos
	 display_init();
	
	 //Inicializa el puerto serie
	 USART3_Init(9600);
	
	 //Inicializa el acelerometro
	 //TM_LIS302DL_LIS3DSH_t Axes_Data;
   TM_LIS302DL_LIS3DSH_Init(TM_LIS3DSH_Sensitivity_2G, TM_LIS3DSH_Filter_800Hz);
    
   //Inicializa el LCD
   TM_HD44780_Init(16, 2);
        
   //Inicializa la entrada analogica
   TM_ADC_Init(ADC1, ADC_Channel_1);
	
	//Inicializa los pulsadores
   buttons_init();
	 
	 TM_HD44780_Clear(); //Borramos lo que haya en el LCD

}