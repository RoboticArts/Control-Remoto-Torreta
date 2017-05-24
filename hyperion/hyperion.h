
#include <stdio.h>
#include <stm32f4xx.h>
#include <string.h>
#include <binario.h>
#include <stdbool.h>

void System_Init(void); // Inicia el sistema, aqui aparecen todos los inits de las funciones

void leds_init(void); //Inicia la tira led del eje X y del eje Y 
void ShowLeds(char axis, uint8_t n); //Gestiona los leds del eje X y eje Y para un mismo puerto D

void delay(uint16_t ms); //Realiza una espera segun el tiempo que se pase al argumento (max 65000 ms)

void buttons_init(void); //Inicializa los pulsadores
bool shoot_button(void); //Devuelve el estado del boton de disparo
bool stabilizer_button(void); // Devuelve el estado del boton de envio de datos

void USART3_Init (uint16_t baudrate); //Inicializa el puerto USART
void USART_SendString(USART_TypeDef* USARTx, char buffer[]); //Envia una cadena de caracteres

void display_init(void); // Inicializa el display
void ShowDisplay(uint8_t n); //Muestra por el display el numero que se pasa por el argumento. 

int16_t map(int16_t valor, int16_t DeMax, int16_t DeMin, int16_t HastaMax, uint16_t HastaMin); // Funcion de mapeado


