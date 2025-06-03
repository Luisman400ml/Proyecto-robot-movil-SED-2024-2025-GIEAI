#include <LPC17xx.h>
#include <Math.h>
#include "QEI.h" 
#include "reg_masks.h"
#include "GLCD.h" 
#include "AsciiLib.h"
#include <stdio.h>
#include <string.h>
#include "uart.h"

/*frecuencias de trabajo de la CPU + periféricos 
*/

#define F_cpu 100e6			    // Frecuencia de trabajo de la CPU (100MHz)
#define F_pclk_pwm F_cpu/4 	// Fuente de reloj del periférico PWM (25MHz) (al no tocar PCLKSEL, se divide entre 4 por defecto)
#define F_pclk_QEI F_cpu/4	// Fuente de reloj del periférico QEI (25MHz) (al no tocar PCLKSEL, se divide entre 4 por defecto)
#define F_pclk_TIMER1 F_cpu/4	// Fuente de reloj del periférico TIMER1 (25MHz) (al no tocar PCLKSEL, se divide entre 4 por defecto)
#define F_pclk_TIMER0 F_cpu/4 // Fuente de reloj del periférico TIMER0 (25MHz) (al no tocar PCLKSEL, se divide entre 4 por defecto)

/*macros movimiento: hemos añadido 3 movimientos más respecto a lo pedido en la descripción del proyecto 
*/

#define STOP 0
#define GO 1
#define RIGHT 2 // Permite giros de entre 0º y 360º hacia la derecha
#define LEFT 3  // Permite giros de entre 0º y 360º hacia la izquierda
#define BACK 4 
#define RIGHTGO 5 //Giro 90º a derecha y avanza cierta distancia
#define LEFTGO 6  //Giro 90º a izquierda y avanza cierta distancia

/*macro señal tiempo en alto complementado (necesario para los movimientos en avance) 
*/

#define COMPLEMENTADO(x) (100-(x))  // En caso de que una rueda avance, el ciclo de trabajo de su señal PWM es la complementaria

/*macros PWM 
*/

#define F_pwm 500			      // Frecuencia del timer counter del PWM (MR0): 500Hz

/*macros QEI 
*/

#define QEICONF_CAPMODE 0x04 // Añade un 1 en el bit 2 del registro QEICONF ( CAPMODE=4, resolución x4 (N=4))
#define QEICON_RESP 0x01	   // Con el bit 0 a '1' en QEICON se resetea el contador de posición (resetea QEIPOS)
#define QEI_TIM_INT 0x0002   // Con el bit 1 a '1' en QEIIES se activa el handler del QEI cuando QEITIME rebosa
#define QEI_POS0_INT 0x0040  // Con el bit 6 a '1' en QEIIES se activa el handler del QEI cuando QEIPOS=CMPOS0

#define PI 3.141592
#define ANGULO_RECTO 90     //Para las acciones de giro y avance
#define PPR 11							// Pulsos del encoder por revolución de la rueda
#define EDGES 4							// Incremento del contador por cada flanco de PhA y PhB (resolución x4)
#define N 35                // La relación con la reductora (1:35 = Por cada 35 vueltas del motor, la rueda realizará solo 1 vuelta)
#define Rrueda 3.35         // Radio rueda 67/2 mm = 33,5 mm = 3,35 cm
#define dist_eje 10         // Distancia en cm desde una rueda hasta el eje central del coche

/* *****¡MUCHO CUIDADO CON LA CAÍDA DE LAS SUSPENSIONES!*****                      


En función del peso puede aumentar ligeramente la distancia entre ejes, por el apoyo de la rueda. 
Al ser un recorrido muy pequeño, afectara poco, pero !Ajuste fino de este valor con el montaje final!
*/                     
//

#define cm_cuenta ((2*PI*Rrueda)/(N*PPR*EDGES)) // Los centímetros que avanza la rueda por unidad de cuenta del encoder	(por pulso; anoto esto porque me aclaro mejor así)
#define arco_giro ((2*PI*dist_eje)/360) // Distancia a recorrer (en cm) por cada grado de giro
#define T_obs_QEITIME 100e-3					 // (100ms) Periodo de observación del contador dentro del QEI para medir velocidades (QEITIME) 

#define VELOCIDAD_RPM(x,y) ((x)*60*F_pclk_QEI/(PPR*EDGES*N*(y))) //Cálculo de la velocidad de la rueda en rpm (x=valor QEICAP, y=valor QEILOAD)
#define VELOCIDAD_M_SEG(x) ((x)*2*PI*Rrueda/6000) 	//Cálculo de la velocidad de la rueda en cm/seg (x=velocidad en rpm)

/*macros ADC
*/
#define Manual_bateria 1  			// En binario: b00000001 (canal entrada AD0.0)
#define Manual_potenciometro 2  // En binario: b00000010 (canal entrada AD0.1)
#define Manual_microfono 32 		// En binario: b00100000 (canal entrada AD0.5)
#define Burst 5 								// En binario: b00000101 (calanes de entrada AD0.0 y AD0.2) (Batería + Sensor distancia)

/*macros DAC
*/
#define N_muestras 32
#define F_SONIDO 1000						// Frecuencia del sonido de 1000Hz

/*variables de control de motores 
*/

uint8_t accion;  // variable que guarda la acción a hacer del coche (STOP=0, GO=1, RIGHT=2, LEFT=3, BACK=4, RIGHTGO=5, LEFTGO=6)
uint8_t mot_1=0;	// variable que guarda la acción a hacer del coche para motor 1 (rueda izquierda)
uint8_t mot_2=0;	// variable que guarda la acción a hacer del coche para motor 2 (rueda derecha)
uint8_t ciclo;    
uint8_t ciclo_motor1;		// Guarda el ciclo de trabajo de la señal PWM que controla el motor 1 (rueda izquierda)
uint8_t ciclo_motor2;   // Guarda el ciclo de trabajo de la señal PWM que controla motor 2 (rueda derecha)
uint8_t acciones[10]={0}; //Array que en un futuro contendra las consignas de acción a recorrer via UART

/*variables de control de QEI
*/

float speed_m=0;
uint16_t dist_actual=0;
uint16_t dist_deseada=0;	
uint16_t speed_rpm = 0;						// Variable limitada a la función (se borran cada vez que salgamos del handler)

/* ATENCIÓN: dist_deseada puede guardar valores de cm o de grados:
	- si avanzamos o retrocedemos, son centimetros a recorrer
	- si giramos, son los grados a girar
*/
//

uint16_t distancias[10]={0}; //Array que en un futuro contendra las distancias (cm) y giros (º) a recorrer via UART
uint8_t QEI_token=0;
uint8_t QEI_index=0;

uint32_t pulsos_CMPOS0;

/*Variables de control ACD
*/
float valor_bateria;
float valor_distancia;
uint8_t modo;										// Modo de trabajo del ADC: "Manual_bateria", "Manual_potenciometro", "Manual_microfono" o "Burst"	
uint16_t potenciometro;
float u_bat;
float vel_teorica;

/*Variables de control DAC
*/
uint8_t token_ruido=0;
uint8_t fin_de_tramo=0;
uint16_t alto_ruido=0;
uint16_t muestras[N_muestras];

/*Variables LCD
*/
uint8_t contador;  // Variable que se representa
uint8_t texto[20];
uint8_t texto2[20];
char texto1[20];
uint16_t Ticks=0;
uint16_t x = 30;
uint16_t y = 45;
uint16_t x1 = 30;
uint16_t y1= 145;
uint16_t x2 = 30;
uint16_t y2= 95;
	uint32_t i=0;
	uint32_t j=0;
	uint32_t k=0;
	int si=0;
	int sj=0;
extern const unsigned char IMAGE_DATA[];

/*Variables UART
*/



char fin=0;
char buffer[30];	// Buffer de recepción
extern char *ptr_rx;		// Buffer de recepción de 30 caracteres
extern char rx_completa;		// Flag de recepción de cadena que se activa a "1" al recibir la tecla return CR(ASCII=13)
extern char tx_completa;		// Flag de transmisión de cadena que se activa al transmitir el caracter null (fin de cadena)
uint8_t comand_index=0;     //Indice de lectura del comando enviado via UART
int dato_aux1;              //Variables auxiliares para la gestión de los datos recibidos via UART
int dato_aux2;
int dato_aux3;
float bat_aux;
int dato_aux_total;
int bat_read=0;
uint8_t load_index;


/*declaración de funciones
*/

void config_ciclo(void);
void config_gpios(void);
void config_pwm(void);
void set_duty_RIGHT_pwm(uint8_t cycle);
void set_duty_LEFT_pwm(uint8_t cycle);
void config_botones(void);
void EINT1_IRQHandler(void);
void EINT2_IRQHandler(void);

void config_QEI(void);
void QEI_IRQHandler(void);
void QEI_pulses();

void config_ADC(uint8_t ADC_modo);
void ADC_IRQHandler(void);
void config_TIMER1(uint8_t ADC_modo);

void genera_muestras();
void config_DAC();
void config_TIMER0();
void TIMER0_IRQHandler(void);

void SysTick_Handler(void); 

/*funciones
*/

int main(void)
{
	
	LCD_Initialization();
	LCD_Clear(Black);
	
	
	
	SysTick_Config(SystemCoreClock/10); 
	
	LCD_DrawLine( x, y, x + 10*25 + 20, y, Blue2); //Como visualizar los datos en el LCD
	LCD_DrawLine( x, y, x, y + 28, Blue2);
	LCD_DrawLine( x, y + 28, x + 10*25 + 20, y + 28, Blue2);
	LCD_DrawLine( x +  10*25 + 20, y, x + 10*25 + 20, y + 28, Blue2);
	
	
	LCD_DrawLine( x1, y1, x1 + 10*25 + 20, y1, Green);
	LCD_DrawLine( x1, y1, x1, y1 + 28, Green);
	LCD_DrawLine( x1, y1 + 28, x1 + 10*25 + 20, y1 + 28, Green);
	LCD_DrawLine( x1 +  10*25 + 20, y1, x1 + 10*25 + 20, y1 + 28, Green);
	
	LCD_DrawLine( x2, y2, x2 + 10*25 + 20, y2, Yellow);
	LCD_DrawLine( x2, y2, x2, y2 + 28, Yellow);
	LCD_DrawLine( x2, y2 + 28, x2 + 10*25 + 20, y2 + 28, Yellow);
	LCD_DrawLine( x2 +  10*25 + 20, y2, x2 + 10*25 + 20, y2 + 28, Yellow);
	
	NVIC_SetPriorityGrouping(2);
	
	genera_muestras();
	config_botones();
	
	// ----- UART

	
	uart0_init(9600);							 // configuración de baudrate, con la función de la API incluida en la libreria UART. Mismo valor que el del módulo BLE				
  uart3_init(9600);		

									
	tx_cadena_UART3("Introduce el comando de movimientos, estas son las directrices:\n"
	                "UXX para umbral de tension\n"
	                "AXXX para avanzar\n"
	                "MXXX para retroceder\n"
	                "DXXX para giro 90 grados dcha mas avance\n"
	                "IXXX para giro 90 grados izqd mas avance\n"
									"RXXX para giro XXX grados dcha\n"
	                "LXXX para giro XXX grados \n\r");  //Solicitud de comendao al usuario via UART3	

	while(tx_completa==0);
	
	while(1){

		if(rx_completa){					 	// Comprabamos la llegada de una cadena por RXD 
			
			if(buffer[0]=='U' && (bat_read==0) ){       //Lectura del umbral de bateria
			  dato_aux1=buffer[1] -'0'; 
			  bat_aux=(buffer[2]-'0');
				bat_aux=bat_aux/10;
			  u_bat=dato_aux1+bat_aux;
				bat_read=1;
				load_index=0;
			  comand_index=3; 
        memset(acciones, 0, sizeof(acciones));  //Limpiamos acciones y distancias
        memset(distancias, 0, sizeof(distancias));					
			}
			
			 switch(buffer[comand_index]){
				 
				 case 'A':                     //Comando avance
			  dato_aux1=buffer[comand_index+1] -'0';
			  dato_aux2=(buffer[comand_index+2]-'0');
			  dato_aux3=(buffer[comand_index+3]-'0');
				dato_aux_total=(dato_aux1*100)+(dato_aux2*10)+dato_aux3;
		    acciones[load_index]=GO;
		    distancias[load_index]=dato_aux_total;
			  load_index++;
		    comand_index+=4;
				 break;
		
			 case 'M':                        //Comando retroceso
			  dato_aux1=buffer[comand_index+1] -'0';
			  dato_aux2=(buffer[comand_index+2]-'0');
			  dato_aux3=(buffer[comand_index+3]-'0');
				dato_aux_total=(dato_aux1*100)+(dato_aux2*10)+dato_aux3;
		    acciones[load_index]=BACK;
		    distancias[load_index]=dato_aux_total;
			  load_index++;
		    comand_index+=4;
			 break;
			 
			 case 'D':                      //Comando 90º dcha mas avance
			  dato_aux1=buffer[comand_index+1] -'0';
			  dato_aux2=(buffer[comand_index+2]-'0');
			  dato_aux3=(buffer[comand_index+3]-'0');
				dato_aux_total=(dato_aux1*100)+(dato_aux2*10)+dato_aux3;
		    acciones[load_index]=RIGHTGO;
		    distancias[load_index]=dato_aux_total;
			  load_index++;
		    comand_index+=4;
			 break;
			 
			case 'I':                       //Comando 90º izq mas avance
		  	dato_aux1=buffer[comand_index+1] -'0';
			  dato_aux2=(buffer[comand_index+2]-'0');
			  dato_aux3=(buffer[comand_index+3]-'0');
				dato_aux_total=(dato_aux1*100)+(dato_aux2*10)+dato_aux3;
		    acciones[load_index]=LEFTGO;
		    distancias[load_index]=dato_aux_total;
			  load_index++;
		    comand_index+=4;
			break;
			 
			 case 'R':                //Comando giro dcha
			dato_aux1=buffer[comand_index+1] -'0';
			dato_aux2=(buffer[comand_index+2]-'0');
			dato_aux3=(buffer[comand_index+3]-'0');
				dato_aux_total=(dato_aux1*100)+(dato_aux2*10)+dato_aux3;
		    acciones[load_index]=RIGHT;
		    distancias[load_index]=dato_aux_total;
			load_index++;
		  comand_index+=4;
			 break;
			 
			case 'L':                   //Comando giro izq
			dato_aux1=buffer[comand_index+1] -'0';
			dato_aux2=(buffer[comand_index+2]-'0');
			dato_aux3=(buffer[comand_index+3]-'0');
				dato_aux_total=(dato_aux1*100)+(dato_aux2*10)+dato_aux3;
		    acciones[load_index]=LEFT;
		    distancias[load_index]=dato_aux_total;
			load_index++;
		  comand_index+=4;
			 break;
						
			 default:
				 rx_completa=0;
				 bat_read=0;
					memset(buffer, 0, sizeof(buffer));  //sentencia que hace que todo el contenido de buffer sea 0
					tx_cadena_UART3("Pulsa KEY1 para iniciar el movimiento\n");
					
				 break;
			 
		 } 
	 }
 }
 //while(!rx_completa);

//tx_cadena_UART3("Pulsa KEY1 para iniciar el movimiento\n\r");

}

/* **** CONFIGURACIÓN DE MOTORES ****
		
		- motor 1 (rueda izquierda)
		- motor 2 (rueda derecha)

		NOTA: 
		- Trabajaremos en modo driver (2º manera mostrada en la descripción del proyecto): GPIO(IN1) + PWM(IN2) y Enable siempre fijo
*/
//

void config_ciclo(void) // Configura el ciclo de trabajo de las señales PWM (ciclo_motor1 y ciclo_motor2) en función del movimiento
{ 
	
  switch (accion) {
	    case STOP:
							ciclo_motor1=0;
							ciclo_motor2=0;
							break;
      case GO:
            	ciclo_motor1= COMPLEMENTADO(ciclo);
							ciclo_motor2= COMPLEMENTADO(ciclo);
							break;
      case RIGHT:
							ciclo_motor1= COMPLEMENTADO(ciclo);
							ciclo_motor2= ciclo;
							break;
      case LEFT:
							ciclo_motor1= ciclo;
							ciclo_motor2= COMPLEMENTADO(ciclo);
							break;
      case BACK:
							ciclo_motor1= ciclo;
							ciclo_motor2= ciclo;
							break;
			case RIGHTGO:
							ciclo_motor1= COMPLEMENTADO(ciclo);
							ciclo_motor2= ciclo;
							break;
      case LEFTGO:
							ciclo_motor1= ciclo;
							ciclo_motor2= COMPLEMENTADO(ciclo);
							break;
			default:
							break;
   }
}

void config_gpios(void) // Configuración pines de sentido para motor 1 y motor 2 (GPIO)
{ 
	 
	 /*
		Pines de control:
				- P0.0 para motor 1 (rueda izquierda) -> será IN1 del driver
				- P0.1 para motor 2 (rueda derecha) -> será IN3 del driver
	 */
	 
	 LPC_PINCON->PINSEL0 |= (0<<0); //P0.0 como GPIO
	 LPC_PINCON->PINSEL0 |= (0<<2); //P0.1 como GPIO
	 LPC_GPIO0->FIODIR |=(1<<0); 		//P0.0 como Salida
	 LPC_GPIO0->FIODIR |=(1<<1);    //P0.1 como Salida
	 
	/*	 OBSERVACIÓN:
	 Tal y como esta diseñado el programa, con la variable "accion" podriamos hacer funcionar los motores. 
	 Se ha previsto una variable por motor adicional, por si sirve para hacer arcos de giro; en caso de que vayamos holgados de tiempo.
	 */
	 //
	
	 mot_1=accion;
	 mot_2=mot_1;
	 
	 //---------- Definir como gira motor 1 (rueda izquierda)
	 
	 if( mot_1==GO || mot_1==RIGHT || mot_1==RIGHTGO){    // Para ir recto o girar a derechas
			LPC_GPIO0->FIOPIN |= (1<<0); //P0.0 a 'H'
	 }
	 
	 if(mot_1==LEFT || mot_1==BACK || mot_1==STOP || mot_1==LEFTGO){  // Para ir atras o girar a izquierdas
			LPC_GPIO0->FIOPIN &= ~(1<<0); //P0.0 a 'L'
	 }
	 
	 //---------- Definir como gira motor 2 (rueda derecha)
	 
	 if(mot_2==GO || mot_2==LEFT || mot_2==LEFTGO){ 		// Para ir recto o girar a izquierdas
			LPC_GPIO0->FIOPIN |= (1<<1); //P0.1 a 'H'
	 }
	 
	 if(mot_2==RIGHT || mot_2==BACK || mot_2==STOP || mot_2==RIGHTGO){  // Para ir atras o girar a izquierdas
			LPC_GPIO0->FIOPIN &= ~(1<<1); //P0.1 a 'L'
	 }
 }
 
void config_pwm(void) 	// Configuración del PWM para motor 1 y motor 2
{
  LPC_SC->PCONP |= (1<<6); 						// Power on
	LPC_SC->PCLKSEL0 &=~ (0x3<<12);     // PWM clk = CCLK/4 (25MHz)
	LPC_PINCON->PINSEL3 |= (1<<11);   	// Configurar pin P1.21 (PWM1.3 output)(MR3) -> para Motor 1 (Rueda izquierda)(IN2)(De momento misma velocidad de giro)
  LPC_PINCON->PINSEL3 |= (1<<17);   	// Configurar pin P1.24 (PWM1.5 output)(MR5) -> para Motor 2 (Rueda derecha)(IN4)(De momento misma velocidad de giro)
	
	LPC_PWM1->MR0 = (F_pclk_pwm/F_pwm) - 1;	// Fijar frecuencia (500 Hz) de las dos PWMs: se hace con el valor de MR0 (modo Single Edge)  
	LPC_PWM1->LER |= (1<<0);
  LPC_PWM1->PCR |= (1<<11)|(1<<13); 	// Todos a modo Single Edge. Habilitación de salidas PWM1.3 (match con MR3) y PWM1.5 (match con MR5)
  LPC_PWM1->MCR |= (1<<1); 						// Reseteo del timer del PWM cuando hay match con MRO
	
  LPC_PWM1->TCR |= (1<<0) | (1<<3); 	// PWM mode ON + Activación PWMs
}

void set_duty_LEFT_pwm(uint8_t cycle) // Ciclo de trabajo (velocidad) motor izquierdo. cycle = ciclo_motor1
{
 
	/*
	recordar que por culpa de la complementación de la PWM hecha por el driver:
	-> ciclo_motor1 NO ES IGUAL A ciclo_motor2
	*/
	
	LPC_PWM1->MR3 = (LPC_PWM1->MR0 * cycle) / 100; //ciclo de trabajo
	LPC_PWM1->LER |= (1<<3);											 //LER para confirmar cambios
}

void set_duty_RIGHT_pwm(uint8_t cycle) // Ciclo de trabajo (velocidad) motor derecho. cycle = ciclo_motor2
{
 
	/*
	recordar que por culpa de la complementación de la PWM hecha por el driver:
	-> ciclo_motor1 NO ES IGUAL A ciclo_motor2
	*/
	
	LPC_PWM1->MR5 = (LPC_PWM1->MR0 * cycle) / 100; //ciclo de trabajo
	LPC_PWM1->LER |= (1<<5);											 //LER para confirmar cambios
}

void config_botones(void)   //Configuración de los botores KEI1 (EINT1) Y KEI2 (EINT2)
{
	//KEY1 -> EINT1 (P2.11)
	//KEY2 -> EINT2 (P2.12)
	
	LPC_PINCON->PINSEL4 |= (0x5 << 22);	//Pines P2.11 y P2.12 como EINT´s (los pines de los botones)
	LPC_SC->EXTMODE |= 7;				 			  //Activas por flanco
	LPC_SC->EXTPOLAR &= ~0x7;			      //flanco de bajada (recuerda pulsar "bien": puede haber glitches)
	
//Habilitación de las IRQ´s de cada botón
	
	NVIC_EnableIRQ(EINT1_IRQn);
	NVIC_EnableIRQ(EINT2_IRQn);
}	

void EINT1_IRQHandler(void)	//Botón KEY1 -> Al presionar este botón, el coche comienza a moverse
{ 
	LPC_SC->EXTINT = (1 << 1); //Borramos flag de interrupción
	
// Para el proyecto final, cargamos acciones[0], con su valor en otro array. Configurar aqui QEI,PWM, GPIOS, DUTY
	
	QEI_token=0;
	accion=acciones[0];     // Cargamos los valores acciones y distancias de los arrays en las variables a trabajar. 
													// Como con el botón arrancamos, siempre empezaremos con la primer accion del array
	dist_deseada=distancias[0];
	QEI_index=0;   //Esta será la variable que recorrerá todo el array. Al inicializarla como cero podremos de esta forma
								 //repetir el mismo camino desde el principio pulsando de nuevo el botón KEY1
	config_QEI();
	QEI_pulses();
	
	LPC_PINCON->PINSEL1 &=~(3<<20); 	 	// Configuración pin a GPIO (P0.26)
	LPC_GPIO0->FIODIR |= (1<<26); 			// GPIO como salida
	LPC_GPIO0->FIOCLR |= (1<<26); 			// P0.26 a 'L'

	alto_ruido=0;
	token_ruido=0;
	fin_de_tramo=0;
	
	modo = Manual_potenciometro;		// ATENCIÓN: antes de leer el potenciómetro, es necesario saber la tensión actual de la batería
	config_ADC(modo);
	config_TIMER1(modo);
}

void EINT2_IRQHandler(void)	//Botón KEY2 
{
	LPC_SC->EXTINT = (1 << 2); //Borramos flag de interrupción
	
 /* Dejamos esto preparado. Seguro que para el proyecto final le encontramos un buen uso.
		Ideas:
		-> Conectar/Desconcetar ventilador
		-> Conectar/Desconectar sensor distancias (le hemos añadido un sensor de distancia al coche)
	*/
}
//
// QEI
//
void config_QEI(void) //Configuración del QEI
{
	LPC_SC->PCONP |= (1<<18);				  // Power ON del QEI
	LPC_SC->PCLKSEL1 &=~ (0x3<<0);		// PCLK = CCLK/4 (25MHz)
	LPC_PINCON->PINSEL3	|= (1<<8);		// MCI0 en P1.20 (leerá "Hall signal A")
	LPC_PINCON->PINSEL3	|= (1<<14);		// MCI1 en P1.23 (leerá "Hall signal B")
	
	LPC_QEI->QEICONF	= QEICONF_CAPMODE; 						// Resolución x4 (edges=4)
	LPC_QEI->QEICON 	= QEICON_RESP;		 						// Reseteo inicial de QEIPOS a cero
	LPC_QEI->QEIMAXPOS = 0xFFFFFFFF; 		 						// Nos da igual, así que ponemos el valor máximo
	LPC_QEI->QEILOAD 	= F_pclk_QEI * T_obs_QEITIME;	// valor de recarga del contador de velocidad QEITIME
		
  LPC_QEI->QEIIEC &=~ 0xFFFFFFFF;                	 //Limpieza de QEIIE
  LPC_QEI->QEIIES |= QEI_TIM_INT | QEI_POS0_INT;  
	
	/* 
		Escritura en QEIIE. Permitimos la interrupción cuando:
	
		- QEITIME rebosa (se usa para leer la velocidad del coche)
		- QEIPOS=CMPOS0
	*/
	
  NVIC_SetPriority(QEI_IRQn,2);   // Solo ajusta el nivel de prioridad
  NVIC_EnableIRQ(QEI_IRQn);   	  // Habilitamos el handler del QEI
	
}


void QEI_pulses()	//Función que actualiza el valor de CMPOS0 y resetea el contador del QEI
	                //Discrimina en función de la acción a realizar, si la variable dist_deseada, son cm a recorrer o grados a girar. 
									//Y tras eso hace la conversión adecuada
{   																		
		/*ATENCIÓN:
	
		- Es MUY IMPORTANTE aquí tener en cuenta que el QEI cuenta los pulsos del encoder del motor de la IZQUIERDA 
	  */
		//
	
	if (QEI_token==0){ //Token usado para GO, BACK, LEFT, RIGHT, STOP y los giros de 90º en LEFTGO Y RIGHTGO
		
		switch(accion){		//Por cada accion siempre debemos: convertir distancia, cargarla en CMPOS0 y resetear contador QEIPOS
			
			case STOP:  //Si paramos, es porque hemos llegado al último valor del array o valor NULL, es decir, accion=STOP
				LPC_QEI->QEIIEC |= 0xFFFFFFFF; //Limpieza de QEIIE para que ya no salte el CMPOS0 por error, tras la parada
				ciclo=0; 				
				config_ciclo(); //Las dos PWMs a con ciclo de trabajo igual a cero
				set_duty_RIGHT_pwm(ciclo_motor2);
			  set_duty_LEFT_pwm(ciclo_motor1);
				config_gpios(); //P0.1 y P0.0 a 'L' -> Ver nota de abajo
			
				/*ATENCIÓN: IMPRESCINDIBLE HACERLO. 
					En un caso extremo, si GPIO a 'H' con PWM a 0%, el driver no discrimina que pin esta conetado a GPIO y cual PWM. 
					De forma que en vez de parar, se puede quedar girando a máxima velocidad la rueda
				*/
			  speed_rpm=0; //Porque si no, al parar tiene cualquier valor JAJAJAJAJAJA
				token_ruido=1;
				fin_de_tramo=1;
				config_DAC();
				config_TIMER0();
				break;
			
			case GO:           
				LPC_QEI->QEICON = QEICON_RESP;		 // Reseteo de QEIPOS a cero
				pulsos_CMPOS0 = (uint32_t) dist_deseada/cm_cuenta;
				LPC_QEI->CMPOS0 = pulsos_CMPOS0;
			  tx_cadena_UART3("Movimiento hacia adelante\n");
				break;
	
			case RIGHT: // ATENCIÓN: El motor de la derecha RETROCEDE para girar a la derecha
				LPC_QEI->QEICON = QEICON_RESP;		 // Reseteo de QEIPOS a cero
				pulsos_CMPOS0 = (uint32_t) (arco_giro*dist_deseada)/cm_cuenta;
				LPC_QEI->CMPOS0 =  pulsos_CMPOS0;
			  tx_cadena_UART3("Giro hacia la derecha\n");
				break;
			
			case LEFT:	// ATENCIÓN: El motor de la derecha AVANZA para girar a la izquierda
				LPC_QEI->QEICON = QEICON_RESP;		 // Reseteo de QEIPOS a cero
				pulsos_CMPOS0 = (uint32_t) (arco_giro*dist_deseada)/cm_cuenta;
				LPC_QEI->CMPOS0 = - pulsos_CMPOS0; // ATENCIÓN: CUENTA DESCENTENDE al ir el motor hacia atrás
				tx_cadena_UART3("Giro hacia la izquierda\n");
				break;
			
			case BACK:
				LPC_QEI->QEICON = QEICON_RESP;	 // Reseteo de QEIPOS a cero
				pulsos_CMPOS0 = (uint32_t) dist_deseada/cm_cuenta;
				LPC_QEI->CMPOS0 = - pulsos_CMPOS0; // ATENCIÓN: CUENTA DESCENTENDE al ir el motor hacia atrás
		  	tx_cadena_UART3("Movimiento hacia atrás\n");
				break;
	
			case RIGHTGO: // ATENCIÓN: El motor de la derecha RETROCEDE para girar a la derecha
				LPC_QEI->QEICON = QEICON_RESP;		 // Reseteo de QEIPOS a cero
				pulsos_CMPOS0 = (uint32_t) (arco_giro*ANGULO_RECTO)/cm_cuenta;  //En esta acción se define el giro de 90 grados con una macro, pues es condición de giro
				LPC_QEI->CMPOS0 = pulsos_CMPOS0;
				
				QEI_token=1;  //Token para que la próxima acción sea el avance con los cm deseados
		  	tx_cadena_UART3("Giro de 90 grados a derecha y recto\n");
				break;
			
			case LEFTGO:	// ATENCIÓN: El motor de la derecha AVANZA para girar a la izquierda
				LPC_QEI->QEICON = QEICON_RESP;		 // Reseteo de QEIPOS a cero
				pulsos_CMPOS0 = (uint32_t) (arco_giro*ANGULO_RECTO)/cm_cuenta; //En esta acción se define el giro de 90 grados con una macro, pues es condición de giro
				LPC_QEI->CMPOS0 = - pulsos_CMPOS0;	 // ATENCIÓN: CUENTA DESCENTENDE al ir el motor hacia atrás
				
				QEI_token=1;  //Token para que la próxima acción sea el avance con los cm deseados
			  tx_cadena_UART3("Giro de 90 grados a izquieda y recto\n");
				break;
		
			default:
				break;
		}
	}
	
	else{		 //Para los casos de RIGHTGO y LEFTGO: una vez realizada la accion de giro, realizamos SIEMPRE un avance con la distancia deseada 

		accion=GO; // IMPORTANTE
		
    LPC_QEI->QEICON = QEICON_RESP;		 // Reseteo de QEIPOS a cero
		pulsos_CMPOS0 = (uint32_t) dist_deseada/cm_cuenta;
		LPC_QEI->CMPOS0 = pulsos_CMPOS0;
		
		QEI_token=0;
	}	
	 
}
void QEI_IRQHandler(void) //Handler de la QEI 
{
	
	if (LPC_QEI->QEIINTSTAT & QEI_TIM_INT)		// Verifica si ha saltado la IRQ por rebosamiento del QEITIME
	{		
		LPC_QEI->QEICLR |= QEI_TIM_INT;		// Borramos flag
		speed_rpm = (uint16_t) VELOCIDAD_RPM(LPC_QEI->QEICAP , LPC_QEI->QEILOAD); //velocidad en rpm (sin decimales)
		speed_m=(speed_rpm*2*PI*Rrueda);
		speed_m=speed_m/(60*100);
		
	}     
	
	if (LPC_QEI->QEIINTSTAT & QEI_POS0_INT)		// Verifica si ha saltado la IRQ porque se ha llegado a la distancia recorrida deseada
	{  
		LPC_QEI->QEICLR |= QEI_POS0_INT;					// Borramos flag
		
		if(QEI_token==0)	//En caso de que la acción previa haya sido GO, BACK, LEFT, RIGHT, STOP o la finalización de RIGHTGO o LEFTGO
		{ 
			QEI_index++; //Actualizamos el indice para la próxima acción 
			accion=acciones[QEI_index]; //Actualizamos próxima accion...
			dist_deseada=distancias[QEI_index]; // y distancia/grados
			
			config_gpios();	//Funciones para poner a "correr" el coche con la acción deseada
			config_ciclo(); 
			set_duty_RIGHT_pwm(ciclo_motor2);
			set_duty_LEFT_pwm(ciclo_motor1);
			QEI_pulses();	//función para actualizar cmpos0 y resetear contador 
		}
		
		else // En caso de que previamente se haya acabado el giro de 90º de LEFTGO Y RIGHTGO y tengamos que avanzar
		{
			QEI_pulses(); // IMPORTANTE que esta sea la primera
			
			config_gpios();
			config_ciclo();
			set_duty_RIGHT_pwm(ciclo_motor2);
			set_duty_LEFT_pwm(ciclo_motor1);

		}
	}
}
//
//ADC
//
void config_ADC(uint8_t ADC_modo) // Se utilizaran 4 canales de ADC para: Bateria (AD0.0), Potenciometro (AD0.1), 
																	// Sensor distancia (radar) (AD0.2) y Micrófono (AD0.5). 
																	// Conversión con flanco de subida en MAT1.1 (Pin P1.25)
{	
	LPC_SC->PCONP|= (1<<12);													// Power ON
	
	LPC_PINCON->PINSEL1|= (1<<14)|(1<<16)|(1<<18);  	// Pines: P0.23 (AD0.0), P0.24 (AD0.1), y P0.25 (AD0.2)
	LPC_PINCON->PINSEL3|= (0x3<<30);                  // Pines: P1.31 (AD0.5)
	
	LPC_PINCON->PINMODE1|= (0x2<<14)|(0x2<<16)|(0x2<<18); 	// Deshabilitación pullup/pulldown
	LPC_PINCON->PINMODE3|= (0x2<<30);
	
	LPC_SC->PCLKSEL0 &=~ (0x3<<24); 									// CCLK/4 (Fpclk después del reset) (100 MHz/4 = 25MHz)
	//Habilitación
	if((ADC_modo==Manual_potenciometro)||(ADC_modo==Manual_microfono)||(ADC_modo==Manual_bateria)){
		
		
					LPC_ADC->ADCR = LPC_ADC->ADCR &~(0xFF<<0) 
													&~(1<<16) 						// Modo Manual
													&~(1<<27);						// Flanco de subida genera la conversión
													
					LPC_ADC->ADCR	|=(ADC_modo << 0)				// Selección del canal AD0.x que funcionará en modo Manual
													|(1<<8)		  	  			// CLKDIV=1 (Fclk_ADC = 25Mhz/(1+1)= 12.5MHz) (12.5MHz < 13 MHz)
													|(1<<21)			 				// PDN=1 (se activa su funcionamiento)
													|(7<<24);							// Inicio de conversión con MAT1.1 (Pin P1.25)
	}

	else{																				 	// MODO BURST para sensor y batería
		
					LPC_ADC->ADCR = LPC_ADC->ADCR &~(0xFF<<0) 
													&~(1<<21) 						// POWER DOWN MODE
													&~(0x07<<24);					// Bits START a cero
		
					LPC_ADC->ADCR |=(ADC_modo << 0)				// Selección de los canales AD0.x que funcionará en modo Burst
													|(1<<8)   	  				// CLKDIV=1 (Fclk_ADC = 25Mhz/(1+1)= 12.5MHz) (12.5MHz < 13 MHz)
													|(1<<16);							// Modo Burst

	}
	
	LPC_ADC->ADINTEN &=~ (0x1FF); 
	LPC_ADC->ADINTEN |= (ADC_modo<<0);		// Hab. interrupción tras conversión 
	
	NVIC_SetPriority(ADC_IRQn,2);	
	NVIC_EnableIRQ(ADC_IRQn);		         
}	

void config_TIMER1(uint8_t ADC_modo)	//Configuración del TIMER1, que activara la conversión del ADC en su handler
{
	uint16_t F_TIMER1;
	
	/* ATENCIÓN
	
		- Es muy probable que tengamos que cambiar MR1 en función de si estamos en Manual (cualquiera de las 3 posibles ) o Burst
		- También es problable que MR1 esté mal escrito : comprobarlo
	*/
	//
	
	if(ADC_modo==Manual_potenciometro || ADC_modo==Manual_bateria){
		
		F_TIMER1=1e3;			// Hay un match cada 1/1e3 = 1ms: Se toma valor del potenciómetro cada 2ms (en los flancos de subida de MAT1.1)
	}
	if(ADC_modo==Burst){
	
		F_TIMER1=2e4;			// Hay un match cada 1/2e4 = 50us: Se toma valor del potenciómetro cada 100us (en los flancos de subida de MAT1.1)
	}
	
	LPC_SC->PCONP|=(1<<2);							 // Power ON TIMER1
	LPC_PINCON->PINSEL3|= (0x3<<18);  	 // Pin P1.25 como MAT1.1
  LPC_TIM1->MCR |= (0x2<<3);   				 // Reset TC con match con  MR1 (NO ACTIVA SU HANDLER)
  LPC_TIM1->MR1 = (F_pclk_TIMER1/F_TIMER1)-1; // Valor de MR1; 
	LPC_TIM1->EMR |= (0x3<<6);   				 // Funcionamiento de toggle de la salida MAT1.1 con cada match
	
	LPC_TIM1->TCR |= (1<<1);    // Reseteo del contador  del TIMER1
	LPC_TIM1->TCR &=~ (1<<1);   
	LPC_TIM1->TCR |= (1<<0);    // Habilitación de la cuenta
	
}	
void ADC_IRQHandler(void) //Cargamos el valor actualizado en las variables respectivas
{
	if(LPC_ADC->ADDR1 & (1<<31)){		// se ha producido una lectura de la tensión del potenciómetro (AD0.1)
		
		potenciometro=( (((LPC_ADC->ADDR1 >>4)&0xFFF)*(100))/(4096-1)  );
		
		// ATENCIÓN: Valor en porcentaje (entre 0% y 100%) del valor del potenciómetro 
		
		if(potenciometro<=30){
			
			ciclo=30; // ¡¡NO DEJAREMOS UN CICLO DE FUNCIONAMIENTO MENOR AL 30!!, puede hacer que no termine de arracar las ruedas
		}
		else{
			
			ciclo= potenciometro > 99 ? 100 : potenciometro ; // Nos aseguramos de que ciclo < 100

		}
		
		// YA TENEMOS EL VALOR DE CICLO: se puede empezar con el funcionamiento del coche (igual al hito 2)
			
		config_pwm();
		config_ciclo();
		set_duty_RIGHT_pwm(ciclo_motor2);
		set_duty_LEFT_pwm(ciclo_motor1);
		config_gpios();
	
		modo = Manual_bateria;
		config_ADC(modo);
		config_TIMER1(modo);
		
		vel_teorica=((potenciometro*7.31*170*2*PI*Rrueda)/(100*6*60*100));
		
		}
		
	if(LPC_ADC->ADDR0 & (1<<31)){		// se ha producido una lectura de la batería (AD0.0)
		
		valor_bateria=((((LPC_ADC->ADDR0 >>4)&0xFFF)*8.94)/3350);
		
		//token_ruido= valor_bateria <= 6.3 ? 1 : 0 ; //  Si valor_batería es <= 6.3 , token_ruido vale 1 y altavoz sonará
		
		if(token_ruido ==1){
			
				config_DAC();
				config_TIMER0();
		}
		
	}
	
	
	if(LPC_ADC->ADDR2 & (1<<31)){		// se ha producido una lectura del sensor de distancias (AD0.2)
		
		// movidas del sensor de distancias
	}
	
	if(LPC_ADC->ADDR5 & (1<<31)){		// se ha producido una lectura del micrófono (AD0.5)
	
		// movidas del micrófono
	}
	
	/*
		
	// LPC_ADC->ADCR&=~(1<<16); // BURST=0     // Deshabilitamos el modo Ráfaga (ojo continua la conversión del siguiente canal) 
  
	//Almacenamos las muestras
	//bateria= (((LPC_ADC->ADDR0 >>4)&0xFFF)*BATERIA);	// flag DONE se borra automat. al leer ADDR0 -> Bateria actual
	//NO HACE FALTA potenciometro= (((LPC_ADC->ADDR1 >>4)&0xFFF));	// flag DONE se borra automat. al leer ADDR2 -> Potenciometro min 1,834...
	radar= (((LPC_ADC->ADDR2 >>4)&0xFFF));	// flag DONE se borra automat. al leer ADDR0 -> Temperatura
	// TEMPERATURA NO SE MIDE CON SPI temperatura= (((LPC_ADC->ADDR4 >>4)&0xFFF));	// flag DONE se borra automat. al leer ADDR0 -> Distancia
  //comprobar_radar(radar);
	
	*/
	//
	//
	
}
//
//DAC
//
void genera_muestras()	//señal senoidal
{
	uint16_t i;

	for(i=0;i<N_muestras;i++){
		
		muestras[i]=1023*(0.5 + 0.5*sin(2*PI*i/N_muestras) ); // Ojo! el DAC es de 10bits
	}
}

void config_DAC (void)	//DAC para generar la señal de alarma (P0.26) (AOUT)
{ 
	LPC_PINCON->PINSEL1|= (2<<20); 	 // Configuración pin P0.26 a DAC (AOUT)
	LPC_PINCON->PINMODE1|= (2<<20);  // Deshabilita pullup/pulldown
}
void config_TIMER0(void)	//TIMER para generar la señal de alarma
{
	LPC_SC->PCONP|=(1<<1);						// 	Power ON
	LPC_TIM1->PR = 0x00;     	 				//  Prescaler =1
  LPC_TIM0->MCR |= 0x03;						//  Reset TC on Match e interrumpe tras match con MR0  
  LPC_TIM0->MR0 = F_pclk_TIMER0/(F_SONIDO*N_muestras)-1;  // Cuentas hasta el Match con MR0
	LPC_TIM1->EMR = 0x00;   					//  No actúa sobre el HW
	
  LPC_TIM0->TCR |= (1<<1);    // Reseteo del contador  del TIMER0
	LPC_TIM0->TCR &=~ (1<<1);   
	LPC_TIM0->TCR |= (1<<0);    // Habilitación de la cuenta
	
	NVIC_SetPriority(TIMER0_IRQn,3);	
	NVIC_EnableIRQ(TIMER0_IRQn);		
}

void TIMER0_IRQHandler(void) //Handler del TIMER0
{
	static uint8_t indice_muestra=0;

	LPC_TIM0->IR|= (1<<0); 						// borrar flag
	
	if(token_ruido==1){
		
		LPC_DAC->DACR = muestras[indice_muestra++] << 6; // Se carga el valor de salida del DAC (bit6..bit15)
		
		if(indice_muestra >= N_muestras){
			indice_muestra=0;
		}
		
		alto_ruido = fin_de_tramo == 1 ? (alto_ruido+1) : 0 ; //  Si fin_de_tramo es igual a 1 , alto_ruido incrementa su valor 1 unidad
		
		if(alto_ruido==30000){
			alto_ruido=0;
			token_ruido=0;
			
			LPC_PINCON->PINSEL1 &=~(3<<20); 	 	// Configuración pin a GPIO (P0.26)
			LPC_GPIO0->FIODIR |= (1<<26); 			// GPIO como salida
			LPC_GPIO0->FIOCLR |= (1<<26); 			// P0.26 a 'L'
		}
	}

	LPC_SC->PLL0FEED  = 0xAA;  //Esto ayuda a depurar y no colapsar el sistema 
  LPC_SC->PLL0FEED  = 0x55;
}	
//
//LCD
//
void SysTick_Handler(void){
	if(Ticks % 10 == 0){
		
		sprintf(texto2,"Velocidad teorica:%.4f m/s",vel_teorica);
		// Representa la cadena de texto en el Display
		GUI_Text(50,50,texto2, White, Black);
		
		sprintf(texto,"Velocidad real:%.4f m/s",speed_m);
		// Representa la cadena de texto en el Display
		GUI_Text(50,100,texto, White, Black);
		
		/*sprintf(texto1,"Valor de bateria:%3d V",valor_bateria);
    GUI_Text(50,150,texto1, White, Black);*/
		sprintf(texto1,"Valor de bateria:%.2f V",valor_bateria);
    GUI_Text(50,150,texto1, White, Black);
	}
	Ticks++;
}

//
// SYSTICK -> para mostrar por pantalla las medidas de distancia recorrida y velocidad (funciones systick_config y systick_IRQHandler)
// -> dentro del handler se tendrá que acualizar en cada momento la variable "distancia_actual"
// -> ¿se podría hacer leyendo cada cierto tiempo el registro QEIPOS y aplicando una formula para pasarlo a distancia recorrida?
// -> preguntar sobre el tiempo de parada entre acciones

//
//UART
//

// IMPORTANTE TENER EN CUENTA que si accion = BACK, va a haber que estar restando (en vez de sumar) a "distancia_actual"
//
//En lo referente al QEI:
//-Recordar al final depurar la conversión de cm a pulsos, para ver si es precisa
//EINT 0 + TOKEN 0+TOKEN 1
//speed_rpm