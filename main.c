#include <LPC17xx.h>
#include <Math.h>
#include "QEI.h" 
#include "reg_masks.h"
#include "GLCD.h" 
#include "AsciiLib.h"
#include <stdio.h>
#include <string.h>
#include "uart.h"

/*frecuencias de trabajo de la CPU + perif�ricos 
*/

#define F_cpu 100e6			    // Frecuencia de trabajo de la CPU (100MHz)
#define F_pclk_pwm F_cpu/4 	// Fuente de reloj del perif�rico PWM (25MHz) (al no tocar PCLKSEL, se divide entre 4 por defecto)
#define F_pclk_QEI F_cpu/4	// Fuente de reloj del perif�rico QEI (25MHz) (al no tocar PCLKSEL, se divide entre 4 por defecto)
#define F_pclk_TIMER0 F_cpu/4 // Fuente de reloj del perif�rico TIMER0 (25MHz) (al no tocar PCLKSEL, se divide entre 4 por defecto)
#define F_pclk_TIMER1 F_cpu/4	// Fuente de reloj del perif�rico TIMER1 (25MHz) (al no tocar PCLKSEL, se divide entre 4 por defecto)
#define F_pclk_TIMER2 F_cpu/4	// Fuente de reloj del perif�rico TIMER2 (25MHz) (al no tocar PCLKSEL, se divide entre 4 por defecto)

/*macros movimiento: hemos a�adido 3 movimientos m�s respecto a lo pedido en la descripci�n del proyecto 
*/

#define STOP 0
#define GO 1
#define RIGHT 2 // Permite giros de entre 0� y 360� hacia la derecha
#define LEFT 3  // Permite giros de entre 0� y 360� hacia la izquierda
#define BACK 4 
#define RIGHTGO 5 //Giro 90� a derecha y avanza cierta distancia
#define LEFTGO 6  //Giro 90� a izquierda y avanza cierta distancia

/*macro se�al tiempo en alto complementado (necesario para los movimientos en avance) 
*/

#define COMPLEMENTADO(x) (100-(x))  // En caso de que una rueda avance, el ciclo de trabajo de su se�al PWM es la complementaria

/*macros PWM 
*/

#define F_pwm 500			      // Frecuencia del timer counter del PWM (MR0): 500Hz
#define calibracion 1

/*macros QEI 
*/

#define QEICONF_CAPMODE 0x04 // A�ade un 1 en el bit 2 del registro QEICONF ( CAPMODE=4, resoluci�n x4 (N=4))
#define QEICON_RESP 0x01	   // Con el bit 0 a '1' en QEICON se resetea el contador de posici�n (resetea QEIPOS)
#define QEI_TIM_INT 0x0002   // Con el bit 1 a '1' en QEIIES se activa el handler del QEI cuando QEITIME rebosa
#define QEI_POS0_INT 0x0040  // Con el bit 6 a '1' en QEIIES se activa el handler del QEI cuando QEIPOS=CMPOS0

#define PI 3.141592
#define ANGULO_RECTO 90     //Para las acciones de giro y avance
#define PPR 11							// Pulsos del encoder por revoluci�n de la rueda
#define EDGES 4							// Incremento del contador por cada flanco de PhA y PhB (resoluci�n x4)
#define N 35                // La relaci�n con la reductora (1:35 = Por cada 35 vueltas del motor, la rueda realizar� solo 1 vuelta)
#define Rrueda 3.35         // Radio rueda 67/2 mm = 33,5 mm = 3,35 cm
#define dist_eje 11.5       // Distancia en cm desde una rueda hasta el eje central del coche

/* *****�MUCHO CUIDADO CON LA CA�DA DE LAS SUSPENSIONES!*****                      


En funci�n del peso puede aumentar ligeramente la distancia entre ejes, por el apoyo de la rueda. 
Al ser un recorrido muy peque�o, afectara poco, pero !Ajuste fino de este valor con el montaje final!
*/                     
//

#define cm_cuenta ((2*PI*Rrueda)/(N*PPR*EDGES)) // Los cent�metros que avanza la rueda por unidad de cuenta del encoder	
																								// (por pulso; anoto esto porque me aclaro mejor as�)
#define arco_giro ((2*PI*dist_eje)/360) // Distancia a recorrer (en cm) por cada grado de giro
#define T_obs_QEITIME 100e-3					 // (100ms) Periodo de observaci�n del contador dentro del QEI para medir velocidades (QEITIME) 

#define frecuencia_paro 0.8				// El TIMER2 hace un tiempo de parada de 1.25 segundos (1/1.25=0.8)

#define VELOCIDAD_RPM(x,y) ((x)*60*F_pclk_QEI/(PPR*EDGES*N*(y))) //C�lculo de la velocidad de la rueda en rpm (x=valor QEICAP, y=valor QEILOAD)
#define VELOCIDAD_M_SEG(x) ((x)*2*PI*Rrueda/6000) 	//C�lculo de la velocidad de la rueda en cm/seg (x=velocidad en rpm)

/*macros ADC
*/
#define Manual_potenciometro 2  // En binario: b00000010 (canal entrada AD0.1)
#define Burst 5 								// En binario: b00000101 (calanes de entrada AD0.0 y AD0.2) (Bater�a + Sensor distancia)

/*macros DAC
*/
#define N_muestras 32
#define F_SONIDO 1000						// Frecuencia del sonido de 1000Hz

/*variables de control de motores 
*/

uint8_t accion;  // variable que guarda la acci�n a hacer del coche (STOP=0, GO=1, RIGHT=2, LEFT=3, BACK=4, RIGHTGO=5, LEFTGO=6)
uint8_t mot_1=0;	// variable que guarda la acci�n a hacer del coche para motor 1 (rueda izquierda)
uint8_t mot_2=0;	// variable que guarda la acci�n a hacer del coche para motor 2 (rueda derecha)
uint8_t ciclo;    
float ciclo_motor1;		// Guarda el ciclo de trabajo de la se�al PWM que controla el motor 1 (rueda izquierda)
float ciclo_motor2;   // Guarda el ciclo de trabajo de la se�al PWM que controla motor 2 (rueda derecha)
uint8_t acciones[10]={0}; //Array que en un futuro contendra las consignas de acci�n a recorrer via UART --> Inicializa todos los valores del array a cero (valor STOP)

/*variables de control de QEI
*/

float speed_m=0;
uint16_t dist_actual=0;
uint16_t dist_deseada=0;	
uint16_t speed_rpm = 0;			 // Variable limitada a la funci�n (se borran cada vez que salgamos del handler)

/* ATENCI�N: dist_deseada puede guardar valores de cm o de grados:
	- si avanzamos o retrocedemos, son centimetros a recorrer
	- si giramos, son los grados a girar
*/
//

uint16_t distancias[10]={0}; //Array que en un futuro contendra las distancias (cm) y giros (�) a recorrer via UART --> Inicializa todos los valores del array a cero
uint8_t QEI_token=0;
uint8_t QEI_index=0;

uint32_t pulsos_CMPOS0;

float regular_giro = 0 ;

/*Variables de control ACD
*/
float valor_bateria;
float valor_distancia;
uint8_t modo;										// Modo de trabajo del ADC: "Manual_bateria", "Manual_potenciometro", "Manual_microfono" o "Burst"	
uint16_t potenciometro;
float u_bat;
float vel_teorica;
float distancia;
uint8_t token_parada;
uint8_t reanudar_recorrido;

/*Variables de control DAC
*/
uint8_t token_ruido_recorrido=0;
uint8_t token_ruido_bateria=0;
uint8_t fin_de_tramo=0;
uint16_t alto_ruido=0;
uint16_t muestras[N_muestras];

/*Variables LCD
*/
uint8_t contador;  // Variable que se representa
uint8_t texto[20];
uint8_t texto2[20];
uint8_t texto3[20];
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

extern char buffer[30];			// Buffer de recepci�n
extern char *ptr_rx;				// Puntero de RECEPCI�N
extern char rx_completa;		// Flag de recepci�n de cadena que se activa a "1" al recibir la tecla return CR(ASCII=13)
extern char tx_completa;		// Flag de transmisi�n de cadena que se activa al transmitir el caracter null (fin de cadena)

char fin=0;
uint8_t comand_index=0;     //Indice de lectura del comando enviado via UART
int dato_aux1;              //Variables auxiliares para la gesti�n de los datos recibidos via UART
int dato_aux2;
int dato_aux3;
float bat_aux;
int dato_aux_total;
int bat_read=0;
uint8_t load_index;


/*Declaraci�n de funciones
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
void config_TIMER2();
void TIMER2_IRQHandler(void);

void config_ADC(uint8_t ADC_modo);
void ADC_IRQHandler(void);
void config_TIMER1(uint8_t ADC_modo);
void TIMER1_IRQHandler(void);

void genera_muestras();
void config_DAC();
void config_TIMER0();
void TIMER0_IRQHandler(void);

void SysTick_Handler(void); 
float calculo_distancia(float);
void parada_emergencia(void);

/*Funciones
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
			
  uart3_init(9600);		// configuraci�n de baudrate
								
	tx_cadena_UART3("Introduce el comando de movimientos, estas son las directrices:\n"
	                "UXX para umbral de tension\n"
	                "AXXX para avanzar\n"
	                "MXXX para retroceder\n"
	                "DXXX para giro 90 grados dcha mas avance\n"
	                "IXXX para giro 90 grados izqd mas avance\n"
									"RXXX para giro XXX grados dcha\n"
	                "LXXX para giro XXX grados izqda \n\r");  //Solicitud que informa al usuario via UART3	

	while(tx_completa==0);
	
	while(1){

		if(rx_completa){					 	// Comprabamos la llegada de una cadena por RXD 
			
			if(buffer[0]=='U' && (bat_read==0) ){   // Lectura del umbral de bateria
				
			  dato_aux1=buffer[1] -'0'; // ej: buffer[1]='3' --> '3'-'0' --> 51 - 48 = 3
			  bat_aux=(buffer[2]-'0');
				bat_aux=bat_aux/10;
			  u_bat=dato_aux1+bat_aux;
				
				bat_read=1;		// valor a 1 para no volver a entrar a este if
				load_index=0;
			  comand_index=3; 
        memset(acciones, 0, sizeof(acciones));  //Limpiamos los arrays "acciones" y "distancias"
        memset(distancias, 0, sizeof(distancias));					
			}
			
			switch(buffer[comand_index]){
				 
			 case 'A':                 //Comando avance
			  dato_aux1=buffer[comand_index+1] -'0';
			  dato_aux2=(buffer[comand_index+2]-'0');
			  dato_aux3=(buffer[comand_index+3]-'0');
				dato_aux_total=(dato_aux1*100)+(dato_aux2*10)+dato_aux3;
		    acciones[load_index]=GO;
		    distancias[load_index]=dato_aux_total;
			  load_index++;
		    comand_index+=4;
			 break;
		
			 case 'M':                 //Comando retroceso
			  dato_aux1=buffer[comand_index+1] -'0';
			  dato_aux2=(buffer[comand_index+2]-'0');
			  dato_aux3=(buffer[comand_index+3]-'0');
				dato_aux_total=(dato_aux1*100)+(dato_aux2*10)+dato_aux3;
		    acciones[load_index]=BACK;
		    distancias[load_index]=dato_aux_total;
			  load_index++;
		    comand_index+=4;
			 break;
			 
			 case 'D':                 //Comando 90� dcha mas avance
			  dato_aux1=buffer[comand_index+1] -'0';
			  dato_aux2=(buffer[comand_index+2]-'0');
			  dato_aux3=(buffer[comand_index+3]-'0');
				dato_aux_total=(dato_aux1*100)+(dato_aux2*10)+dato_aux3;
		    acciones[load_index]=RIGHTGO;
		    distancias[load_index]=dato_aux_total;
			  load_index++;
		    comand_index+=4;
			 break;
			 
			 case 'I':                 //Comando 90� izq mas avance
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
			  memset(buffer, 0, sizeof(buffer));  // memset en C es utilizada para rellenar un bloque de memoria con un valor espec�fico.
																						// Esta sentencia hace que todo el contenido de "buffer" sea 0
			  tx_cadena_UART3("Pulsa KEY1 para iniciar el movimiento\n");
			 break;
			 
		 } 
	 }
 }

}

/* **** CONFIGURACI�N DE MOTORES ****
		
		- motor 1 (rueda izquierda)
		- motor 2 (rueda derecha)

		NOTA: 
		- Trabajaremos en modo driver (2� manera mostrada en la descripci�n del proyecto): GPIO(IN1) + PWM(IN2) y Enable siempre fijo
*/
//

void config_ciclo(void) // Configura el ciclo de trabajo de las se�ales PWM (ciclo_motor1 y ciclo_motor2) en funci�n del movimiento
{ 
	
  switch (accion) {
	    case STOP:
							ciclo_motor1=0;
							ciclo_motor2=0;
							break;
      case GO:
            	ciclo_motor1= COMPLEMENTADO(ciclo-calibracion);
							ciclo_motor2= COMPLEMENTADO(ciclo);
							break;
      case RIGHT:
							ciclo_motor1= COMPLEMENTADO(ciclo-calibracion);
							ciclo_motor2= ciclo;
							break;
      case LEFT:
							ciclo_motor1= ciclo-calibracion;
							ciclo_motor2= COMPLEMENTADO(ciclo);
							break;
      case BACK:
							ciclo_motor1= ciclo-calibracion;
							ciclo_motor2= ciclo;
							break;
			case RIGHTGO:
							ciclo_motor1= COMPLEMENTADO(ciclo-calibracion);
							ciclo_motor2= ciclo;
							break;
      case LEFTGO:
							ciclo_motor1= ciclo-calibracion;
							ciclo_motor2= COMPLEMENTADO(ciclo);
							break;
			default:
							break;
   }
}

void config_gpios(void) // Configuraci�n pines de sentido para motor 1 y motor 2 (GPIO)
{ 
	 
	 /*
		Pines de control:
				- P0.0 para motor 1 (rueda izquierda) -> ser� IN1 del driver
				- P0.1 para motor 2 (rueda derecha) -> ser� IN3 del driver
	 */
	 
	 LPC_PINCON->PINSEL0 |= (0<<0); //P0.0 como GPIO
	 LPC_PINCON->PINSEL0 |= (0<<2); //P0.1 como GPIO
	 LPC_GPIO0->FIODIR |=(1<<0); 		//P0.0 como Salida
	 LPC_GPIO0->FIODIR |=(1<<1);    //P0.1 como Salida
	 
	/*	 OBSERVACI�N:
	 Tal y como esta dise�ado el programa, con la variable "accion" podriamos hacer funcionar los motores. 
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
 
void config_pwm(void) 	// Configuraci�n del PWM para motor 1 y motor 2
{
  LPC_SC->PCONP |= (1<<6); 						// Power on
	LPC_SC->PCLKSEL0 &=~ (0x3<<12);     // PWM clk = CCLK/4 (25MHz)
	LPC_PINCON->PINSEL3 |= (1<<11);   	// Configurar pin P1.21 (PWM1.3 output)(MR3) -> para Motor 1 (Rueda izquierda)(IN2)(De momento misma velocidad de giro)
  LPC_PINCON->PINSEL3 |= (1<<17);   	// Configurar pin P1.24 (PWM1.5 output)(MR5) -> para Motor 2 (Rueda derecha)(IN4)(De momento misma velocidad de giro)
	
	LPC_PWM1->MR0 = (F_pclk_pwm/F_pwm) - 1;	// Fijar frecuencia (500 Hz) de las dos PWMs: se hace con el valor de MR0 (modo Single Edge)  
	LPC_PWM1->LER |= (1<<0);
  LPC_PWM1->PCR |= (1<<11)|(1<<13); 	// Todos a modo Single Edge. Habilitaci�n de salidas PWM1.3 (match con MR3) y PWM1.5 (match con MR5)
  LPC_PWM1->MCR |= (1<<1); 						// Reseteo del timer del PWM cuando hay match con MRO
	
  LPC_PWM1->TCR |= (1<<0) | (1<<3); 	// PWM mode ON + Activaci�n PWMs
}

void set_duty_LEFT_pwm(uint8_t cycle) // Ciclo de trabajo (velocidad) motor izquierdo. cycle = ciclo_motor1
{
 
	/*
	Recordar que por culpa de la complementaci�n de la PWM hecha por el driver:
	-> ciclo_motor1 NO ES IGUAL A ciclo_motor2
	*/
	
	LPC_PWM1->MR3 = (LPC_PWM1->MR0 * cycle) / 100; //ciclo de trabajo
	LPC_PWM1->LER |= (1<<3);											 //LER para confirmar cambios
}

void set_duty_RIGHT_pwm(uint8_t cycle) // Ciclo de trabajo (velocidad) motor derecho. cycle = ciclo_motor2
{
 
	/*
	recordar que por culpa de la complementaci�n de la PWM hecha por el driver:
	-> ciclo_motor1 NO ES IGUAL A ciclo_motor2
	*/
	
	LPC_PWM1->MR5 = (LPC_PWM1->MR0 * cycle) / 100; //ciclo de trabajo
	LPC_PWM1->LER |= (1<<5);											 //LER para confirmar cambios
}

void config_botones(void)   //Configuraci�n de los botores KEY1 (EINT1) Y KEY2 (EINT2)
{
	//KEY1 -> EINT1 (P2.11)
	//KEY2 -> EINT2 (P2.12)
	
	LPC_PINCON->PINSEL4 |= (0x5 << 22);	//Pines P2.11 y P2.12 como EINT�s (los pines de los botones)
	LPC_SC->EXTMODE |= 7;				 			  //Activas por flanco
	LPC_SC->EXTPOLAR &= ~0x7;			      //flanco de bajada (recuerda pulsar "bien": puede haber glitches)
	
//Habilitaci�n de las IRQ�s de cada bot�n
	
	NVIC_EnableIRQ(EINT1_IRQn);
	NVIC_EnableIRQ(EINT2_IRQn);
}	

void EINT1_IRQHandler(void)	//Bot�n KEY1 -> Al presionar este bot�n, el coche comienza a moverse
{ 
	LPC_SC->EXTINT = (1 << 1); //Borramos flag de interrupci�n
	
	QEI_token=0;
	accion=acciones[0];     // Cargamos los valores acciones y distancias de los arrays en las variables a trabajar. 
													// Como con el bot�n arrancamos, siempre empezaremos con la primer accion del array
	dist_deseada=distancias[0];
	QEI_index=0;   //Esta ser� la variable que recorrer� todo el array. Al inicializarla como cero podremos de esta forma
								 //repetir el mismo camino desde el principio pulsando de nuevo el bot�n KEY1
	config_QEI();
	
	LPC_PINCON->PINSEL1 &=~(3<<20); 	 	// Configuraci�n pin a GPIO (P0.26)
	LPC_GPIO0->FIODIR |= (1<<26); 			// GPIO como salida
	LPC_GPIO0->FIOCLR |= (1<<26); 			// P0.26 a 'L'

	alto_ruido=0;
	token_ruido_bateria=0;
	token_ruido_recorrido=0;
	fin_de_tramo=0;
	reanudar_recorrido=0;
	
	modo = Manual_potenciometro;
	config_ADC(modo);
	config_TIMER1(modo);
}

void EINT2_IRQHandler(void)	//Bot�n KEY2 -> Al presionar este bot�n, el coche reanuda el movimiento tras detectarse un obst�culo y 
														//desaparecer el obst�culo
{
	LPC_SC->EXTINT = (1 << 2); //Borramos flag de interrupci�n
	
	if(reanudar_recorrido==1){
	
		reanudar_recorrido=0;
		
		accion=acciones[QEI_index];
		ciclo=potenciometro; 				
		config_ciclo(); 
		set_duty_RIGHT_pwm(ciclo_motor2);
		set_duty_LEFT_pwm(ciclo_motor1);
		config_gpios();	
		tx_cadena_UART3("Boton KEI2 pulsado. El coche esta continuando su recorrido\n");
		
	}

}
//
// QEI
//
void config_QEI(void) //Configuraci�n del QEI
{
	LPC_SC->PCONP |= (1<<18);				  // Power ON del QEI
	LPC_SC->PCLKSEL1 &=~ (0x3<<0);		// PCLK = CCLK/4 (25MHz)
	LPC_PINCON->PINSEL3	|= (1<<8);		// MCI0 en P1.20 (leer� "Hall signal A")
	LPC_PINCON->PINSEL3	|= (1<<14);		// MCI1 en P1.23 (leer� "Hall signal B")
	
	LPC_QEI->QEICONF	= QEICONF_CAPMODE; 						// Resoluci�n x4 (edges=4)
	LPC_QEI->QEICON 	= QEICON_RESP;		 						// Reseteo inicial de QEIPOS a cero
	LPC_QEI->QEIMAXPOS = 0xFFFFFFFF; 		 						// Nos da igual, as� que ponemos el valor m�ximo
	LPC_QEI->QEILOAD 	= F_pclk_QEI * T_obs_QEITIME;	// valor de recarga del contador de velocidad QEITIME
		
  LPC_QEI->QEIIEC &=~ 0xFFFFFFFF;                	 //Limpieza de QEIIE
  LPC_QEI->QEIIES |= QEI_TIM_INT | QEI_POS0_INT;  
	
	/* 
		Escritura en QEIIE. Permitimos la interrupci�n cuando:
	
		- QEITIME rebosa (se usa para leer la velocidad del coche)
		- QEIPOS=CMPOS0
	*/
	
  NVIC_SetPriority(QEI_IRQn,2);   
  NVIC_EnableIRQ(QEI_IRQn);   	  // Habilitamos el handler del QEI
	
}


void QEI_pulses()	//Funci�n que actualiza el valor de CMPOS0 y resetea el contador del QEI
	                //Discrimina en funci�n de la acci�n a realizar, si la variable dist_deseada, son cm a recorrer o grados a girar. 
									//Y tras eso hace la conversi�n adecuada
{   																		
		/*ATENCI�N:
	
		- Es MUY IMPORTANTE aqu� tener en cuenta que el QEI cuenta los pulsos del encoder del motor de la IZQUIERDA 
	  */
		//
	
	if (QEI_token==0){ //Token usado para GO, BACK, LEFT, RIGHT, STOP y los giros de 90� en LEFTGO Y RIGHTGO
		
		switch(accion){		//Por cada accion siempre debemos: convertir distancia, cargarla en CMPOS0 y resetear contador QEIPOS
			
			case STOP:  //Si paramos, es porque hemos llegado al �ltimo valor del array o valor NULL, es decir, accion=STOP
				LPC_QEI->QEIIEC |= 0xFFFFFFFF; //Limpieza de QEIIE para que ya no salte el CMPOS0 por error, tras la parada
				ciclo=0; 				
				config_ciclo(); //Las dos PWMs a con ciclo de trabajo igual a cero
				set_duty_RIGHT_pwm(ciclo_motor2);
			  set_duty_LEFT_pwm(ciclo_motor1);
				config_gpios(); //P0.1 y P0.0 a 'L' -> Ver nota de abajo
			
				/*ATENCI�N: IMPRESCINDIBLE HACERLO. 
					En un caso extremo, si GPIO a 'H' con PWM a 0%, el driver no discrimina que pin esta conetado a GPIO y cual PWM. 
					De forma que en vez de parar, se puede quedar girando a m�xima velocidad la rueda
				*/
			
			  speed_m=0; //Porque si no, al parar tiene cualquier valor
				token_ruido_recorrido=1;
				fin_de_tramo=1;
				config_DAC();
				config_TIMER0();
				
				tx_cadena_UART3("Final de recorrido\n Pulsa KEY1 para repetir o envia otro recorrido\n");
				
				
				break;
			
			case GO:           
				LPC_QEI->QEICON = QEICON_RESP;		 // Reseteo de QEIPOS a cero
				pulsos_CMPOS0 = (uint32_t) dist_deseada/cm_cuenta;
				LPC_QEI->CMPOS0 = pulsos_CMPOS0;
			  tx_cadena_UART3("Movimiento hacia adelante\n");
				break;
	
			case RIGHT: // ATENCI�N: El motor de la izquierda AVANZA para girar a la derecha
				LPC_QEI->QEICON = QEICON_RESP;		 // Reseteo de QEIPOS a cero
				pulsos_CMPOS0 = (uint32_t) (arco_giro*dist_deseada)/cm_cuenta;
				LPC_QEI->CMPOS0 =  pulsos_CMPOS0;
			  tx_cadena_UART3("Giro hacia la derecha\n");
				break;
			
			case LEFT:	// ATENCI�N: El motor de la izquierda RETROCEDE para girar a la izquierda
				LPC_QEI->QEICON = QEICON_RESP;		 // Reseteo de QEIPOS a cero
				pulsos_CMPOS0 = (uint32_t) (arco_giro*dist_deseada)/cm_cuenta;
				LPC_QEI->CMPOS0 = - pulsos_CMPOS0; // ATENCI�N: CUENTA DESCENTENDE al ir el motor hacia atr�s
				tx_cadena_UART3("Giro hacia la izquierda\n");
				break;
			
			case BACK:
				LPC_QEI->QEICON = QEICON_RESP;	 // Reseteo de QEIPOS a cero
				pulsos_CMPOS0 = (uint32_t) dist_deseada/cm_cuenta;
				LPC_QEI->CMPOS0 = - pulsos_CMPOS0; // ATENCI�N: CUENTA DESCENTENDE al ir el motor hacia atr�s
		  	tx_cadena_UART3("Movimiento hacia atras\n");
				break;
	
			case RIGHTGO: // ATENCI�N: El motor de la izquierda AVANZA para girar a la derecha
				LPC_QEI->QEICON = QEICON_RESP;		 // Reseteo de QEIPOS a cero
				pulsos_CMPOS0 = (uint32_t) (arco_giro*ANGULO_RECTO)/cm_cuenta;  //En esta acci�n se define el giro de 90 grados con una macro, pues es condici�n de giro
				LPC_QEI->CMPOS0 = pulsos_CMPOS0;
				
				QEI_token=1;  //Token para que la pr�xima acci�n sea el avance con los cm deseados
		  	tx_cadena_UART3("Giro de 90 grados a derecha y recto\n");
				break;
			
			case LEFTGO:	// ATENCI�N: El motor de la izquierda RETROCEDE para girar a la izquierda
				LPC_QEI->QEICON = QEICON_RESP;		 // Reseteo de QEIPOS a cero
				pulsos_CMPOS0 = (uint32_t) (arco_giro*ANGULO_RECTO)/cm_cuenta; //En esta acci�n se define el giro de 90 grados con una macro, pues es condici�n de giro
				LPC_QEI->CMPOS0 = - pulsos_CMPOS0;	 // ATENCI�N: CUENTA DESCENTENDE al ir el motor hacia atr�s
				
				QEI_token=1;  //Token para que la pr�xima acci�n sea el avance con los cm deseados
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
		
		if( (acciones[QEI_index+1]!=STOP) || (QEI_token ==1) ){
			
			accion=STOP;                 //Inicio secuencia de parada
			ciclo=0; 				
			config_ciclo(); 
			set_duty_RIGHT_pwm(ciclo_motor2);
			set_duty_LEFT_pwm(ciclo_motor1);
			config_gpios();
			
			config_TIMER2();
		}
		else{
			
			QEI_index++; 								//Actualizamos el indice para la pr�xima acci�n 
			accion=acciones[QEI_index]; //Actualizamos pr�xima accion
		
			QEI_pulses();	//funci�n para actualizar cmpos0 y resetear contador 
		}
			
	}

}
void config_TIMER2(void)
{
	LPC_SC->PCONP|=(1<<22);					// 	Power ON
	LPC_TIM2->PR = 0x00;     	 			//  Prescaler = 1
  LPC_TIM2->MCR = 0x07;						//  Contador parado tras Match con MR0, reset del contador y activaci�n de la interrupci�n tras Match con MR0  
  LPC_TIM2->MR0 = F_pclk_TIMER2/(frecuencia_paro)-1;  // Cuentas hasta el Match con MR0
	LPC_TIM2->EMR = 0x00;   				//  No act�a sobre el HW
	
	LPC_TIM2->TCR |= (1<<0);    // Habilitaci�n de la cuenta
	
	NVIC_SetPriority(TIMER2_IRQn,3);	
	NVIC_EnableIRQ(TIMER2_IRQn);		
}
void TIMER2_IRQHandler(void) //Handler del TIMER2
{
	LPC_TIM2->IR|= (1<<0); 						// borrar flag
	
	if(QEI_token==0)	//En caso de que la acci�n previa haya sido GO, BACK, LEFT, RIGHT o la finalizaci�n de RIGHTGO o LEFTGO
	{ 
		QEI_index++; 								//Actualizamos el indice para la pr�xima acci�n 
		accion=acciones[QEI_index]; //Actualizamos pr�xima accion
		ciclo=potenciometro; 				//Volvemos a definir el ciclo de trabajo previamente medido en el potenci�metro
		dist_deseada=distancias[QEI_index]; // Actualizamos la pr�xima distancia o grados a recorrer
			
		config_ciclo(); //Funciones para poner a "correr" el coche con la acci�n deseada
		set_duty_RIGHT_pwm(ciclo_motor2);
		set_duty_LEFT_pwm(ciclo_motor1);
		config_gpios();	
		
		QEI_pulses();	//funci�n para actualizar cmpos0 y resetear contador 
		
		LPC_SC->PCONP &=~ (1<<22);					// 	Power OFF TIMER2
	}
		
	else // En caso de que previamente se haya acabado el giro de 90� de LEFTGO Y RIGHTGO y tengamos que avanzar
	{
		QEI_pulses(); // IMPORTANTE que esta sea la primera
		ciclo=potenciometro;
		
		config_ciclo();
		set_duty_RIGHT_pwm(ciclo_motor2);
		set_duty_LEFT_pwm(ciclo_motor1);
		config_gpios();
		
		LPC_SC->PCONP &=~ (1<<22);					// 	Power OFF TIMER2
	}
	
}
//
//ADC
//
void config_ADC(uint8_t ADC_modo) // Se utilizaran 3 canales de ADC para: Bateria (AD0.0), Potenciometro (AD0.1) y Sensor distancia (radar) (AD0.2).
																	// Conversi�n con flanco de subida en MAT1.1 (Pin P1.25)
{	
	LPC_SC->PCONP|= (1<<12);													// Power ON
	
	LPC_PINCON->PINSEL1|= (1<<14)|(1<<16)|(1<<18);  	// Pines: P0.23 (AD0.0), P0.24 (AD0.1), y P0.25 (AD0.2)
	LPC_PINCON->PINSEL3|= (0x3<<30);                  // Pines: P1.31 (AD0.5)
	
	LPC_PINCON->PINMODE1|= (0x2<<14)|(0x2<<16)|(0x2<<18); 	// Deshabilitaci�n pullup/pulldown
	LPC_PINCON->PINMODE3|= (0x2<<30);
	
	LPC_SC->PCLKSEL0 &=~ (0x3<<24); 									// CCLK/4 (Fpclk despu�s del reset) (100 MHz/4 = 25MHz)
	
	//Habilitaci�n
	
	if(ADC_modo==Manual_potenciometro){
		
		
					LPC_ADC->ADCR = LPC_ADC->ADCR &~(0xFF<<0) 
													&~(1<<16) 						// Modo Manual
													&~(1<<27);						// Flanco de subida genera la conversi�n
													
					LPC_ADC->ADCR	|=(ADC_modo << 0)				// Selecci�n del canal AD0.x que funcionar� en modo Manual
													|(1<<8)		  	  			// CLKDIV=1 (Fclk_ADC = 25Mhz/(1+1)= 12.5MHz) (12.5MHz < 13 MHz)
													|(1<<21)			 				// PDN=1 (se activa su funcionamiento)
													|(7<<24);							// Inicio de conversi�n con MAT1.1 (Pin P1.25)
		
		LPC_ADC->ADINTEN &=~ (0x1FF); 
	  LPC_ADC->ADINTEN |= (ADC_modo<<0);		// Hab. interrupci�n tras conversi�n
	}

	else if(ADC_modo==Burst){																				 	// MODO BURST para sensor y bater�a
		
					LPC_ADC->ADCR = LPC_ADC->ADCR &~(0xFF<<0) //Limpiar canales a convertir 
													&~(0x07<<24);					// Bits START a cero
		
					LPC_ADC->ADCR |=(ADC_modo << 0)				// Selecci�n de los canales AD0.x que funcionar� en modo Burst
													|(1<<8);   	  				// CLKDIV=1 (Fclk_ADC = 25Mhz/(1+1)= 12.5MHz) (12.5MHz < 13 MHz)
		
		LPC_ADC->ADINTEN &=~ (0x1FF); 
	  LPC_ADC->ADINTEN |= (1<<2);		// Hab. interrupci�n tras conversi�n		
	}
	
	  NVIC_SetPriority(ADC_IRQn,2);	
	  NVIC_EnableIRQ(ADC_IRQn);
}	

void config_TIMER1(uint8_t ADC_modo)	//Configuraci�n del TIMER1, que activara la conversi�n del ADC en su handler
{
	uint32_t F_TIMER1;
	
	/* ATENCI�N
	
		- Es muy probable que tengamos que cambiar MR1 en funci�n de si estamos en Manual (cualquiera de las 3 posibles ) o Burst
		- Tambi�n es problable que MR1 est� mal escrito : comprobarlo
	*/
	//
	
	if(ADC_modo==Manual_potenciometro){
		
		F_TIMER1=1e3;												 // Hay un match cada 1/1e3 = 1ms
	  LPC_SC->PCONP|=(1<<2);							 // Power ON TIMER1
	  LPC_PINCON->PINSEL3|= (0x3<<18);  	 // Pin P1.25 como MAT1.1
		
    LPC_TIM1->MCR = (0x2<<3);   				 // Reset TC con match con  MR1 (NO ACTIVA SU HANDLER)
    LPC_TIM1->MR1 = (F_pclk_TIMER1/F_TIMER1)-1; // Valor de MR1; 
	  LPC_TIM1->EMR = (0x3<<6);   				 // Funcionamiento de toggle de la salida MAT1.1 con cada match
	
	  LPC_TIM1->TCR |= (1<<1);    // Reseteo del contador  del TIMER1
	  LPC_TIM1->TCR &=~ (1<<1);   
	  LPC_TIM1->TCR |= (1<<0);    // Habilitaci�n de la cuenta
	}
	if(ADC_modo==Burst){
	
		F_TIMER1=5;													// Hay un match cada 1/5 = 200ms
		
		LPC_TIM1->MCR = 0x18;   					  // Interrupci�n con Match 1 y Reset TC
    LPC_TIM1->MR1 = (F_pclk_TIMER1/F_TIMER1)-1; //  Periodo de muestreo de TODAS las entradas!!!!   
    LPC_TIM1->EMR = 0x00C2; 
		 
    LPC_TIM1->TCR |= (1<<1);    // Reseteo del contador  del TIMER1
	  LPC_TIM1->TCR &=~ (1<<1);   
	  LPC_TIM1->TCR |= (1<<0);    // Habilitaci�n de la cuenta
		
		NVIC_SetPriority(TIMER1_IRQn,1);			
		NVIC_EnableIRQ(TIMER1_IRQn);	
	}
	
}	

void TIMER1_IRQHandler(void)
{	
	LPC_ADC->ADCR|=(1<<16); // BURST=1 --> Se toma una muestra de cada canal comenzando desde el m�s bajo
	LPC_TIM1->IR|=(1<<1);		// Borrar flag interrupci�n	

}

void ADC_IRQHandler(void) //Cargamos el valor actualizado en las variables respectivas
{
	LPC_ADC->ADCR&=~(1<<16); // BURST=0 
	
	if(LPC_ADC->ADDR1 & (1<<31)){		// se ha producido una lectura de la tensi�n del potenci�metro (AD0.1)
		
		potenciometro=( (((LPC_ADC->ADDR1 >>4)&0xFFF)*(100))/(4096-1)  );
		
		// ATENCI�N: Valor en porcentaje (entre 0% y 100%) del valor del potenci�metro 
		
		if(potenciometro <= 30){
			
			potenciometro=30;	// ��NO DEJAREMOS UN CICLO DE FUNCIONAMIENTO MENOR AL 30% !!, ya que puede hacer que no termine de arracar las ruedas
		}
		else{
			
			if (potenciometro >= 100){
				
				potenciometro=100;	// Nos aseguramos de que ciclo no sea mayor que 100
			}
		}
			
		ciclo= potenciometro;

		
		// YA TENEMOS EL VALOR DE CICLO: se puede empezar con el funcionamiento del coche
			
		config_pwm();
		config_ciclo();
		set_duty_RIGHT_pwm(ciclo_motor2);
		set_duty_LEFT_pwm(ciclo_motor1);
		config_gpios();
		QEI_pulses();
	
		modo = Burst;
		config_ADC(modo);
		config_TIMER1(modo);
		
		vel_teorica=((potenciometro*7.31*170*2*PI*Rrueda)/(100*6*60*100));
		
		}
		
	
	if(LPC_ADC->ADDR2 & (1<<31)){	// se ha producido una lectura del sensor de distancias (AD0.2)
		
		valor_bateria= ((((LPC_ADC->ADDR0 >>4)&0xFFF)*8.94)/3350) - 0.6;
		
		token_ruido_bateria = valor_bateria <= u_bat ? 1 : 0 ; //  Si valor_bater�a es <= u_bat, token_ruido vale 1 y altavoz sonar�
		
		if(token_ruido_bateria ==1){
			
				config_DAC();
				config_TIMER0();
		}
		
		// FUNCIONES QUE REGULAN LA DISTANCIA DE LOS OBJETOS AL SENSOR DEL COCHE
		
		valor_distancia=((((LPC_ADC->ADDR2 >>4)&0xFFF)*(3.3))/(4096-1));
		calculo_distancia(valor_distancia); //Conversi�n a cm
		parada_emergencia();
	}
	
}
float calculo_distancia(float voltios) // Para calcular la distancia de los objetos al sensor de distancias
{
	float v_table[] = {0.45, 0.48, 0.50, 0.60, 0.65, 0.70, 0.75, 0.80, 0.90, 1.10, 1.25, 1.55, 2.00, 2.50, 2.75};	//Hay 15 valores
	float d_table[] = {150, 140, 130, 120, 110, 100, 90, 80, 70, 60, 50, 40, 30, 20, 15};
	float pendiente;
	int i;

	for(i=0; i<14; i++){
		if( (voltios>=v_table[i]) &  (voltios<v_table[i+1]) ) {							//Si el valor es� entre esos limites
			pendiente = (d_table[i+1] - d_table[i]) / (v_table[i+1] - v_table[i]);		//Calcula la pendiente
			distancia = pendiente * (voltios - v_table[i]) + d_table[i];				//Calcula la distancia
			
		}
	}

	return distancia;
}


void parada_emergencia(void)	// Controla la parada de emergencia tras encontrar un obst�culo
{
	
	if(distancia<27){	 // Si se ha encontrado un obst�culo a menos de 27 cm de distancia
		
		token_parada=1;
		
		accion=STOP;
		ciclo=0; 				
		config_ciclo(); 
		set_duty_RIGHT_pwm(ciclo_motor2);
		set_duty_LEFT_pwm(ciclo_motor1);
		config_gpios();	
		tx_cadena_UART3("ALERTA, parada de emergencia\n");
			
	} else if( (token_parada==1) && (distancia>=27) ){	// Si ha habido un obst�culo previamente y ya no se encuentra a menos 27 cm del coche
			
		token_parada=0;
		reanudar_recorrido=1; // Variable usada para hacer que el coche contin�e con el recorrido tras pulsar KEY2
		
		tx_cadena_UART3("No hay objeto delante. Pulsa KEY2 en caso de que el coche estuviera moviendose y haya ocurrido una parada de emergencia\n");
			
	}

}
//
//DAC
//
void genera_muestras()	//Creaci�n de la se�al senoidal para los altavoces
{
	uint16_t i;

	for(i=0;i<N_muestras;i++){
		
		muestras[i]=1023*(0.5 + 0.5*sin(2*PI*i/N_muestras) ); // Ojo! el DAC es de 10bits
	}
}

void config_DAC (void)	//DAC para generar la se�al de alarma (P0.26) (AOUT)
{ 
	LPC_PINCON->PINSEL1|= (2<<20); 	 // Configuraci�n pin P0.26 a DAC (AOUT)
	LPC_PINCON->PINMODE1|= (2<<20);  // Deshabilita pullup/pulldown
}
void config_TIMER0(void)	//TIMER para generar la se�al de alarma
{
	LPC_SC->PCONP|=(1<<1);						// 	Power ON
	LPC_TIM0->PR = 0x00;     	 				//  Prescaler =1
  LPC_TIM0->MCR |= 0x03;						//  Reset TC on Match e interrumpe tras match con MR0  
  LPC_TIM0->MR0 = F_pclk_TIMER0/(F_SONIDO*N_muestras)-1;  // Cuentas hasta el Match con MR0
	LPC_TIM0->EMR = 0x00;   					//  No act�a sobre el HW
	
  LPC_TIM0->TCR |= (1<<1);    // Reseteo del contador  del TIMER0
	LPC_TIM0->TCR &=~ (1<<1);   
	LPC_TIM0->TCR |= (1<<0);    // Habilitaci�n de la cuenta
	
	NVIC_SetPriority(TIMER0_IRQn,3);	
	NVIC_EnableIRQ(TIMER0_IRQn);		
}

void TIMER0_IRQHandler(void) //Handler del TIMER0
{
	static uint8_t indice_muestra=0;

	LPC_TIM0->IR|= (1<<0); 			// Borrar flag
	
	if( (token_ruido_bateria==1) || (token_ruido_recorrido==1) ){
		
		LPC_DAC->DACR = muestras[indice_muestra++] << 6; // Se carga el valor de salida del DAC (bit6..bit15)
		
		if(indice_muestra >= N_muestras){
			indice_muestra=0;
		}
		
		alto_ruido = fin_de_tramo == 1 ? (alto_ruido+1) : 0 ; //  Si fin_de_tramo es igual a 1 , alto_ruido incrementa su valor 1 unidad
		
		if(alto_ruido==30000){
			alto_ruido=0;
			token_ruido_recorrido=0;
			
			LPC_PINCON->PINSEL1 &=~(3<<20); 	 	// Configuraci�n pin a GPIO (P0.26)
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
void SysTick_Handler(void) // Para mostrar por pantalla las medidas de distancia recorrida 
													 // y velocidad (funciones systick_config y systick_IRQHandler)
{
	if(Ticks % 10 == 0){
		
		sprintf(texto2,"Velocidad teorica:%.4f m/s",vel_teorica);
		// Representa la cadena de texto en el Display
		GUI_Text(50,50,texto2, White, Black);
		
		sprintf(texto,"Velocidad real:%.4f m/s",speed_m);
		// Representa la cadena de texto en el Display
		GUI_Text(50,100,texto, White, Black);
		
		sprintf(texto3,"Distancia:%.2f cm ",distancia);
		// Representa la cadena de texto en el Display
		GUI_Text(50,200,texto3, White, Black);
		
	
		sprintf(texto1,"Valor de bateria:%.2f V",valor_bateria);
    GUI_Text(50,150,texto1, White, Black);
	}
	Ticks++;
}

