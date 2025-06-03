#include <LPC17xx.h>
#include <Math.h>
#include "QEI.h" 
#include "reg_masks.h"

/*frecuencias de trabajo de la CPU + perif�ricos 
*/

#define F_cpu 100e6			    // Frecuencia de trabajo de la CPU (100MHz)
#define F_pclk_pwm F_cpu/4 	// Fuente de reloj del perif�rico PWM (25MHz) (al no tocar PCLKSEL, se divide entre 4 por defecto)
#define F_pclk_QEI F_cpu/4	// Fuente de reloj del perif�rico QEI (25MHz) (al no tocar PCLKSEL, se divide entre 4 por defecto)
#define F_pclk_TIMER1 F_cpu/4	// Fuente de reloj del perif�rico TIMER1 (25MHz) (al no tocar PCLKSEL, se divide entre 4 por defecto)
#define F_pclk_TIMER0 F_cpu/4 // Fuente de reloj del perif�rico TIMER0 (25MHz) (al no tocar PCLKSEL, se divide entre 4 por defecto)

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
#define dist_eje 10         // Distancia en cm desde una rueda hasta el eje central del coche

/* *****�MUCHO CUIDADO CON LA CA�DA DE LAS SUSPENSIONES!*****                      


En funci�n del peso puede aumentar ligeramente la distancia entre ejes, por el apoyo de la rueda. 
Al ser un recorrido muy peque�o, afectara poco, pero !Ajuste fino de este valor con el montaje final!
*/                     
//

#define cm_cuenta ((2*PI*Rrueda)/(N*PPR*EDGES)) // Los cent�metros que avanza la rueda por unidad de cuenta del encoder	(por pulso; anoto esto porque me aclaro mejor as�)
#define arco_giro ((2*PI*dist_eje)/360) // Distancia a recorrer (en cm) por cada grado de giro
#define T_obs_QEITIME 100e-3					 // (100ms) Periodo de observaci�n del contador dentro del QEI para medir velocidades (QEITIME) 

#define VELOCIDAD_RPM(x,y) ((x)*60*F_pclk_QEI/(PPR*EDGES*N*(y))) //C�lculo de la velocidad de la rueda en rpm (x=valor QEICAP, y=valor QEILOAD)
#define VELOCIDAD_CM_SEG(x) ((x)*2*PI*Rrueda/60) 	//C�lculo de la velocidad de la rueda en cm/seg (x=velocidad en rpm)

/*macros ADC
*/
#define Manual_bateria 1  			// En binario: b00000001 (canal entrada AD0.0)
#define Manual_potenciometro 2  // En binario: b00000010 (canal entrada AD0.1)
#define Manual_microfono 32 		// En binario: b00100000 (canal entrada AD0.5)
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
uint8_t ciclo_motor1;		// Guarda el ciclo de trabajo de la se�al PWM que controla el motor 1 (rueda izquierda)
uint8_t ciclo_motor2;   // Guarda el ciclo de trabajo de la se�al PWM que controla motor 2 (rueda derecha)
uint8_t acciones[4]={GO,BACK,RIGHTGO,STOP}; //Array que en un futuro contendra las consignas de acci�n a recorrer via UART

/*variables de control de QEI
*/

uint16_t speed_cm=0;
uint16_t dist_actual=0;
uint16_t dist_deseada=0;		

/* ATENCI�N: dist_deseada puede guardar valores de cm o de grados:
	- si avanzamos o retrocedemos, son centimetros a recorrer
	- si giramos, son los grados a girar
*/
//

uint16_t distancias[4]={90,50,40,0}; //Array que en un futuro contendra las distancias (cm) y giros (�) a recorrer via UART
uint8_t QEI_token=0;
uint8_t QEI_index=0;

uint32_t pulsos_CMPOS0;

/*Variables de control ACD
*/
float valor_bateria;						
uint8_t modo;										// Modo de trabajo del ADC: "Manual_bateria", "Manual_potenciometro", "Manual_microfono" o "Burst"	
uint16_t potenciometro;

/*Variables de control DAC
*/
uint8_t token_ruido=0;
uint8_t fin_de_tramo=0;
uint16_t alto_ruido=0;
uint16_t muestras[N_muestras];

/*declaraci�n de funciones
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

/*funciones
*/

int main(void)
{
	NVIC_SetPriorityGrouping(2);
	
	genera_muestras();
	config_botones();

  while(1){
		
	}
}

//
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

void config_gpios(void) // Configuraci�n pines de sentido para motor 1 y motor 2 (GPIO).
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
 
void config_pwm(void) 	//Configuraci�n del PWM para motor 1 y motor 2
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
	recordar que por culpa de la complementaci�n de la PWM hecha por el driver:
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

void config_botones(void)   //Configuraci�n de los botores KEI1 (EINT1) Y KEI2 (EINT2)
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
	
// Para el proyecto final, cargamos acciones[0], con su valor en otro array. Configurar aqui QEI,PWM, GPIOS, DUTY
	
	QEI_token=0;
	accion=acciones[0];     // Cargamos los valores acciones y distancias de los arrays en las variables a trabajar. 
													// Como con el bot�n arrancamos, siempre empezaremos con la primer accion del array
	dist_deseada=distancias[0];
	QEI_index=0;   //Esta ser� la variable que recorrer� todo el array. Al inicializarla como cero podremos de esta forma
								 //repetir el mismo camino desde el principio pulsando de nuevo el bot�n KEY1
	config_QEI();
	QEI_pulses();
	
	LPC_PINCON->PINSEL1 &=~(3<<20); 	 	// Configuraci�n pin a GPIO (P0.26)
	LPC_GPIO0->FIODIR |= (1<<26); 			// GPIO como salida
	LPC_GPIO0->FIOCLR |= (1<<26); 			// P0.26 a 'L'

	alto_ruido=0;
	token_ruido=0;
	fin_de_tramo=0;
	
	modo = Manual_potenciometro;		// ATENCI�N: antes de leer el potenci�metro, es necesario saber la tensi�n actual de la bater�a
	config_ADC(modo);
	config_TIMER1(modo);
}

void EINT2_IRQHandler(void)	//Bot�n KEY2 
{
	LPC_SC->EXTINT = (1 << 2); //Borramos flag de interrupci�n
	
 /* Dejamos esto preparado. Seguro que para el proyecto final, le encontramos un buen uso.
		Ideas:
		-> Conectar/Desconcetar ventilador
		-> Conectar/Desconectar sensor distancias (le hemos a�adido un sensor de distancia al coche)
	*/
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
	
  NVIC_SetPriority(QEI_IRQn,2);   // Solo ajusta el nivel de prioridad
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
			
				token_ruido=1;
				fin_de_tramo=1;
				config_DAC();
				config_TIMER0();
				break;
			
			case GO:           
				LPC_QEI->QEICON = QEICON_RESP;		 // Reseteo de QEIPOS a cero
				pulsos_CMPOS0 = (uint32_t) dist_deseada/cm_cuenta;
				LPC_QEI->CMPOS0 = pulsos_CMPOS0;
				break;
	
			case RIGHT: // ATENCI�N: El motor de la derecha RETROCEDE para girar a la derecha
				LPC_QEI->QEICON = QEICON_RESP;		 // Reseteo de QEIPOS a cero
				pulsos_CMPOS0 = (uint32_t) (arco_giro*dist_deseada)/cm_cuenta;
				LPC_QEI->CMPOS0 =  pulsos_CMPOS0;
				break;
			
			case LEFT:	// ATENCI�N: El motor de la derecha AVANZA para girar a la izquierda
				LPC_QEI->QEICON = QEICON_RESP;		 // Reseteo de QEIPOS a cero
				pulsos_CMPOS0 = (uint32_t) (arco_giro*dist_deseada)/cm_cuenta;
				LPC_QEI->CMPOS0 = - pulsos_CMPOS0; // ATENCI�N: CUENTA DESCENTENDE al ir el motor hacia atr�s
				break;
			
			case BACK:
				LPC_QEI->QEICON = QEICON_RESP;	 // Reseteo de QEIPOS a cero
				pulsos_CMPOS0 = (uint32_t) dist_deseada/cm_cuenta;
				LPC_QEI->CMPOS0 = - pulsos_CMPOS0; // ATENCI�N: CUENTA DESCENTENDE al ir el motor hacia atr�s
				break;
	
			case RIGHTGO: // ATENCI�N: El motor de la derecha RETROCEDE para girar a la derecha
				LPC_QEI->QEICON = QEICON_RESP;		 // Reseteo de QEIPOS a cero
				pulsos_CMPOS0 = (uint32_t) (arco_giro*ANGULO_RECTO)/cm_cuenta;  //En esta acci�n se define el giro de 90 grados con una macro, pues es condici�n de giro
				LPC_QEI->CMPOS0 = pulsos_CMPOS0;
				
				QEI_token=1;  //Token para que la pr�xima acci�n sea el avance con los cm deseados
				break;
			
			case LEFTGO:	// ATENCI�N: El motor de la derecha AVANZA para girar a la izquierda
				LPC_QEI->QEICON = QEICON_RESP;		 // Reseteo de QEIPOS a cero
				pulsos_CMPOS0 = (uint32_t) (arco_giro*ANGULO_RECTO)/cm_cuenta; //En esta acci�n se define el giro de 90 grados con una macro, pues es condici�n de giro
				LPC_QEI->CMPOS0 = - pulsos_CMPOS0;	 // ATENCI�N: CUENTA DESCENTENDE al ir el motor hacia atr�s
				
				QEI_token=1;  //Token para que la pr�xima acci�n sea el avance con los cm deseados
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
		uint16_t speed_rpm = 0;						// Variable limitada a la funci�n (se borran cada vez que salgamos del handler)
		speed_rpm = (uint16_t) VELOCIDAD_RPM(LPC_QEI->QEICAP , LPC_QEI->QEILOAD); //velocidad en rpm (sin decimales)
		speed_cm = (uint16_t) VELOCIDAD_CM_SEG(speed_rpm); // velocidad en cm/s  (sin decimales)         		
	}     
	
	if (LPC_QEI->QEIINTSTAT & QEI_POS0_INT)		// Verifica si ha saltado la IRQ porque se ha llegado a la distancia recorrida deseada
	{  
		LPC_QEI->QEICLR |= QEI_POS0_INT;					// Borramos flag
		
		if(QEI_token==0)	//En caso de que la acci�n previa haya sido GO, BACK, LEFT, RIGHT, STOP o la finalizaci�n de RIGHTGO o LEFTGO
		{ 
			QEI_index++; //Actualizamos el indice para la pr�xima acci�n 
			accion=acciones[QEI_index]; //Actualizamos pr�xima accion...
			dist_deseada=distancias[QEI_index]; // y distancia/grados
			
			config_gpios();	//Funciones para poner a "correr" el coche con la acci�n deseada
			config_ciclo(); 
			set_duty_RIGHT_pwm(ciclo_motor2);
			set_duty_LEFT_pwm(ciclo_motor1);
			QEI_pulses();	//funci�n para actualizar cmpos0 y resetear contador 
		}
		
		else // En caso de que previamente se haya acabado el giro de 90� de LEFTGO Y RIGHTGO y tengamos que avanzar
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
																	// Sensor distancia (radar) (AD0.2) y Micr�fono (AD0.5). 
																	// Conversi�n con flanco de subida en MAT1.1 (Pin P1.25)
{	
	LPC_SC->PCONP|= (1<<12);													// Power ON
	
	LPC_PINCON->PINSEL1|= (1<<14)|(1<<16)|(1<<18);  	// Pines: P0.23 (AD0.0), P0.24 (AD0.1), y P0.25 (AD0.2)
	LPC_PINCON->PINSEL3|= (0x3<<30);                  // Pines: P1.31 (AD0.5)
	
	LPC_PINCON->PINMODE1|= (0x2<<14)|(0x2<<16)|(0x2<<18); 	// Deshabilitaci�n pullup/pulldown
	LPC_PINCON->PINMODE3|= (0x2<<30);
	
	LPC_SC->PCLKSEL0 &=~ (0x3<<24); 									// CCLK/4 (Fpclk despu�s del reset) (100 MHz/4 = 25MHz)
	//Habilitaci�n
	if((ADC_modo==Manual_potenciometro)||(ADC_modo==Manual_microfono)||(ADC_modo==Manual_bateria)){
		
		
					LPC_ADC->ADCR = LPC_ADC->ADCR &~(0xFF<<0) 
													&~(1<<16) 						// Modo Manual
													&~(1<<27);						// Flanco de subida genera la conversi�n
													
					LPC_ADC->ADCR	|=(ADC_modo << 0)				// Selecci�n del canal AD0.x que funcionar� en modo Manual
													|(1<<8)		  	  			// CLKDIV=1 (Fclk_ADC = 25Mhz/(1+1)= 12.5MHz) (12.5MHz < 13 MHz)
													|(1<<21)			 				// PDN=1 (se activa su funcionamiento)
													|(7<<24);							// Inicio de conversi�n con MAT1.1 (Pin P1.25)
	}

	else{																				 	// MODO BURST para sensor y bater�a
		
					LPC_ADC->ADCR = LPC_ADC->ADCR &~(0xFF<<0) 
													&~(1<<21) 						// POWER DOWN MODE
													&~(0x07<<24);					// Bits START a cero
		
					LPC_ADC->ADCR |=(ADC_modo << 0)				// Selecci�n de los canales AD0.x que funcionar� en modo Burst
													|(1<<8)   	  				// CLKDIV=1 (Fclk_ADC = 25Mhz/(1+1)= 12.5MHz) (12.5MHz < 13 MHz)
													|(1<<16);							// Modo Burst

	}
	
	LPC_ADC->ADINTEN &=~ (0x1FF); 
	LPC_ADC->ADINTEN |= (ADC_modo<<0);		// Hab. interrupci�n tras conversi�n 
	
	NVIC_SetPriority(ADC_IRQn,2);	
	NVIC_EnableIRQ(ADC_IRQn);		         
}	

void config_TIMER1(uint8_t ADC_modo)	//Configuraci�n del TIMER1, que activara la conversi�n del ADC en su handler
{
	uint16_t F_TIMER1;
	
	/* ATENCI�N
	
		- Es muy probable que tengamos que cambiar MR1 en funci�n de si estamos en Manual (cualquiera de las 3 posibles ) o Burst
		- Tambi�n es problable que MR1 est� mal escrito : comprobarlo
	*/
	//
	
	if(ADC_modo==Manual_potenciometro || ADC_modo==Manual_bateria){
		
		F_TIMER1=1e3;			// Hay un match cada 1/1e3 = 1ms: Se toma valor del potenci�metro cada 2ms (en los flancos de subida de MAT1.1)
	}
	if(ADC_modo==Burst){
	
		F_TIMER1=2e4;			// Hay un match cada 1/2e4 = 50us: Se toma valor del potenci�metro cada 100us (en los flancos de subida de MAT1.1)
	}
	
	LPC_SC->PCONP|=(1<<2);							 // Power ON TIMER1
	LPC_PINCON->PINSEL3|= (0x3<<18);  	 // Pin P1.25 como MAT1.1
  LPC_TIM1->MCR |= (0x2<<3);   				 // Reset TC con match con  MR1 (NO ACTIVA SU HANDLER)
  LPC_TIM1->MR1 = (F_pclk_TIMER1/F_TIMER1)-1; // Valor de MR1; 
	LPC_TIM1->EMR |= (0x3<<6);   				 // Funcionamiento de toggle de la salida MAT1.1 con cada match
	
	LPC_TIM1->TCR |= (1<<1);    // Reseteo del contador  del TIMER1
	LPC_TIM1->TCR &=~ (1<<1);   
	LPC_TIM1->TCR |= (1<<0);    // Habilitaci�n de la cuenta
	
}	
void ADC_IRQHandler(void) //Cargamos el valor actualizado en las variables respectivas
{
	if(LPC_ADC->ADDR1 & (1<<31)){		// se ha producido una lectura de la tensi�n del potenci�metro (AD0.1)
		
		potenciometro=( (((LPC_ADC->ADDR1 >>4)&0xFFF)*(100))/(4096-1)  );
		
		// ATENCI�N: Valor en porcentaje (entre 0% y 100%) del valor del potenci�metro 
		
		if(potenciometro<=30){
			
			ciclo=30; // ��NO DEJAREMOS UN CICLO DE FUNCIONAMIENTO MENOR AL 30!!, puede hacer que no termine de arracar las ruedas
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
		
		/*
		modo = Burst;						// OBSERVACI�N: funcionamiento en modo Burst del ADC para obtener los valores actualizados de Bater�a 
														// y distancia (radar del coche)
		config_ADC(modo);
		config_TIMER1(modo);
		*/
		
		}
		
	if(LPC_ADC->ADDR0 & (1<<31)){		// se ha producido una lectura de la bater�a (AD0.0)
		
		valor_bateria= 3 * ( (((LPC_ADC->ADDR0 >>4)&0xFFF)*(3.3 - 0))/(4096-1)  );
		
		token_ruido= valor_bateria <= 6.3 ? 1 : 0 ; //  Si valor_bater�a es <= 6.3 , token_ruido vale 1 y altavoz sonar�
		
		if(token_ruido ==1){
			
				config_DAC();
				config_TIMER0();
		}
		
	}
	
	
	if(LPC_ADC->ADDR2 & (1<<31)){		// se ha producido una lectura del sensor de distancias (AD0.2)
		
		// movidas del sensor de distancias
	}
	
	if(LPC_ADC->ADDR5 & (1<<31)){		// se ha producido una lectura del micr�fono (AD0.5)
	
		// movidas del micr�fono
	}
	
	/*
		
	// LPC_ADC->ADCR&=~(1<<16); // BURST=0     // Deshabilitamos el modo R�faga (ojo continua la conversi�n del siguiente canal) 
  
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
void genera_muestras()	//se�al senoidal
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
	LPC_TIM1->PR = 0x00;     	 				//  Prescaler =1
  LPC_TIM0->MCR |= 0x03;						//  Reset TC on Match e interrumpe tras match con MR0  
  LPC_TIM0->MR0 = F_pclk_TIMER0/(F_SONIDO*N_muestras)-1;  // Cuentas hasta el Match con MR0
	LPC_TIM1->EMR = 0x00;   					//  No act�a sobre el HW
	
  LPC_TIM0->TCR |= (1<<1);    // Reseteo del contador  del TIMER0
	LPC_TIM0->TCR &=~ (1<<1);   
	LPC_TIM0->TCR |= (1<<0);    // Habilitaci�n de la cuenta
	
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
			
			LPC_PINCON->PINSEL1 &=~(3<<20); 	 	// Configuraci�n pin a GPIO (P0.26)
			LPC_GPIO0->FIODIR |= (1<<26); 			// GPIO como salida
			LPC_GPIO0->FIOCLR |= (1<<26); 			// P0.26 a 'L'
		}
	}

	LPC_SC->PLL0FEED  = 0xAA;  //Esto ayuda a depurar y no colapsar el sistema 
  LPC_SC->PLL0FEED  = 0x55;
}	

//
// SYSTICK -> para mostrar por pantalla las medidas de distancia recorrida y velocidad (funciones systick_config y systick_IRQHandler)
// -> dentro del handler se tendr� que acualizar en cada momento la variable "distancia_actual"
// -> �se podr�a hacer leyendo cada cierto tiempo el registro QEIPOS y aplicando una formula para pasarlo a distancia recorrida?
// -> preguntar sobre el tiempo de parada entre acciones

// IMPORTANTE TENER EN CUENTA que si accion = BACK, va a haber que estar restando (en vez de sumar) a "distancia_actual"

//En lo referente al QEI:
//-Recordar al final depurar la conversi�n de cm a pulsos, para ver si es precisa
//EINT 0 + TOKEN 0+TOKEN 1
