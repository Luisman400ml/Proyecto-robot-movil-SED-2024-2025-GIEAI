#include <LPC17xx.h>
#include "QEI.h" 
#include "reg_masks.h"

/*frecuencias de trabajo de la CPU + periféricos 
*/

#define F_cpu 100e6			    // Frecuencia de trabajo de la CPU (100MHz)
#define F_pclk_pwm F_cpu/4 	// Fuente de reloj del periférico PWM (25MHz) (al no tocar PCLKSEL, se divide entre 4 por defecto)
#define F_pclk_QEI F_cpu/4	// Fuente de reloj del periférico QEI (25MHz) (al no tocar PCLKSEL, se divide entre 4 por defecto)

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
#define dist_eje 19         // Distancia en cm desde una rueda hasta el eje central del coche (distacia rueda a rueda de 38cm: 19=38/2)

/* *****¡MUCHO CUIDADO CON LA CAÍDA DE LAS SUSPENSIONES!*****                      


En función del peso puede aumentar ligeramente la distancia entre ejes, por el apoyo de la rueda. 
Al ser un recorrido muy pequeño, afectara poco, pero !Ajuste fino de este valor con el montaje final!
*/                     
//

#define cm_cuenta ((2*PI*Rrueda)/(N*PPR*EDGES)) // Los centímetros que avanza la rueda por unidad de cuenta del encoder	(por pulso; anoto esto porque me aclaro mejor así)
#define arco_giro ((2*PI*dist_eje)/360) // Distancia a recorrer (en cm) por cada grado de giro
#define T_obs_QEITIME 100e-3					 // (100ms) Periodo de observación del contador dentro del QEI para medir velocidades (QEITIME) 

#define VELOCIDAD_RPM(x,y) ((x)*60*F_pclk_QEI/(PPR*EDGES*N*(y))) //Cálculo de la velocidad de la rueda en rpm (x=valor QEICAP, y=valor QEILOAD)
#define VELOCIDAD_CM_SEG(x) ((x)*2*PI*Rrueda/60) 	//Cálculo de la velocidad de la rueda en cm/seg (x=velocidad en rpm)


/*variables de control de motores 
*/

uint8_t accion;  // variable que guarda la acción a hacer del coche (STOP=0, GO=1, RIGHT=2, LEFT=3, BACK=4, RIGHTGO=5, LEFTGO=6)
uint8_t mot_1=0;	// variable que guarda la acción a hacer del coche para motor 1 (rueda izquierda)
uint8_t mot_2=0;	// variable que guarda la acción a hacer del coche para motor 2 (rueda derecha)
uint8_t ciclo;    
uint8_t ciclo_motor1;		// Guarda el ciclo de trabajo de la señal PWM que controla el motor 1 (rueda izquierda)
uint8_t ciclo_motor2;   // Guarda el ciclo de trabajo de la señal PWM que controla motor 2 (rueda derecha)
uint8_t acciones[4]={BACK,LEFT,BACK,STOP}; //Array que en un futuro contendra las consignas de acción a recorrer via UART

/*variables de control de QEI
*/

uint16_t speed_cm=0;
uint16_t dist_actual=0;
uint16_t dist_deseada=0;		

/* ATENCIÓN: dist_deseada puede guardar valores de cm o de grados:
	- si avanzamos o retrocedemos, son centimetros a recorrer
	- si giramos, son los grados a girar
*/
//

uint16_t distancias[4]={90,50,40,0}; //Array que en un futuro contendra las distancias (cm) y giros (º) a recorrer via UART
uint8_t QEI_token=0;
uint8_t QEI_index=0;

uint32_t pulsos_CMPOS0;

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

/*funciones
*/

int main(void)
{
	 	//Fijada para la demo un ciclo de trabajo inicial de 50%
							//A mayor ciclo, mas rapido va. He visto que tal cual esta, por debajo de un 25% de th deja de moverse
	config_gpios();
	config_ciclo();
	config_pwm();
	config_botones();
	config_QEI();

  while(1){
		
	}
}

//
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
							ciclo_motor1=ciclo;
							ciclo_motor2= COMPLEMENTADO(ciclo);
							break;
      case LEFT:
			        ciclo_motor1= COMPLEMENTADO(ciclo);
							ciclo_motor2=ciclo;
							break;
      case BACK:
							ciclo_motor1=ciclo;
							ciclo_motor2=ciclo;
							break;
			case RIGHTGO:
							ciclo_motor1=ciclo;
							ciclo_motor2= COMPLEMENTADO(ciclo);
							break;
      case LEFTGO:
			        ciclo_motor1= COMPLEMENTADO(ciclo);
							ciclo_motor2=ciclo;
							break;
			default:
							break;
   }
}

void config_gpios(void) // Configuración pines de sentido para motor 1 y motor 2 (GPIO).
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
 
void config_pwm(void) 	//Configuración del PWM para motor 1 y motor 2
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
	accion=acciones[0];     // Cargamos los valores acciones y distancias de los arrays en las variables a trabajar. 
													// Como con el botón arrancamos, siempre empezaremos con la primer accion del array
	dist_deseada=distancias[0];
	QEI_pulses();
	QEI_index=1;   //Esta será la variable que recorrerá todo el array. Al inicializarla como cero podremos de esta forma
								 //repetir el mismo camino desde el principio pulsando de nuevo el botón KEY1
  ciclo=30;      //Probamos a un ciclo de 30
	config_gpios();
	config_ciclo();
	set_duty_RIGHT_pwm(ciclo_motor1);
	set_duty_LEFT_pwm(ciclo_motor2);
	}

void EINT2_IRQHandler(void)	//Botón KEY2 
{
	LPC_SC->EXTINT = (1 << 2); //Borramos flag de interrupción
	
 /* Dejamos esto preparado. Seguro que para el proyecto final, le encontramos un buen uso.
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
	LPC_SC->PCLKSEL1 &=~ (0x2<<0);		// PCLK = CCLK/4 (25MHz)
	LPC_PINCON->PINSEL3	|= (1<<8);		// MCI0 en P1.20 (leerá "Hall signal A")
	LPC_PINCON->PINSEL3	|= (1<<14);		// MCI1 en P1.23 (leerá "Hall signal B")
	
	LPC_QEI->QEICONF	= QEICONF_CAPMODE; 						// Resolución x4 (edges=4)
	LPC_QEI->QEICON 	= QEICON_RESP;		 						// Reseteo inicial de QEIPOS a cero
	LPC_QEI->QEILOAD 	= F_pclk_QEI * T_obs_QEITIME;	// valor de recarga del contador de velocidad QEITIME
	LPC_QEI->QEIMAXPOS = 0xFFFFFFFF;   // Nos da igual, así que ponemos el valor máximo
	
  LPC_QEI->QEIIEC &=~ 0xFFFFFFFF;                	 //Limpieza de QEIIE
  LPC_QEI->QEIIES = (QEI_TIM_INT | QEI_POS0_INT); 
	
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
	
		- Es MUY IMPORTANTE aquí tener en cuenta que el QEI cuenta los pulsos del encoder del motor de la DERECHA 
			(¡Te prometo que lo cambiare, de momento en la izq.!)
	  */
		//
	
	if (QEI_token==0){ //Token usado para GO, BACK, LEFT, RIGHT, STOP y los giros de 90º en LEFTGO Y RIGHTGO
		
		switch(accion){		//Por cada accion siempre debemos: convertir distancia, cargarla en CMPOS0 y resetear contador QEIPOS

			case GO:           
				LPC_QEI->QEICON = QEICON_RESP;		 // Reseteo de QEIPOS a cero
				pulsos_CMPOS0 = (uint32_t) dist_deseada/cm_cuenta;
				LPC_QEI->CMPOS0 = pulsos_CMPOS0;
				break;
	
			case RIGHT: // ATENCIÓN: El motor de la derecha RETROCEDE para girar a la derecha
				LPC_QEI->QEICON = QEICON_RESP;		 // Reseteo de QEIPOS a cero
				pulsos_CMPOS0 = (uint32_t) (arco_giro*dist_deseada)/cm_cuenta;
				LPC_QEI->CMPOS0 =  pulsos_CMPOS0;
				break;
			
			case LEFT:	// ATENCIÓN: El motor de la derecha AVANZA para girar a la izquierda
				LPC_QEI->QEICON = QEICON_RESP;		 // Reseteo de QEIPOS a cero
				pulsos_CMPOS0 = (uint32_t) (arco_giro*dist_deseada)/cm_cuenta;
				LPC_QEI->CMPOS0 = - pulsos_CMPOS0; // ATENCIÓN: CUENTA DESCENTENDE al ir el motor hacia atrás
				break;
			
			case BACK:
				LPC_QEI->QEICON = QEICON_RESP;	 // Reseteo de QEIPOS a cero
				pulsos_CMPOS0 = (uint32_t) dist_deseada/cm_cuenta;
				LPC_QEI->CMPOS0 = - pulsos_CMPOS0; // ATENCIÓN: CUENTA DESCENTENDE al ir el motor hacia atrás
				break;
	
			case RIGHTGO: // ATENCIÓN: El motor de la derecha RETROCEDE para girar a la derecha
				LPC_QEI->QEICON = QEICON_RESP;		 // Reseteo de QEIPOS a cero
				pulsos_CMPOS0 = (uint32_t) (arco_giro*ANGULO_RECTO)/cm_cuenta;  //En esta acción se define el giro de 90 grados con una macro, pues es condición de giro
				LPC_QEI->CMPOS0 = pulsos_CMPOS0;
				
				QEI_token=1;  //Token para que la próxima acción sea el avance con los cm deseados
				break;
			
			case LEFTGO:	// ATENCIÓN: El motor de la derecha AVANZA para girar a la izquierda
				LPC_QEI->QEICON = QEICON_RESP;		 // Reseteo de QEIPOS a cero
				pulsos_CMPOS0 = (uint32_t) (arco_giro*ANGULO_RECTO)/cm_cuenta; //En esta acción se define el giro de 90 grados con una macro, pues es condición de giro
				LPC_QEI->CMPOS0 = - pulsos_CMPOS0;	 // ATENCIÓN: CUENTA DESCENTENDE al ir el motor hacia atrás
				
				QEI_token=1;  //Token para que la próxima acción sea el avance con los cm deseados
				break;
			
			case STOP:  //Si paramos, es porque hemos llegado al último valor del array o valor NULL, es decir, accion=STOP
		  ciclo=0;
		  config_ciclo();  //PWM A 0
		  LPC_QEI->CMPOS0 = 0xF0F0F0F0; //Valor aleatorio por si acso, para que ya no salte el CMPOS0 por error, tras la parada
		  LPC_GPIO0->FIOPIN &= ~(1<<0); //P0.0 a 'L' -> Ver nota de abajo
		  LPC_GPIO0->FIOPIN &= ~(1<<1); //P0.1 a 'L' -> Ver nota de abajo
		  /*Imperativo hacerlo. Caso extremo: Si GPIO a 'H' con PWM a 0%, el driver no discrimina que pin esta conetado a GPIO y cual PWM. De forma que en vez de parar, se 
		   puede quedar girando a máxima velocidad la rueda*/
		
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
		uint16_t speed_rpm = 0;						// Variable limitada a la función (se borran cada vez que salgamos del handler)
		speed_rpm = (uint16_t) VELOCIDAD_RPM(LPC_QEI->QEICAP , LPC_QEI->QEILOAD); //velocidad en rpm (sin decimales)
		speed_cm = (uint16_t) VELOCIDAD_CM_SEG(speed_rpm); // velocidad en cm/s  (sin decimales)         		
	}     
	
	if (LPC_QEI->QEIINTSTAT & QEI_POS0_INT)		// Verifica si ha saltado la IRQ porque se ha llegado a la distancia recorrida deseada
	{  
		LPC_QEI->QEICLR |= QEI_POS0_INT;					// Borramos flag
		
		if(QEI_token==0)	//En caso de que la acción previa haya sido GO, BACK, LEFT, RIGHT, STOP o la finalización de RIGHTGO o LEFTGO
		{ 
			accion=acciones[QEI_index]; //Actualizamos próxima accion...
			dist_deseada=distancias[QEI_index]; // y distancia/grados
			QEI_index++; //Actualizamos el indice para la próxima acción 
			QEI_pulses();
			
			config_gpios();	//Funciones para poner a "correr" el coche con la acción deseada
			config_ciclo(); 
			set_duty_RIGHT_pwm(ciclo_motor1);
			set_duty_LEFT_pwm(ciclo_motor2);
			QEI_pulses();	//función para actualizar cmpos0 y resetear contador 
		}
		
		else // En caso de que previamente se haya acabado el giro de 90º de LEFTGO Y RIGHTGO y tengamos que avanzar
		{
			QEI_pulses(); // IMPORTANTE que esta sea la primera
			
			config_gpios();
			config_ciclo();
			set_duty_RIGHT_pwm(ciclo_motor1);
			set_duty_LEFT_pwm(ciclo_motor2);

		}
	}
}

//
// SYSTICK -> para mostrar por pantalla las medidas de distancia recorrida y velocidad (funciones systick_config y systick_IRQHandler)
// -> dentro del handler se tendrá que acualizar en cada momento la variable "distancia_actual"
// -> ¿se podría hacer leyendo cada cierto tiempo el registro QEIPOS y aplicando una formula para pasarlo a distancia recorrida?
// -> preguntar sobre el tiempo de parada entre acciones

// IMPORTANTE TENER EN CUENTA que si accion = BACK, va a haber que estar restando (en vez de sumar) a "distancia_actual"

//En lo referente al QEI:
//-Recordar al final depurar la conversión de cm a pulsos, para ver si es precisa
//EINT 0 + TOKEN 0+TOKEN 1
