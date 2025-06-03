#include <LPC17xx.h>

//frecuencias de trabajo de la CPU + periféricos
#define F_cpu 100e6			    // Frecuencia de trabajo de la CPU (100MHz) (por defecto tenemos la fuente de reloj interna IRC)
#define F_pclk_pwm F_cpu/4 	// Fuente de reloj del periférico PWM (25MHz) (al no tocar PCLKSEL, se divide entre 4 por defecto)

//macros movimiento
#define STOP 0
#define GO 1
#define RIGHT 2
#define LEFT 3
#define BACK 4 

//macro señal tiempo en alto complementado (necesario para los movimientos en avance)
#define COMPLEMENTADO(x) (100-(x))

//macros PWM
#define F_pwm 500			      // Frecuencia del timer counter del PWM (MR0): 500Hz

/* *****Posible uso de struct. No es necesario, pero puede quedar algo más pro*****
struct motor {
		uint8_t accion;
		uint8_t mot_1;
		uint8_t mot_2;
		uint8_t ciclo;
		uint8_t ciclo_motor1;
		uint8_t ciclo_motor2;
};

struct motor motor
*/


//variables de control de motores
uint8_t accion;
uint8_t mot_1=0;
uint8_t mot_2=0;
uint8_t ciclo;
uint8_t ciclo_motor1;
uint8_t ciclo_motor2;

void config_ciclo(void);
void config_gpios(void);
void config_pwm(void);
void set_duty_RIGHT_pwm(uint8_t cycle);
void set_duty_LEFT_pwm(uint8_t cycle);
void config_botones(void);
void EINT1_IRQHandler(void);
void EINT2_IRQHandler(void);

int main(void)
{
	ciclo=25; 	//Fijada para la demo un ciclo inicial de 25%
	accion=0;	  //Fijada para la demo la accion 0
	
	config_ciclo();
	config_gpios();
  config_pwm();
  set_duty_RIGHT_pwm(ciclo_motor2);
	set_duty_LEFT_pwm(ciclo_motor1);
  config_botones();	// a mayor ciclo, mas rapido va. He visto que tal cual esta, por debajo de un 25% de th deja de moverse
	
  while(1);
}
//

//EN LA ENTREGA DE ESTE HITO SE HA DOTADO DE UNA DEMO PARA VISUALIZAR SI LA CONFIGURACIÓN DE PINES Y CICLO DE PWM ES CORRECTA. 

//DICHA DEMO PERMITE AUMENTAR EL CICLO DE TRABAJO(LA VELOCIDAD DE LOS MOTORES) PULSADO KEY1 HASTA EL 100%. DESPUÉS REGRESA AL 10%
//MEDIANTE KEY 2 SE INTERCAMBIA EL MOVIMIENTO A REALIZAR, SEGÚN ESTA COMENTADA LA SECUENCIA EN EL HANDLER DEL BOTÓN

//
/* **** Configuración de motores****
		- motor 1 (rueda izquierda)
		- motor 2 (rueda derecha)

		NOTA: 
		- Trabajaremos en modo driver (2º manera mostrada en la descripción del proyecto): GPIO+PWM y enable siempre fijo
*/


void config_ciclo(void) // configura el ciclo de trabajo de las señales PWM (ciclo_motor1 y ciclo_motor2) en función del movimiento
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
							ciclo_motor2=ciclo;
							break;
      case LEFT:
							ciclo_motor1=ciclo;
							ciclo_motor2= COMPLEMENTADO(ciclo);
							break;
      case BACK:
							ciclo_motor1=ciclo;
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
	 
	 LPC_GPIO0->FIODIR |=(1<<0); 		//P0.0 Salida
	 LPC_PINCON->PINSEL0 |= (0<<0); //P0.0 como GPIO
	 LPC_GPIO0->FIODIR |=(1<<1);    //P0.1 Salida
	 LPC_PINCON->PINSEL0 |= (0<<2); //P0.1 como GPIO
	 
	 /*	 
	 Tal y como esta diseñado el programa, con la variable "accion" podriamos hacer funcionar los motores. 
	 Se ha previsto una variable por motor adicional, por si sirve para hacer arcos de giro; en caso de que vayamos holgados de tiempo.
	 */
	 
	 mot_1=accion;
	 mot_2=mot_1;
	 
	 //---------- Definir como gira motor 1 (rueda izquierda)
	 
	 if(mot_1==GO|mot_1==RIGHT){    // Para ir recto o girar a derechas
			LPC_GPIO0->FIOPIN |= (1<<0); //P0.0 a 'H'
	 }
	 
	 if(mot_1==LEFT|mot_1==BACK){  // Para ir atras o girar a izquierdas
			LPC_GPIO0->FIOPIN &= ~(1<<0); //P0.0 a 'L'
	 }
	 
	 //---------- Definir como gira motor 2 (rueda derecha)
	 
	 if(mot_2==GO|mot_2==LEFT){ 		// Para ir recto o girar a izquierdas
			LPC_GPIO0->FIOPIN |= (1<<1); //P0.1 a 'H'
	 }
	 
	 if(mot_2==RIGHT|mot_2==BACK){  // Para ir atras o girar a izquierdas
			LPC_GPIO0->FIOPIN &= ~(1<<1); //P0.1 a 'L'
	 }
 }
 
void config_pwm(void) 	//configuración del PWM para motor 1 y motor 2
{
  LPC_SC->PCONP |= (1<<6); 						// Power on
	LPC_SC->PCLKSEL0 &=~ (0x3<<12);     // PWM clk = CCLK/4 (25MHz)
	LPC_PINCON->PINSEL3 |= (1<<11);   	// Configurar pin P1.21 (PWM1.3 output)(MR3) -> para Motor 1 (Rueda izquierda)(IN2)(De momento misma velocidad de giro)
  LPC_PINCON->PINSEL3 |= (1<<5);   		// Configurar pin P1.18 (PWM1.1 output)(MR1) -> para Motor 2 (Rueda derecha)(IN4)(De momento misma velocidad de giro)
	
	LPC_PWM1->MR0 = (F_pclk_pwm/F_pwm) - 1;	// Fijar frecuencia del PWM1 (se hace con el valor de MR0)  
	LPC_PWM1->LER |= (1<<0);
  LPC_PWM1->PCR |= (1<<9)|(1<<11); 		// Todos a modo Single Edge. Habilitación de salidas PWM1.1 (match con MR1) y PWM1.3 (match con MR3)
  LPC_PWM1->MCR |= (1<<1); 						// Reseteo del timer del PWM cuando hay match con MRO
	
  LPC_PWM1->TCR |= (1<<0) | (1<<3); 	// Arranque del PWM
}

void set_duty_LEFT_pwm(uint8_t cycle) // ciclo de trabajo (velocidad) motor izquierdo. cycle = ciclo_motor1
{
 
	/*
	recordar que por culpa de la complementación de la PWM hecha por el driver:
	-> ciclo_motor1 NO ES IGUAL A ciclo_motor2
	*/
	
	LPC_PWM1->MR3 = (LPC_PWM1->MR0 * cycle) / 100; //ciclo de trabajo
	LPC_PWM1->LER |= (1<<3);											 //LER para confirmar cambios
}

void set_duty_RIGHT_pwm(uint8_t cycle) // ciclo de trabajo (velocidad) motor derecho. cycle = ciclo_motor2
{
 
	/*
	recordar que por culpa de la complementación de la PWM hecha por el driver:
	-> ciclo_motor1 NO ES IGUAL A ciclo_motor2
	*/
	
	LPC_PWM1->MR1 = (LPC_PWM1->MR0 * cycle) / 100; //ciclo de trabajo
	LPC_PWM1->LER |= (1<<1);											 //LER para confirmar cambios
}

void config_botones(void)  //Preparamos los botones; en este caso se usaran para la demo
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

void EINT1_IRQHandler(void)	//Botón KEY1 -> Cambio de velocidad
{ 
	LPC_SC->EXTINT = (1 << 1); //Borramos flag de interrupción
	
	ciclo = (ciclo<100)? (ciclo+25) : 25; //Subimos ciclo de trabajo en pasos de 25%
	
	config_ciclo(); //Ajustamos ciclos de cada motor
	set_duty_RIGHT_pwm(ciclo_motor2); //metemos los ciclos en la PWM de cada motor
	set_duty_LEFT_pwm(ciclo_motor1);

}

void EINT2_IRQHandler(void)	//Botón KEY2 -> Cambio de modo, secuencia: STOP -> GO -> RIGHT -> LEFT -> BACK -> STOP...
{
	LPC_SC->EXTINT = (1 << 2); //Borramos flag de interrupción
	
	accion= (accion>=4)? 0 : (accion+1); //Subimos ciclo de trabajo en pasos de 25%
	
	
	config_ciclo(); //Si cambia la acción, puede cambiar el ciclo del motor, por culpa de la complementación de la PWM en el driver
	config_gpios(); //Actualizamos pines de control de motores
  set_duty_RIGHT_pwm(ciclo_motor2); //Cargamos nuevos ciclos
	set_duty_LEFT_pwm(ciclo_motor1);
}
