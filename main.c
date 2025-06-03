#include <LPC17xx.h>
#define F_cpu 100e6			// Defecto Keil (xtal=12Mhz)
#define F_pclk F_cpu/4 	// Defecto despues del reset
#define F_pwm 500			// 500Hz


//macros modos
#define GO 1
#define STOP 0
#define RIGHT 2
#define LEFT 3
#define BACK 4 //¿Hace falta que vaya hacia atras?
////////////////Con esto ya controlamos 1 motor

//variables de control de motores
uint8_t accion;
uint8_t mot_1=0;
uint8_t mot_2=0;
uint8_t ciclo=90;
uint8_t ciclo_A;
uint8_t ciclo_B;



/////Configuración de pines como GPIOS (Estan preparados dos pines gpios/motor, por si usamos el otro modo. Pero de momento modo driver: gpio+pwm y enable siempre fijo)
 void config_gpios(void){
	 //----------configurar pines control motor 1. NOTA: motor 1, motor con pegatina. Con clema a la izq-> OUT1 ARRIBA; OUT2 ABAJO
	 LPC_GPIO1->FIODIR |=(1<<0); //P1.0 Salida
	 //LPC_GPIO1->FIODIR |=(1<<1); //P1.1 ""
	 LPC_PINCON->PINSEL2 |= (0<<0);//p1.0  COMO GPIO
	 //LPC_PINCON->PINSEL2 |= (0<<2);//p1.1   ""
	 
	 //****Definir como gira motor 1
	 if(mot_1==GO|mot_1==RIGHT){
		 //Supongo motor uno como motor izq.: Para girar derecha o ir recto
	 LPC_GPIO1->FIOPIN |= (1<<0); //P1.0 a 'H'
	 //LPC_GPIO1->FIOPIN &= ~(1<<1); //P1.1 a 'L'
	 }
	 
	 if(mot_1==LEFT|mot_1==BACK){
		 //Supongo motor uno como motor izq.: Para girar izquierda o ir atras
	 LPC_GPIO1->FIOPIN |= (1<<1); //P1.1 a 'H'
	 //LPC_GPIO1->FIOPIN &= ~(1<<0); //P1.0 a 'L'
	 }
	 
	 //-------------------configurar pines motor 2. NOTA: motor 2, motor SIN pegatina. Con clema a la dech-> OUT4 ABAJO; OUT3 ARRIBA
	 LPC_GPIO1->FIODIR |=(1<<4); //P1.4 Salida
	 //LPC_GPIO1->FIODIR |=(1<<8); //P1.8 ""
	 LPC_PINCON->PINSEL2 |= (0<<8);//p1.4  COMO GPIO
	 //LPC_PINCON->PINSEL2 |= (0<<16);//p1.8   ""
	 //*****Definir como gira motor 2
	 if(mot_2==GO|mot_2==LEFT){
		 //Supongo motor uno como motor izq.: Para girar derecha o ir recto
	 LPC_GPIO1->FIOPIN |= (1<<4); //P1.4 a 'H'
	 //LPC_GPIO1->FIOPIN &= ~(1<<8); //P1.8 a 'L'
	 }
	 
	 if(mot_2==RIGHT|mot_2==BACK){
		 //Supongo motor uno como motor izq.: Para girar izquierda o ir atras
	 //LPC_GPIO1->FIOPIN |= (1<<8); //P1.8 a 'H'
	 LPC_GPIO1->FIOPIN &= ~(1<<4); //P1.4 a 'L'
	 }
	 
	 //Si los pines estan bien puestos, tomando motor 1, como motor izq y 2 como dcho, ya estan bien definidos los giros. FALTA: complementar la pwm
 }
 
void config_pwm(void)
	//Este codigo es el que hay subido en BB para configurar el PWM, solo he cambiado el pin (no se porque lo cambie; pero da igual)
{
	//Configuración PWM motor 1 y 2
  LPC_SC->PCONP |= (1<<6); 				// power on
  LPC_PINCON->PINSEL3 |= (1<<5);   // 1.18 PWM1.1 output ///para Motor 1 (De momento da igual, misma velocidad de giro)
	LPC_PINCON->PINSEL3 |= (1<<21);   // 1.26 PWM1.6 output ///para Motor 1 (De momento da igual, misma velocidad de giro)
  LPC_PWM1->MR0 = (F_pclk/F_pwm) - 1;	// set frequency of PWM1           
  LPC_PWM1->PCR |= (1<<9)|(1<<14); 		// PWMENA1=1 y 6
  LPC_PWM1->MCR |= (1<<1); 						// reset timer on Match0
  LPC_PWM1->TCR |= (1<<0) | (1<<3); 	// start timer, timer enable
	
	
}
void set_duty_pwm(uint8_t cycle)
{
	//Mover motor 1
	LPC_PWM1->MR1 = (LPC_PWM1->MR0 * cycle) / 100; //duty cycle
	//mover motor 2
	LPC_PWM1->MR6 = (LPC_PWM1->MR0 * cycle) / 100; //duty cycle
	//LER para confirmar
	LPC_PWM1->LER |= (1<<6); //PWMLER[0-1]=1 (LE de Match0 y Match1)
	LPC_PWM1->LER |= (1<<1);
	LPC_PWM1->LER |= (1<<0);
}
void config_botones(void)
{
	
	LPC_PINCON->PINSEL4 |= (0x15 << 20);	//P2.12:10 como EINT2:0 
	LPC_SC->EXTMODE |= 7;					// Activas por flanco
	LPC_SC->EXTPOLAR &= ~0x7;				//de bajada
	NVIC_EnableIRQ(EINT0_IRQn);
	NVIC_EnableIRQ(EINT1_IRQn);
	NVIC_EnableIRQ(EINT2_IRQn);
}	

int main(void)
{
	
	//para probar:
	accion=RIGHT;
	//if(accion==BACK)
	ciclo=50;
	mot_1=accion;
	mot_2=mot_1;
	//
	 config_gpios();
   config_pwm();
   set_duty_pwm(ciclo); 
   config_botones();	// a mayor ciclo, mas rapido va. He visto que tal cual esta, por debajo de un 25% de th deja de moverse
   while(1);
}
