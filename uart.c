/* uart.c
 * contiene las funciones:
 1  UART3_IRQHandler(void)
 2  tx_cadena_UART3(char *ptr)
 3  uart3_set_baudrate(unsigned int baudrate) 
 4  uart3_init(int baudrate) 
 */
 
#include <LPC17xx.h>
#include "uart.h"

char buffer[30];
char *ptr_rx;						// Puntero de RECEPCIÓN
char rx_completa;				// Flag de recepción de cadena completa que se activa al recibir CR (\r)(0x0D)
static char *ptr_tx;		// Puntero de TRANSMISIÓN
char tx_completa;				// Flag de transmisión de cadena completa

/* Funciones
*/

//
//	Modificaciones de la UART3 para que funcione el modulo BTH
// 

void tx_cadena_UART3(char *cadena) // Función para enviar una cadena de texto.
																	 // Se le pasa como argumento directamente el texto, guardado en "cadena"
{
	ptr_tx=cadena;						 // El puntero "prt_tx" apunta a la dirección inicial de "cadena"
	tx_completa=0;
	LPC_UART3->THR=*ptr_tx++;	 // Comienza transfiriéndose el primer caracter de "cadena", y después incrementa en 1 unidad
														 // la posición de memoria apuntada por el puntero "ptr_tx" dentro de "cadena"
}							 						 	 // IMPORTANTE: activar flag interrupción por registro transmisor vacio


void UART3_IRQHandler(void) {
	
  switch(LPC_UART3->IIR & 0x0E) {		// Permite identificar a qué se ha debido esa interrupción
	
		case 0x04:	// RBR, Receiver Buffer Ready --> existencia de un nuevo dato en RBR pendiente de leer
	
			*ptr_rx = LPC_UART3->RBR; //  Lee el dato recibido y lo almacena en el puntero
	
	    if (*ptr_rx != 13) {  // Comprueba si el último caracter recibido es \r --> Cadena completa.
														 
					ptr_rx++;	// El puntero apunta a una unidad por encima dentro de "buffer"
	    }
			else{
					*ptr_rx=0;				// Añadimos el caracter null (\0) para tratar los datos recibidos como una cadena
					rx_completa = 1;  // rx_completa=1 permite comenzar con el código dentro del while(1)
					ptr_rx=buffer;	  // Puntero al inicio del buffer para nueva recepción
			}	
		break;
    
		case 0x02:	// THRE, Transmit Holding Register empty ---> el registro THR se encuentra vacío
		 
			if (*ptr_tx!=0){
				LPC_UART3->THR=*ptr_tx++;	// El dato apuntado por del puntero para ser mandado por THR. 
																	// Después incrementa en 1 unidad la posición de memoria apuntada por el puntero "ptr_tx" dentro de "cadena"
			}
			else{ 
				tx_completa=1;
			}
		break;
 
  }
}


static int uart3_set_baudrate(unsigned int baudrate) {
    int errorStatus = -1; //< Failure

    // UART clock (FCCO / PCLK_UART0)
   // unsigned int uClk = SystemCoreClock / 4;
    unsigned int uClk =SystemCoreClock/4;
    unsigned int calcBaudrate = 0;
    unsigned int temp = 0;

    unsigned int mulFracDiv, dividerAddFracDiv;
    unsigned int divider = 0;
    unsigned int mulFracDivOptimal = 1;
    unsigned int dividerAddOptimal = 0;
    unsigned int dividerOptimal = 0;

    unsigned int relativeError = 0;
    unsigned int relativeOptimalError = 100000;

    uClk = uClk >> 4; /* div by 16 */

    /*
     *  The formula is :
     * BaudRate= uClk * (mulFracDiv/(mulFracDiv+dividerAddFracDiv) / (16 * DLL)
     *
     * The value of mulFracDiv and dividerAddFracDiv should comply to the following expressions:
     * 0 < mulFracDiv <= 15, 0 <= dividerAddFracDiv <= 15
     */
    for (mulFracDiv = 1; mulFracDiv <= 15; mulFracDiv++) {
        for (dividerAddFracDiv = 0; dividerAddFracDiv <= 15; dividerAddFracDiv++) {
            temp = (mulFracDiv * uClk) / (mulFracDiv + dividerAddFracDiv);

            divider = temp / baudrate;
            if ((temp % baudrate) > (baudrate / 2))
                divider++;

            if (divider > 2 && divider < 65536) {
                calcBaudrate = temp / divider;

                if (calcBaudrate <= baudrate) {
                    relativeError = baudrate - calcBaudrate;
                } else {
                    relativeError = calcBaudrate - baudrate;
                }

                if (relativeError < relativeOptimalError) {
                    mulFracDivOptimal = mulFracDiv;
                    dividerAddOptimal = dividerAddFracDiv;
                    dividerOptimal = divider;
                    relativeOptimalError = relativeError;
                    if (relativeError == 0)
                        break;
                }
            }
        }

        if (relativeError == 0)
            break;
    }

    if (relativeOptimalError < ((baudrate * UART_ACCEPTED_BAUDRATE_ERROR) / 100)) {

        LPC_UART3->LCR |= DLAB_ENABLE; 	// importante poner a 1
        LPC_UART3->DLM = (unsigned char) ((dividerOptimal >> 8) & 0xFF);
        LPC_UART3->DLL = (unsigned char) dividerOptimal;
        LPC_UART3->LCR &= ~DLAB_ENABLE;	// importante poner a 0

        LPC_UART3->FDR = ((mulFracDivOptimal << 4) & 0xF0) | (dividerAddOptimal & 0x0F);

        errorStatus = 0; //< Success
    }

    return errorStatus;
}
 					   					  
void uart3_init(int baudrate) {
	
    LPC_SC ->PCONP |= (1<<25);
    LPC_PINCON->PINSEL9 |=  ((3 << 24) | (3 << 26));// Change P4.29 and P4.28 mode to TXD3 and RXD3
    LPC_UART3->LCR &= ~STOP_1_BIT & ~PARITY_NONE; // Set 8N1 mode (8 bits/dato, sin pariad, y 1 bit de stop)
    LPC_UART3->LCR |= CHAR_8_BIT;

    uart3_set_baudrate(baudrate);// Set the baudrate
    
		ptr_rx = buffer;		// Inicializamos el puntero de recepción al comienzo de buffer
		tx_completa = 0;
		rx_completa = 0;
     
    LPC_UART3->IER = THRE_IRQ_ENABLE|RBR_IRQ_ENABLE;// Enable UART TX and RX interrupt (for LPC17xx UART)   
    NVIC_EnableIRQ(UART3_IRQn);// Enable the UART interrupt (for Cortex-CM3 NVIC)

}

