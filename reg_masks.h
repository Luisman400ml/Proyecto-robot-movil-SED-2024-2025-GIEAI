#ifndef REG_MASKS_H
#define REG_MASKS_H

// Interrupt Register (IR):

#define MR0_INT 0x01
#define MR1_INT 0x02
#define MR2_INT 0x04
#define MR3_INT 0x08

#define CR0_INT 0x10
#define CR1_INT 0x20

#define CAP0RE  0x01
#define CAP0FE  0x02
#define CAP0I   0x04

#define CAP1RE  0x08
#define CAP1FE  0x10
#define CAP1I   0x20

// Timer Control Register (TCR):
#define CNT_EN    0x01
#define CNT_RST   0x02
#define PWM_EN    0x08


// Match Control Register (MCR):

#define MR0I    0x0001
#define MR0R    0x0002
#define MR0S    0x0004

#define MR1I    0x0008
#define MR1R    0x0010
#define MR1S    0x0020

#define MR2I    0x0040
#define MR2R    0x0080
#define MR2S    0x0100

#define MR3I    0x0200
#define MR3R    0x0400
#define MR3S    0x0800

//--------------------------

// PWM Control Register (PCR)

#define PWMSEL2 0x0004 // 2   
#define PWMSEL3 0x0008 // 3   
#define PWMSEL4 0x0010 // 4   
#define PWMSEL5 0x0020 // 5   
#define PWMSEL6 0x0040 // 6   

#define PWMENA1 0x0200 // 9   
#define PWMENA2 0x0400 // 10  
#define PWMENA3 0x0800 // 11  
#define PWMENA4 0x1000 // 12  
#define PWMENA5 0x2000 // 13  
#define PWMENA6 0x4000 // 14  

// PWM Latch Enable Register (LER)

#define M0LEN 0x0001
#define M1LEN 0x0002
#define M2LEN 0x0004
#define M3LEN 0x0008
#define M4LEN 0x0010
#define M5LEN 0x0020
#define M6LEN 0x0040
#define M7LEN 0x0080

#define PWM_EN  0x08


// Power Control for Peripherals register (PCONP):

#define PCTIM0     0x00000002 //  1 
#define PCTIM1     0x00000004 //  2 
#define PCUART0    0x00000008 //  3 
#define PCUART1    0x00000010 //  4

#define PCPWM1     0x00000040 //  6 
#define PCI2C0     0x00000080 //  7 
#define PCSPI      0x00000100 //  8 
#define PCRTC      0x00000200 //  9 
#define PCSSP1     0x00000400 // 10
 
#define PCADC      0x00001000 // 12   // <= !!!!
#define PCCAN1     0x00002000 // 13 
#define PCCAN2     0x00004000 // 14 
#define PCGPIO     0x00008000 // 15 
#define PCRIT      0x00010000 // 16 
#define PCMCPWM    0x00020000 // 17 
#define PCQEI      0x00040000 // 18 
#define PCI2C1     0x00080000 // 19 

#define PCSSP0     0x00200000 // 21 
#define PCTIM2     0x00400000 // 22 
#define PCTIM3     0x00800000 // 23 
#define PCUART2    0x01000000 // 24 
#define PCUART3    0x02000000 // 25 
#define PCI2C2     0x04000000 // 26 
#define PCI2S      0x08000000 // 27 

#define PCGPDMA    0x20000000 // 29 
#define PCENET     0x40000000 // 30 
#define PCUSB      0x80000000 // 31 


// External Interrupt Mode register (EXTMODE):
#define EXTMODE0  0x0001  // 0
#define EXTMODE1  0x0002  // 1
#define EXTMODE2  0x0004  // 2
#define EXTMODE3  0x0008  // 3

// External Interrupt Polarity register (EXTPOLAR):
#define EXTPOLAR0 0x0001  // 0
#define EXTPOLAR1 0x0002  // 1
#define EXTPOLAR2 0x0004  // 2
#define EXTPOLAR4 0x0008  // 3

// A/D Control Register (AD0CR):
#define ADC_CLK_DIV(x)  (((x) & 0xFF) << 8)
#define ADC_BURST_MODE  (1U << 16)
#define ADC_NOT_PDN     (1U << 21)  // power_down

#define ADC_NO_START    (0x00 << 24)
#define ADC_SW_START    (0x01 << 24)
#define ADC_EDGE        (1U << 27)

#define ADC_START_MASK  (0x7U << 24)

// A/D Global Data Register (ADGDR):
#define ADC_GET_CHANNEL(x)  (((x) >> 24) & 0x7)
#define ADC_GET_RESULT(x)   (((x) >> 4) & 0xFFF)
#define ADC_OVR         (1U << 30)
#define ADC_DONE        (1U << 31)

// A/D Interrupt Enable register (ADINTEN):
#define ADINTEN0  0x0001U
#define ADINTEN1  0x0002U
#define ADINTEN2  0x0004U
#define ADINTEN3  0x0008U
#define ADINTEN4  0x0010U
#define ADINTEN5  0x0020U
#define ADINTEN6  0x0040U
#define ADINTEN7  0x0080U

#define ADGINTEN  0x0100U

// A/D Status register (ADSTAT):
#define ADC_DONE0   0x00001
#define ADC_DONE1   0x00002
#define ADC_DONE2   0x00004
#define ADC_DONE3   0x00008
#define ADC_DONE4   0x00010
#define ADC_DONE5   0x00020
#define ADC_DONE6   0x00040
#define ADC_DONE7   0x00080

#define ADC_OVR0    0x00100
#define ADC_OVR1    0x00200
#define ADC_OVR2    0x00400
#define ADC_OVR3    0x00800
#define ADC_OVR4    0x01000
#define ADC_OVR5    0x02000
#define ADC_OVR6    0x04000
#define ADC_OVR7    0x08000

#define ADC_ADINT   0x10000




#endif // REG_MASKS_H
