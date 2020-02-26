/*
 * gpio.c
 *
 *  Created on: 19 Feb 2020
 *      Author: Eslam
 */

#include "gpio.h"
#include "regmap.h"
#include "BIT_MODE.h"
#include "CLOCK_int.h"


    void SetMask (volatile u32* reg , u8 pins)
    {
        u32 data = *reg;
        data &= ~(pins);
        data |= (0xff & pins);
        *reg = data;
    }
    void ClrMask (volatile u32* reg , u8 pins)
    {
        u32 data = *reg;
        data &= ~(pins);
        data |= (0x00 & pins);
        *reg = data;
    }
    void ConfigureBus()
    {
         #if (PORTF_BUS == AHB)
                 SETBIT(GPIOHBCTL , 0);
         #elif (PORTF_BUS == APB)
                 CLRBIT(GPIOHBCTL , 0);
         #endif

         #if (PORTB_BUS == AHB)
                 SETBIT(GPIOHBCTL , 1);
         #elif (PORTB_BUS == APB)
                 CLRBIT(GPIOHBCTL , 1);
         #endif

         #if (PORTC_BUS == AHB)
                 SETBIT(GPIOHBCTL , 2);
         #elif (PORTC_BUS == APB)
                 CLRBIT(GPIOHBCTL , 2);
         #endif

         #if (PORTD_BUS == AHB)
                 SETBIT(GPIOHBCTL , 3);
         #elif (PORTD_BUS == APB)
                 CLRBIT(GPIOHBCTL , 3);
         #endif

         #if (PORTE_BUS == AHB)
                 SETBIT(GPIOHBCTL , 4);
         #elif (PORTE_BUS == APB)
                 CLRBIT(GPIOHBCTL , 4);
         #endif

         #if  (PORTF_BUS == AHB)
                 SETBIT(GPIOHBCTL , 5);
         #elif (PORTF_BUS == APB)
                 CLRBIT(GPIOHBCTL , 5);
         #endif
    }

void GPIOClockSet(gpio_port_t port)
{
    /* SYS CLOCK */
//    SETBIT(RCC , 11);           /*PLL BYPASS*/
//    SETBIT(RCC , 13);           /*PLL DIASBLE*/
//    CLRBIT(RCC , 22);           /*SYSDIV*/

   CLOCK_voidInit();

    /* Enable clock to ports */
    if (port == PORTA)
        SETBIT (RCGCGPIO , 0);
    else if (port == PORTB)
        SETBIT (RCGCGPIO , 1);
    else if (port == PORTC)
        SETBIT (RCGCGPIO , 2);
    else if (port == PORTD)
        SETBIT (RCGCGPIO , 3);
    else if (port == PORTE)
        SETBIT (RCGCGPIO , 4);
    else if (port == PORTF)
        SETBIT (RCGCGPIO , 5);
}
void GPIOClockRst(gpio_port_t port)
{
    if (port == PORTA)
        CLRBIT (RCGCGPIO , 5);
    else if (port == PORTB)
        CLRBIT (RCGCGPIO , 4);
    else if (port == PORTC)
        CLRBIT (RCGCGPIO , 3);
    else if (port == PORTD)
        CLRBIT (RCGCGPIO , 2);
    else if (port == PORTE)
        CLRBIT (RCGCGPIO , 1);
    else if (port == PORTF)
        CLRBIT (RCGCGPIO , 0);
}
char GPIOClockGet(gpio_port_t port)
{
        if (port == PORTA)
          return GETBIT (RCGCGPIO , 5);
       else if (port == PORTB)
          return GETBIT (RCGCGPIO , 4);
       else if (port == PORTC)
          return GETBIT (RCGCGPIO , 3);
       else if (port == PORTD)
          return GETBIT (RCGCGPIO , 2);
       else if (port == PORTE)
          return GETBIT (RCGCGPIO , 1);
       else if (port == PORTF)
          return GETBIT (RCGCGPIO , 0);
}

void GPIODirModeSet(gpio_port_t port, u8 pins, gpio_mode_t mode)     //GPIODirModeSet(PORTA,0b01100100,MODE_OUT);
{
    volatile u32* reg = port + GPIODEN;             /* DIGITAL ENABLE PINS */
    u32 data = *reg;
        data &= ~(pins);
        data |= (0xff & pins);
        *reg = data;

   reg = port + GPIOAFSEL;                           //ADRESS_PORTA_AFSEL
   data = *reg;                                      // DATA PORTA AFSEL say XXXXXXXX... 00101101

    data &= ~(pins);                                 // ~pins=10011011 ,, data &= ~(pins)=XXXXXXX 00001001

    if (mode == MODE_AF)
        data |= (0xff & pins);
    else
        data |= (0x00 & pins);                                  // data = XXXXXXX 00001001

    *reg = data;                                                 //DATAPORTAAFSEL = XXXXXXX 00001001
/*-----------------------------------------------------------------------------------------------------*/

    reg = port + GPIODIR;                                   //ADRESS _ PORtA DIR
        data = *reg;                                        // DATA PORTA dir say XXXXXXXX... 00101101

     data &= ~(pins);                                        // ~pins=10011011 ,, data &= ~(pins)=XXXXXXX 00001001
                                                             // PINS 0b01100100
     if (mode == MODE_OUT)
     {
         data |= (0xff & pins);
         *reg = data;                                            //     01101101
     }
     else if (mode == MODE_IN)
     {
         data |= (0x00 & pins);
         *reg = data;                                       // data = XXXXXXX 00001001
     }
}
u8 GPIODirGet(gpio_port_t port, u8 pins)
{
    volatile u32* reg = port + GPIODIR;
    u32 data = *reg;
    data &= 0x000000FF;
return ((u8)data & pins);
}

u8 GPIOModeGet(gpio_port_t port, u8 pins)
{
    volatile u32* reg = port + GPIOAFSEL;
    u32 data = *reg;
    data &= 0x000000FF;                                                                                                                                       //  PINS 00010001
return ((u8)data & pins);
}

void GPIOPadSet(gpio_port_t port, u8 pins, gpio_drive_t str, gpio_pad_t pad)
{
    volatile u32* reg = port +  str ;                       // pins          11100101
    u32 data = *reg;                                        // data XXXXXXXX 00110011
                                                            //   ~pins         00011010
    data &= ~(pins);                                                       //  00010010
    data |= (0xff & pins);                                                 //11110111
    *reg = data;
/*--------------------------------------------------------------------------------------------------*/
   if ( pad == PAD_NPU_NPD)
   {
       reg = port + PAD_PU;
            data = *reg;
            data &= ~(pins);
            data |= (0x00 & pins);

            reg = port + PAD_PD;
            data = *reg;
            data &= ~(pins);
            data |= (0x00 & pins);
   }
   else
   {
    reg = port + pad;
    data = *reg;
    data &= ~(pins);
    data |= (0xff & pins);
    *reg = data;
   }
}
u8 GPIOPadDriveStrGet(gpio_port_t port, u8 pin)       /* return value of amperes of only 1 pin */
{
    volatile u32* reg = port + Drive_2mA;
     if (GETBIT(*reg,pin))
            return 0x02;
    reg = port + Drive_4mA;
    if (GETBIT(*reg,pin))
            return 0x04;
    reg = port + Drive_8mA;
    if (GETBIT(*reg,pin))
            return 0x08;
}
u8 GPIOPadOpenDrainGet(gpio_port_t port, u8 pins)
{
    volatile u32* reg = port + GPIOODR;
    u32 data = *reg;
    data &= 0x000000FF;
return ((u8)data & pins);
}

u8 GPIOPadPullUpGet(gpio_port_t port, u8 pins)
{
    volatile u32* reg = port + GPIOPUR;
    u32 data = *reg;
    data &= 0x000000FF;
return ((u8)data & pins);
}

u8 GPIOPadPullDownGet(gpio_port_t port, u8 pins)
{
    volatile u32* reg = port + GPIOPDR;
    u32 data = *reg;
    data &= 0x000000FF;
return ((u8)data & pins);
}

void GPIOWrite(gpio_port_t port, u8 pins, gpio_data_t gpio_data)
{
 volatile u32* reg = port + GPIODATA;
         u32 data = *reg;
        data &= ~(pins);
        if (gpio_data == HIGH)
        {
            data |= (0xff & pins);
            *reg = data;
        }
        else if (gpio_data == LOW)
        {
            data |= (0x00 & pins);
            *reg = data;
        }
}
u8 GPIORead(gpio_port_t port, u8 pins)
{
    volatile u32* reg = port + GPIODATA;
        u32 data = *reg;
        data &= 0x000000FF;                                                                                                                                 //  PINS 00010001
        return ((u8)data & pins);
}
u8 INT_pins=0x00;
void InterruptEnable(gpio_port_t port , unsigned char pins , interrupt_t type  )
{
    INT_pins = pins;
    volatile u32* reg = port + GPIOIM;
                ClrMask(reg,pins);

           if (type <= 2)
           {
               reg = port + GPIOIS;
               ClrMask(reg,pins);
           }
           else
           {
               reg = port + GPIOIS;
               SetMask(reg,pins);
           }

           if (type == RISING_FALLING)
           {
               reg = port + GPIOIBE;
               SetMask(reg,pins);
           }
           else
           {
               reg = port + GPIOIBE;
               ClrMask(reg,pins);
           }
           if (type == RISING)
           {
               reg = port + GPIOIEV;
               SetMask(reg,pins);
           }
               else if (type == FALLING)
               {
                   reg = port + GPIOIEV;
                   ClrMask(reg,pins);
               }


           reg = port + GPIORIS;
           ClrMask(reg,pins);
           reg = port +GPIOICR;
           SetMask(reg,pins);
           reg = port +GPIOIM;
           SetMask(reg,pins);

           if (port == PORTA)
           {
               SETBIT(INT_EN0,0);
           }
           else   if (port == PORTB)
           {
               SETBIT(INT_EN0,1);
           }
           else   if (port == PORTC)
           {
               SETBIT(INT_EN0,2);
           }
           else   if (port == PORTD)
           {
               SETBIT(INT_EN0,3);
           }
           else   if (port == PORTE)
           {
               SETBIT(INT_EN0,4);
           }
           else   if (port == PORTF)
           {
               SETBIT(INT_EN0,30);
           }
}

void (*ISRA)(void) = '\0';
void (*ISRB)(void) = '\0';
void (*ISRC)(void) = '\0';
void (*ISRD)(void) = '\0';
void (*ISRE)(void) = '\0';
void (*ISRF)(void) = '\0';
void SetISR(gpio_port_t port , void (*Local_ISR)(void))
{
    if (port == PORTA)
        ISRA = Local_ISR;
    else if (port == PORTB)
        ISRB = Local_ISR;
    else if (port == PORTC)
        ISRC = Local_ISR;
    else if (port == PORTD)
        ISRD = Local_ISR;
    else if (port == PORTE)
        ISRE = Local_ISR ;
    else if (port == PORTF)
        ISRF = Local_ISR;
}
void ISR_PORTA (void)
{
    ISRA();
    volatile u32 *reg = PORT_A + GPIOICR;
    SetMask(reg,INT_pins);
}
void ISR_PORTB (void)
{
    ISRA();
    volatile u32 *reg = PORT_B + GPIOICR;
    SetMask(reg,INT_pins);
}
void ISR_PORTC (void)
{
    ISRA();
    volatile u32 *reg = PORT_C + GPIOICR;
    SetMask(reg,INT_pins);
}
void ISR_PORTD (void)
{
    ISRA();
    volatile u32 *reg = PORT_D + GPIOICR;
    SetMask(reg,INT_pins);
}
void ISR_PORTE (void)
{
    ISRA();
    volatile u32 *reg = PORT_E + GPIOICR;
    SetMask(reg,INT_pins);
}
void ISR_PORTF (void)
{
    ISRF();

    volatile u32 *reg = PORT_F + GPIOICR;
    SetMask(reg,INT_pins);
}

