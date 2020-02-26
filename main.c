

/**
 * main.c
 */
#include "gpio.h"
#include "regmap.h"
#include "BIT_MODE.h"
//   volatile u32 *ICR = PORT_F + 0x41C;
//   volatile u32 *EN0 = 0xE000E100;
void PORTF_FN (void);
void ISR_TIMER0(void);
#define TIMER0_base 0x40030000
int main(void)
{



    u8 flag =0;
    int i =0;
    ConfigureBus();
    GPIOClockSet(PORTF);
    GPIODirModeSet(PORTF,0b00010001, MODE_IN);
    GPIODirModeSet(PORTF,0b00001110, MODE_OUT);
    GPIOPadSet(PORTF, 0b00011110, Drive_8mA ,PAD_PU);
    InterruptEnable(PORTF , 0b00010000 , L_LEVEL);
    SetISR(PORTF ,  PORTF_FN);


   volatile u32 * reg = 0x400FE000 +0x604;
    SETBIT(*reg , 0);                                   // ENABLE CLOCK TO TIMER

    SETBIT(INT_EN0 , 19);                               // ENABLE GLOBAL INT

        reg = TIMER0_base + 0x00C;     //ctrl
        CLRBIT(*reg , 0);                               // DISABLE TIMER

        reg = TIMER0_base + 0x000;
        CLRBIT(*reg , 0);
        CLRBIT(*reg , 1);
        CLRBIT(*reg , 2);                               // ENABLE 16/32 TIMER

        reg = TIMER0_base + 0x004;
        SETBIT(*reg,1);
        CLRBIT(*reg,0);                                 // PERIODIC


        reg = TIMER0_base + 0x028;
        *reg = 0xFFFFFF;                                // INTERVAL LOAD

        reg = TIMER0_base + 0x018;
        SETBIT(*reg , 0);                               // ENABLE TIME-OUT INTERRUPT

        reg = TIMER0_base + 0x00C;
        SETBIT(*reg , 0);                               // START COUNT

        reg = TIMER0_base +0x024;
        SETBIT(*reg , 0);                               // CLEAR INTERRUPT

        while (1)
        {

        }



}
void ISR_TIMER0(void)
{
    static u8 flag = 0;
    if (flag ==0 )
       {
       GPIOWrite(PORTF,0b00001110, HIGH);
       flag++;
       }
       else if ( flag == 1)
       {
           GPIOWrite(PORTF,0b000010, HIGH);
           GPIOWrite(PORTF,0b001100, LOW);
           flag ++;
       }

       else if ( flag == 2)
       {
           GPIOWrite(PORTF,0b000100, HIGH);
                 GPIOWrite(PORTF,0b001010, LOW);
           flag ++;
       }
       else if ( flag == 3)
       {
           GPIOWrite(PORTF,0b001000, HIGH);
                 GPIOWrite(PORTF,0b000110, LOW);
           flag ++;
       }
       else if ( flag == 4)
       {
           GPIOWrite(PORTF,0b000110, HIGH);
                 GPIOWrite(PORTF,0b001000, LOW);
           flag ++;
       }
       else if ( flag == 5)
       {
           GPIOWrite(PORTF,0b001100, HIGH);
                 GPIOWrite(PORTF,0b000010, LOW);
           flag ++;
       }
       else if ( flag == 6)
       {
           GPIOWrite(PORTF,0b001010, HIGH);
                 GPIOWrite(PORTF,0b000100, LOW);        flag =0;
       }

       volatile u32 *reg = TIMER0_base +0x024;
                  SETBIT(*reg , 0);
}
void PORTF_FN (void)
{
          GPIOWrite(PORTF,0b001110, LOW);
}
