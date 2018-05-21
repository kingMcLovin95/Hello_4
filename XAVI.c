
// CONFIG1L
#pragma config PLLSEL = PLL4X   // PLL Selection (4x clock multiplier)
#pragma config CFGPLLEN = OFF   // PLL Enable Configuration bit (PLL Disabled (firmware controlled))
#pragma config CPUDIV = NOCLKDIV// CPU System Clock Postscaler (CPU uses system clock (no divide))
#pragma config LS48MHZ = SYS24X4// Low Speed USB mode with 48 MHz system clock (System clock at 24 MHz, USB clock divider is set to 4)
//#pragma config PLLDIV = 1       // PLL Prescaler Selection bits (No prescale (4 MHz oscillator input drives PLL directly))
//#pragma config CPUDIV = OSC1_PLL2// System Clock Postscaler Selection bits ([Primary Oscillator Src: /1][96 MHz PLL Src: /2])
//#pragma config USBDIV = 1       // USB Clock Selection bit (used in Full-Speed USB mode only; UCFG:FSEN = 1) (USB clock source comes directly from the primary oscillator block with no postscale)

// CONFIG1H
#pragma config FOSC = INTOSCIO  // Oscillator Selection (Internal oscillator)
#pragma config PCLKEN = OFF     // Primary Oscillator Shutdown (Primary oscillator shutdown firmware controlled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor (Fail-Safe Clock Monitor disabled)
#pragma config IESO = OFF       // Internal/External Oscillator Switchover (Oscillator Switchover mode disabled)

// CONFIG2L
#pragma config nPWRTEN = OFF    // Power-up Timer Enable (Power up timer disabled)
#pragma config BOREN = OFF      // Brown-out Reset Enable (BOR disabled in hardware (SBOREN is ignored))
#pragma config BORV = 190       // Brown-out Reset Voltage (BOR set to 1.9V nominal)
#pragma config nLPBOR = OFF     // Low-Power Brown-out Reset (Low-Power Brown-out Reset disabled)

// CONFIG2H
#pragma config WDTEN = OFF      // Watchdog Timer Enable bits (WDT disabled in hardware (SWDTEN ignored))
#pragma config WDTPS = 32768    // Watchdog Timer Postscaler (1:32768)

// CONFIG3H
#pragma config CCP2MX = RC1     // CCP2 MUX bit (CCP2 input/output is multiplexed with RC1)
#pragma config PBADEN = ON      // PORTB A/D Enable bit (PORTB<5:0> pins are configured as analog input channels on Reset)
#pragma config T3CMX = RC0      // Timer3 Clock Input MUX bit (T3CKI function is on RC0)
#pragma config SDOMX = RB3      // SDO Output MUX bit (SDO function is on RB3)
#pragma config MCLRE = ON       // Master Clear Reset Pin Enable (MCLR pin enabled; RE3 input disabled)

// CONFIG4L
#pragma config STVREN = ON      // Stack Full/Underflow Reset (Stack full/underflow will cause Reset)
#pragma config LVP = OFF        // Single-Supply ICSP Enable bit (Single-Supply ICSP disabled)
#pragma config ICPRT = OFF      // Dedicated In-Circuit Debug/Programming Port Enable (ICPORT disabled)
#pragma config XINST = OFF      // Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode disabled)

// CONFIG5L
#pragma config CP0 = OFF        // Block 0 Code Protect (Block 0 is not code-protected)
#pragma config CP1 = OFF        // Block 1 Code Protect (Block 1 is not code-protected)
#pragma config CP2 = OFF        // Block 2 Code Protect (Block 2 is not code-protected)
#pragma config CP3 = OFF        // Block 3 Code Protect (Block 3 is not code-protected)

// CONFIG5H
#pragma config CPB = OFF        // Boot Block Code Protect (Boot block is not code-protected)
#pragma config CPD = OFF        // Data EEPROM Code Protect (Data EEPROM is not code-protected)

// CONFIG6L
#pragma config WRT0 = OFF       // Block 0 Write Protect (Block 0 (0800-1FFFh) is not write-protected)
#pragma config WRT1 = OFF       // Block 1 Write Protect (Block 1 (2000-3FFFh) is not write-protected)
#pragma config WRT2 = OFF       // Block 2 Write Protect (Block 2 (04000-5FFFh) is not write-protected)
#pragma config WRT3 = OFF       // Block 3 Write Protect (Block 3 (06000-7FFFh) is not write-protected)

// CONFIG6H
#pragma config WRTC = OFF       // Configuration Registers Write Protect (Configuration registers (300000-3000FFh) are not write-protected)
#pragma config WRTB = OFF       // Boot Block Write Protect (Boot block (0000-7FFh) is not write-protected)
#pragma config WRTD = OFF       // Data EEPROM Write Protect (Data EEPROM is not write-protected)

// CONFIG7L
#pragma config EBTR0 = OFF      // Block 0 Table Read Protect (Block 0 is not protected from table reads executed in other blocks)
#pragma config EBTR1 = OFF      // Block 1 Table Read Protect (Block 1 is not protected from table reads executed in other blocks)
#pragma config EBTR2 = OFF      // Block 2 Table Read Protect (Block 2 is not protected from table reads executed in other blocks)
#pragma config EBTR3 = OFF      // Block 3 Table Read Protect (Block 3 is not protected from table reads executed in other blocks)

// CONFIG7H
#pragma config EBTRB = OFF      // Boot Block Table Read Protect (Boot block is not protected from table reads executed in other blocks)


#include <pic18f45k50.h>              //----Include Reg file of Pic18f4550
#include<xc.h>

/* LCD Commands */
#define LCD_16x2_INIT         0x38    /**< Initialize 16x2 Lcd in 8-bit Mode.*/
#define LCD_DISP_ON_CUR_ON    0x0E    /**< LCD Display On Cursor On.*/
#define LCD_DISP_ON_CUR_OFF   0x0C    /**< LCD Display On Cursor Off.*/
#define LCD_DISP_ON_CUR_BLNK  0x0F    /**< LCD Display On Cursor Blink.*/
#define LCD_ENTRY_MODE        0x06    /**< LCD Entry Mode. */
#define LCD_FIRST_ROW         0x80    /**< Move Pointer to First Row.*/
#define LCD_SECOND_ROW        0xC0    /**< Move Pointer to Second Row.*/
#define LCD_CLEAR             0x01    /**< Clear LCD Display.*/
#define LCD_ROTATE_LEFT       0x18    /**< Rotate LCD Data Left.*/

/*PIR Motion Sensor*/
#define Motion_detection PORTAbits.RA0  /* Read PIR sensor's data on this pin */
#define PORT_Dir TRISAbits.RA0          /* define for setting direction */
#define LED PORTCbits.RC6               /* connect LED to the PORT pin */
#define LED_Dir TRISCbits.RC6           /* define for setting direction */

/*Bateria*/
#define Battery PORTEbits.RE0   
#define Battery_Dir TRISEbits.RE0

#define Battery_LED PORTCbits.RC7
#define Battery_LED_Dir TRISCbits.RC7

//#define  _XTAL_FREQ 4000000
//#pragma config WDTEN = OFF      // Watchdog Timer Enable bits (WDT disabled in hardware (SWDTEN ignored))
#define lcd PORTD                   //----Assing PORTD as lcd
 
//LCD
#define rs PORTCbits.RC0            //----Register Select Pin for Control of LCD
#define rw PORTCbits.RC1            //----Read/Write Pin for Control of LCD
#define en PORTCbits.RC2            //----Enable Pin for Control of LCD
//RENGLONES
#define R1 PORTBbits.RB0
#define R2 PORTBbits.RB1
#define R3 PORTBbits.RB2
#define R4 PORTBbits.RB3
//COLUMNAS
#define C1 PORTBbits.RB4
#define C2 PORTBbits.RB5
#define C3 PORTBbits.RB6
#define C4 PORTBbits.RB7

/*VARIABLES*/
int aux=0;
int contrasenia_guardada[4] = {1,2,3,4};
int contrasenia_ingresada [4]; 
int contrasenia_valida;
int contador;
int i;
int clave_correcta;
int active, sensor;
int alerta, reposo;

/*FUNCIONES*/
void lcdinit();
void lcddata(char z);
void lcd_msg(unsigned char *c);     //----Function to Send String of Data to LCD
void lcd_lat();                     //----Function to Latch data into LCD
void lcd_ready();  
void delay_ms(unsigned int time);
//void delay(unsigned int ms);        //----Delay Function for 1ms
void Check_Col1();
void Check_Col2();
void Check_Col3();
void Check_Col4();
void Motion();
void Bateria();


void lcd_lat()                    //----To Latch data into LCD
{
    en = 1;                       //----Enable Pin goes high
    delay_ms(1);                     //----delay of 1ms
    en = 0;                       //----Enable Pin goes Low
}
void lcd_ready()
{
    lcd = 0xFF;                   //----PORTD is High
    lcd &= 0x80;                  //----D7 is set as high
    rs = 0;                       //----Command Register is Selected
    rw = 1;                       //----Read/Write Pin is High => Read
    en = 0; delay_ms(1); en = 1;     //----Low to High to read data from LCD
    if(lcd == 0x80)
    {
        en = 0; delay_ms(1); en = 1;     //----Low to High to read data from LCD
    }
    else
    {
        //---Do nothing.
    }
} 

void delay_ms(unsigned int time)
{
unsigned int x;
unsigned char y;
for(x=0;x<time;x++)
{
 for(y=0;y<5;y++);
}
}
void lcdcmd (char y)
  {
    lcd_ready();                  //----To Check whether lcd is busy
    lcd = y;  
    LATD = y;
    rs = 0;
    rw = 0;
    en = 1;
    lcd_lat();  
    delay_ms(30);
    en = 0;
  }

void lcd_dwr(unsigned char x)
{
    lcd_ready();                  //----To Check whether lcd is busy
    lcd = x;                      //----8-Bit Data is Send to PORTD
    rs = 1;                       //----Register Select Pin is High => Data Register
    rw = 0;                       //----Read/Write Pin is Low => Write.
    lcd_lat();                    //----Latch data into LCD
}
 
void lcd_msg(unsigned char *c)
{
    while(*c != 0)              //---Check till last data is send
        lcd_dwr(*c++);          //---Send data to lcd and increment
    
}

void lcddata (char z)
{
    lcdcmd(LCD_CLEAR);
     LATD = z;
     rs = 1;
     rw = 0;
     en = 1;
     delay_ms(30);
     en = 0;
}
void lcdinit()
{

 lcdcmd(LCD_16x2_INIT); //Function Set: 8-bit, 2 Line, 5x7 Dots
 delay_ms(1);
 lcdcmd(LCD_DISP_ON_CUR_ON); //Display on Cursor on 
 delay_ms(1);
 lcdcmd(LCD_CLEAR); //Clear Display Screen
 delay_ms(1);
 lcdcmd(LCD_ENTRY_MODE); //Entry Mode
 delay_ms(1);
 lcdcmd(0x81); //Set Contrsat Control (0x81=129)
 delay_ms(1);

}

void Bateria()
{
    if (Battery==1)
        Battery_LED = 1;
        
    if(Battery == 0)
        Battery_LED = 0;

}

void Motion()
{
    if((contrasenia_valida==2&&sensor ==1) || contrasenia_valida==0&&sensor ==1)
    {
       
        if(Motion_detection == 1 && active == 0)
        {
            active = 1;
            contrasenia_valida = 0;
            contador = 0;
            i = 0;
        }
        
    while(Motion_detection == 1 || active == 1)   
    {
        aux=0;
        LED = 0;//PRENDE LED Y BUZZER
        lcdcmd(0x81);
        lcdcmd(LCD_FIRST_ROW);
        lcdcmd(LCD_ROTATE_LEFT);
        delay_ms(200);
        lcd_msg("The motion sensor has been triggered"); //-----Data
        delay_ms(200);
        aux=1;
        
    }
     if(aux==1)
            lcdcmd(LCD_CLEAR);
     aux=2;
    if (Motion_detection == 0)
    {
        
        LED = 1;
    }
    }
}



void Check_Col1()
 {
  R2=R3=R4=0;


   R1=1;
   if (C1==1)
    {
            if(contrasenia_valida==0)
       {  
          contrasenia_ingresada[contador]=1;  
          if(contador <3)
          {
          contador +=1;
          }  
          
       }
       if(contrasenia_valida==2 && alerta == 1)
       {   
           lcdcmd(0x0C);
           lcdcmd(LCD_FIRST_ROW);
           lcd_msg("ALERT");
           lcdcmd(LCD_SECOND_ROW);
           lcd_msg("#318, FAM.GONZALES");
           delay_ms(10000); 
          
        }
       if(contrasenia_valida==2 && alerta == 2)
       {   
           lcdcmd(0x0C);
           lcdcmd(LCD_FIRST_ROW);
           lcd_msg("BECAREFUL");
           lcdcmd(LCD_SECOND_ROW);
           lcd_msg("#318, FAM.GONZALES");
           delay_ms(10000);          
       }
       
       while(C1==1);
    }
//numero 4
    if (C2==1)
        {
             if(contrasenia_valida==0)
        {
          contrasenia_ingresada[contador]=4;  
          if(contador < 3)
          {
          contador +=1;
          }
        }
       if(contrasenia_valida==2)
       {   
           lcdcmd(0x0C);
           lcdcmd(LCD_FIRST_ROW);
           lcd_msg("EMPTY");
           delay_ms(10000);    
           lcdcmd(LCD_CLEAR);  
       }
             while(C2==1);
         }
//numero 7
    if (C3==1)
         {
                if(contrasenia_valida==0)
        {
          contrasenia_ingresada[contador]=7;  
          if(contador < 3)
          {
          contador +=1;
          }
        }
       if(contrasenia_valida==2)
       {   
           lcdcmd(0x0C);
           lcdcmd(LCD_FIRST_ROW);
           lcd_msg("EMPTY");
           delay_ms(10000);    
           lcdcmd(LCD_CLEAR);           
       }
               while(C3==1);
         }
//sisirisco
    if  (C4==1)
         {
              if(contrasenia_valida==2)
       {   
           lcdcmd(0x0C);
           lcdcmd(LCD_FIRST_ROW);
           lcd_msg("*");
           delay_ms(10000);    
           lcdcmd(LCD_CLEAR);           
       }
             while(C4==1);
         }
        R1=0;
}

void Check_Col2()
 {
  R1=R3=R4=0;


   R2=1;
   if (C1==1)
    {
            if(contrasenia_valida==0)
        {
          contrasenia_ingresada[contador]=2;  
          if(contador < 3)
          {
          contador +=1;
          }
        }
       if(contrasenia_valida==2 && alerta == 1)
       {   
           lcdcmd(0x0C);
           lcdcmd(LCD_FIRST_ROW);
           lcd_msg("ALERT");
           lcdcmd(LCD_SECOND_ROW);
           lcd_msg("#318, FAM.RODRIGUEZ");
           delay_ms(10000); 
          
        }
       if(contrasenia_valida==2 && alerta == 2)
       {   
           lcdcmd(0x0C);
           lcdcmd(LCD_FIRST_ROW);
           lcd_msg("BECAREFUL");
           lcdcmd(LCD_SECOND_ROW);
           lcd_msg("#318, FAM.RODRIGUEZ");
           delay_ms(10000);          
       }
            while(C1==1);
    }
//numero 5
    if (C2==1)
        {
               if(contrasenia_valida==0)
        {
          contrasenia_ingresada[contador]=5;  
          if(contador < 3)
          {
          contador +=1;
          }
        }
       if(contrasenia_valida==2)
       {   
           lcdcmd(0x0C);
           lcdcmd(LCD_FIRST_ROW);
           lcd_msg("EMPTY");
           delay_ms(10000);    
           lcdcmd(LCD_CLEAR);           
       }
             while(C2==1);
         }

    if (C3==1)
         {
               if(contrasenia_valida==0)
        {
          contrasenia_ingresada[contador]=8;  
          if(contador < 3)
          {
          contador +=1;
          }
        }
       if(contrasenia_valida==2)
       {   
           lcdcmd(0x0C);
           lcdcmd(LCD_FIRST_ROW);
           lcd_msg("EMPTY");
           delay_ms(10000);    
           lcdcmd(LCD_CLEAR);           
       }
               while(C3==1);
         }

    if  (C4==1)
         {
            if(contrasenia_valida==0)
        {
          contrasenia_ingresada[contador]=0;  
          if(contador < 3)
          {
          contador +=1;
          }
        }
       if(contrasenia_valida==2)
       {   
           lcdcmd(0x0C);
           lcdcmd(LCD_FIRST_ROW);
           lcd_msg("EMPTY");
           delay_ms(10000);    
           lcdcmd(LCD_CLEAR);           
       }
             while(C4==1);
         }
        R2=0;
}

void Check_Col3()
 {
  R1=R2=R4=0;


   R3=1;
   if (C1==1)
    {
            if(contrasenia_valida==0)
        {
          contrasenia_ingresada[contador]=3;  
          if(contador < 3)
          {
          contador +=1;
          }
        }
       if(contrasenia_valida==2 && alerta == 1)
       {   
           lcdcmd(0x0C);
           lcdcmd(LCD_FIRST_ROW);
           lcd_msg("ALERT");
           lcdcmd(LCD_SECOND_ROW);
           lcd_msg("#318, FAM.GARCIA");
           delay_ms(10000); 
          
        }
       if(contrasenia_valida==2 && alerta == 2)
       {   
           lcdcmd(0x0C);
           lcdcmd(LCD_FIRST_ROW);
           lcd_msg("BECAREFUL");
           lcdcmd(LCD_SECOND_ROW);
           lcd_msg("#318, FAM.GARCIA");
           delay_ms(10000);          
       }
            while(C1==1);
    }

    if (C2==1)
        {
              if(contrasenia_valida==0)
        {
          contrasenia_ingresada[contador]=6;  
          if(contador < 3)
          {
          contador +=1;
          }
        }
       if(contrasenia_valida==2)
       {   
           lcdcmd(0x0C);
           lcdcmd(LCD_FIRST_ROW);
           lcd_msg("EMPTY");
           delay_ms(10000);    
           lcdcmd(LCD_CLEAR);          
       }
             while(C2==1);
         }

    if (C3==1)
         {
              if(contrasenia_valida==0)
        {
          contrasenia_ingresada[contador]=9;  
          if(contador < 3)
          {
          contador +=1;
          }
        }
       if(contrasenia_valida==2)
       {   
           lcdcmd(0x0C);
           lcdcmd(LCD_FIRST_ROW);
           lcd_msg("EMPTY");
           delay_ms(10000);    
           lcdcmd(LCD_CLEAR);           
       }
               while(C3==1);
         }

    if  (C4==1)
         {
              if(contrasenia_valida==2)
       {   
           lcdcmd(0x0C);
           lcdcmd(LCD_FIRST_ROW);
           lcd_msg("EMPTY");
           delay_ms(10000);    
           lcdcmd(LCD_CLEAR);           
       }
             while(C4==1);
         }
        R3=0;
}

void Check_Col4()
 {
  R1=R2=R3=0;


   R4=1;
   if (C1==1)
    {
            if(contrasenia_valida==2)
           {   
           if(alerta !=1)
           {
               if(alerta==2)
                   alerta=2;
               else
                   alerta=1;               
           }
           }
            while(C1==1);
    }

    if (C2==1)
        {
             if(contrasenia_valida==2)
           {   
            if(alerta !=2)
           {
               if(alerta==1)
                   alerta=1;
               else
                   alerta=2;               
           }         
           }
             while(C2==1);
         }

    if (C3==1)
         {
               if (contador>=3 && contrasenia_valida == 0 && active == 0)
            {           
                for( i= 0;i<4;i++)
                {
                    if(contrasenia_guardada[i]!=contrasenia_ingresada[i])
                    {
                        contrasenia_valida=3;
                        contador=0;
                        i=0;
                        break;
                    }
                    
                    if(i>=3)
                    {
                        contrasenia_valida=1;
                    }
                }
            }
                     if (contador>=3 && contrasenia_valida == 0 && active == 1)
            {           
                for( i= 0;i<4;i++)
                {
                    if(contrasenia_guardada[i]!=contrasenia_ingresada[i])
                    {
                        contador=0;
                        i=0;
                        break;
                    }
                    
                    if(i>=3)
                    {
                        lcdcmd(LCD_CLEAR);  
                        sensor = 0;
                        active = 0;
                        contador = 0;
                        i = 0;
                    }
                }
            }
               
            if (contador>=3 && contrasenia_valida == 2)
            {
                if(alerta != 0)
                {
                    alerta = 0;
                    lcdcmd(LCD_CLEAR);  
                }
            }
               while(C3==1);
         }

    if  (C4==1)
         {
            if(contrasenia_valida==2)
       {   
                sensor = 1;
       }
             while(C4==1);
         }
        R4=0;
}

void main()
{
    //int aux=1;
    //LATA = 0;       // Define direccion
    //TRISA = 0;      // Escribir 
    //ANSELD = 0;     // Puerto analogicos 
    //OSCCON = 83;    // Oscilador 
    //PORTA = 0;
    ADCON1 = 0x0F;
    TRISD  = 0x00;
    TRISC  = 0x00;
    TRISB  = 0xF0;
    ANSELB = 0x00;
    ANSELA = 0x00;
    ANSELC = 0x00;
    PORTB  = 0xF0;
    LATB   = 0xF0;
    LATC   = 0x00;
    LATD   = 0x00;
    TRISE  = 0x00;
    ANSELE = 0x00;
    LATE   = 0x00;
    
    //PIR
    Motion_detection = 0;
    PORT_Dir = 1;       /* set as input port */
    LED_Dir = 0;       /* set as output port */
    LED = 1;           /* initially turned OFF LED */
    
    //BATTERY
    Battery = 0;
    Battery_Dir = 1;
    Battery_LED_Dir = 0;
    Battery_LED = 1;
    /*CONTRASEÑA E INICIO*/
    sensor = 0;
    active = 0;
    contador=0;
    clave_correcta=1;
    alerta=0;
    contrasenia_valida=0;
 
  
    
    
    INTCON2bits.RBPU = 1; // turn off  weak pull ups for RB7-RB4 switches
    lcdinit();
    lcdcmd(LCD_FIRST_ROW);
    lcd_msg(" |:. Welcome .:|"); //-----Data
    lcdcmd(LCD_SECOND_ROW);
     lcd_msg(" |:. McLovin .:|"); //-----Data
    delay_ms(10000);                 //-----250 msec delay
    lcdcmd(LCD_FIRST_ROW);
    lcd_msg("       ^_^       "); //-----Data
    lcdcmd(LCD_SECOND_ROW);
    lcd_msg("||||||||||||||||"); //-----Data
    delay_ms(2000);                 //-----250 msec delay
    lcdcmd(LCD_FIRST_ROW);
    lcd_msg("       ^o^    "); //-----Data
    lcdcmd(LCD_SECOND_ROW);
    lcd_msg("||||||||||||||||"); //-----Data
    delay_ms(2000); 
    lcdcmd(LCD_CLEAR);
    //if(Motion_detection == 1)
      // lcd_msg("AGUAS"); //-----Data
   
 while(1)
 {
   //if(Motion_detection == 1)
     //  lcd_msg("AGUAS"); //-----Data
       
   lcdcmd(LCD_FIRST_ROW);
   Motion();
   Bateria();
   Check_Col1();  
   Check_Col2();   
   Check_Col3();   
   Check_Col4();
   delay_ms(40);  
   
    if(contrasenia_valida==1)
        {
            lcdcmd(LCD_FIRST_ROW);
            lcd_msg("RIGHT");
            delay_ms(10000); 
            lcdcmd(LCD_CLEAR);
            contrasenia_valida = 2;
        } 
   
     if(contrasenia_valida==3)
        {
            lcdcmd(LCD_FIRST_ROW);
            lcd_msg("ERROR, PLEASE");
            lcdcmd(LCD_SECOND_ROW);
            lcd_msg("TRY AGAIN");
            delay_ms(10000); 
            lcdcmd(LCD_CLEAR);
            contrasenia_valida = 0;
        }  
  }
}