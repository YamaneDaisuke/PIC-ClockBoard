/* 
 * File:   main.c
 * Author: Yamaned
 *
 * Created on 2015/02/13, 16:38
 */

#include <stdio.h>
#include <stdlib.h>
#include <xc.h>
#include <pic.h>
// CONFIG
#pragma config FOSC = HS        // Oscillator Selection bits (HS oscillator)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled)
#pragma config PWRTE = ON       // Power-up Timer Enable bit (PWRT enabled)
#pragma config BOREN = OFF      // Brown-out Reset Enable bit (BOR disabled)
#pragma config LVP = OFF         // Low-Voltage (Single-Supply) In-Circuit Serial Programming Enable bit (RB3/PGM pin has PGM function; low-voltage programming enabled)
#pragma config CPD = OFF        // Data EEPROM Memory Code Protection bit (Data EEPROM code protection off)
#pragma config WRT = OFF        // Flash Program Memory Write Enable bits (Write protection off; all program memory may be written to by EECON control)
#pragma config CP = OFF         // Flash Program Memory Code Protection bit (Code protection off)


#define HALF_MIN 31
#define TMR0TIME -195
#define _XTAL_FREQ 20000000
/*
 * 
 */
void initPORT(void);
void initTMR0(void);
void InitI2C(void);
static void interrupt isr(void);
unsigned int cnt=0;
unsigned char number_low=0;
unsigned char number_high=0;
const char addr = 0x94;
static unsigned char address_flag, i2c_command, seg_data[4];
int main(int argc, char** argv) {
    //unsigned char number=0;
    unsigned char mask_seg[17] = {0x3F, 0x03, 0x6D, 0x67, 0x53, 0x76, 0x7E, 0x33, 0x7F, 0x77, 0x7B, 0x5E, 0x3C, 0x4F, 0x7C, 0x78, 0x00};
    unsigned char temp_mask=0;

    initPORT();
    //initTMR0();
    InitI2C();
    
    while(1){
    
            PORTA = 0x3F;
            PORTB |= 0xFE;
            PORTC |= 0xE7;
            PORTD |= 0xFF;
            PORTE |= 0x01;
            __delay_ms(1);
            
//            PORTA &=  mask_seg[seg_data[0]];
//            PORTE &= (mask_seg[seg_data[0]]>>6);
//            PORTB &= (mask_seg[seg_data[1]]<<1);
//            PORTD &= (mask_seg[seg_data[2]]<<1) |0x01;
//            temp_mask  = (mask_seg[seg_data[3]]>>1) & 0x07;
//            temp_mask |= (mask_seg[seg_data[3]]<<1) & 0xe0;
//            temp_mask |= 0x18;
//            PORTC &= temp_mask;
//            PORTD &= mask_seg[seg_data[3]] | 0xfe;
//           
            PORTA =  mask_seg[seg_data[1]];
            PORTE &= (mask_seg[seg_data[1]]>>6);
            PORTB &= (mask_seg[seg_data[3]]<<1);
            PORTD &= (mask_seg[seg_data[2]]<<1) |0x01;
            temp_mask  = (mask_seg[seg_data[0]]>>1) & 0x07;
            temp_mask |= (mask_seg[seg_data[0]]<<1) & 0xe0;
            temp_mask |= 0x18;
            PORTC &= temp_mask;
            PORTD &= mask_seg[seg_data[0]] | 0xfe;
          
            __delay_ms(1);
            PORTA = ~0x3F;
            PORTB &= ~0xFE;
            PORTC &= ~0xE7;
            PORTD &= ~0xFF;
            PORTE &= ~0x01;
            __delay_ms(18);
        //cnt++;
        if(cnt>=50){
            seg_data[0]++;
            if(seg_data[0]>=10){
                seg_data[0]=0;
                seg_data[1]++;
            }
            if(seg_data[1]>=10){
                seg_data[1]=0;
                seg_data[2]++;
            }
            if(seg_data[2]>=10){
                seg_data[2]=0;
                seg_data[3]++;
            }
            if(seg_data[3]>=10){
                seg_data[3]=0;
            }
            number_high ++;
            if(number_high>=0x10)
                number_high=0;
            cnt=0;
        }
    }
    return (EXIT_SUCCESS);
}

void initPORT(void)
{
//    ANSEL = 0x00;   //All ports are Digital pin
//    ANSELH = 0x00;
    ADCON1 = 0x06;
    PORTA = 0x00;
    PORTB = 0x00;
    PORTC = 0x00;
    PORTD = 0x00;
    PORTE = 0x00;
    TRISA = 0x00;
    TRISB = 0x00;
    TRISC = 0x18;
    TRISD = 0x00;
    TRISE = 0x00;
    //PORTE = 0x07;
    //PORTB = 0xff;

}

void InitI2C(void)
{
    INTCONbits.GIE = 0;
    PIR1bits.SSPIF = 0;
    PIR2bits.BCLIF = 0;
    PORTC &= 0xE7;  //clear SCL and SDA
    TRISC |= 0x18;  //SCL and SDA are inputs
    SSPSTAT  = 0x80;
    SSPCON   = 0x3e;    //i2c srave mode (use SSPCONbits.CKP)
    SSPCON2  = 0x00;
    SSPADD = addr;
    INTCONbits.PEIE = 1;
    PIE1bits.SSPIE = 1;
    INTCONbits.GIE = 1;

    

}

void initTMR0(void)
{
    cnt = 0;
    OPTION_REG = 0xC7;
    TMR0 = TMR0TIME;	//10ms timer
    INTCONbits.T0IE = 1;
    INTCONbits.GIE = 1;
}



static void interrupt isr(void)
{
    unsigned int i, stat;
    unsigned char dat;
    unsigned char temp, add;

    /* This code stub shows general interrupt handling.  Note that these
    conditional statements are not handled within 3 seperate if blocks.
    Do not use a seperate if block for each interrupt flag to avoid run
    time errors. */

    //python write word
    //i2c.write_word_data (address, command, data)
    //sequesnce:
    //1: state 1, buf contains address
    //2: state 2, buf contains command
    //3: state 2, first byte
    //4: state 2, second byte
    //state 5 gets called at end of message

    //python to read word from pic
    //i2c.read_word_data (address, command)
    //sequence:
    //1: state 1, buf contains address
    //2: state 2, buf contains command
    //3: state 3, first byte
    //4: state 4, second byte
    //state 5 gets called at end of message

    if (SSPIF == 1) //i2c interupt
    {
        if (SSPOV == 1)
        {
            SSPOV = 0;  //clear overflow
            dat = SSPBUF;
            temp = 30;
        }
        else
        {
            stat = SSPSTAT;
            stat = stat & 0b00101101;
            //find which state we are in
            //state 1 Master just wrote our address
            if ((stat ^ 0b00001001) == 0) //S=1 && RW==0 && DA==0 && BF==1
            {
                /*
                for (i=0; i<BUF_SIZE; ++i) //clear buffer
                    buf[i] = 0;
                buf_index = 0;  //clear buffer index
                */
                add = SSPBUF; //read address so we know later if read or write
                address_flag=1;

            }
            //state 2 Master just wrote data 
            else if ((stat ^ 0b00101001) == 0) //S=1 && RW==0 && DA==1 && BF==1
            {
                dat = SSPBUF;
                if(address_flag==1){
                    i2c_command = dat;
                    address_flag=0;
                }else{
                    if((i2c_command-1)<4){
                        seg_data[i2c_command-1]=dat;
                    }
                    i2c_command = 0;
                }
                
            }
            //state 3 Master want to read, just wrote address
            else if ((stat ^ 0b00001100) == 0) //S=1 && RW==1 && DA==0 && BF==0
            {

                add = SSPBUF; //keep the address
                /*
                buf_index = buf[0];  //index to read
                if (buf_index >= OUT_BUF_SIZE)
                    buf_index = 0;
                */
                WCOL = 0;   //clear write collision flag
                dat = seg_data[0]; //low byte
                SSPBUF = dat & 0x00FF; //data to send
            }
            //state 4 Master want to read, last byte was already data
            else if ((stat ^ 0b00101100) == 0) //S=1 && RW==1 && DA==1 && BF==0
            {


                /*
                if (buf_index >= OUT_BUF_SIZE)
                    buf_index = 0;
                */
                WCOL = 0;   //clear write collision flag
                dat = temp; //outbuf[buf_index] >> 8;
                SSPBUF = dat; //data to send, low byte
                //++buf_index;
            }
            //state 5 Master sends NACK to end message
            else if ((stat ^ 0b00101000) == 0) //S=1 && RW==0 && DA==1 && BF==0
            {
                dat = SSPBUF;
            }
            else //undefined, clear buffer
            {
                dat = SSPBUF;
            }
        }
    CKP = 1; //release the clk
    SSPIF = 0; //clear interupt flag
    }

    if(INTCONbits.T0IF){    //timer0_interrupt
        INTCONbits.T0IF = 0;
        TMR0 = TMR0TIME;
        cnt ++;			//counter
        

    }

}
