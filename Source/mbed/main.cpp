#include "mbed.h"
#include "RTC8564.h"
#include "ReceiverIR.h"
#define TIMELAG 32400
#define NUM_SEGMETS 4
#define DEBUG 0 // if DEBUG is 0 , debug mode is disable.
#pragma O3
#pragma Otime


I2C         i2c(dp5,dp27);     // SDA, SCL
RTC8564     rtc8564(i2c);      // RTC(RTC8564) (Fixed address)
ReceiverIR  ir_rx(dp13);
char        PIC_ADDR = 0x94;
char        wait_flag;
char        send_data[4];
char        pre_send_data[4];
struct tm   clockTime;

typedef enum {
    CLOCK = 0,
    TEMPERATURE,
    TIMER,
    STOP,
    ADJUST
} MODE;



int receive(RemoteIR::Format *format, uint8_t *buf, int bufsiz, int timeout = 50)
{
    int cnt = 0;
    while (ir_rx.getState() != ReceiverIR::Received) {
        wait_ms(1);
        cnt++;
        if (timeout < cnt) {
            return -1;
        }
    }
    return ir_rx.getData(format, buf, bufsiz * 8);
}

void send_i2c(char* send_data)
{
    char data[2];
    for(char i = 0; i<NUM_SEGMETS; i++) {
        if(pre_send_data[i]!=send_data[i]) {
            data[0]=i+1;
            data[1]=send_data[i];
            i2c.write(PIC_ADDR , data, 2);
            wait(0.2);
        }
        pre_send_data[i]=send_data[i];
    }
}


void SendTime(void)
{
    //time_t ctTime;
    //ctTime=time(NULL)+32400;
    //struct tm *t = localtime(&ctTime);
    //struct tm *t;

    //printf("loop test\n");
    send_data[0]=clockTime.tm_min%10;
    send_data[1]=clockTime.tm_min/10;
    send_data[2]=clockTime.tm_hour%10;
    send_data[3]=clockTime.tm_hour/10;
    send_i2c(send_data);
    //printf("%d,%d :: %d,%d\n",send_data[3],send_data[2],send_data[1],send_data[0]);
}

void clock_mode(void)
{
    rtc8564.read_rtc_std(&clockTime);
    SendTime();
}

void clock_stop(void)
{
    for(uint8_t i = 0; i < 4; i++)send_data[i] = 0x00;
    send_i2c(send_data);
    wait(2);
    for(uint8_t i = 0; i < 4; i++)send_data[i] = 0xff;
    send_i2c(send_data);
    wait(2);

}

int main()
{
    char mode = CLOCK;
    RemoteIR::Format format;
    uint8_t buf1[4] = {0};
    //tm t;
    //time_t seconds;
    //char buf[40];

    //rtc8564.get_time_rtc(&t);   // read RTC data
    //seconds = mktime(&t);
    //strftime(buf, 40, "%I:%M:%S %p (%Y/%m/%d)", localtime(&seconds));
    //printf("Date: %s\r\n", buf);
    //printf("Start\n");
    while(1) {
        if(mode == CLOCK) {
            clock_mode();
        } else if( mode == ADJUST ) {
            ;//printf("%d:%d\n",clockTime.tm_hour,clockTime.tm_min);
        }
        //wait(0.5);
        // recieve remote IR signal
#define IR_SIGNAL_CLOCK         0xb94601fe  // "1"   button
#define IR_SIGNAL_TEMPERATURE   0xb94602fd  // "2"   button
#define IR_SIGNAL_STOP          0xb94613ec  // "SRC" button
#define IR_SIGNAL_ADJUST        0xb94600ff  // "0"   button
#define IR_SIGNAL_ADDHOUR       0xb94607f8  // "7"   button
#define IR_SIGNAL_ADDTENMINS    0xb94608f7  // "8"   button
#define IR_SIGNAL_ADDMIN        0xb94609f6  // "9"   button
        int bitlength;
        bitlength = receive(&format, buf1, sizeof(buf1), 5);
        if (bitlength == 32) {
            uint32_t data = 0;
            for(uint8_t i = 0; i < 4; i++) data = (data << 8) + buf1[i];
            /*            lcd.cls();
                        lcd.printf("%d:mode = %d\n",bitlength, mode);
                        //for(uint8_t i = 0 ; i < 4; i++)lcd.printf("%02x",buf1[i]);
                        lcd.printf("%08x",data);
            */
            //printf("%08x\n",data);
            if(data == IR_SIGNAL_CLOCK) {
                mode = CLOCK;
            } else if(data == IR_SIGNAL_TEMPERATURE) {
                mode = TEMPERATURE;
            } else if(data == IR_SIGNAL_STOP) {
                if(mode == STOP) {
                    mode = CLOCK;
                    //printf("MOVE\n");
                } else {
                    //printf("STOPING\n");
                    clock_stop();
                    //printf("STOPED\n");
                    mode = STOP;
                }
            } else if(data == IR_SIGNAL_ADJUST) {
                if(mode == ADJUST) {
                    //set time
                    rtc8564.set_time_rtc(&clockTime);
                    mode = CLOCK;
                } else if(mode == CLOCK) {
                    //go adjust mode
                    mode = ADJUST;
                }
            } else if(data == IR_SIGNAL_ADDHOUR) {
                if(mode == ADJUST) {
                    //add an hour
                    clockTime.tm_hour++;
                    clockTime.tm_sec = 0;
                    if(clockTime.tm_hour>=24) {
                        clockTime.tm_hour-=24;
                    }
                    SendTime();
                    wait(0.5);
                }
            } else if(data == IR_SIGNAL_ADDTENMINS) {
                if(mode == ADJUST) {
                    //add ten minute
                    clockTime.tm_min+=10;
                    clockTime.tm_sec = 0;
                    if(clockTime.tm_min>=60) {
                        clockTime.tm_min-=60;
                    }
                    SendTime();
                    wait(0.5);
                }
            } else if(data == IR_SIGNAL_ADDMIN) {
                if(mode == ADJUST) {
                    //add a minute
                    clockTime.tm_min++;
                    clockTime.tm_sec = 0;
                    if(clockTime.tm_min>=60) {
                        clockTime.tm_min-=60;
                    }
                    SendTime();
                    wait(0.5);
                }
            }
        }

    }
}

