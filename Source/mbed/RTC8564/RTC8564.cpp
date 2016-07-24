/*
 * mbed library program
 *  Control RTC-8564NB Real Time Clock Module
 *  EPSON TOYOCOM
 *
 * Copyright (c) 2015 Kenji Arai / JH1PJL
 *  http://www.page.sannet.ne.jp/kenjia/index.html
 *  http://mbed.org/users/kenjiArai/
 *      Created: Feburary   1st, 2015
 *      Revised: March     14th, 2015
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
 * INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE
 * AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
 * DAMAGES OR OTHER  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#include "mbed.h"
#include "RTC8564.h"

RTC8564::RTC8564 (PinName p_sda, PinName p_scl) : _i2c(p_sda, p_scl)
{
    RTC8564_addr = RTC8564ADDR;
}

RTC8564::RTC8564 (I2C& p_i2c) : _i2c(p_i2c)
{
    RTC8564_addr = RTC8564ADDR;
}

/////////////// Read RTC data /////////////////////////////
void RTC8564::get_time_rtc (tm *t)
{
    read_rtc_std(t);
}

void RTC8564::read_rtc_std (tm *t)
{
    rtc_time time;
    read_rtc_direct(&time);
    t->tm_sec  = time.rtc_seconds;
    t->tm_min  = time.rtc_minutes;
    t->tm_hour = time.rtc_hours;
    t->tm_mday = time.rtc_date;
    if ( time.rtc_weekday == RTC_Wk_Sunday) {
        t->tm_wday = 0; // Sun is not 7 but 0
    } else {
        t->tm_wday = time.rtc_weekday;
    }
    
    t->tm_mon  = time.rtc_month - 1;
    t->tm_year = time.rtc_year_raw + 100;
    t->tm_isdst= 0;
}

/////////////// Write data to RTC /////////////////////////
void RTC8564::set_time_rtc (tm *t)
{
    write_rtc_std(t);
}

void RTC8564::write_rtc_std (tm *t)
{
    rtc_time time;

    time.rtc_seconds  = t->tm_sec;
    time.rtc_minutes  = t->tm_min;
    time.rtc_hours    = t->tm_hour;
    time.rtc_date     = t->tm_mday;
    if ( t->tm_wday == 0) {
        time.rtc_weekday = RTC_Wk_Sunday;
    } else {
        time.rtc_weekday = t->tm_wday;
    }
    time.rtc_month    = t->tm_mon + 1;
    time.rtc_year_raw = t->tm_year - 100;
    write_rtc_direct(&time);
}

/////////////// Read/Write specific register //////////////
uint8_t RTC8564::read_reg_byte(uint8_t reg)
{
    uint8_t dt[2];

    dt[0] = reg;
    _i2c.write((int)RTC8564_addr, (char *)dt, 1, true);
    _i2c.read((int)RTC8564_addr, (char *)dt, 1, false);
    return dt[0];
}

uint8_t RTC8564::write_reg_byte(uint8_t reg, uint8_t data)
{
    uint8_t dt[2];

    dt[0] = reg;
    dt[1] = data;
    _i2c.write((int)RTC8564_addr, (char *)dt, 2, false);
    dt[1] = read_reg_byte(reg);
    return dt[1];
}

/////////////// I2C Freq. /////////////////////////////////
void RTC8564::frequency(int hz)
{
    _i2c.frequency(hz);
}

/////////////// Read/Write RTC another format /////////////
void RTC8564::read_rtc_direct (rtc_time *tm)
{
    uint8_t dt;

    dt = RTC8564_REG_CONTL1;
    _i2c.write((int)RTC8564_addr, (char *)dt, 1, true);
    _i2c.read((int)RTC8564_addr, (char *)rtc_buf, RTC8564_REG_TIMER + 1, false);
    tm->rtc_seconds = bcd2bin(rtc_buf[RTC8564_REG_SEC]  & 0x7f);
    tm->rtc_minutes = bcd2bin(rtc_buf[RTC8564_REG_MIN]  & 0x7f);
    tm->rtc_hours   = bcd2bin(rtc_buf[RTC8564_REG_HOUR] & 0x3f);
    tm->rtc_date    = bcd2bin(rtc_buf[RTC8564_REG_DAY]  & 0x3f);
    tm->rtc_weekday = rtc_buf[RTC8564_REG_WDAY] & 0x07;
    tm->rtc_month   = bcd2bin(rtc_buf[RTC8564_REG_MON]  & 0x1f);
    tm->rtc_year_raw= bcd2bin(rtc_buf[RTC8564_REG_YEAR]);
    tm->rtc_year = tm->rtc_year_raw + 100 + 1900;

}

void RTC8564::write_rtc_direct (rtc_time *tm)
{
    uint8_t dt[2];

    rtc_buf[RTC8564_REG_YEAR - 1] = bin2bcd(tm->rtc_year_raw);
    rtc_buf[RTC8564_REG_MON  - 1] = bin2bcd(tm->rtc_month);
    rtc_buf[RTC8564_REG_WDAY - 1] = (tm->rtc_weekday & 0x07);
    rtc_buf[RTC8564_REG_DAY  - 1] = bin2bcd(tm->rtc_date);
    rtc_buf[RTC8564_REG_HOUR - 1] = bin2bcd(tm->rtc_hours);
    rtc_buf[RTC8564_REG_MIN  - 1] = bin2bcd(tm->rtc_minutes);
    rtc_buf[RTC8564_REG_SEC  - 1] = bin2bcd(tm->rtc_seconds);
    dt[0] = RTC8564_REG_CONTL1;
    dt[1] = 0x20;
    _i2c.write((int)RTC8564_addr, (char *)dt, 2, false);
    rtc_buf[0] = RTC8564_REG_SEC;
    _i2c.write((int)RTC8564_addr, (char *)rtc_buf, 8, false);
    dt[0] = RTC8564_REG_CONTL1;
    dt[1] = 0x00;
    _i2c.write((int)RTC8564_addr, (char *)dt, 2, false);
}

uint8_t RTC8564::bin2bcd (uint8_t dt)
{
    uint8_t bcdhigh = 0;

    while (dt >= 10) {
        bcdhigh++;
        dt -= 10;
    }
    return  ((uint8_t)(bcdhigh << 4) | dt);
}

uint8_t RTC8564::bcd2bin (uint8_t dt)
{
    uint8_t tmp = 0;

    tmp = ((uint8_t)(dt & (uint8_t)0xf0) >> (uint8_t)0x4) * 10;
    return (tmp + (dt & (uint8_t)0x0f));
}
