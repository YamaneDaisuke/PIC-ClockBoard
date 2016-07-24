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
/*
 *---------------- REFERENCE ----------------------------------------------------------------------
 * Original Information
 *  http://www.epsondevice.com/docs/qd/ja/DownloadServlet?id=ID000514
 *  http://www5.epsondevice.com/ja/quartz/product/rtc/i2c_bus/rtc8564je_nb.html
 * Sensor board
 *  http://akizukidenshi.com/catalog/g/gI-00233/
 */

#ifndef RTC8564_H
#define RTC8564_H

#include "mbed.h"

// RTC EPSON TOYOCOM RTC8564
//  7bit address = 0b1010001(No other choice)
#define RTC8564ADDR  (0x51 << 1)

#define RTC_Wk_Monday          ((uint8_t)0x01)
#define RTC_Wk_Tuesday         ((uint8_t)0x02)
#define RTC_Wk_Wednesday       ((uint8_t)0x03)
#define RTC_Wk_Thursday        ((uint8_t)0x04)
#define RTC_Wk_Friday          ((uint8_t)0x05)
#define RTC_Wk_Saturday        ((uint8_t)0x06)
#define RTC_Wk_Sunday          ((uint8_t)0x07)

// Register definition
#define RTC8564_REG_CONTL1      0
#define RTC8564_REG_CONTL2      1
#define RTC8564_REG_SEC         2
#define RTC8564_REG_MIN         3
#define RTC8564_REG_HOUR        4
#define RTC8564_REG_DAY         5
#define RTC8564_REG_WDAY        6
#define RTC8564_REG_MON         7
#define RTC8564_REG_YEAR        8
#define RTC8564_REG_ALARM_MIN   9
#define RTC8564_REG_ALARM_HOUR  0xa
#define RTC8564_REG_ALARM_DAY   0xb
#define RTC8564_REG_ALARM_WDAY  0xc
#define RTC8564_REG_CLK_OUT     0xd
#define RTC8564_REG_TMR_CTRL    0xe
#define RTC8564_REG_TIMER       0xf

// Buffer size
#define RTC_BUF_SIZ             (RTC8564_REG_TIMER + 5)

/** Interface for RTC (I2C Interface)  EPSON TOYOCOM RTC8564
 *
 *  Standalone type RTC via I2C interface
 *
 * @code
 * #include "mbed.h"
 *
 * // I2C Communication
 *  RTC8564     rtc8564(dp5,dp27); // RTC(RTC8564) SDA, SCL (Fixed address)
 * // If you connected I2C line not only this device but also other devices,
 * //     you need to declare following method.
 *  I2C         i2c(dp5,dp27);     // SDA, SCL
 *  RTC8564     rtc8564(i2c);      // RTC(RTC8564) (Fixed address)
 *
 * int main() {
 * tm t;
 * time_t seconds;
 * char buf[40];
 *
 *   rtc8564.get_time_rtc(&t);   // read RTC data
 *   seconds = mktime(&t);
 *   strftime(buf, 40, "%I:%M:%S %p (%Y/%m/%d)", localtime(&seconds));
 *   printf("Date: %s\r\n", buf);
 * }
 * @endcode
 */

class RTC8564
{
public:

    typedef struct {    // BCD format
        uint8_t rtc_seconds;
        uint8_t rtc_minutes;
        uint8_t rtc_hours;
        uint8_t rtc_weekday;
        uint8_t rtc_date;
        uint8_t rtc_month;
        uint8_t rtc_year_raw;
        uint16_t rtc_year;
    } rtc_time;

    /** Configure data pin
      * @param data SDA and SCL pins
      */
    RTC8564(PinName p_sda, PinName p_scl);

    /** Configure data pin (with other devices on I2C line)
      * @param I2C previous definition
      */
    RTC8564(I2C& p_i2c);

    /** Read RTC data with Standard C "struct tm" format
      * @param tm (data save area)
      * @return none but all data in tm
      */
    void read_rtc_std(tm *);
    void get_time_rtc(tm *);

    /** Write data to RTC data with Standard C "struct tm" format
      * @param tm (save writing data)
      * @return none but all data in tm
      */
    void write_rtc_std(tm *);
    void set_time_rtc(tm *);

    /** Read one byte from specific register
      * @param register address
      * @return register data
      */
    uint8_t read_reg_byte(uint8_t reg);

    /** Write one byte into specific register
      * @param register address, data
      * @return register saved data
      */
    uint8_t write_reg_byte(uint8_t reg, uint8_t data);

    /** Read RTC data with own format
      * @param tm (data save area)
      * @return none but all data in tm
      */
    void read_rtc_direct(rtc_time *);

    /** Read RTC data with own format
      * @param tm (save writing data)
      * @return none but all data in tm
      */
    void write_rtc_direct(rtc_time *);

    /** Set I2C clock frequency
      * @param freq.
      * @return none
      */
    void frequency(int hz);

protected:
    uint8_t bin2bcd(uint8_t);
    uint8_t bcd2bin(uint8_t);

    I2C _i2c;

private:
    uint8_t RTC8564_addr;
    uint8_t rtc_buf[RTC_BUF_SIZ];   // buffer for RTC
};

#endif      // RTC8564_H
