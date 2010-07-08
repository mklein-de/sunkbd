#include <avr/io.h>
#include <util/delay.h>
#include <string.h>

#include "lcd.h"

#define PORTNAME      C

#define CONCAT(a,b)     a ## b

#define INPORT(name)    CONCAT(PIN, name)
#define OUTPORT(name)   CONCAT(PORT, name)
#define DDRPORT(name)   CONCAT(DDR, name)

#define LCD_PIN         INPORT(PORTNAME)
#define LCD_PORT        OUTPORT(PORTNAME)
#define LCD_DDR         DDRPORT(PORTNAME)

#define LCD_BITS        (_BV(0)|_BV(1)|_BV(2)|_BV(3))
#define LCD_RS          (_BV(5))
#define LCD_EN          (_BV(4))

static void lcd_write_nibble(unsigned char c, unsigned int exec_time_us)
{
    LCD_PORT = (LCD_PORT & ~(LCD_BITS|LCD_RS|LCD_EN)) | c;
    LCD_PORT ^= LCD_EN;
    _delay_us(1);
    LCD_PORT ^= LCD_EN;
    while(exec_time_us--)
        _delay_us(1);
}

void lcd_write_command(unsigned char c, unsigned int exec_time_us)
{
    lcd_write_nibble(c>>4, 0);
    lcd_write_nibble(c&0x0f, exec_time_us);
}

void lcd_char(unsigned char c)
{
    lcd_write_nibble(LCD_RS|(c>>4), 0);
    lcd_write_nibble(LCD_RS|(c&0x0f), 41);
}

void lcd_clear()
{
    lcd_write_command(0x01, 1640);
}

void lcd_init()
{
    LCD_DDR = LCD_BITS|LCD_RS|LCD_EN;
    DDRD |= _BV(PD5);

    _delay_ms(15);
    LCD_PORT &= ~(LCD_BITS|LCD_RS|LCD_EN);

    /* Initializing by Instruction */
    _delay_ms(15);
    lcd_write_nibble(0x03, 5000);
    lcd_write_nibble(0x03, 100);
    lcd_write_nibble(0x03, 37);

    /* Function set: data length 4 bit */
    lcd_write_nibble(0x02, 37);

    /* Function set: data length 4 bit, 2 lines, 5x8 font */
    lcd_write_command(0x28, 37);

    /* Display on/off control: display on, cursor off, no blinking */
    lcd_write_command(0x0c, 37);

    /* Entry mode set: increment, no shift */
    lcd_write_command(0x06, 37);

    lcd_clear();
}

void lcd_string(const char *s)
{
    if (s) while(*s) lcd_char((unsigned char)*(s++));
}

void lcd_hexnibble(unsigned char c)
{
    c &= 0x0f;
    lcd_char((c >= 10) ? c + 'A' - 10 : c + '0');
}

void lcd_hexbyte(unsigned char c)
{
    lcd_hexnibble(c >> 4);
    lcd_hexnibble(c & 0x0f);
}

void lcd_decimal_rtl(unsigned int d)
{
    /* Entry mode set: decrement, no shift */
    lcd_write_command(0x04, 37);

    if (!d)
        lcd_char('0');
    else
        for(;d;d/=10)
            lcd_char((d % 10) + '0');

    /* Entry mode set: increment, no shift */
    lcd_write_command(0x06, 37);
}

void lcd_set_cursor(unsigned char x, unsigned char y)
{
    lcd_write_command(0x80 + 0x40 * y + x, 37);
}
