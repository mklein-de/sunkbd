void lcd_clear();
void lcd_init();
void lcd_char(unsigned char c);
void lcd_hexnibble(unsigned char c);
void lcd_hexbyte(unsigned char c);
void lcd_string(const char *s);
void lcd_set_cursor(unsigned char x, unsigned char y);
void lcd_decimal_rtl(unsigned int d);
void lcd_write_command(unsigned char c, unsigned int exec_time_us);
