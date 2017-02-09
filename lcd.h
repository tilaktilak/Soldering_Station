#ifndef LCD_H
#define LCD_H

// LCD interface (should agree with the diagram above)
//   make sure that the LCD RW pin is connected to GND

//LiquidCrystal lcd(12, 11,     7, 8, 4, 5);
    //LiquidCrystal(rs, enable, d4, d5, d6, d7)  
#define lcd_D7_port     PORTD                   // lcd D7 connection
#define lcd_D7_bit      PORTD5
#define lcd_D7_ddr      DDRD

#define lcd_D6_port     PORTD                   // lcd D6 connection
#define lcd_D6_bit      PORTD4
#define lcd_D6_ddr      DDRD

#define lcd_D5_port     PORTB                   // lcd D5 connection
#define lcd_D5_bit      PORTB0
#define lcd_D5_ddr      DDRB

#define lcd_D4_port     PORTD                   // lcd D4 connection
#define lcd_D4_bit      PORTD7
#define lcd_D4_ddr      DDRD

#define lcd_E_port      PORTB                   // lcd Enable pin
#define lcd_E_bit       PORTB3
#define lcd_E_ddr       DDRB

#define lcd_RS_port     PORTB                   // lcd Register Select pin
#define lcd_RS_bit      PORTB4
#define lcd_RS_ddr      DDRB

// LCD module information
#define lcd_LineOne     0x00                    // start of line 1
#define lcd_LineTwo     0x40                    // start of line 2
//#define   lcd_LineThree   0x14                  // start of line 3 (20x4)
//#define   lcd_lineFour    0x54                  // start of line 4 (20x4)
#define   lcd_LineThree   0x10                  // start of line 3 (16x4)
#define   lcd_lineFour    0x50                  // start of line 4 (16x4)

// LCD instructions
#define lcd_Clear           0b00000001          // replace all characters with ASCII 'space'
#define lcd_Home            0b00000010          // return cursor to first position on first line
#define lcd_EntryMode       0b00000110          // shift cursor from left to right on read/write
#define lcd_DisplayOff      0b00001000          // turn display off
#define lcd_DisplayOn       0b00001100          // display on, cursor off, don't blink character
#define lcd_FunctionReset   0b00110000          // reset the LCD
#define lcd_FunctionSet4bit 0b00101000          // 4-bit data, 2-line display, 5 x 7 font
#define lcd_SetCursor       0b10000000          // set cursor position
void lcd_write_4(uint8_t);
void lcd_write_instruction_4d(uint8_t);
void lcd_write_character_4d(uint8_t);
void lcd_write_string_4d(uint8_t *);
void lcd_init_4d(void);
int hidden_main(void);
void new_line(uint8_t line);
#endif
