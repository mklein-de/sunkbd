#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>       /* PROGMEM */
#include <avr/wdt.h>

#include <util/delay.h>

#include <string.h>

#include "usbdrv/oddebug.h"
#include "usbdrv/usbdrv.h"

#ifdef LCD_DEBUG
#include "lcd.h"
#endif

#define BAUDRATE 1200L

#define DEBUG_LED_BIT   PD7
#define DEBUG_LED_PORT  PORTD

enum KEYBOARD_COMMAND
{
    KBD_RESET     = 0x01,
    KBD_BELL_ON   = 0x02,
    KBD_BELL_OFF  = 0x03,
    KBD_CLICK_ON  = 0x0a,
    KBD_CLICK_OFF = 0x0b,
    KBD_LED       = 0x0e,
    KBD_LAYOUT    = 0x0f
};

enum KEYBOARD_RESPONSE
{
    KBD_IDLE_RESPONSE   = 0x7f,
    KBD_LAYOUT_RESPONSE = 0xfe,
    KBD_RESET_RESPONSE  = 0xff
};


#define KEY_COUNT 6

static union
{
    struct report_s
    {
        uchar modifierMask;
        uchar reserved;
        uchar keys[KEY_COUNT];
    }
    report;

    uchar bytes[sizeof(struct report_s)];
}
reportBuffer;


static uchar LEDState;
static uchar idleRate;           /* in 4 ms units */
static uchar expectReport;
static volatile uchar updateNeeded;
static enum { KEYCLICK_OFF, KEYCLICK_ON, KEYCLICK_CAPS } keyClickMode;

enum { KEYCLICK_FLAG_RESET = 8, KEYCLICK_FLAG_SAVE = 4 };
enum { FEATURE_SET_KEYCLICK = 1, FEATURE_OVERRIDE_KEYCODE, FEATURE_BOOTLOADER };

static uchar protocolVer=1;      /* 0 is the boot protocol, 1 is report protocol */

static uchar suspended;

static void led(uchar onoff)
{
    if (onoff)
        DEBUG_LED_PORT |= _BV(DEBUG_LED_BIT);
    else
        DEBUG_LED_PORT &= ~_BV(DEBUG_LED_BIT);
}

static void hardwareInit(void)
{
    PORTD |= _BV(PD0); /* enable RX pullup */
    DDRD   = _BV(USB_CFG_DMINUS_BIT) | _BV(USB_CFG_DPLUS_BIT) |
        _BV(DEBUG_LED_BIT);

    PORTB = _BV(PB0); /* enable bootload jumper pullup */

    led(protocolVer == 0);

    /* USB Reset by device only required on Watchdog Reset */
    _delay_ms(11);   /* delay >10ms for USB reset */

    DDRD &= ~(_BV(USB_CFG_DMINUS_BIT) | _BV(USB_CFG_DPLUS_BIT));
    /* configure timer 0 for a rate of 12M/(1024 * 256) = 45.78 Hz (~22ms) */
    TCCR0 = 5;      /* timer 0 prescaler: 1024 */
}



PROGMEM const char usbHidReportDescriptor[USB_CFG_HID_REPORT_DESCRIPTOR_LENGTH] = { /* USB report descriptor */
    0x05, 0x01,                    // USAGE_PAGE (Generic Desktop)
    0x09, 0x06,                    // USAGE (Keyboard)
    0xa1, 0x01,                    // COLLECTION (Application)

    0x05, 0x07,                    //   USAGE_PAGE (Keyboard)

    0x19, 0xe0,                    //   USAGE_MINIMUM (Keyboard LeftControl)
    0x29, 0xe7,                    //   USAGE_MAXIMUM (Keyboard Right GUI)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x25, 0x01,                    //   LOGICAL_MAXIMUM (1)
    0x75, 0x01,                    //   REPORT_SIZE (1)
    0x95, 0x08,                    //   REPORT_COUNT (8)
    0x81, 0x02,                    //   INPUT (Data,Var,Abs)

    0x95, 0x01,                    //   REPORT_COUNT (1)
    0x75, 0x08,                    //   REPORT_SIZE (8)
    0x81, 0x03,                    //   INPUT (Cnst,Var,Abs)

    0x95, 0x05,                    //   REPORT_COUNT (5)
    0x75, 0x01,                    //   REPORT_SIZE (1)
    0x05, 0x08,                    //   USAGE_PAGE (LEDs)
    0x19, 0x01,                    //   USAGE_MINIMUM (Num Lock)
    0x29, 0x05,                    //   USAGE_MAXIMUM (Kana)
    0x91, 0x02,                    //   OUTPUT (Data,Var,Abs)

    0x95, 0x01,                    //   REPORT_COUNT (1)
    0x75, 0x03,                    //   REPORT_SIZE (3)
    0x91, 0x03,                    //   OUTPUT (Cnst,Var,Abs)

    0x95, 0x06,                    //   REPORT_COUNT (6)
    0x75, 0x08,                    //   REPORT_SIZE (8)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (1)
    0x26, 0x81, 0x00,              //   LOGICAL_MAXIMUM (129)
    0x05, 0x07,                    //   USAGE_PAGE (Keyboard)
    0x19, 0x00,                    //   USAGE_MINIMUM (Reserved (no event indicated))
    0x2a, 0x81, 0x00,              //   USAGE_MAXIMUM (Keyboard Application)
    0x81, 0x00,                    //   INPUT (Data,Ary,Abs)

    0x95, 0x03,                    //   REPORT_COUNT (3)
    0x75, 0x08,                    //   REPORT_SIZE (8)
    0xb1, 0x02,                    //   FEATURE (Data,Var,Abs)

    0xc0,                          // END_COLLECTION
};

static void updateLEDs()
{
    loop_until_bit_is_set(UCSRA, UDRE);
    UDR = KBD_LED;

    loop_until_bit_is_set(UCSRA, UDRE);
    UDR = LEDState;
}


uchar usbFunctionSetup(uchar data[8])
{
    usbRequest_t *rq = (void *)data;
    if((rq->bmRequestType & USBRQ_TYPE_MASK) == USBRQ_TYPE_CLASS)
    {
        switch(rq->bRequest)
        {
            case USBRQ_HID_GET_REPORT:
                if(rq->wValue.bytes[1] == 3)
                {
                    /* Feature Report */
                    usbMsgPtr = (uchar*)&keyClickMode;
                    return 1;
                }
                else
                {
                    usbMsgPtr = reportBuffer.bytes;
                    return sizeof(reportBuffer);
                }

            case USBRQ_HID_SET_REPORT:
                expectReport = rq->wValue.bytes[1];
                return 0xff; /* Call usbFunctionWrite with data */

            case USBRQ_HID_GET_IDLE:
                usbMsgPtr = &idleRate;
                return 1;

            case USBRQ_HID_SET_IDLE:
                idleRate = rq->wValue.bytes[1];
                break;

            case USBRQ_HID_GET_PROTOCOL:
                usbMsgPtr = &protocolVer;
                return 1;

            case USBRQ_HID_SET_PROTOCOL:
                protocolVer = rq->wValue.bytes[0] & 1;
                led(protocolVer == 0);
                break;

        }
    }
    return 0;
}


static void updateKeyClick()
{
    uchar c;
    switch(keyClickMode)
    {
        case KEYCLICK_ON:
            c = KBD_CLICK_ON;
            break;
        case KEYCLICK_CAPS:
            c = (LEDState & 0x08) ? KBD_CLICK_ON : KBD_CLICK_OFF;
            break;
        default:
            keyClickMode = KEYCLICK_OFF;
        case KEYCLICK_OFF:
            c = KBD_CLICK_OFF;
            break;
    }

    loop_until_bit_is_set(UCSRA, UDRE);
    UDR = c;
}


void uart_init()
{
    unsigned int ubrr;

    ubrr = ((F_CPU + (BAUDRATE<<2)) / (BAUDRATE<<4)) - 1;

    UBRRH = (unsigned char)(ubrr >> 8);
    UBRRL = (unsigned char)ubrr;

    UCSRB = _BV(RXCIE) | _BV(RXEN) | _BV(TXEN);
    UCSRC = _BV(URSEL) | _BV(UCSZ1) | _BV(UCSZ0); /* 8 bit */

}


enum HIDCodes
{
    HID_Reserved, HID_ErrorRollOver, HID_POSTFail, HID_ErrorUndefined,
    HID_A, HID_B, HID_C, HID_D, HID_E, HID_F, HID_G, HID_H, HID_I,
    HID_J, HID_K, HID_L, HID_M, HID_N, HID_O, HID_P, HID_Q, HID_R,
    HID_S, HID_T, HID_U, HID_V, HID_W, HID_X, HID_Y, HID_Z,
    HID_1, HID_2, HID_3, HID_4, HID_5, HID_6, HID_7, HID_8, HID_9,
    HID_0,
    HID_Return, HID_Escape, HID_Backspace, HID_Tab, HID_Space,
    HID_Minus, HID_Equal, HID_LeftBracket, HID_RightBracket,
    HID_Backslash, HID_INTL_Hash, HID_Semicolon, HID_Apostrophe,
    HID_AccentGrave, HID_Comma, HID_Dot, HID_Slash, HID_CapsLock,
    HID_F1, HID_F2, HID_F3, HID_F4, HID_F5, HID_F6, HID_F7, HID_F8,
    HID_F9, HID_F10, HID_F11, HID_F12,
    HID_PrintScreen, HID_ScrollLock, HID_Pause,
    HID_Insert, HID_Home, HID_PageUp, HID_Delete, HID_End, HID_PageDown,
    HID_RightArrow, HID_LeftArrow, HID_DownArrow, HID_UpArrow,
    HID_KP_NumLock, HID_KP_Divide, HID_KP_Multiply, HID_KP_Subtract,
    HID_KP_Add, HID_KP_Enter,
    HID_KP_1, HID_KP_2, HID_KP_3, HID_KP_4, HID_KP_5, HID_KP_6,
    HID_KP_7, HID_KP_8, HID_KP_9, HID_KP_0, HID_KP_Comma,
    HID_INTL_Backslash, HID_Compose, HID_Power, HID_KP_Equal,
    HID_F13, HID_F14, HID_F15, HID_F16, HID_F17, HID_F18, HID_F19,
    HID_F20, HID_F21, HID_F22, HID_F23, HID_F24,
    HID_Execute, HID_Help, HID_Menu, HID_Select, HID_Stop, HID_Again,
    HID_Undo, HID_Cut, HID_Copy, HID_Paste, HID_Find, HID_Mute,
    HID_VolumeUp, HID_VolumeDown,
    HID_LeftControl = 0xe0, HID_LeftShift, HID_LeftAlt, HID_LeftGUI,
    HID_RightControl, HID_RightShift, HID_RightAlt, HID_RightGUI,
};

/*uchar keycode2hidcode_default[128] PROGMEM =*/
static uchar keycode2hidcode[128] =
{
            HID_POSTFail, /* no actual key */
/* 0x01	*/  HID_Stop,
            HID_VolumeDown,
/* 0x03	*/  HID_Again,
            HID_VolumeUp,
/* 0x05	*/  HID_F1,
/* 0x06	*/  HID_F2,
/* 0x07	*/  HID_F10,
/* 0x08	*/  HID_F3,
/* 0x09	*/  HID_F11,
/* 0x0A	*/  HID_F4,
/* 0x0B	*/  HID_F12,
/* 0x0C	*/  HID_F5,
/* 0x0D	*/  HID_RightAlt,
/* 0x0E	*/  HID_F6,
            HID_Reserved,
/* 0x10	*/  HID_F7,
/* 0x11	*/  HID_F8,
/* 0x12	*/  HID_F9,
/* 0x13	*/  HID_LeftAlt,
            HID_UpArrow,
/* 0x15	*/  HID_Pause,
/* 0x16	*/  HID_PrintScreen,
/* 0x17	*/  HID_ScrollLock,
            HID_LeftArrow,
/* 0x19	*/  HID_Reserved, /* XXX Props */
/* 0x1A	*/  HID_Undo,
            HID_DownArrow,
            HID_RightArrow,
/* 0x1D	*/  HID_Escape,
/* 0x1E	*/  HID_1,
/* 0x1F	*/  HID_2,
/* 0x20	*/  HID_3,
/* 0x21	*/  HID_4,
/* 0x22	*/  HID_5,
/* 0x23	*/  HID_6,
/* 0x24	*/  HID_7,
/* 0x25	*/  HID_8,
/* 0x26	*/  HID_9,
/* 0x27	*/  HID_0,
/* 0x28	*/  HID_Minus,
/* 0x29	*/  HID_Equal,
/* 0x2A	*/  HID_AccentGrave,
/* 0x2B	*/  HID_Backspace,
            HID_Insert,
/* 0x2D	*/  HID_Mute,
/* 0x2E	*/  HID_KP_Divide,
/* 0x2F	*/  HID_KP_Multiply,
            HID_Power,
/* 0x31	*/  HID_Reserved, /* XXX Front */
/* 0x32	*/  HID_KP_Comma,
/* 0x33	*/  HID_Copy,
            HID_Home,
/* 0x35	*/  HID_Tab,
/* 0x36	*/  HID_Q,
/* 0x37	*/  HID_W,
/* 0x38	*/  HID_E,
/* 0x39	*/  HID_R,
/* 0x3A	*/  HID_T,
/* 0x3B	*/  HID_Y,
/* 0x3C	*/  HID_U,
/* 0x3D	*/  HID_I,
/* 0x3E	*/  HID_O,
/* 0x3F	*/  HID_P,
/* 0x40	*/  HID_LeftBracket,
/* 0x41	*/  HID_RightBracket,
/* 0x42	*/  HID_Delete,
/* 0x43	*/  HID_Compose,
/* 0x44	*/  HID_KP_7,
/* 0x45	*/  HID_KP_8,
/* 0x46	*/  HID_KP_9,
/* 0x47	*/  HID_KP_Subtract,
/* 0x48	*/  HID_Execute,
/* 0x49	*/  HID_Paste,
            HID_End,
            HID_Reserved,
/* 0x4C	*/  HID_LeftControl,
/* 0x4D	*/  HID_A,
/* 0x4E	*/  HID_S,
/* 0x4F	*/  HID_D,
/* 0x50	*/  HID_F,
/* 0x51	*/  HID_G,
/* 0x52	*/  HID_H,
/* 0x53	*/  HID_J,
/* 0x54	*/  HID_K,
/* 0x55	*/  HID_L,
/* 0x56	*/  HID_Semicolon,
/* 0x57	*/  HID_Apostrophe,
/* 0x58	*/  HID_Backslash,
/* 0x59	*/  HID_Return,
/* 0x5A	*/  HID_KP_Enter,
/* 0x5B	*/  HID_KP_4,
/* 0x5C	*/  HID_KP_5,
/* 0x5D	*/  HID_KP_6,
/* 0x5E	*/  HID_KP_0,
/* 0x5F	*/  HID_Find,
            HID_PageUp,
/* 0x61	*/  HID_Cut,
/* 0x62	*/  HID_KP_NumLock,
/* 0x63	*/  HID_LeftShift,
/* 0x64	*/  HID_Z,
/* 0x65	*/  HID_X,
/* 0x66	*/  HID_C,
/* 0x67	*/  HID_V,
/* 0x68	*/  HID_B,
/* 0x69	*/  HID_N,
/* 0x6A	*/  HID_M,
/* 0x6B	*/  HID_Comma,
/* 0x6C	*/  HID_Dot,
/* 0x6D	*/  HID_Slash,
/* 0x6E	*/  HID_RightShift,
/* 0x6F	*/  HID_Reserved, /* XXX Line Feed */
/* 0x70	*/  HID_KP_1,
/* 0x71	*/  HID_KP_2,
/* 0x72	*/  HID_KP_3,
            HID_Reserved,
            HID_Reserved,
            HID_Reserved,
/* 0x76	*/  HID_Help,
/* 0x77	*/  HID_CapsLock,
/* 0x78	*/  HID_LeftGUI,
/* 0x79	*/  HID_Space,
/* 0x7A	*/  HID_RightGUI,
            HID_PageDown,
            HID_INTL_Backslash,
/* 0x7D	*/  HID_KP_Add,
            HID_Reserved,
            HID_Reserved
};

static uchar pressedKeys[(sizeof(keycode2hidcode)+7)>>3];


static void readEEPROM()
{
    uint8_t buffer[8];
    eeprom_read_block(&buffer, 0, sizeof(buffer));
    keyClickMode = KEYCLICK_CAPS;
    if (!strncmp((char*)buffer, "SuNkBd", 6))
    {
/*        uint16_t keycode_override;*/
/*        uint16_t * p;*/

        keyClickMode = buffer[6];
        buffer[7] = eeprom_read_byte((const uint8_t*)8);

        /* not yet...
        for (p = (uint16_t*)8; ((keycode_override = eeprom_read_word(p)) & 0xff) != 0xff; p++)
        {
            keycode2hidcode[keycode_override & 0x7f] = keycode_override >> 8;
        }
        */
    }

}

static void writeEEPROM()
{
    char buffer[7];
    strcpy(buffer, "SuNkBd");
    buffer[6] = keyClickMode;
    eeprom_write_block(&buffer, 0, sizeof(buffer));
}


uchar usbFunctionWrite(uchar *data, uchar len)
{
    /*
     * HID bits: { NUM LOCK=0, CAPS LOCK=1, SCROLL LOCK=2, COMPOSE=3 }
     * kbd bits: { NUM LOCK=0, COMPOSE=1, SCROLL LOCK=2, CAPS LOCK=3 }
     */
    static uchar HIDModifiers2KeyboardLEDs[] =
    {
        0x00, 0x01, 0x08, 0x09,
        0x04, 0x05, 0x0c, 0x0d,
        0x02, 0x03, 0x0a, 0x0b,
        0x06, 0x07, 0x0e, 0x0f,
    };

    switch(expectReport)
    {
    case 2:
        /* Output Report */
        if (len == 1)
        {
            LEDState = HIDModifiers2KeyboardLEDs[*data & 0x0f];
            updateLEDs();
            if(keyClickMode == KEYCLICK_CAPS)
                updateKeyClick();
        }
        break;

    case 3:
        /* Feature Report */
        if (len == 1 || data[0] == FEATURE_SET_KEYCLICK)
        {
            // len==1 legacy report

            if (len > 1)
                ++data;

            if (*data & KEYCLICK_FLAG_RESET)
                /* read from EEPROM */
                readEEPROM();
            else
                keyClickMode = *data & 0x03;

            updateKeyClick();
            if (*data & KEYCLICK_FLAG_SAVE)
                /* write to EEPROM */
                writeEEPROM();
        }
        else if (len > 1 && data[0] == FEATURE_BOOTLOADER)
        {
            for(;;); // watchdog reset
        }
        else if (len == 4 && data[0] == FEATURE_OVERRIDE_KEYCODE)
        {
            keycode2hidcode[data[2]] = data[3];
        }
    }
    expectReport=0;
    return 1;
}


ISR(USART_RXC_vect)
{
    uchar i, j, k, idx, mask;
    uchar hid_code;

    static uchar state = KBD_IDLE_RESPONSE;
/*    static uchar c;*/

    k = UDR;

/*    if (!(c % 8))*/
/*        lcd_clear();*/
/*    lcd_set_cursor((c % 4)  * 2, c / 4);*/
/*    lcd_hexbyte(k);*/

    if (state == KBD_LAYOUT_RESPONSE)
    {
        state = KBD_IDLE_RESPONSE;
        return;
    }

    if (suspended)
    {
        PORTD &= ~(_BV(USB_CFG_DMINUS_BIT) | _BV(USB_CFG_DPLUS_BIT));
        DDRD  |= _BV(USB_CFG_DMINUS_BIT) | _BV(USB_CFG_DPLUS_BIT);
        PORTD |= _BV(USB_CFG_DPLUS_BIT);
        _delay_ms(5);
        DDRD  &= ~(_BV(USB_CFG_DMINUS_BIT) | _BV(USB_CFG_DPLUS_BIT));
    }

    switch(k)
    {
        case KBD_RESET_RESPONSE:
        case KBD_LAYOUT_RESPONSE:
            state = k;
            break;

        default:
            if (state == KBD_RESET_RESPONSE)
            {
                state = KBD_IDLE_RESPONSE;
                if (k == 4)
                    return; /* no error */

                k = 0; /* -> HID_POSTFail */
            }

            idx = (k & ~_BV(7)) >> 3;
            mask = _BV(k & 0x07);

            if (k & _BV(7))
                /* break */
                pressedKeys[idx] &= ~mask;
            else
                /* make */
                pressedKeys[idx] |= mask;

            for (i = 0; i < sizeof(reportBuffer.bytes); i++)
                reportBuffer.bytes[i] = 0;

            for (i = 0, j = 0; i < sizeof(keycode2hidcode); i++)
            {
                if (pressedKeys[i>>3] & _BV(i & 0x07))
                {
                    hid_code = keycode2hidcode[i];
                    if (hid_code)
                    {
                        if (hid_code < HID_LeftControl)
                        {
                            /* non-modifier key */
                            if (j < KEY_COUNT)
                                reportBuffer.report.keys[j++] = hid_code;
                            else
                                for (j = 0; j < KEY_COUNT; j++)
                                    reportBuffer.report.keys[j] = HID_ErrorRollOver;
                        }
                        else
                        {
                            /* modifier */
                            reportBuffer.report.modifierMask |= _BV(hid_code - HID_LeftControl);
                        }
                    }
                }
            }

            updateNeeded = 1;
    }
}


int main(void)
{
#ifdef LCD_DEBUG
    uchar i;
#endif
    uchar idleCounter = 0;
    uchar prevSofCount = 0;

    wdt_enable(WDTO_2S); /* Enable watchdog timer 2s */
    hardwareInit(); /* Initialize hardware (I/O) */

    odDebugInit();

    usbInit(); /* Initialize USB stack processing */

    uart_init();
#ifdef LCD_DEBUG
    lcd_init();
#endif

    readEEPROM();

    updateLEDs();
    updateKeyClick();

    sei(); /* Enable global interrupts */

    for(;;){  /* Main loop */
        wdt_reset(); /* Reset the watchdog */
        usbPoll(); /* Poll the USB stack */

        if (bit_is_clear(PINB, PB0))
            while(1);

        /* Check timer if we need periodic reports */
        if(TIFR & (1<<TOV0)){
            TIFR = 1<<TOV0; /* Reset flag */
            suspended = (prevSofCount == usbSofCount);
            prevSofCount = usbSofCount;

            if(idleRate != 0){ /* Do we need periodic reports? */
                if(idleCounter > 4){ /* Yes, but not yet */
                    idleCounter -= 5;   /* 22 ms in units of 4 ms */
                }else{ /* Yes, it is time now */
                    updateNeeded = 1;
                    idleCounter = idleRate;
                }
            }
        }

        /* If an update is needed, send the report */
        if(updateNeeded && usbInterruptIsReady()){
            updateNeeded = 0;
            usbSetInterrupt(reportBuffer.bytes, sizeof(reportBuffer));
#ifdef LCD_DEBUG
            for (i = 0; i < sizeof(reportBuffer); i++)
            {
                lcd_set_cursor((i % 4)  * 2, i / 4);
                lcd_hexbyte(reportBuffer.bytes[i]);
            }
#endif
        }
    }
    return 0;
}
