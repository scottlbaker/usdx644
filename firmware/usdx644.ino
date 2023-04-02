
// ============================================================================
// usdx644.ino
//
// Much of this code was inspired by the uSDX project by Guido PE1NNZ
// The uSDX project which used an Atmega328 processor
// This project uses the Atmega644 processor which is similar to the 328
// but which has more RAM and Flash memory
//
// Permission is hereby granted, free of charge, to any person obtaining
// a copy of this software and associated documentation files (the "Software"),
// to deal in the Software without restriction, including without limitation
// the rights to use, copy, modify, merge, publish, distribute, sublicense,
// and or sell copies of the Software, and to permit persons to whom the
// Software is furnished to do so, subject to the following conditions: The
// above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software. THE SOFTWARE IS PROVIDED
// "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
// NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
// PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
// HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN
// AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
// CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//
// Arduino IDE settings:
// board: Atmega644 (MightyCore)
// clock: external 16MHz
// BOD: BOD 2.7V
// eeprom: eeprom retained
// variant: 644P 644PA
// pinout: standard
// bootloader: no bootloader
// programmer: AVRISP mkII (MightyCore)
//
// ============================================================================

// #define DEBUG  1           // uncomment for debugging

#define VERSION   "1.00a"
#define TIMESTAMP "6/27/2022"

#define SI5351_ADDR   0x60    // Si5351 I2C address
#define IOEXP16_ADDR  0x20    // band switching address

#define QSDQ      PIN_PA0     // Q
#define QSDI      PIN_PA1     // I
#define MIC       PIN_PA2     // microphone
#define XP3       PIN_PA3     // extra pin (not used)
#define XP2       PIN_PA4     // extra pin (not used)
#define VBAT      PIN_PA5     // battery voltage
#define VFWD      PIN_PA6     // SWR fwd voltage
#define VREV      PIN_PA7     // SWR rev voltage

#define BLINKY    PIN_PB0     // diag LED
#define XP4       PIN_PB1     // extra pin (not used)
#define SDA0      PIN_PB2     // I2C bus #0 (Si5351)
#define SCL0      PIN_PB3     // I2C bus #0 (Si5351)
#define RXEN      PIN_PB4     // Receive control
#define MOSI      PIN_PB5     // SPI programming pin
#define MISO      PIN_PB6     // SPI programming pin
#define SCK       PIN_PB7     // SPI programming pin

#define SCL1      PIN_PC0     // I2C bus #1 (OLED)
#define SDA1      PIN_PC1     // I2C bus #1 (OLED)
#define UI2       PIN_PC2     // user button (exit)
#define UI1       PIN_PC3     // user button (menu)
#define UI3       PIN_PC4     // user button (encoder)
#define ROTA      PIN_PC5     // rotary encoder A
#define ROTB      PIN_PC6     // rotary encoder B
#define XPTT      PIN_PC7     // external amp control

#define RXD       PIN_PD0     // serial port Rx
#define TXD       PIN_PD1     // serial port Tx
#define CSS       PIN_PD2     // SPI select
#define DIT       PIN_PD3     // dit/ptt
#define PTT       PIN_PD3     // dit/ptt
#define TXPWM     PIN_PD4     // transmit PWM
#define SIDETONE  PIN_PD5     // sidetone
#define DAH       PIN_PD6     // dah
#define XP1       PIN_PD7     // extra pin (not used)

// uncomment to enable

//#define CAT             1  // CAT control
//#define CAT_EXT         1  // CAT extensions
//#define CAT_STREAMING   1  // CAT streaming
//#define LPF_NOLATCH     1  // non-latching relay support
//#define QUAD            1  // improve tx quality?

#define SWR_METER         1  // SWR meter support

// used by switch_rxtx()
#define NOSHOW  0
#define SHOW    1

// generic
#define OFF   0
#define ON    1
#define NO    0
#define YES   1
#define LAST  1
#define DOWN  0
#define UP    1
#define NONE  0

#define ONE_SECOND   1000
#define TWO_SECONDS  2000

#define TIC   0
#define TOC   1

// used by decoder
#define DITX  0
#define DAHX  1

// used by setCursor
#define NOSCALE  0
#define SCALE    1

// CW decoder modes
#define XMIT  1
#define RECV  2
#define XTRA  3

// OLED display rows
#define ROW1  0
#define ROW2  1
#define ROW3  2
#define ROW4  3

// OLED display columns
#define STEPCOL   2
#define TWPMCOL   7
#define RWPMCOL  13
#define DBMCOL   12
#define SVALCOL  13
#define RMODECOL 13

// OLED display columns
#define MAXCOL  15

// agc settings
#define FAST  1
#define SLOW  2

// S-meter modes
#define DBM        1
#define SVAL       2
#define RXWPM      3

// SWR-meter modes
#define WATTS      1
#define VOLTS      2

// keyer modes
#define STRAIGHT   0   // straight key
#define IAMBIC_A   1   // Iambic A
#define IAMBIC_B   2   // Iambic B

// vfo settings
#define VFOA  0
#define VFOB  1

// SSB filters
#define FULL       0   // no bandwidth filtering
#define BW3000     1   // 3000Hz low-pass filter
#define BW2400     2   // 2400Hz low-pass filter
#define BW1800     3   // 1800Hz low-pass filter

// CW filters
#define BW500      1   // 500Hz band-pass filter
#define BW250      2   // 250Hz band-pass filter
#define BW100      3   // 100Hz band-pass filter
#define BW50       4   //  50Hz band-pass filter

// supported bands
#define B10M       0   // 10M band
#define B15M       1   // 15M band
#define B17M       2   // 17M band
#define B20M       3   // 20M band
#define B30M       4   // 30M band
#define B40M       5   // 40M band
#define B60M       6   // 60M band
#define B80M       7   // 80M band

// radio modes
#define  LSB    0
#define  USB    1
#define  CW     2
#define  FM     3
#define  AM     4

// CW decoder modes
#define  CWRX   1
#define  CWTX   2
#define  RXTX   3

// tuning step sizes
#define STEP_1M      0
#define STEP_100k    1
#define STEP_10k     2
#define STEP_1k      3
#define STEP_100     4
#define STEP_10      5
#define STEP_1       6

// menu variables
uint8_t volume     = 11;
uint8_t tonevol    = 15;
uint8_t radiomode  = USB;
uint8_t bwfilter   = FULL;
uint8_t cwfilter   = FULL;
uint8_t bandsel    = B20M;
uint8_t stepsize   = STEP_1k;
uint8_t vfosel     = VFOA;
uint8_t ritmode    = OFF;
uint8_t smeter     = RXWPM;
uint8_t swrmeter   = OFF;
uint8_t vox        = OFF;
uint8_t voxlevel   = 4;
uint8_t agc        = SLOW;
uint8_t nrlevel    = 2;
uint8_t attmic     = 0;    // mic attenuation
uint8_t attrx      = 0;    // Rx  attenuation
uint8_t att2       = 2;    // shift data to avoid overflow
uint8_t drive      = 4;
uint8_t txdelay    = 10;
uint8_t cwdec      = CWRX;
uint8_t semiqsk    = OFF;
uint8_t keyermode  = IAMBIC_A;
uint8_t keyerspeed = 25;
uint8_t keyswap    = OFF;
uint8_t practice   = ON;   // practice mode (no Tx)
uint8_t pwm_min    = 0;    // Tx PWM tuning
uint8_t pwm_max    = 128;  // Tx PWM tuning
uint8_t iqphase    = 90;
uint8_t notused    = 0;

uint8_t ditkey = DIT;
uint8_t dahkey = DAH;
uint8_t quad = 0;
uint8_t tx  = 0;
uint8_t acc_init = 0;
uint8_t skip_dsp = NO;
uint8_t cwdec_save = CWRX;
uint8_t practice_save = ON;
uint32_t t0;

// for display blank/timeout
#define ONE_MINUTE 60000
uint8_t display  = ON;
uint8_t xtimeout = 5;       // blank after 5 minutes
uint8_t xtimer   = 0;
uint32_t xdt;

uint32_t fxtal = 27006137;  // Si5351 calibrated frequency

#if (ARDUINO < 10810)
   #error "Arduino IDE 1.8.10 or newer is required"
#endif
#if !(defined(ARDUINO_ARCH_AVR))
   #error "Unsupported architecture in Arduino IDE"
#endif
#if (F_CPU != 16000000)
   #error "Arduino IDE must specify 16MHz clock"
#endif

#undef  F_CPU
#define F_CPU 20007000  // actual frequency of 20MHz crystal
#define F_MCU 20000000  // 20MHz crystal

#include <avr/sleep.h>
#include <avr/wdt.h>
#include <EEPROMex.h>
#include "./include/font.h"
#include "./include/user.h"

// prototype defs
int8_t menuAction(uint8_t id);
void goertzel(uint8_t show);
void switch_rxtx(uint8_t tx_enable, uint8_t show);
void run_diags();
void saveAll();

inline void vox_process(uint8_t trigger);
inline int16_t arctan3(int16_t q, int16_t i);
inline int16_t ssb(int16_t in);
inline void minsky_init();
inline void process_minsky();
inline void insertchar();
inline void spinchar();
inline void gbuffer(int16_t ac);
inline void rxcw_decode(uint8_t show);
inline int16_t process_agc_fast(int16_t in);
inline int16_t process_agc(int16_t in);
inline int16_t process_nr(int16_t ac);
inline int16_t iirfilter(int16_t ac);
inline int16_t _arctan3(int16_t q, int16_t i);
inline int16_t slow_dsp(int16_t ac);
inline int16_t sdr_rx_common_q();
inline int16_t sdr_rx_common_i();

// button definitions
#define EXIT       0x10  // exit
#define MENU       0x20  // menu
#define ENCO       0x40  // encoder

// button events
#define CLICK      0x01  // click
#define LONG       0x02  // long press
#define DLP        0x04  // double-long press
#define SLP        0x08  // super-long press

// button times (ms)
#define MS_LONG     300  // long
#define MS_DLP      600  // double-long
#define MS_SLP     1200  // super-long

// menu IDs
enum menu_t {
  VOLUME, TONEVOL, RADIOMODE, BWFILTER, CWFILTER,
  NRLEVEL, BAND, STEP, PRACTICE,
  CWDEC, KEYERMODE, KEYERWPM, KEYSWAP, SEMIQSK,
  CWMSG1, CWMSG2, CWMSG3, CWMSG4, CWMSG5, CWMSG6,
  VFOSEL, SMETER, SWRMETER, VOX, VOXLEVEL, AGC, 
  ATTMIC, ATTRX, ATT2,
  DRIVE, TXDELAY, PWM_MIN, PWM_MAX, IQPHASE, REFCAL,
  CALLSIGN, NAME, ANT, RIG, PWR, QTH, FREQMEM,
  DIAG, SAVE, RIT, CWCAL, BATT, SWVER, SHDN,
  BLANK, N_PARAMS };

// user button macros
#define EXIT_PRESSED  !digitalRead(UI1)
#define MENU_PRESSED  !digitalRead(UI2)
#define ENCO_PRESSED  !digitalRead(UI3)
#define BUTTON_PRESSED (EXIT_PRESSED||MENU_PRESSED||ENCO_PRESSED)

#define DIT_PRESSED  !digitalRead(ditkey)
#define DAH_PRESSED  !digitalRead(dahkey)
#define DIT_OR_DAH   (DIT_PRESSED || DAH_PRESSED)

// menumode definitions
#define NOT_IN_MENU   0
#define SELECT_MENU   1
#define SELECT_VALUE  2
#define EDIT_STRING   3

uint8_t menumode = NOT_IN_MENU;  // menu state variable
int8_t  menu     = VOLUME;       // currently selected menu item

#ifdef CAT_EXT
uint8_t cat_key = 0;
// reads pin or (via CAT) artificially overriden pins
uint8_t _digitalRead(uint8_t pin) {
  serialEvent();  // allows CAT update
  if (cat_key) {
    switch (pin) {
      case UI1:
        return (~cat_key&0x01);
      case UI2:
        return (~cat_key&0x02);
      case UI3:
        return (~cat_key&0x04);
      case DIT:
        return (~cat_key&0x10);
      case DAH:
        return (~cat_key&0x20);
      default:
        return (0);
    }
  }
  return digitalRead(pin);
}
#else
#define _digitalRead(x) digitalRead(x)
#endif // CAT_EXT

// keyerinfo bit definitions
#define DIT_REG     0x01     // dit pressed
#define DAH_REG     0x02     // dah pressed
#define KEY_REG     0x03     // dit or dah pressed
#define WAS_DIT     0x04     // last key was dit
#define BOTH_REG    0x08     // both dit/dah pressed

// keyer state machine
#define KEY_IDLE    0
#define CHK_KEY     1
#define KEY_WAIT    2
#define IDD_WAIT    3
#define LTR_GAP     4
#define WORD_GAP    5

uint8_t  keyerstate = KEY_IDLE;
uint8_t  keyerinfo  = 0;

uint16_t dittime;     // dit timing
uint16_t dahtime;     // dah timing
uint16_t lettergap;   // letter-gap timing
uint16_t wordgap;     // word-gap   timing

#define GOTKEY  (keyerinfo & KEY_REG)
#define NOKEY   !GOTKEY
#define GOTBOTH  GOTKEY == KEY_REG
#define NOTBOTH  GOTKEY != KEY_REG

// interruptable delay
void wait_ms(uint16_t dly) {
  uint32_t curTime = millis();
  uint32_t endTime = curTime + dly;
  while(curTime < endTime) {
    delayMicroseconds(100);
    wdt_reset();
    curTime = millis();
  }
}

// read and debounce paddles
void read_paddles() {
  uint8_t ditv1 = DIT_PRESSED;
  uint8_t dahv1 = DAH_PRESSED;
  if (ditv1 || dahv1) {
    wait_ms(1);
    uint8_t ditv2 = DIT_PRESSED;
    uint8_t dahv2 = DAH_PRESSED;
    if (ditv1 && ditv2) {
      keyerinfo |= DIT_REG;
    }
    if (dahv1 && dahv2) {
      keyerinfo |= DAH_REG;
    }
    if (GOTBOTH) keyerinfo |= BOTH_REG;
  }
}

// Secondary I2C bus used by OLED
class I2C_ {
public:
  // initialize
  void init() {
    TWBR = 12;  // set SCL delay
    TWSR = 0;   // set prescaler to 1
    TWCR = (1<<TWEN);  // enable
  }
  // start of transmission
  void start(uint8_t address) {
    // send start condition
    TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);
    while (!(TWCR & (1<<TWINT)));
    // send address
    TWDR = address<<1;
    TWCR = (1<<TWINT) | (1<<TWEN);
    while (!(TWCR & (1<<TWINT)));
  }
  // write data
  void write(uint8_t data) {
    TWDR = data;
    TWCR = (1<<TWINT) | (1<<TWEN);
    while (!(TWCR & (1<<TWINT)));
  }
  // end of transaction
  void stop() {
    TWCR = (1<<TWINT) | (1<<TWSTO) | (1<<TWEN);
    while (TWCR & (1<<TWSTO));
  }
};
I2C_ Wire;

// SSD1306 OLED initialization sequence
const uint8_t oled_init_sequence [] = {
  0xD5, 0x80,   // set display clock divide ratio
  0xA8, 0x3F,   // Set multiplex ratio to 1:64
  0xD3, 0x00,   // set display offset = 0
  0x40,         // set display start line address
  0x8D, 0x14,   // set charge pump, internal VCC
  0x20, 0x02,   // set memory addressing
  0xA4,         // output RAM to display
  0xA1,         // set segment re-map
  0xC8,         // set COM output scan direction
  0xDA, 0x12,   // Set com pins hardware configuration
  0x81, 0x80,   // set contrast control register
  0xDB, 0x40,   // set vcomh
  0xD9, 0xF1,   // 0xF1=brighter
  0xB0,         // set page address (0-7)
  0xA6,         // set display mode to normal
  0xAF,         // display ON
};

// SSD1306 OLED routines
class OLEDDevice: public Print {
public:
  #define OLED_ADDR    0x3C
  #define OLED_PAGES   4
  #define OLED_COMMAND 0x00
  #define OLED_DATA    0x40
  #define OLED_FRAME   0xB0
  #define NODISPLAY    0xAE
  #define ONDISPLAY    0xAF

  uint8_t oledX = 0;
  uint8_t oledY = 0;
  uint8_t wrap = OFF;

  // send a command
  void sendcmd(uint8_t b) {
    Wire.start(OLED_ADDR);
    Wire.write(OLED_COMMAND);
    Wire.write(b);
    Wire.stop();
  }

  // send data
  void senddata(uint8_t data) {
    Wire.start(OLED_ADDR);
    Wire.write(OLED_DATA);
    Wire.write(data);
    Wire.stop();
  }

  // turn off the display
  void noDisplay() { sendcmd(NODISPLAY); }

  // turn on the display
  void onDisplay() { sendcmd(ONDISPLAY); }

  // set the cursor position to row/col
  void setCursor(uint8_t col, uint8_t row, uint8_t scale=1) {
    if (scale) {oledX = col*FONT_W; oledY = row*FONT_H;}
    else {oledX = col; oledY = row;}
    Wire.start(OLED_ADDR);
    Wire.write(OLED_COMMAND);
    Wire.write(OLED_FRAME | (oledY & 0x07));
    uint8_t _oledX = oledX;
    Wire.write(0x10 | ((_oledX & 0xf0) >> 4));
    Wire.write(_oledX & 0x0f);
    Wire.stop();
  }

  // goto the next line
  void newLine() {
    oledY+=FONT_H;
    if (oledY > OLED_PAGES - FONT_H) {
      oledY = OLED_PAGES - FONT_H;
    }
    setCursor(0, oledY, NOSCALE);
  }

  // clear to end of line
  void clr2eol() {
    for (uint8_t x=oledX; x<128; x++) senddata(0);
    setCursor(oledX, oledY+1, 0);
    for (uint8_t x=oledX; x<128; x++) senddata(0);
  }

  // clear a line
  void clrLine(uint8_t row) {
    setCursor(0, row);
    for (uint8_t x=0; x<128; x++) senddata(0);
    setCursor(0, oledY+1, 0);
    for (uint8_t x=0; x<128; x++) senddata(0);
    setCursor(0, row);
  }

  // clear the screen
  void clrScreen() {
    for (uint8_t row=0; row<4; row++) clrLine(row);
  }

  // clear rows 2 and 3
  void clrR23() {
    for (uint8_t row=1; row<3; row++) clrLine(row);
    setCursor(0, 3);
  }

  // clear rows 2, 3, and 4
  void clrR234() {
    for (uint8_t row=1; row<4; row++) clrLine(row);
    setCursor(0, 3);
  }

  void begin(uint8_t cols, uint8_t rows, uint8_t charsize = 0) {
    Wire.start(OLED_ADDR);
    Wire.write(OLED_COMMAND);
    for (uint8_t i=0; i < sizeof(oled_init_sequence); i++) {
      Wire.write(oled_init_sequence[i]);
    }
    Wire.stop();
    delayMicroseconds(100);
    clrScreen();
  }

  size_t write(byte c) {
    if ((c == '\n') || (oledX > (128 - FONT_W))) return 1;
    c = ((c < 9) ? (c + '~') : c) - ' ';
    uint16_t offset = ((uint16_t)c) * FONT_W/(FONT_STRETCHH+1) * FONT_H;
    uint8_t line = FONT_H;
    do {
      if (FONT_STRETCHV) offset = ((uint16_t)c) * FONT_W/(FONT_STRETCHH+1) * FONT_H/(2*FONT_STRETCHV);
      Wire.start(OLED_ADDR);
      Wire.write(OLED_DATA);
      for (uint8_t i=0; i < (FONT_W/(FONT_STRETCHH+1)); i++) {
        uint8_t b = pgm_read_byte(&(font[offset++]));
        if (FONT_STRETCHV) {
          uint8_t b2 = 0;
          if (line > 1) for (uint8_t i=0; i<4; i++)
            b2 |=/* ! */(b & (1<<i)) ? (1<<(i*2)) | (1<<((i*2)+1)): 0x00;
          else for (uint8_t i=0; i<4; i++)
            b2 |=/* ! */(b & (1<<(i+4))) ? (1<<(i*2)) | (1<<((i*2)+1)): 0x00;
          Wire.write(b2);
          if (FONT_STRETCHH) Wire.write(b2);
        } else { Wire.write(b); if (FONT_STRETCHH) Wire.write(b); }
      }
      Wire.stop();
      if (FONT_H == 1) {
        oledX+=FONT_W;
      }
      else {
        if (line > 1) {
          setCursor(oledX, oledY + 1, NOSCALE);
        }
        else {
          setCursor(oledX + FONT_W, oledY - (FONT_H - 1), NOSCALE);
        }
      }
    }
    while (--line);
    return 1;
  }
};

// this class spoofs display contents and cursor state
template<class parent>class Display : public parent {
public:

#ifdef CAT_EXT
  uint8_t x, y;
  uint8_t curs;
  char text[2*16+1];
  Display() : parent() { clear(); };

  size_t write(uint8_t b) {
    if ((x<16) && (y<2)) {
      text[y*16+x] = ((b < 9) ? "> :*#AB"[b-1] : b);
      x++;
    }
    return parent::write(b);
  }

  void setCursor(uint8_t _x, uint8_t _y) { x = _x; y = _y; parent::setCursor(_x, _y); }

  void clear() {
    for (uint8_t i=0; i<32; i++) text[i] = ' ' ;
    text[32] = '\0';
    x = 0;
    y = 0;
  }
#endif // CAT_EXT
};

Display<OLEDDevice> oled;

volatile int8_t encoder_val = 0;
uint8_t  encoder_state;
uint8_t  ui_lock = OFF;  // lock the UI buttons

// rotary encoder pin change interrupt handler
ISR(PCINT2_vect) {
  if (ui_lock) return;  // check if UI is locked
  encoder_state = (encoder_state << 4) | (digitalRead(ROTB) << 1) | digitalRead(ROTA);
  switch (encoder_state) {
    case 0x23:  encoder_val++; break;
    case 0x32:  encoder_val--; break;
    default: break;
  }
}

// rotary encoder init
void encoder_init() {
  // enable pin change interrupts on PortC pins 5,6
  PCMSK2 = 0x60;  // PCINT22 | PCINT21
  PCICR  = 0x04;  // PCIE2
  encoder_state = (digitalRead(ROTB) << 1) | digitalRead(ROTA);
  interrupts();
}

// Primary I2C bus used by Si5351
class I2C {
public:

  #define I2C_DELAY   4    // I2C Speed (2=939kb/s too fast!!)
  #define I2C_DDR  DDRB
  #define I2C_PIN  PINB
  #define I2C_PORT PORTB
  #define I2C_SDA (1 << 2) // PB2
  #define I2C_SCL (1 << 3) // PB3
  #define DELAY(n) for (uint8_t i=0; i<n; i++) asm("nop");
  #define I2C_SDA_GET() I2C_PIN & I2C_SDA
  #define I2C_SCL_GET() I2C_PIN & I2C_SCL
  #define I2C_SDA_HI() I2C_DDR &= ~I2C_SDA;
  #define I2C_SDA_LO() I2C_DDR |=  I2C_SDA;
  #define I2C_SCL_HI() I2C_DDR &= ~I2C_SCL; DELAY(I2C_DELAY);
  #define I2C_SCL_LO() I2C_DDR |=  I2C_SCL; DELAY(I2C_DELAY);

  I2C() {
    I2C_PORT &= ~( I2C_SDA | I2C_SCL );
    I2C_SCL_HI();
    I2C_SDA_HI();
    I2C_SDA_LO();
  }

  ~I2C() {
    I2C_PORT &= ~( I2C_SDA | I2C_SCL );
    I2C_DDR &= ~( I2C_SDA | I2C_SCL );
  }

  inline void start() {
    I2C_SCL_LO();
    I2C_SDA_HI();
  }

  inline void stop() {
    I2C_SDA_LO();
    I2C_SCL_HI();
    I2C_SDA_HI();
    I2C_DDR &= ~(I2C_SDA | I2C_SCL);
    I2C_SDA_LO();
  }

  #define SendBit(data, mask) \
    if (data & mask) { \
      I2C_SDA_HI();  \
    } else {         \
      I2C_SDA_LO();  \
    }                \
    I2C_SCL_HI();    \
    I2C_SCL_LO();

  inline void SendByte(uint8_t data) {
    SendBit(data, 1 << 7);
    SendBit(data, 1 << 6);
    SendBit(data, 1 << 5);
    SendBit(data, 1 << 4);
    SendBit(data, 1 << 3);
    SendBit(data, 1 << 2);
    SendBit(data, 1 << 1);
    SendBit(data, 1 << 0);
    I2C_SDA_HI();  // recv ACK
    DELAY(I2C_DELAY);
    I2C_SCL_HI();
    I2C_SCL_LO();
  }

  inline uint8_t RecvBit(uint8_t mask) {
    I2C_SCL_HI();
    // wait until SCL HIGH or timeout at 3ms
    for (uint16_t i=60000; !(I2C_SCL_GET()) && i; i--);
    uint8_t data = I2C_SDA_GET();
    I2C_SCL_LO();
    return data ? mask : 0;
  }

  inline uint8_t RecvByte(uint8_t last) {
    uint8_t data = 0;
    data |= RecvBit(1 << 7);
    data |= RecvBit(1 << 6);
    data |= RecvBit(1 << 5);
    data |= RecvBit(1 << 4);
    data |= RecvBit(1 << 3);
    data |= RecvBit(1 << 2);
    data |= RecvBit(1 << 1);
    data |= RecvBit(1 << 0);
    if (last) {
      I2C_SDA_HI();  // NACK
    } else {
      I2C_SDA_LO();  // ACK
    }
    DELAY(I2C_DELAY);
    I2C_SCL_HI();
    I2C_SDA_HI();
    I2C_SCL_LO();
    return data;
  }

  void begin() {};
  void beginTransmission(uint8_t addr) { start(); SendByte(addr << 1);  };
  uint8_t write(uint8_t dat) { SendByte(dat); return 1; };
  uint8_t endTransmission() { stop(); return 0; };
};

I2C i2c;

uint8_t log2(uint16_t x) {
  uint8_t y = 0;
  while (x>>=1) y++;
  return y;
}

class SI5351 {
public:
  volatile int32_t _fout;
  volatile uint8_t _div;  // note: uint8_t asserts fout > 3.5MHz with R_DIV=1
  volatile uint16_t _msa128min512;
  volatile uint32_t _msb128;
  volatile uint8_t pll_regs[8];

  #define BB0(x) ((uint8_t)(x))
  #define BB1(x) ((uint8_t)((x)>>8))
  #define BB2(x) ((uint8_t)((x)>>16))

  #define OFAST __attribute__((optimize("Ofast")))

  // note: relies on _msb128, _msa128min512, _div, _fout, fxtal
  inline void OFAST freq_calc_fast(int16_t df) {
    #define _MSC  0x10000
    uint32_t msb128 = _msb128 + ((int64_t)(_div * (int32_t)df) * _MSC * 128) / fxtal;
    uint16_t msp1 = _msa128min512 + msb128 / _MSC;
    // = msb128 % _MSC;  assuming MSC is covering exact uint16_t
    // so the mod operation can dissapear and the upper BB2 byte
    uint16_t msp2 = msb128;
    pll_regs[4] = BB0(msp1);
    pll_regs[5] = ((_MSC&0xF0000)>>(16-4));
    // top nibble MUST be same as top nibble of _MSC
    // assuming that BB2(msp2) is always 0 -> so reg is constant
    pll_regs[6] = BB1(msp2);
    pll_regs[7] = BB0(msp2);
  }

  inline void SendPLLRegisterBulk() {
    i2c.start();
    i2c.SendByte(SI5351_ADDR << 1);
    i2c.SendByte(26+0*8 + 4);  // Write to PLLA
    i2c.SendByte(pll_regs[4]);
    i2c.SendByte(pll_regs[5]);
    i2c.SendByte(pll_regs[6]);
    i2c.SendByte(pll_regs[7]);
    i2c.stop();
  }

  void SendRegister(uint8_t reg, uint8_t* data, uint8_t n) {
    i2c.start();
    i2c.SendByte(SI5351_ADDR << 1);
    i2c.SendByte(reg);
    while (n--) i2c.SendByte(*data++);
    i2c.stop();
  }
  void SendRegister(uint8_t reg, uint8_t val) { SendRegister(reg, &val, 1); }
  int16_t iqmsa; // to detect a need for a PLL reset

  enum ms_t { PLLA=0, PLLB=1, MSNA=-2, MSNB=-1, MS0=0, MS1=1, MS2=2, MS3=3, MS4=4, MS5=5 };

  void ms(int8_t n, uint32_t div_nom, uint32_t div_denom, uint8_t pll = PLLA,
    uint8_t _int = 0, uint16_t phase = 0, uint8_t rdiv = 0) {
    uint16_t msa; uint32_t msb, msc, msp1, msp2, msp3;
    // integer part: msa must be in range 15..90 for PLL, 8+1/1048575..900 for MS
    msa = div_nom / div_denom;
    // To satisfy the MSx_INT=1 requirement of AN619, section 4.1.3
    // which basically says that for MS divider a value of 4 and integer mode must be used
    if (msa == 4) _int = 1;
    // fractional part
    msb = _int ? 0 : (((uint64_t)(div_nom % div_denom)*_MSC) / div_denom);
    msc = _int ? 1 : _MSC;
    msp1 = 128*msa + 128*msb/msc - 512;
    msp2 = 128*msb - 128*msb/msc * msc;
    msp3 = msc;
    uint8_t ms_reg2 = BB2(msp1) | (rdiv<<4) | ((msa == 4)*0x0C);
    uint8_t ms_regs[8] = {
      BB1(msp3), BB0(msp3), ms_reg2, BB1(msp1), BB0(msp1),
      BB2(((msp3 & 0x0F0000)<<4) | msp2), BB1(msp2), BB0(msp2) };

    SendRegister(n*8+42, ms_regs, 8); // Write to MSx
    if (n < 0) {
      SendRegister(n+16+8, 0x80|(0x40*_int)); // MSNx PLLn: 0x40=FBA_INT; 0x80=CLKn_PDN
    } else {
      // MSx CLKn: 0x0C=PLLA,0x2C=PLLB local msynth; 3=8mA; 0x40=MSx_INT; 0x80=CLKx_PDN
      // make sure to configure MS in fractional-mode, perform reset afterwards
      SendRegister(n+16, ((pll)*0x20)|0x0C|3|(0x40*_int));
      SendRegister(n+165, (!_int) * phase * msa / 90);
    }
  }

  // when using: make sure to configure MS in fractional-mode!, perform reset afterwards
  void phase(int8_t n, uint32_t div_nom, uint32_t div_denom, uint16_t phase) {
    SendRegister(n+165, phase * (div_nom / div_denom) / 90);
  }

  // 0x20 reset PLLA; 0x80 reset PLLB
  void reset() { SendRegister(177, 0xA0); }

  // output-enable mask: CLK2=4; CLK1=2; CLK0=1
  void oe(uint8_t mask) { SendRegister(3, ~mask); }

  // Set a CLK0,1,2 to fout Hz with phase i, q (on PLLA)
  void freq(int32_t fout, uint16_t i, uint16_t q) {
    uint8_t rdiv = 0; // CLK pin sees fout/(2^rdiv)
    // for higher freqs, use 3rd harmonic
    if (fout > 300000000) { i/=3; q/=3; fout/=3; }
    // Divide by 128 for fout 4..500kHz
    if (fout < 500000) { rdiv = 7; fout *= 128; }
    uint16_t d; if (fout < 30000000) d = (16 * fxtal) / fout; else d = (32 * fxtal) / fout;
    // PLL at 189MHz to cover 160m (freq>1.48MHz) when using 27MHz crystal
    // for f=140..300MHz; AN619; 4.1.3, this implies integer mode
    if (fout < 3500000) d = (7 * fxtal) / fout;
    if (fout > 140000000) d = 4;
    if (d % 2) d++;  // even numbers preferred for divider (AN619 p.4 and p.6)
    if ( (d * (fout - 5000) / fxtal) != (d * (fout + 5000) / fxtal) ) d += 2;
    // Variable PLLA VCO frequency at integer multiple of fout at around 27MHz*16 = 432MHz
    // Si5351 spectral purity considerations: https://groups.io/g/QRPLabs/message/42662
    // PLLA in fractional mode
    // Multisynth stage with integer divider but in frac mode due to phase setting
    uint32_t fvcoa = d * fout;
    ms(MSNA, fvcoa, fxtal);
    ms(MS0,  fvcoa, fout, PLLA, 0, i, rdiv);
    ms(MS1,  fvcoa, fout, PLLA, 0, q, rdiv);
    ms(MS2,  fvcoa, fout, PLLA, 0, 0, rdiv);
    if (iqmsa != (((int8_t)i-(int8_t)q)*((int16_t)(fvcoa/fout))/90)) {
      iqmsa = ((int8_t)i-(int8_t)q)*((int16_t)(fvcoa/fout))/90; reset();
    }
    oe(0b00000011);  // output enable CLK0, CLK1
    _fout = fout;    // cache
    _div = d;
    _msa128min512 = fvcoa / fxtal * 128 - 512;
    _msb128=((uint64_t)(fvcoa % fxtal)*_MSC*128) / fxtal;
  }

  // Set a CLK2 to fout Hz on PLLB
  void freqb(uint32_t fout) {
    uint16_t d = (16 * fxtal) / fout;
    // even numbers preferred for divider (AN619 p.4 and p.6)
    if (d % 2) d++;
    // Variable PLLA VCO frequency at integer multiple of fout at around 27MHz*16 = 432MHz
    uint32_t fvcoa = d * fout;
    ms(MSNB, fvcoa, fxtal);
    ms(MS2,  fvcoa, fout, PLLB, 0, 0, 0);
  }

  // read Si5351 register
  uint8_t RecvRegister(uint8_t reg) {
    i2c.start();  // Data write to set the register address
    i2c.SendByte(SI5351_ADDR << 1);
    i2c.SendByte(reg);
    i2c.stop();
    i2c.start(); // Data read to retrieve the data from the set address
    i2c.SendByte((SI5351_ADDR << 1) | 1);
    uint8_t data = i2c.RecvByte(LAST);
    i2c.stop();
    return data;
  }

  // Si5351 power down
  void powerDown() {
    SendRegister(3,  0b11111111);   // disable all CLK outputs
    SendRegister(24, 0b00010000);   // CLK2 enabled, CLK0 & CLK1 disabled
    SendRegister(25, 0b00000000);   // disable state (0 state when disabled)
    for (uint8_t addr=16; addr<24; addr++) SendRegister(addr, 0b10000000);
    SendRegister(187, 0);           // disable fanout
    SendRegister(149, 0);           // disable spread spectrum enable
    SendRegister(183, 0b11010010);  // internal CL = 10 pF (default)
  }
  #define SI_CLK_OE 3

};

SI5351 si5351;

// I2c port expander for LPF band selection
class IOExpander16 {
public:
  inline void SendRegister(uint8_t reg, uint8_t val) {
    i2c.begin();
    i2c.beginTransmission(IOEXP16_ADDR);
    i2c.write(reg);
    i2c.write(val);
    i2c.endTransmission();
  }
  inline void init() {
    write(0);
  }
  inline void write(uint16_t data) {
    SendRegister(0x07, 0xff);
    SendRegister(0x06, 0xff);
    SendRegister(0x02, data);
    SendRegister(0x06, 0x00);
    SendRegister(0x03, data >> 8);
    SendRegister(0x07, 0x00);
  }
};

IOExpander16 ioext;
enum iox_t { IO0_0, IO0_1, IO0_2, IO0_3, IO0_4, IO0_5, IO0_6, IO0_7,
             IO1_0, IO1_1, IO1_2, IO1_3, IO1_4, IO1_5, IO1_6, IO1_7 };

// set or reset latches
void set_latch(uint8_t i, uint8_t common, uint8_t latch = ON) {
  #define LATCH_TIME  30
  if (latch) {
    // set a latch
    ioext.write((1<<i)| 0x0000);
    delay(LATCH_TIME);
    ioext.write(0x0000);
  } else {
    if (i == 0xff) {
      ioext.init();
      // reset all latches
      for (uint8_t i=0; i<16; i++) set_latch(i, common, latch);
    } else {
      ioext.write( (~(1<<i))| (1 << common));
      delay(LATCH_TIME);
      ioext.write(0x0000);
    }
  }
}

uint8_t prev_lpf_io = 0xff; // inits and resets all latches

// I2C port expander band to I/O pin mapping
void set_lpf(uint8_t band) {
  uint8_t lpf_io;
  switch (band) {
    case 0:  lpf_io = IO1_3; break;  // 10M
    case 1:  lpf_io = IO1_4; break;  // 15M
    case 2:  lpf_io = IO1_2; break;  // 17M
    case 3:  lpf_io = IO1_5; break;  // 20M
    case 4:  lpf_io = IO1_1; break;  // 30M
    case 5:  lpf_io = IO1_6; break;  // 40M
    case 6:  lpf_io = IO1_0; break;  // 60M
    case 7:  lpf_io = IO1_7; break;  // 80M
    default: lpf_io = IO1_7;
  }

#ifdef LPF_NOLATCH
  if (prev_lpf_io != lpf_io) {
    ioext.write(1 << lpf_io);
    prev_lpf_io = lpf_io;
  }
#else
  if (prev_lpf_io != lpf_io) {
    set_latch(prev_lpf_io, IO0_0, OFF);
    set_latch(lpf_io, IO0_0);
    prev_lpf_io = lpf_io;
  }
#endif // LPF_NOLATCH
}

// VOX processing
inline void vox_process(uint8_t trigger) {
  if (trigger) {
    // hangtime = 255 / 4402 = 58ms (the time that TX at least stays on
    // when not triggered again). tx == 255 when triggered first
    // 254 follows for subsequent triggers, until tx is off.
    tx = tx ? 254 : 255;
  } else {
    if (tx) tx--;
  }
}

// ADC Tx sample-rate  OCR2A = ((F_CPU / 64) / F_SAMP_TX) - 1
#define F_SAMP_TX 4800

// one full circle of 2pi radians = F_SAMP_TX/8
#define _UA  600

// maximum phase change
#define MAX_DP  ((bwfilter == FULL) ? _UA : (bwfilter == BW1800) ? _UA/4 : _UA/2)

// arctan approximation
inline int16_t arctan3(int16_t q, int16_t i) {
  // note that atan2 can overflow easily so keep _UA low
  #define _atan2(z)  (_UA/8 + _UA/22 - _UA/22 * z) * z
  int16_t r;
  if (abs(q) > abs(i))
    r = _UA / 4 - _atan2(abs(i) / abs(q));        // arctan(z) = 90-arctan(1/z)
  else
    r = (i == 0) ? 0 : _atan2(abs(q) / abs(i));   // arctan(z)
  r = (i < 0) ? _UA / 2 - r : r;                  // arctan(-z) = -arctan(z)
  return (q < 0) ? -r : r;                        // arctan(-z) = -arctan(z)
}

// approximation of: magnitude = sqrt(i*i + q*q); error 0.95dB
#define magn(i, q) (abs(i) > abs(q) ? abs(i) + abs(q) / 4 : abs(q) + abs(i) / 4)

uint8_t lut[256];
volatile uint8_t amp;

#define MORE_MIC_GAIN  1

// SSB Rx processing (Hilbert)
inline int16_t ssb(int16_t in) {
  static int16_t dc, z1;
  int16_t i, q;
  uint8_t j;
  static int16_t v[16];
  for (j=0; j<15; j++) v[j] = v[j + 1];
#ifdef MORE_MIC_GAIN
#ifdef DIG_MODE
  int16_t ac = in;
  dc = (ac + (7) * dc) / (7 + 1);  // hpf slow average
  v[15] = (ac - dc) / 2;           // hpf (dc decoupling with -6dB gain)
#else
  int16_t ac = in * 2;             // 6dB gain
  ac = ac + z1;                    // lpf
  z1 = (in - (2) * z1) / (2 + 1);  // lpf notch at fs/2 (alias rejecting)
  dc = (ac + (2) * dc) / (2 + 1);  // hpf slow average
  v[15] = (ac - dc);               // hpf (dc decoupling)
#endif // DIG_MODE
  // 6dB gain for i, q  (to prevent quanitization issues in
  // Hilbert transformer and phase calculation, corrected for magnitude calc)
  i = v[7] * 2;
  // Hilbert transform, 40dB side-band rejection in 400..1900Hz (@4kSPS)
  // when used in image-rejection scenario
  q = ((v[0] - v[14]) * 2 + (v[2] - v[12]) * 8 + (v[4] - v[10]) * 21 + (v[6] - v[8]) * 16) / 64 + (v[6] - v[8]);

  uint16_t amp16 = magn(i / 2, q / 2);  // -6dB gain (correction)
#else  // !MORE_MIC_GAIN
  dc = (in + dc) / 2;          // average
  int16_t ac = (in - dc);      // DC decoupling
  v[15] = (ac + z1);           // low-pass filter with notch at fs/2
  z1 = ac;
  i = v[7];
  // Hilbert transform, 40dB side-band rejection in 400..1900Hz (@4kSPS)
  // when used in image-rejection scenario
  q = ((v[0] - v[14]) * 2 + (v[2] - v[12]) * 8 + (v[4] - v[10]) * 21 + (v[6] - v[8]) * 15) / 128 + (v[6] - v[8]) / 2;

  uint16_t amp16 = magn(i, q);
#endif  // MORE_MIC_GAIN

  vox_process(amp16 > voxlevel);
  amp16 = amp16 << (drive);
  // clip or when drive=8 use max output
  amp16 = ((amp16 > 255) || (drive == 8)) ? 255 : amp16;
  amp = tx ? lut[amp16] : 0;

  static int16_t prev_phase;
  int16_t phase = arctan3(q, i);

  int16_t dp = phase - prev_phase;  // phase difference
  prev_phase = phase;

  // make negative phase shifts positive
  // prevents negative frequencies and reduce sideband spurs
  if (dp < 0) dp = dp + _UA;
#ifdef QUAD
  if (dp >= (_UA/2)) { dp = dp - _UA/2; quad = !quad; }
#endif

#ifdef MAX_DP
  if (dp > MAX_DP) {
    // dp should be less than half unit-angle
    // in order to keep frequencies below F_SAMP_TX/2
    prev_phase = phase - (dp - MAX_DP);  // substract restdp
    dp = MAX_DP;
  }
#endif
  if (radiomode == USB)
    return dp * ( F_SAMP_TX / _UA);
  else
    return dp * (-F_SAMP_TX / _UA);
}

#define AF_BIAS    32

#define TX1RX0  0b11111011
#define TX1RX1  0b11111000
#define TX0RX1  0b11111100
#define TX0RX0  0b11111111

int16_t _adc;

void dsp_tx() {
  int16_t adc;  // current ADC sample
  adc = ADC;
  ADCSRA |= (1 << ADSC);
  // submit frequency registers to Si5351 over 731kbit/s I2C
  // transfer takes 64/731 = 88us
  // then the PLL needs 50us to stabalize)
  si5351.SendPLLRegisterBulk();
#ifdef QUAD
  // Invert/non-invert CLK2 in case of a huge phase-change
  si5351.SendRegister(18, quad ? 0x1f : 0x0f);
#endif // QUAD
  // submit amplitude to PWM register
  // amplitude-phase-alignment error is about 30-50us
  OCR1BL = amp;
  adc += ADC;
  // ENABLE this line when using direct biasing!!
  ADCSRA |= (1 << ADSC);
  // convert analog input into phase-shifts (carrier out by periodic frequency shifts)
  int16_t df = ssb(_adc >> attmic);
  adc += ADC;
  ADCSRA |= (1 << ADSC);
  // calculate Si5351 registers based on frequency shift and carrier frequency
  si5351.freq_calc_fast(df);
  adc += ADC;
  ADCSRA |= (1 << ADSC);
  // now make sure that we keep a postive bias offset (to prevent the phase
  // swapping 180 degrees and potentially causing negative feedback (RFI)
  _adc = (adc/4 - (512 - AF_BIAS));

  if (tx == 1) { OCR1BL = 0; si5351.SendRegister(SI_CLK_OE, TX0RX0); }  // disable carrier
  if (tx == 255) { si5351.SendRegister(SI_CLK_OE, TX1RX0); } // enable carrier
}

volatile uint16_t acc;
volatile uint32_t cw_offset = 600;

// alpha = cw_offset * 2 * pi / fs
// alpha = cw_offset * 798 / F_SAMP_TX;
#define ALPHA 99

// pwm midpoint (50% duty cycle)
#define MIDPOINT  0x80

int8_t p_sin;
int8_t n_cos;

// Minsky circle init
inline void minsky_init() {
  p_sin = 0;
  n_cos = 112;  // sets the amplitude
  OCR1AL = MIDPOINT;
}

// Minsky circle sample
inline void process_minsky() {
  p_sin += (ALPHA * n_cos) >> 7;
  n_cos -= (ALPHA * p_sin) >> 7;
}

// raised-cosine(i) = 255 * sq(cos(PI/2 * i/32))
const uint8_t rcos[] = { 255,254,252,249,245,239,233,226,217,208,
198,187,176,164,152,139,127,115,102,90,78,67,56,46,37,28,21,15,9,5,2 };

void dummy() {}

void dsp_tx_cw() {
  uint8_t i=31;
  uint8_t j=rcos[i];
  if (OCR1BL < lut[255]) {
    // ramp up the transmit PWM amplitude
    // soft rising edge to prevent key clicks
    while (OCR1BL < lut[j]) { i--; j=rcos[i]; }
    OCR1BL = lut[j];
  }
  process_minsky();
  OCR1AL = (p_sin >> (16 - tonevol)) + MIDPOINT;
}

void dsp_tx_am() {
  ADCSRA |= (1 << ADSC);   // start ADC
  // submit amplitude to PWM about 140us in advance of phase-change
  // so that phase-delays in key-shaping circuit filter can settle)
  OCR1BL = amp;
  int16_t adc = ADC - 512; // current ADC sample 10-bits analog input, NOTE: first ADCL, then ADCH
  int16_t in = (adc >> attmic);
  in = in << (drive-4);
  #define AM_BASE 32
  in=max(0, min(255, (in + AM_BASE)));
  amp=in;// lut[in];
}

void dsp_tx_fm() {
  ADCSRA |= (1 << ADSC);  // start ADC
  // submit amplitude to PWM register
  // actually this is done in advance (about 140us) of phase-change
  // so that phase-delays in key-shaping circuit filter can settle
  OCR1BL = lut[255];
  // submit frequency registers to Si5351 over 731kbit/s I2C
  // transfer takes 64/731 = 88us
  // then PLL-loopfilter needs 50us to stabalize
  si5351.SendPLLRegisterBulk();
  // current ADC sample 10-bits analog input
  // NOTE: first ADCL, then ADCH
  int16_t adc = ADC - 512;
  int16_t in = (adc >> attmic);
  in = in << (drive);
  int16_t df = in;
  // calculate Si5351 registers based on frequency shift and carrier frequency
  si5351.freq_calc_fast(df);
}

// Morse-to-ASCII lookup table
const char m2a[128] =
  {'~',' ','E','T','I','A','N','M','S','U','R','W','D','K','G','O',
   'H','V','F','*','L','*','P','J','B','X','C','Y','Z','Q','*','*',
   '5','4','S','3','*','*','*','2','&','*','+','*','*','*','J','1',
   '6','=','/','*','*','#','(','*','7','*','*','*','8','*','9','0',
   '^','*','*','*','*','*','*','*','*','*','*','*','?','_','*','*',
   '*','\\','"','*','*','.','*','*','*','*','@','*','*','*','\'','*',
   '*','-','*','*','*','*','*','*','*','*',';','!','*',')','*','*',
   '*','*','*',',','*','*','*','*',':','*','*','*','*','*','*','*'};

// ASCII-to-Morse lookup table
const uint8_t a2m[64] =
  {0x01,0x6b,0x52,0x4c,0x89,0x4c,0x28,0x5e,  //   ! " # $ % & '
   0x36,0x6d,0x4c,0x2a,0x73,0x61,0x55,0x32,  // ( ) * + , - . /
   0x3f,0x2f,0x27,0x23,0x21,0x20,0x30,0x38,  // 0 1 2 3 4 5 6 7
   0x3c,0x3e,0x78,0x6a,0x4c,0x31,0x4c,0x4c,  // 8 9 : ; < = > ?
   0x5a,0x05,0x18,0x1a,0x0c,0x02,0x12,0x0e,  // @ A B C D E F G
   0x10,0x04,0x17,0x0d,0x14,0x07,0x06,0x0f,  // H I J K L M N O
   0x16,0x1d,0x0a,0x08,0x03,0x09,0x11,0x0b,  // P Q R S T U V W
   0x19,0x1b,0x1c,0x4c,0x40,0x4c,0x4c,0x4d}; // X Y Z [ \ ] ^ _

// table lookup for CW decoder
char lookup_cw(uint8_t addr) {
  char ch = '*';
  if (addr < 128) ch = m2a[addr];
  return ch;
}

uint8_t txsym    = 1;    // tx decoder symbol
uint8_t rxsym    = 1;    // rx decoder symbol
uint8_t xmit_col = 0;    // tx CW decoder write char at this column
uint8_t recv_col = 0;    // rx CW decoder write char at this column
uint8_t xtra_col = 0;    // for long messages

// print a CW character
void print_cwchar(char ch, uint8_t line) {
  static uint8_t row = XMIT;
  switch (line) {
    case XMIT:
      switch (cwdec) {
        case CWTX:
          // use lines 2 and 3
          if (xmit_col > MAXCOL) {
            xmit_col = 0;
            if (row == XMIT) row = RECV;
            else row = XMIT;
          }
          break;
        case RXTX:
          // use line 2
          if (xmit_col > MAXCOL) xmit_col = 0;
          row = XMIT;
          break;
        default:
          // else do not print
          return;
          break;
      }
      oled.setCursor(xmit_col++, row);
      oled.print(ch);
      oled.print(' ');
      break;
    case RECV:
      switch (cwdec) {
        case CWRX:
          // use lines 2 and 3
          if (recv_col > MAXCOL) {
            recv_col = 0;
            if (row == XMIT) row = RECV;
            else row = XMIT;
          }
          break;
        case RXTX:
          // use line 3
          if (recv_col > MAXCOL) recv_col = 0;
          row = RECV;
          break;
        default:
          // else do not print
          return;
          break;
      }
      oled.setCursor(recv_col++, row);
      oled.print(ch);
      oled.print(' ');
      break;
    case XTRA:
      if (xtra_col > MAXCOL) xtra_col = 0;
      oled.setCursor(xtra_col++, XTRA);
      oled.print(ch);
      oled.print(' ');
      break;
    default:
      break;
  }
}

// print a CW string
void print_cwstr(char* str, uint8_t line) {
  for (uint8_t i=0; str[i]; i++) {
    print_cwchar(str[i], line);
  }
}

// convert morse to ascii and print
void print_cwsym(uint8_t mode) {
  // check the decoder modes
  if (!cwdec) return;
  if ((mode == RECV) && !((cwdec == CWRX) || (cwdec == RXTX))) return;
  if ((mode == XMIT) && !((cwdec == CWTX) || (cwdec == RXTX))) return;
  if (menumode) return;
  uint8_t sym;
  uint8_t row = XMIT;
  if (mode == XMIT) sym = txsym;
  else {
    sym = rxsym;
    row = RECV;
  }
  switch (sym) {
    case 0x0:
      if (mode == XMIT) {
        oled.clrR23();
        xmit_col=0;
      }
      break;
    case 0xc5:
      print_cwstr("BK", mode);
      break;
    case 0x45:
      print_cwstr("SK", mode);
      break;
    default:
      char ch = lookup_cw(sym);
      print_cwchar(ch, mode);
  }
}

// shift a bit into morse (m2a) lookup table address
void shift_txsym(uint8_t abit) {
  txsym = txsym<<1;
  txsym |= abit;
}

int8_t  editpos  = 0;
int8_t  spinpos  = 0;
char   *editstr;

// return the length of string
uint8_t len(char *str) {
  uint8_t i=0;
  while (str[i++]);
  return i-1;
}

// copy a string
void cpy(char *dst, char *src) {
  uint8_t i=0;
  while (src[i]) dst[i++] = src[i];
  dst[i] = '\0';
}

// copy up to n characters of a string
void ncpy(char *dst, char *src, uint8_t n) {
  uint8_t i=0;
  while (src[i] && (i<n)) dst[i++] = src[i];
  dst[i] = '\0';
}

// concatenate a string and return length
uint8_t cat(char *dst, char *src) {
  uint8_t i=0;
  uint8_t x= len(dst);
  while (src[i]) dst[x++] = src[i++];
  dst[x] = '\0';
  return i;
}

void print_label(const char* label) {
  oled.setCursor(0, ROW1);
  oled.print(label);
  oled.clr2eol();
}

// check edit position limits
void check_editpos(char* str) {
  if (editpos > len(str)-1) editpos = 0;
  if (editpos < 0) editpos = len(str)-1;
}

// check edit position limits
void show_editstr() {
  oled.setCursor(0, ROW4);
  oled.print(editstr);
  oled.clr2eol();
}

// show cursor for editing
void show_cursor(uint8_t on) {
  uint8_t i, j;
  oled.clrLine(ROW3);
  if (on) {
    check_editpos(editstr);
    oled.setCursor(editpos, ROW3);
    oled.print('\x04');
  } else {
    // check for deleted characters
    i=0;
    while (editstr[i] != '\0') {
      // % is a marker for deleting a char
      if (editstr[i] == '%') {
        for (j=i; editstr[j]; j++)
          editstr[j] = editstr[j+1];
      } else  i++;
    }
  }
}

// insert a character to the edit string
inline void insertchar() {
  uint8_t x = len(editstr);
  // check limits
  if (x >= MAXSTR-1) return;
  editpos++;
  for (uint8_t i=x; i>editpos; i--)
    editstr[i] = editstr[i-1];
  editstr[editpos] = '%';
  editstr[x+1] = '\0';
  show_cursor(ON);
  show_editstr();
}

// use encoder for editing a string
inline void spinchar() {
  const char spinstr[40] =
  "ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789%";
  spinpos += encoder_val;
  if (spinpos > 36) spinpos = 0;
  if (spinpos <  0) spinpos = 36;
  check_editpos(editstr);
  editstr[editpos] = spinstr[spinpos];
  encoder_val = 0;
  show_editstr();
}

// get spin position
void get_spinpos(char ch) {
  // ascii to index calculations
  if ((ch >= 'A') && (ch <= 'Z')) spinpos = ch-65;
  else if ((ch >= '0') && (ch <= '9')) spinpos = ch-22;
  else spinpos = 37;
}

#define CWMSGSIZE  50
#define NMSG        7

char cwmsg[NMSG][CWMSGSIZE] = {
  "CQ CQ DE $CALL $CALL K",
  "BENS BEST BENT WIRE",
// "RST 5NN 5NN NAME IS $NAME $NAME HW CPY? BK",
  "FB QTH IS $QTH $QTH BT",
  "RIG IS $RIG $PWR $PWR BT",
  "ANT IS $ANT BK",
  "TU FER CALL CU AGN 73 AR E E",
  "TMPSTR"
};

// re-use last CW message as a temp var
char *tmpstr = &cwmsg[NMSG-1][0];

// transmit a character in CW
void send_cwchar(char ch) {
  uint8_t mcode;
  // remove the ascii code offset
  uint8_t addr = (uint8_t)ch - 0x20;
  // ascii-to-morse lookup
  if (addr < 64) mcode = a2m[addr];
  else mcode = 0x4c; // unknown ascii char is ?
  // check for word space
  if (mcode == 0x01) wait_ms(dittime << 2);
  else {
    uint8_t mask = 0x80;
    // use a bit mask to find dit or dah
    while (!(mask & mcode)) mask = mask >> 1;
    while (mask != 1) {
      mask = mask >> 1;
      switch_rxtx(ON, NOSHOW);
      // turn on the side-tone for a dit or dah
      wait_ms((mcode & mask) ? dahtime : dittime);
      switch_rxtx(OFF, NOSHOW);
      // turn off the side-tone for a symbol space
      wait_ms(dittime);
    }
    wait_ms(lettergap);
  }
}

// parse for user variables in CW messages
void parsemsg(char *msg) {
  uint8_t i=1;
  uint8_t j=0;
  char ch = msg[0];
  tmpstr[0] = '\0';
  while (ch) {
    // check for user variables
    if (ch == '$') {
      ch = msg[i++];
      switch (ch) {
        case 'C':
          j += cat(tmpstr, mycall);
          break;
        case 'N':
          j += cat(tmpstr, myname);
          break;
        case 'A':
          j += cat(tmpstr, myant);
          break;
        case 'R':
          j += cat(tmpstr, myrig);
          break;
        case 'P':
          j += cat(tmpstr, mypwr);
          break;
        case 'Q':
          j += cat(tmpstr, myqth);
          break;
        default:
          break;
      }
      tmpstr[j++] = ' ';
      tmpstr[j] = '\0';
      while ((ch != ' ') && (ch != '\0')) {
        ch = msg[i++];
      }
      // handle the case where a user variable
      // is at the end of the message
      if (ch == '\0') break;
    } else {
      tmpstr[j++] = ch;
      tmpstr[j] = '\0';
    }
    ch = msg[i++];
  }
  tmpstr[j] = '\0';
}

// print a CW message
void print_cwmsg(uint8_t id) {
  noInterrupts();
  xmit_col = 0;
  recv_col = 0;
  xtra_col = 0;
  parsemsg(cwmsg[id]);
  oled.clrR234();
  for (uint8_t i=0; tmpstr[i]; i++) {
    if (i <  MAXCOL)    oled.setCursor(xmit_col++, XMIT);
    if (i >= MAXCOL)    oled.setCursor(recv_col++, RECV);
    if (i >= MAXCOL<<1) oled.setCursor(xtra_col++, XTRA);
    oled.print(tmpstr[i]);
  }
  interrupts();
}

// beep and print a CW message
void send_cwmsg(uint8_t id) {
  noInterrupts();
  skip_dsp = YES;
  cwdec = OFF;
  xmit_col = 0;
  recv_col = 0;
  xtra_col = 0;
  parsemsg(cwmsg[id]);
  oled.clrR234();
  for (uint8_t i=0; tmpstr[i]; i++) {
    if (i <  MAXCOL)    oled.setCursor(xmit_col++, XMIT);
    if (i >= MAXCOL)    oled.setCursor(recv_col++, RECV);
    if (i >= MAXCOL<<1) oled.setCursor(xtra_col++, XTRA);
    oled.print(tmpstr[i]);
    send_cwchar(tmpstr[i]);
  }
  delay(ONE_SECOND);
  oled.clrR234();
  menuAction(menu);
  // restore settings
  cwdec = cwdec_save;
  skip_dsp = NO;
  interrupts();
}

// beep a CW message
void beep_cwmsg(uint8_t id) {
  noInterrupts();
  skip_dsp = YES;
  practice = ON;
  cwdec = OFF;
  parsemsg(cwmsg[id]);
  for (uint8_t i=0; tmpstr[i]; i++) {
    send_cwchar(tmpstr[i]);
  }
  // restore settings
  practice = practice_save;
  cwdec = cwdec_save;
  skip_dsp = NO;
  interrupts();
}

// play sidetone beep (no transmit)
void playtone(uint8_t onoff) {
  if (onoff) {
    skip_dsp = YES;
    noInterrupts();
    practice = ON;
    switch_rxtx(ON, NOSHOW);
  } else {
    switch_rxtx(OFF, NOSHOW);
    practice = practice_save;
    skip_dsp = NO;
    interrupts();
  }
}

// echo a CW string
int beep_cw(char *msg) {
  cpy(cwmsg[NMSG-1],msg);
  beep_cwmsg(NMSG-1);
}

#define N     24        // goertzel buffer size
#define DSHFT  8        // goertzel data scaler
#define GSHFT 11        // goertzel coef scaler
#define GCOEF 3547      // goertzel coef = 1.73205 * 2048

int16_t gdbuf[N];       // goertzel data buffer
uint8_t gstate = TIC;   // goertzel data buffer state
uint8_t gdbrdy = NO;    // goertzel data ready flag
uint8_t gcdone = YES;   // goertzel calculation complete
uint8_t go2tic = NO;    // goertzel data buffer control

int32_t gdbtime       = 0;   // goertzel decoder timer
int32_t starttimehigh = 0;   // start time of dit/dah
int32_t starttimelow  = 0;   // start time of gap
int16_t highduration  = 0;   // duration of dit/dah
int16_t prevduration  = 0;   // duration of dit/dah
int16_t lowduration   = 0;   // duration of gap
int16_t dittimeavg    = 0;   // average dit time
int16_t newdittime    = 0;   // calculated dit time
int16_t lastlowtime   = 0;   // to capture last symbol


#define DXLO   0
#define DXW1   1
#define DXW2   2
#define DXHI   3
#define DXW3   4
#define DXW4   5

int32_t cwhigh   = 400;       // CW decoder high threshold
int32_t cwlow    = 100;       // CW decoder low  threshold
uint8_t dxstate  = DXLO;      // CW decoder state
int32_t maxcwmag = 0;         // max CW magnitude during interval

#define GDPMAX      650       // =~ 2 second interval
#define RXCWCONST  9200

// interruptable delay with scan checks
uint8_t scan_wait_ms(uint16_t dly, uint8_t checkexit=0) {
  uint32_t curTime = millis();
  uint32_t endTime = curTime + dly;
  while(curTime < endTime) {
    delayMicroseconds(100);
    wdt_reset();
    curTime = millis();
    // check for exit button click
    if (EXIT_PRESSED) return CLICK;
    // run goertzel if data is ready
    if (gdbrdy) {
      goertzel(SHOW);
      // goertzel calculation complete
      gcdone = YES;
      go2tic = YES;
    }
  }
  return 0;
}

// print the measured rx wpm
void show_rxwpm() {
  static uint16_t wpm;
  uint16_t newwpm;
  newwpm = RXCWCONST/dittimeavg;
  if (newwpm > 99) newwpm = 99;
  // only print if changed
  if (wpm != newwpm) {
    wpm = newwpm;
    oled.setCursor(RWPMCOL, ROW1);
    oled.print(wpm);
    oled.clr2eol();
    wdt_reset();
  }
}

// print the maximum goertzel magnitude
void show_maxcwmag() {
  oled.setCursor(0, ROW2);
  oled.print(maxcwmag);
  oled.print(" ");
  oled.print(cwhigh);
  oled.print(" ");
  oled.print(cwlow);
  oled.clr2eol();
  wdt_reset();
}

// collect Goertzel data
inline void gbuffer(int16_t ac) {
  static uint8_t gdbcnt = 0;
  // in the TIC state: gather data
  // in the TOC state: run the goertzel calculations
  switch (gstate) {
    case TIC:
      for (uint8_t i=N-1; i>0; i--) gdbuf[i] = gdbuf[i-1];
      gdbuf[0] = ac;
      gdbcnt++;
      if (gdbcnt == N-1) {
        gdbcnt = 0;
        // when the buffer is full then allow the next
        // goertzel calculation to start
        gstate = TOC;
        gdbrdy = YES;
      }
      break;
    case TOC:
      // stay in TOC state until ready
      if (go2tic) gstate = TIC;
      go2tic = NO;
      break;
  }
  gdbtime++;
}

#define MINHIGH  500
#define MINLOW   50
#define DXWAIT   5

// Goertzel CW calculations
void goertzel(uint8_t show = 0) {
  int32_t gmag;                   // goertzel magnitude
  int32_t Q0;                     // goertzel state Q0
  static int32_t Q1 = 0;          // goertzel state Q1
  static int32_t Q2 = 0;          // goertzel state Q2
  static int32_t cwmag = 0;       // average magnitude
  static int8_t  wct;             // wait counter
  static int16_t gdptime = 0;     // goertzel period timer
  gcdone = NO;                    // stall next goertzel until done
  gdbrdy = NO;                    // ready for more data

  if ((dxstate == DXLO) || (dxstate == DXHI)) {
    // process buffer
    for (uint8_t i=0; i<N; i++){
      Q0 = ((GCOEF * Q1)>>GSHFT) - Q2 + gdbuf[i];
      Q2 = Q1;
      Q1 = Q0;
    }
    gmag = (((Q1*Q1) + (Q2*Q2))>>DSHFT) - ((((Q1*Q2)>>DSHFT) * GCOEF)>>GSHFT);
    cwmag = cwmag + ((gmag - cwmag)>>1);
    Q2 = 0;
    Q1 = 0;
  }
  // keep track of high and low times
  switch (dxstate) {
    case DXLO:
      if ((cwmag > MINHIGH) && (cwmag > cwhigh)) {
        // at transition from low to high
        // mark the start of the high state and
        // measure the previous low duration
        starttimehigh = gdbtime;
        lowduration   = gdbtime - starttimelow;
        dxstate = DXW1;
      }
      break;
    case DXW1:
      wct = 0;
      dxstate = DXW2;
      break;
    case DXW2:
      wct++;
      if (wct == DXWAIT) dxstate = DXHI;
      break;
    case DXHI:
      if (cwmag < cwlow) {
        // at transition from high to low
        // mark the start of the low state and
        // measure the previous high duration
        starttimelow = gdbtime;
        prevduration = highduration;
        highduration = gdbtime - starttimehigh;
        if ((highduration > (prevduration<<1)) ||
            (prevduration > (highduration<<1))) {
          newdittime = (highduration + prevduration)>>2;
          dittimeavg = dittimeavg + ((newdittime - dittimeavg)>>2);
        }
        dxstate = DXW3;
      }
      break;
    case DXW3:
      wct = 0;
      dxstate = DXW4;
      break;
    case DXW4:
      wct++;
      if (wct == DXWAIT) dxstate = DXLO;
      break;
    default:
      break;
  }
  if (cwmag > maxcwmag) maxcwmag = cwmag;
  // check if it's time to update the CW thresholds
  gdptime++;
  if (gdptime == GDPMAX) {
    // track with quick rise and exponential averaging
    if (maxcwmag > (cwhigh<<2)) cwhigh = maxcwmag>>1;
    else cwhigh = cwhigh + (((maxcwmag>>1) - cwhigh)>>2);
    cwlow  = cwhigh - (cwhigh>>2);
    if (show) {
      // print goertzel data and rx wpm
      show_maxcwmag();
      show_rxwpm();
    }
    maxcwmag = 0;
    gdptime = 0;
  }
}

// CW Decoding for Rx
inline void rxcw_decode(uint8_t show = 0) {
  goertzel(show);
  switch (dxstate) {
    case DXW3:
      // we just calculated the high duration
      // check if it was a dit or dah
      rxsym = rxsym<<1;
      if (highduration > (dittimeavg<<1)) rxsym |= 1;
      break;
    case DXW1:
      // we just calculated the low duration
      // check if it was a word or letter gap
      if (lowduration > (dittimeavg<<1)) {
        // letter gap found so print char
        if (rxsym > 1) {
          print_cwsym(RECV);
          rxsym = 1;
        }
      }
      // word gap > 4x dittime
      if (lowduration > (dittimeavg<<2)) {
        // word gap found so print space
        rxsym = 1;
        print_cwsym(RECV);
      }
      break;
    default:
      break;
  }
  if ((dxstate == DXLO) && (rxsym > 1)) {
    // print last char if low time >= 4x dittime
    lastlowtime = gdbtime - starttimelow;
    if (lastlowtime >= (dittimeavg<<2)) {
      print_cwsym(RECV);
      rxsym = 1;
    }
  }
  // goertzel calculation complete
  gcdone = YES;
  go2tic = YES;
}

#define F_SAMP_PWM (78125/1)
#define F_SAMP_RX 62500

// was 192307 but as noted this produces clicks in audio stream
// slower ADC clock cures this but is a problem for VOX
// when sampling mic-input simulatanously
#define F_ADC_CONV (192307/2)

int16_t gain = 1024;

// fast AGC
inline int16_t process_agc_fast(int16_t in) {
  int16_t out = (gain >= 1024) ? (gain >> 10) * in : in;
  int16_t accum = (1 - abs(out >> 10));
  if ((INT16_MAX - gain) > accum) gain = gain + accum;
  if (gain < 1) gain = 1;
  return out;
}

int16_t centiGain = 128;
#define DECAY_FACTOR 400      // AGC decay occurs <DECAY_FACTOR> slower than attack.
uint16_t decayCount = DECAY_FACTOR;
#define HI(x)  ((x) >> 8)
#define LO(x)  ((x) & 0xFF)

// slow AGC
inline int16_t process_agc(int16_t in) {
  static uint8_t small = ON;
  int16_t out;

  if (centiGain >= 128)
    out = (centiGain >> 5) * in;         // net gain >= 1
  else
    out = (centiGain >> 2) * (in >> 3);  // net gain < 1
  out >>= 2;

  if (HI(abs(out)) > HI(1536)) {
    // Fast attack time when big signal encountered (relies on CentiGain >= 16)
    centiGain -= (centiGain >> 4);
  } else {
    if (HI(abs(out)) > HI(1024))
      small = OFF;
    if (--decayCount == 0) {
      // But slowly increase gain when signal disappears
      // 400 samples below lower threshold - increase gain
      if (small) {
        if (centiGain < (INT16_MAX-(INT16_MAX >> 4)))
          centiGain += (centiGain >> 4);
        else
          centiGain = INT16_MAX;
      }
      decayCount = DECAY_FACTOR;
      small = ON;
    }
  }
  return out;
}

// noise reduction using exponential averaging
inline int16_t process_nr(int16_t ac) {
  static int16_t avg;
  avg = avg + ((ac - avg)>>nrlevel);
  return avg;
}

// IIR filter state variables
int16_t x0,x1,y1;
int32_t y0;

// IIR bandwidth filters
inline int16_t iirfilter(int16_t za0) {
  static int16_t za1, za2;
  static int16_t zb0, zb1, zb2;
  static int16_t zc0, zc1, zc2;
  static int16_t zz1, zz2;
  if (radiomode != CW) {
    // SSB filters
    if (!bwfilter) return za0;  // full
    zz2 = zz1;
    zz1 = za0;
    za0 = (30 * (za0 - zz2) + 25 * zz1) >> 5;  // 300-Hz
    switch (bwfilter) {
      case BW3000:
        // 3000Hz SSB low-pass filter
        zb0 = ((za0 + 2 * za1 + za2) >>1) - ((13 * zb1 + 11 * zb2) >>4);
        zc0 = ((zb0 + 2 * zb1 + zb2) >>1) - ((18 * zc1 + 11 * zc2) >>4);
        break;
      case BW2400:
        // 2400Hz SSB low-pass filter
        zb0 = ((za0 + 2 * za1 + za2) >>1) - ((2 * zb1 + 8 * zb2) >>4);
        zc0 = ((zb0 + 2 * zb1 + zb2) >>2) - ((4 * zc1 + 8 * zc2) >>4);
        break;
      case BW1800:
        // 1800Hz SSB low-pass filter
        zb0 = ((za0 + 2 * za1 + za2) >>1) - ((0 * zb1 + 4 * zb2) >>4);
        zc0 = ((zb0 + 2 * zb1 + zb2) >>2) - ((0 * zc1 + 4 * zc2) >>4);
        break;
    }
  } else {
    // CW filters
    if (!cwfilter) return za0;  // full
    switch (cwfilter) {
      case BW500:
        // 500Hz band-pass filter
        zb0 = (0 * za0 + 1 * za1 + 0 * za2) + ((114L * zb1 - 57L * zb2) >>6);
        zc0 = (zb0 - 2 * zb1 + zb2) + ((95L * zc1 - 52L * zc2) >>6);
        break; // 600Hz +-250Hz
      case BW250:
        // 200Hz band-pass filter
        zb0 = (0 * za0 + 1 * za1 + 0 * za2) + ((113L * zb1 - 60L * zb2) >>6);
        zc0 = ((zb0 - 2 * zb1 + zb2) >>2) + ((106L * zc1 - 59L * zc2) >>6);
        break; //600Hz +- 100Hz
      case BW100:
        // 100Hz band-pass filter
        zb0 = (0 * za0 + 1 * za1 + 0 * za2) + ((110L * zb1 - 62L * zb2) >>6);
        zc0 = ((zb0 - 2 * zb1 + zb2) >>4) + ((113L * zc1 - 62L * zc2) >>6);
        break; //600Hz +- 50Hz
      case BW50:
        // 50Hz band-pass filter
        zb0 = (0 * za0 + 1 * za1 + 0 * za2) + ((110L * zb1 - 61L * zb2) >>6);
        zc0 = ((zb0 - 2 * zb1 + zb2) >>5) + ((112L * zc1 - 62L * zc2) >>6);
        break; //600Hz +-25Hz
      default:
        break;
    }
  }
  zc2 = zc1;
  zc1 = zc0;
  zb2 = zb1;
  zb1 = zb0;
  za2 = za1;
  za1 = za0;
  if (ssb) return zc0;
  else     return zc0>>3; // compensate the front-end gain
}

#define __UA   256

// another arctan approximation
inline int16_t _arctan3(int16_t q, int16_t i) {
  #define __atan2(z)  (__UA/8  + __UA/22) * z     // not accurate but fast
  int16_t r;
  if (abs(q) > abs(i))
    r = __UA / 4 - __atan2(abs(i) / abs(q));      // arctan(z) = 90-arctan(1/z)
  else
    r = (i == 0) ? 0 : __atan2(abs(q) / abs(i));  // arctan(z)
  r = (i < 0) ? __UA / 2 - r : r;                 // arctan(-z) = -arctan(z)
  return (q < 0) ? -r : r;
}

int16_t  cwac;
uint8_t  absavg256cnt = 0;
uint32_t absavg256 = 0;

volatile uint32_t _absavg256 = 0;
volatile int16_t i, q;

// slow DSP processes
inline int16_t slow_dsp(int16_t ac) {
  if (skip_dsp) return 0;
  if (!absavg256cnt) {
    _absavg256 = absavg256;
    absavg256 = 0;
  } else {
    absavg256 += abs(ac);
  }
  if (radiomode == AM) {
    ac = magn(i, q);
    { static int16_t dc;   // DC decoupling
      dc += (ac - dc) / 2;
      ac = ac - dc; }
  } else if (radiomode == FM) {
    // https://www.veron.nl/wp-content/uploads/2014/01/FmDemodulator.pdf
    static int16_t zi;
    ac = ((ac + i) * zi);  // -qh = ac + i
    zi =i;
  }
  switch (agc) {
    case SLOW:
      ac = process_agc(ac);
      ac = ac >> (18-volume);
      break;
    case FAST:
      ac = process_agc_fast(ac);
      ac = ac >> (18-volume);
      break;
    default:
      // if no AGC allow volume control to boost weak signals
      if (volume <= 13)
        ac = ac >> (13-volume);
      else
        ac = ac << (volume-13);
      break;
  }
  if (nrlevel) ac = process_nr(ac);   // noise reduction
  if (cwdec && (radiomode == CW)) {
    gbuffer(ac);                      // buffer goertzel data
  }
  ac = iirfilter(ac);
  absavg256cnt++;
  ac = min(max(ac, -512), 511);       // clipping
  return ac;
}

volatile uint8_t cat_streaming = 0;
volatile uint8_t _cat_streaming = 0;

typedef void (*func_t)(void);
volatile func_t func_ptr;

// Decimating 2nd order CIC filter
// rate change from 62500/2 kSPS to 7812.5SPS, providing 12dB gain
#define CIC_R  4

volatile uint8_t admux[3];
volatile int16_t ocomb, qh;
volatile uint8_t rx_state = 0;

#pragma GCC push_options
#pragma GCC optimize ("Ofast")  // compiler-optimization for speed

uint8_t rxlpcnt = 0;   // rx loop counter
int16_t ac3;

void process_rx(int16_t i_ac2, int16_t q_ac2) {
#ifdef CAT_STREAMING
  if (cat_streaming) {
    uint8_t out = ac3 + 128;
    if (out == ';') out++;
    Serial.write(out);
  }
#endif // CAT_STREAMING
  static int16_t ozd1, ozd2;  // Output stage
  // hack: on first sample init accumlators of further stages (to prevent instability)
  if (acc_init) { ac3 = 0; ozd1 = 0; ozd2 = 0; acc_init = 0; }
  int16_t od1 = ac3 - ozd1; // Comb section
  ocomb = od1 - ozd2;

  if (rxlpcnt++ == 0) interrupts();  // prevent recursion
  // since slow_dsp process exceeds rx sample-time, allow subsequent 7
  // interrupts for further rx sampling
  ozd2 = od1;
  ozd1 = ac3;
  int16_t qh;
  {
    q_ac2 >>= att2;        // digital gain control
    static int16_t v[14];  // process Q (down-sampled) samples
    // Hilbert transform
    // outi= fir(inl,  0, 0, 0, 0, 0,  0,  0, 1,   0, 0,   0, 0,  0, 0, 0, 0)
    // outq = fir(inr, 2, 0, 8, 0, 21, 0, 79, 0, -79, 0, -21, 0, -8, 0, -2, 0) / 128;
    qh = ((v[0] - q_ac2) + (v[2] - v[12]) * 4) / 64 + ((v[4] - v[10]) + (v[6] - v[8])) / 8 + ((v[4] - v[10]) * 5 - (v[6] - v[8]) ) / 128 + (v[6] - v[8]) / 2;
    v[0]=v[1];
    v[1]=v[2];
    v[2]=v[3];
    v[3]=v[4];
    v[4]=v[5];
    v[5]=v[6];
    v[6]=v[7];
    v[7]=v[8];
    v[8]=v[9];
    v[9]=v[10];
    v[10]=v[11];
    v[11]=v[12];
    v[12]=v[13];
    v[13]=q_ac2;
  }
  i_ac2 >>= att2;         // digital gain control
  static int16_t v[7];    // Post processing I and Q (down-sampled) results
  i = i_ac2; q = q_ac2;   // tbd: this can be more efficient
  // Delay to match Hilbert transform on Q branch
  int16_t i = v[0];
  v[0] = v[1];
  v[1] = v[2];
  v[2] = v[3];
  v[3] = v[4];
  v[4] = v[5];
  v[5] = v[6];
  v[6] = i_ac2;
  // inverting I and Q helps dampening a feedback-loop
  // between PWM out and ADC inputs
  ac3 = slow_dsp(-i - qh);
  rxlpcnt--;
}

int16_t i_s0za1, i_s0zb0, i_s0zb1, i_s1za1, i_s1zb0, i_s1zb1;
int16_t q_s0za1, q_s0zb0, q_s0zb1, q_s1za1, q_s1zb0, q_s1zb1, q_ac2;

#define M_SR  1  // CIC N=3

void sdr_rx_00() {
  int16_t ac = sdr_rx_common_i();
  func_ptr = sdr_rx_01;
  int16_t i_s1za0 = (ac + (i_s0za1 + i_s0zb0) * 3 + i_s0zb1) >> M_SR;
  i_s0za1 = ac;
  int16_t ac2 = (i_s1za0 + (i_s1za1 + i_s1zb0) * 3 + i_s1zb1);
  i_s1za1 = i_s1za0;
  process_rx(ac2, q_ac2);
}

void sdr_rx_02() {
  int16_t ac = sdr_rx_common_i();
  func_ptr = sdr_rx_03;
  i_s0zb1 = i_s0zb0;
  i_s0zb0 = ac;
}

void sdr_rx_04() {
  int16_t ac = sdr_rx_common_i();
  func_ptr = sdr_rx_05;
  i_s1zb1 = i_s1zb0;
  i_s1zb0 = (ac + (i_s0za1 + i_s0zb0) * 3 + i_s0zb1) >> M_SR;
  i_s0za1 = ac;
}

void sdr_rx_06() {
  int16_t ac = sdr_rx_common_i();
  func_ptr = sdr_rx_07;
  i_s0zb1 = i_s0zb0;
  i_s0zb0 = ac;
}

void sdr_rx_01() {
  int16_t ac = sdr_rx_common_q();
  func_ptr = sdr_rx_02;
  q_s0zb1 = q_s0zb0;
  q_s0zb0 = ac;
}

void sdr_rx_03() {
  int16_t ac = sdr_rx_common_q();
  func_ptr = sdr_rx_04;
  q_s1zb1 = q_s1zb0;
  q_s1zb0 = (ac + (q_s0za1 + q_s0zb0) * 3 + q_s0zb1) >> M_SR;
  q_s0za1 = ac;
}

void sdr_rx_05() {
  int16_t ac = sdr_rx_common_q();
  func_ptr = sdr_rx_06;
  q_s0zb1 = q_s0zb0;
  q_s0zb0 = ac;
}

void sdr_rx_07() {
  int16_t ac = sdr_rx_common_q();
  func_ptr = sdr_rx_00;
  int16_t q_s1za0 = (ac + (q_s0za1 + q_s0zb0) * 3 + q_s0zb1) >> M_SR;
  q_s0za1 = ac;
  q_ac2 = (q_s1za0 + (q_s1za1 + q_s1zb0) * 3 + q_s1zb1);
  q_s1za1 = q_s1za0;
}

int16_t ozi1, ozi2;

// ADC read (Q)
inline int16_t sdr_rx_common_q() {
  ADMUX = admux[0];
  ADCSRA |= (1 << ADSC);
  int16_t ac = ADC - 511;
  return ac;
}

// ADC read (I)
inline int16_t sdr_rx_common_i() {
  ADMUX = admux[1];
  ADCSRA |= (1 << ADSC);
  int16_t adc = ADC - 511;
  static int16_t prev_adc;
  int16_t ac = (prev_adc + adc) / 2;
  prev_adc = adc;
  if (acc_init) { ocomb=0; ozi1 = 0; ozi2 = 0; } // hack
  ozi2 = ozi1 + ozi2;          // Integrator section
  ozi1 = ocomb + ozi1;
  OCR1AL = min(max((ozi2>>4), 0), 255);
  return ac;
}

// Timer2 COMPA interrupt
ISR(TIMER2_COMPA_vect) {
  func_ptr();
}

#pragma GCC pop_options  // end of DSP section

void adc_start(uint8_t adcpin, uint8_t ref1v1, uint32_t fs) {
  DIDR0 |= (1 << adcpin);
  ADCSRA = 0;
  ADCSRB = 0;
  ADMUX = 0;
  ADMUX |= (adcpin & 0x0f);
  ADMUX |= (ref1v1 ? (1 << REFS1) : 0) | (1 << REFS0);
  ADCSRA |= ((uint8_t)log2((uint8_t)(F_CPU / 13 / fs))) & 0x07;
  ADCSRA |= (1 << ADEN);    // enable ADC
#ifdef ADC_NR
  set_sleep_mode(SLEEP_MODE_IDLE);
  sleep_enable();
#endif
}

void adc_stop() {
  ADCSRA &= ~(1 << ADIE);
  // 128 prescaler for 9.6kHz
  ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
#ifdef ADC_NR
  sleep_disable();
#endif // ADC_NR
  // restore 5V ref voltage
  ADMUX = (1 << REFS0);
}

// start timer 1 (OC1A and OC1B in PWM mode)
void timer1_start(uint32_t fs) {
  TCCR1A = 0;
  TCCR1B = 0;
  TCCR1A |= (1 << COM1A1) | (1 << COM1B1) | (1 << WGM11);
  TCCR1B |= (1 << CS10) | (1 << WGM13) | (1 << WGM12);
  ICR1H = 0x00;
  ICR1L = min(255, F_CPU / fs);
  OCR1AH = 0x00;
  OCR1AL = 0x00;  // OC1A (SIDETONE)
  OCR1BH = 0x00;
  OCR1BL = 0x00;  // OC1B (TXPWM)
}

// stop timer 1
void timer1_stop() {
  OCR1AL = 0x00;
  OCR1BL = 0x00;
}

// Timer 2: interrupt mode
void timer2_start(uint32_t fs) {
  ASSR &= ~(1 << AS2);
  TCCR2A = 0;
  TCCR2B = 0;
  TCNT2 = 0;
  TCCR2A |= (1 << WGM21);
  TCCR2B |= (1 << CS22);
  TIMSK2 |= (1 << OCIE2A);
  uint8_t ocr = ((F_CPU / 64) / fs) - 1;
  OCR2A = ocr;
}

// Stop Timer 2 interrupt
void timer2_stop() {
  TIMSK2 &= ~(1 << OCIE2A);
  delay(1);
}

const char* step_label[]  = { "1M ", ".1M", "10k", " 1k", "100", " 10", " 1 "};

// display the top line
void show_banner() {
  // display the Tx/Rx status
  oled.setCursor(0, ROW1);
  oled.print('R');
  oled.print(' ');
  // display the step size
  oled.setCursor(STEPCOL, ROW1);
  oled.print(step_label[stepsize]);
  oled.print("  ");
  oled.setCursor(TWPMCOL, ROW1);
  // display the keyer wpm value
  if (radiomode == CW) oled.print(keyerspeed);
  else oled.print("  ");
  oled.clr2eol();
}

// wait for button release
void wait4release(uint8_t button = 0) {
  switch (button) {
    case EXIT:
      while (EXIT_PRESSED) {
        wait_ms(1);
        wdt_reset();
      }
    break;
    case MENU:
      while (MENU_PRESSED) {
        wait_ms(1);
        wdt_reset();
      }
    break;
    case ENCO:
      while (ENCO_PRESSED) {
        wait_ms(1);
        wdt_reset();
      }
    break;
    default:
      while (BUTTON_PRESSED) {
        wait_ms(1);
        wdt_reset();
      }
    break;
  }
  delay(20);  // debounce
}

// display locked status
void show_lock() {
  oled.setCursor(0, ROW1);
  oled.print("UI Locked");
  oled.print('\x01');
  oled.clr2eol();
  wait4release(EXIT);
  wait4release(MENU);
  show_banner();
}

// display unlocked status
void show_unlock() {
  oled.setCursor(0, ROW1);
  oled.print("UI Unlocked");
  oled.print('\x01');
  oled.clr2eol();
  wait4release(EXIT);
  show_banner();
}

// display edit string status
void show_edit() {
  oled.setCursor(0, ROW1);
  oled.print("Edit");
  oled.print('\x01');
  oled.clr2eol();
}

// select a menu entry
void select_menu() {
  menu += encoder_val;
  if (menu == N_PARAMS) menu = 0;        // wrap
  else if (menu < 0) menu = N_PARAMS-1;  // wrap
  menuAction(menu);
  encoder_val = 0;
}

// Calculate new keyer time constants
void loadWPM (uint8_t wpm) {
  dittime   = (1200 * 5/4)/wpm;
  dahtime   = (dittime<<1) + dittime;       // 3x   dittime
  lettergap = (dittime<<1) + (dittime>>1);  // 2.5x dittime
  wordgap   = (dittime<<2);                 // 4x   dittime
}

// performs classical analogRead with default Arduino sample-rate
// and analog reference setting; restores previous settings
int analogSafeRead(uint8_t pin, uint8_t ref1v1 = OFF) {
  noInterrupts();
  for (;!(ADCSRA & (1 << ADIF)););  // wait for previous ADC
  uint8_t adcsra = ADCSRA;
  uint8_t admux = ADMUX;
  // disable interrupts when measurement complete
  ADCSRA &= ~(1 << ADIE);
  // 128 prescaler for 9.6kHz
  ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
  // restore reference voltage
  if (ref1v1) ADMUX &= ~(1 << REFS0);  // AREF (1V1)
  else ADMUX = (1 << REFS0);           // AREF (5V)
  delay(1);
  int val = analogRead(pin);
  ADCSRA = adcsra;
  ADMUX = admux;
  interrupts();
  return val;
}

uint16_t analogSampleMic() {
  uint16_t adc;
  noInterrupts();
  // hack: a faster conversion rate is necessary for VOX
  ADCSRA = (1 << ADEN) | (((uint8_t)log2((uint8_t)(F_CPU / 13 / (192307/1)))) & 0x07);
  // disable RF input, only for SDR mod and with low VOX threshold
  if ((voxlevel >= 32)) digitalWrite(RXEN, LOW);
  uint8_t oldmux = ADMUX;
  for (;!(ADCSRA & (1 << ADIF)););  // wait for previous ADC
  ADMUX = admux[2];                 // set ADC MUX for next conversion
  ADCSRA |= (1 << ADSC);            // start ADC
  for (;!(ADCSRA & (1 << ADIF)););  // wait until ADC is completed
  ADMUX = oldmux;
  // enable RF input, only for SDR mod and with low VOX threshold
  if (voxlevel >= 32) digitalWrite(RXEN, HIGH);
  adc = ADC;
  interrupts();
  return adc;
}

uint8_t outofband = NO;
uint8_t update_freq = YES;
int32_t curfreq = 14060000;
int16_t ritval = 0;

uint32_t vfo[] = { 14060000, 14060000 };
uint8_t  vfomode[] = { CW, CW };

uint32_t max_absavg256 = 0;
uint16_t smeter_cnt = 0;

#define SMCMAX 2048   // smeter calculation rate

// S-meter calculations
void calc_smeter() {
  char tmp[5];
  uint8_t sm;
  int8_t  sb;
  float   rms;
  int16_t dbm;
  max_absavg256 = max(_absavg256, max_absavg256);
  // check if calculation is needed
  if (smeter && !smeter_cnt) {
    if (smeter != RXWPM) {
      rms = (float)max_absavg256 * (float)(1 << att2);
      rms /= (256.0 * 1024.0 * (float)CIC_R * 8.0 * 500.0 * 1.414 / (0.707 * 1.1));
      dbm = 10 * log10((rms * rms) / 50) + 30;  // rms to dBm at 50R
    }
    switch (smeter) {
      case DBM:
        // print the power in dBm
        oled.setCursor(DBMCOL, ROW1);
        oled.print(dbm);
        oled.print("dBm ");
        break;
      case SVAL:
        // print the S-value
        sm = 9;
        if (dbm < -73)  sm = 8;
        if (dbm < -79)  sm = 7;
        if (dbm < -85)  sm = 6;
        if (dbm < -91)  sm = 5;
        if (dbm < -97)  sm = 4;
        if (dbm < -103) sm = 3;
        if (dbm < -109) sm = 2;
        if (dbm < -115) sm = 1;
        oled.setCursor(SVALCOL, ROW1);
        oled.print('S');
        oled.print(sm);
        break;
      case RXWPM:
        // print the measured rx wpm
        show_rxwpm();
        break;
      default:
        break;
    }
    max_absavg256 /= 2;  // peak hold/decay
  }
  if (smeter_cnt++ == SMCMAX) smeter_cnt = 0;
}

void start_rx() {
  acc_init = 1;
  rx_state = 0;
  func_ptr = sdr_rx_00;  //enable RX DSP/SDR
  adc_start(0, !(attrx == 1), F_ADC_CONV); admux[0] = ADMUX;
  adc_start(1, !(attrx == 1), F_ADC_CONV); admux[1] = ADMUX;
  adc_start(2, ON, F_ADC_CONV*4);          admux[2] = ADMUX;
  timer1_start(F_SAMP_PWM);
  timer2_start(F_SAMP_RX);
  TCCR1A &= ~(1 << COM1B1);
  digitalWrite(TXPWM, LOW); // disable TXPWM
}

int16_t _centiGain = 0;
uint32_t semiqsk_timeout = 0;

// switch receive/transmit
void switch_rxtx(uint8_t tx_enable, uint8_t show) {
  // disable timer compare interrupt
  TIMSK2 &= ~(1 << OCIE2A);
  // wait until potential RX interrupt is finalized
  delayMicroseconds(20);
  noInterrupts();
  if (!semiqsk_timeout)
    if (txdelay && tx_enable && !tx && !practice) {
      // key-up TX relay in advance before actual transmission
      digitalWrite(RXEN, LOW);  // TX (disable RX)
#ifdef NTX
      digitalWrite(NTX, LOW);   // TX (enable TX)
#endif // NTX
#ifdef PTX
      digitalWrite(PTX, HIGH);  // TX (enable TX)
#endif // PTX
      for (uint8_t i=0; i<txdelay; i++) {
        delayMicroseconds(100);
      }
    }
  tx = tx_enable;
  if (tx_enable) {
    _centiGain = centiGain;  // backup AGC setting
    semiqsk_timeout = 0;
    switch (radiomode) {
      case USB:
      case LSB: func_ptr = dsp_tx; break;
      case CW:  func_ptr = dsp_tx_cw; break;
      case AM:  func_ptr = dsp_tx_am; break;
      case FM:  func_ptr = dsp_tx_fm; break;
    }
  } else {  // rx
    if ((radiomode == CW) && (!(semiqsk_timeout))) {
      semiqsk_timeout = millis() + dittime * 8;
      if (semiqsk) func_ptr = dummy;
      else func_ptr = sdr_rx_00;
    } else {
      centiGain = _centiGain;  // restore AGC setting
      semiqsk_timeout = 0;
      func_ptr = sdr_rx_00;
    }
  }
  interrupts();
  if (tx_enable) ADMUX = admux[2];
  else acc_init = 1;
  rx_state = 0;
  if (tx_enable) { // tx
    if (practice) {
      digitalWrite(RXEN, LOW); // TX (disable RX)
      if (show) {
        oled.setCursor(0, ROW1);
        // print the practice indicator
        oled.print('P');
      }
      si5351.SendRegister(SI_CLK_OE, TX0RX0);
      // Do not enable PWM (TXPWM), do not enble CLK2
    } else {
      digitalWrite(RXEN, LOW);  // TX (disable RX)
#ifdef NTX
      digitalWrite(NTX, LOW);   // TX (enable TX)
#endif // NTX
#ifdef PTX
      digitalWrite(PTX, HIGH);  // TX (enable TX)
#endif // PTX
      if (show) {
        oled.setCursor(0, ROW1);
        // print the Tx indicator
        oled.print('T');
      }
      if (radiomode == CW) {
        si5351.freq_calc_fast(-cw_offset);
        si5351.SendPLLRegisterBulk();
      } else if (ritmode) {
        si5351.freq_calc_fast(0);
        si5351.SendPLLRegisterBulk();
      }
      si5351.SendRegister(SI_CLK_OE, TX1RX0);
      OCR1AL = MIDPOINT;  // SIDETONE to 2.5V
      if (radiomode != CW) TCCR1A &= ~(1 << COM1A1);  // disable SIDETONE
      TCCR1A |= (1 << COM1B1);  // enable TXPWM PWM
    }
  } else {  // rx
    if (OCR1BL != 0) {
      // ramp down the transmit PWM amplitude
      // soft falling edge to prevent key clicks
      for (uint16_t i=0; i!=31; i++) {
        if (OCR1BL > rcos[i]) {
          OCR1BL = rcos[i];
          delayMicroseconds(60);
        }
      }
      OCR1BL = 0;
    }
    // enable SIDETONE (was disabled to prevent interference during ssb tx)
    // disable TXPWM prevents interference during RX
    TCCR1A |= (1 << COM1A1);
    TCCR1A &= ~(1 << COM1B1);
    digitalWrite(TXPWM, LOW);
    OCR1BL = 0; // make sure PWM (TXPWM) is set to 0%
#ifdef QUAD
    si5351.SendRegister(18, 0x0f);  // disable invert on CLK2
#endif //QUAD
    si5351.SendRegister(SI_CLK_OE, TX0RX1);
    // enable RX when not in semi-qsk phase; so RX and
    // NTX/PTX outputs are switching only when in RX mode
    if (!semiqsk_timeout || !semiqsk) {
      digitalWrite(RXEN, !(attrx == 2)); // enable RX if attenuator not on
#ifdef NTX
      digitalWrite(NTX, HIGH);  // RX (disable TX)
#endif // NTX
#ifdef PTX
      digitalWrite(PTX, LOW);   // TX (disable TX)
#endif // PTX
    }
    si5351.freq_calc_fast(ritval);
    si5351.SendPLLRegisterBulk();  // restore original PLL RX frequency
    if (show && (menumode == NOT_IN_MENU)) {
      oled.setCursor(0, ROW1);
      // print the Rx indicator
      oled.print('R');
    }
  }
  OCR2A = ((F_CPU / 64) / (tx_enable ? F_SAMP_TX : F_SAMP_RX)) - 1;
  TIMSK2 |= (1 << OCIE2A);  // enable timer compare interrupt
}

void set_freq() {
  switch (radiomode) {
    case CW:
      si5351.freq(curfreq + cw_offset, iqphase, 0);
      break;
    case LSB:
      si5351.freq(curfreq, iqphase, 0);
      break;
    case USB:
      si5351.freq(curfreq, 0, iqphase);
      break;
    default:
      si5351.freq(curfreq, 0, iqphase);
      break;
  }
  if (ritmode) {
    si5351.freq_calc_fast(ritval);
    si5351.SendPLLRegisterBulk();
   }
}

#define N_BANDS 8

// band lower limits
const uint32_t bandlow[N_BANDS] = {
// 10m       15m       17m       20m
// 30m       40m       60m       80m
   28000000, 21000000, 18070000, 14000000,
   10100000, 7000000,  5300000,  3500000
};

// band CW upper limits
const uint32_t cwupr[N_BANDS] = {
// 10m       15m       17m       20m
// 30m       40m       60m       80m
   28300000, 21200000, 18110000, 14070000,
   10150000, 7125000,  5500000,  3600000,
};

// band upper limits
const uint32_t bandtop[N_BANDS] = {
// 10m       15m       17m       20m
// 30m       40m       60m       80m
   29700000, 21450000, 18170000, 14350000,
   10150000, 7300000,  5500000,  4000000,
};

// uncomment to enable

//#define FT8_FREQS   1  // use FT8 frequencies
//#define CWF_FREQS   1  // use CW FISTS frequencies
#define   QRP_FREQS   1  // use QRP frequencies

#ifdef FT8_FREQS
uint32_t bandfreq[N_BANDS] = { 28074000, 21074000, 18100000, 14074000,\
                               10136000, 7074000, 5357000, 3573000 };
#endif
#ifdef CWF_FREQS
uint32_t bandfreq[N_BANDS] = { 28058000, 21058000, 18085000, 14058000,\
                               10118000, 7028000, 5351500, 3558000 };
#endif
#ifdef QRP_FREQS
uint32_t bandfreq[N_BANDS] = { 28060000, 21060000, 18096000, 14060000,\
                               10106000, 7030000, 5351500, 3560000 };
#endif

const uint32_t stepsizes[7] = { 1000000, 100000, 10000, 1000, 100, 10, 1 };

#define MAXRIT      9999
#define MINRIT     -9999
#define MAXFREQ  30000000  // upper frequency limit = 30 MHz
#define MINFREQ   1000000  // lower frequency limit =  1 MHz

void tune_freq() {
  if (ritmode) {
    int16_t newrit = ritval + (encoder_val * stepsizes[stepsize]);
    if      (newrit < MINRIT) newrit = MINRIT;
    else if (newrit > MAXRIT) newrit = MAXRIT;
    ritval = newrit;
  } else {
    int32_t newfreq = curfreq + (encoder_val * stepsizes[stepsize]);
    // check frequency bounds
    if      (newfreq < MINFREQ) newfreq = MINFREQ;
    else if (newfreq > MAXFREQ) newfreq = MAXFREQ;
    // check band bounds
    if ((newfreq < bandlow[bandsel])||(newfreq > bandtop[bandsel])) {
      if (!outofband) {
        oled.clrLine(ROW3);
        oled.print("Out of Band..");
        outofband = YES;
      }
    } else {
      if (outofband) {
        oled.clrLine(ROW3);
        outofband = NO;
      }
    }
    curfreq = newfreq;
  }
  update_freq = YES;
  encoder_val = 0;
}

// change the frequency step size
void stepsize_change() {
  stepsize++;
  if (ritmode) {
    if (stepsize < STEP_100) stepsize = STEP_1;    // wrap
    if (stepsize > STEP_1)   stepsize = STEP_100;  // wrap
  } else {
    if (stepsize < STEP_1M) stepsize = STEP_1;     // wrap
    if (stepsize > STEP_1)  stepsize = STEP_1M;    // wrap
  }
  show_banner();
}

// change the reference frequency step size
void ref_stepsize_change() {
  stepsize++;
  if (stepsize < STEP_1M) stepsize = STEP_1;   // wrap
  if (stepsize > STEP_1)  stepsize = STEP_1M;  // wrap
  // display the new step size
  oled.setCursor(5, ROW3);
  oled.print(stepsizes[stepsize]);
  oled.clr2eol();
}

void powerDown() {
  oled.setCursor(0, ROW1);
  oled.print("Going to Sleep..");
  oled.clrR234();
  oled.print("turn VFO to wake");
  oled.clr2eol();
  MCUSR = ~(1<<WDRF);  // must be done before wdt_disable()
  wdt_disable();
  timer2_stop();
  timer1_stop();
  adc_stop();
  si5351.powerDown();
  delay(TWO_SECONDS);
  oled.noDisplay();
  // disable interrupts
  PCMSK0 = 0;
  PCMSK1 = 0;
  TIMSK0 = 0;
  TIMSK1 = 0;
  TIMSK2 = 0;
  WDTCSR = 0;
  // power-down sub-systems
  PRR = 0xff;
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();
  interrupts();
  sleep_bod_disable();
  // go to sleep mode and wake-up on encoder change
  sleep_cpu();
  sleep_disable();
  // soft reset by trigger watchdog timeout
  do { wdt_enable(WDTO_15MS); while(1); } while (0);  // wtf??
}

const char* vfosel_label[] = { "A", "B" };
const char* rmode_label[]  = { "LSB", "USB", "CW ", "FM ", "AM " };

void show_vfo(int32_t fx, uint8_t mode) {
  int32_t scale = 10000000;  // 10 MHz
  oled.setCursor(0, ROW4);
  if (ritmode) {
    fx = ritval;
    scale = 1000;
    oled.print("RIT ");
    oled.print(ritval < 0 ? '-' : '+');
  } else {
    if (vfosel == VFOA) oled.print('\x02');  // A
    else oled.print('\x03');                 // B
    if (fx/scale == 0) {
      oled.print(' ');  // initial space instead of zero
      scale/=10;
    }
  }
  for (; scale; fx%=scale, scale/=10) {
    oled.print(abs(fx/scale));
    if (scale == 1000 || scale == 1000000) oled.print(',');
  }
  oled.print("   ");
  oled.setCursor(RMODECOL, ROW4);
  oled.print(rmode_label[mode]);
}

void check_ditdah() {
  // play tone if dit or dah is pressed
  if (DIT_OR_DAH) {
    playtone(ON);
    while (DIT_OR_DAH) {
      wait_ms(1);
      wdt_reset();
    }
    playtone(OFF);
    wait_ms(20);  // debounce
  }
}

// refresh LUT based on pwm_min, pwm_max
void build_lut() {
  for (uint16_t i=0; i<256; i++)
    lut[i] = (i * (pwm_max - pwm_min)) / 255 + pwm_min;
}

uint8_t sfindex = 0;

#define MAXIDX 10  // number of saved-array entries

// saved-frequency array
uint32_t sfmem[] = { 0,
   14060010, 14060020,
   28060000, 21060000, 18096000, 14060000,
   10106000, 7030000,  5351500,  3560000};

// saved-radiomode array
uint32_t srmem[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

// display a saved frequency
void show_savedfreq() {
  oled.setCursor(0, ROW1);
  if (sfindex==0) {
    oled.print("Current Freq ");
  } else {
    oled.print("Freq Mem ");
    oled.print(sfindex);
  }
  oled.clr2eol();
  show_vfo(sfmem[sfindex], srmem[sfindex]);
}

#define F_MAX    28000000   // Si5351 crystal max value
#define F_MIN    24000000   // Si5351 crystal min value

// menu parameter actions
void paramAction(uint8_t id, uint8_t wrap, uint8_t* ptr,
  const char* label, const char* etag[], uint8_t min, uint8_t max) {
  uint8_t value = *ptr;
  // print the menu label
  print_label(label);
  if (menumode == SELECT_VALUE) {
    // check for encoder input
    if (encoder_val) {
      if (id == REFCAL) {
        // Si5351 crystal calibration value
        fxtal += (encoder_val * stepsizes[stepsize]);
        if      (fxtal < F_MIN) fxtal = F_MIN;
        else if (fxtal > F_MAX) fxtal = F_MAX;
      } else {
        // for all other parameters
        if (wrap) {
          if      ((value + encoder_val) < min) value = max;
          else if ((value + encoder_val) > max) value = min;
          else value += encoder_val;
        } else {
          // no wrap
          if      ((value + encoder_val) < min) value = min;
          else if ((value + encoder_val) > max) value = max;
          else value += encoder_val;
        }
      }
      // additional parameter actions
      switch (id) {
        case BAND:
          set_lpf(value);
          curfreq = bandfreq[value];
          break;
        case RADIOMODE:
          vfomode[vfosel] = value;
          if (value != CW) gdbtime=0;
          si5351.iqmsa = 0;
          break;
        case VFOSEL:
          curfreq = vfo[vfosel];
          radiomode = vfomode[vfosel];
          update_freq = YES;
          break;
        case TXDELAY:
          semiqsk = value;
          break;
        case PRACTICE:
          // save the practice state
          practice_save = value;
          break;
        case CWDEC:
          // save the CW decoder state
          cwdec_save = value;
          if (cwdec != value) {
            xmit_col = 0;
            recv_col = 0;
          }
          break;
        case KEYERWPM:
          loadWPM(value);
          break;
        case KEYERMODE:
          keyerinfo = 0;
          break;
        case KEYSWAP:
          if (value) {
            ditkey = DAH;
            dahkey = DIT;
          } else {
            ditkey = DIT;
            dahkey = DAH;
          }
          break;
        case ATTRX:
          // Rx attenuation
          // att bit 0 ON: attenuate -13dB by changing the ADC reference
          noInterrupts();
          adc_start(0, !(attrx & 0x01), F_ADC_CONV); admux[0] = ADMUX;
          adc_start(1, !(attrx & 0x01), F_ADC_CONV); admux[1] = ADMUX;
          interrupts();
          // att bit 1 ON: attenuate -20dB by disabling RX line
          // switching antenna input into 100k resistence
          digitalWrite(RXEN, !(attrx & 0x02));
          // att bit 2 ON: attenuate -40dB by terminating ADC inputs with 10R
          pinMode(QSDQ, (attrx & 0x04) ? OUTPUT : INPUT);
          pinMode(QSDI, (attrx & 0x04) ? OUTPUT : INPUT);
          break;
        case SWRMETER:
          // restore CW decoder state
          if (!value) cwdec = cwdec_save;
          break;
        case IQPHASE:
        case REFCAL:
          update_freq = YES;
          break;
        case PWM_MIN:
        case PWM_MAX:
          build_lut();
          break;
        default:
          break;
      }
      *ptr = value;
    }
  }
  // print value
  if (etag == NULL) {
    switch (id) {
      case REFCAL:
        oled.clrLine(ROW2);
        oled.setCursor(0, ROW3);
        if (menumode == SELECT_VALUE) oled.print('>');
        oled.print("Step ");
        oled.print(stepsizes[stepsize]);
        oled.clr2eol();
        oled.setCursor(0, ROW4);
        // print the xtal reference frequency
        oled.print(fxtal);
        break;
      default:
        oled.clrR234();
        // print the parameter numeric value
        if (menumode == SELECT_VALUE) oled.print('>');
        if (id == BLANK) {
          if (!value) {
            oled.print("OFF");
          } else {
            oled.print(value);
            oled.print(" MINUTE");
            if (value != 1) oled.print("S");
          }
        } else {
          oled.print(value);
        }
        break;
    }
  } else {
    oled.clrR234();
    // print the parameter string
    if (menumode == SELECT_VALUE) oled.print('>');
    oled.print(etag[value]);
  }
  encoder_val = 0;
  oled.clr2eol();
}

// CW message action
void cwmsgAction(uint8_t id, const char* label) {
  print_label(label);
  print_cwmsg(id-CWMSG1);
}

// string edit action
void editsAction(char* ptr, const char* label) {
  print_label(label);
  oled.setCursor(0, ROW2);
  oled.print("--> press menu  ");
  oled.setCursor(0, ROW3);
  oled.print("to edit string  ");
  oled.setCursor(0, ROW4);
  editstr = ptr;
  oled.print(editstr);
  oled.clr2eol();
}

// other menu parameter action
void otherAction(uint8_t id, const char* label) {
  uint8_t event;      // click or long press
  print_label(label);
  float vbatt = (float)analogSafeRead(VBAT, OFF) / 18.6;
  switch (id) {
    case FREQMEM:
      if (menumode == SELECT_MENU) {
        oled.setCursor(0, ROW2);
        oled.print("--> press menu  ");
        oled.setCursor(0, ROW3);
        oled.print("  to view list  ");
        oled.clrLine(ROW4);
      } else {
        oled.clrR23();
        sfindex = 0;
        sfmem[0] = curfreq;   // save the current freq
        srmem[0] = radiomode; // save the current radio mode
        show_savedfreq();
        // scroll through the saved frequencies
        while (!EXIT_PRESSED) {
          if (encoder_val) {
            int8_t i = sfindex + encoder_val;
            if (i < 0) i = MAXIDX;        // wrap
            else if (i > MAXIDX) i = 0;   // wrap
            sfindex = i;
            show_savedfreq();
            encoder_val = 0;
          }
          if (MENU_PRESSED) {
            // save current frequency
            oled.setCursor(0, ROW1);
            oled.print("Saved to ");
            oled.print(sfindex);
            sfmem[sfindex] = curfreq;
            srmem[sfindex] = radiomode;
            show_savedfreq();
            oled.clr2eol();
            wait_ms(1000);   // delay for display
            wait4release(MENU);
          }
          wait_ms(1);
        }
        wait_ms(20);   // debounce
        // the exit button was pressed
        // so load the stored frequency and exit
        curfreq   = sfmem[sfindex];
        radiomode = srmem[sfindex];
        menumode = NOT_IN_MENU;
        wait4release(EXIT);
        update_freq = YES;
        show_banner();
      }
      break;
    case DIAG:
      if (menumode == SELECT_MENU) {
        oled.setCursor(0, ROW2);
        oled.print("--> press menu  ");
        oled.setCursor(0, ROW3);
        oled.print("  to run diags  ");
        oled.clrLine(ROW4);
      } else {
        // run diagnotics and shut down
        run_diags();
      }
      break;
    case SHDN:
      if (menumode == SELECT_MENU) {
        oled.setCursor(0, ROW2);
        oled.print("--> press menu  ");
        oled.setCursor(0, ROW3);
        oled.print("  to shut down  ");
        oled.clrLine(ROW4);
      } else {
        powerDown();
      }
      break;
    case SAVE:
      if (menumode == SELECT_MENU) {
        oled.setCursor(0, ROW2);
        oled.print("--> press menu  ");
        oled.setCursor(0, ROW3);
        oled.print("to save params  ");
        oled.clrLine(ROW4);
      } else {
        // save parameters and shut down
        saveAll();
      }
      break;
    case RIT:
      if (menumode == SELECT_MENU) {
        oled.clrR234();
        ritmode = ON;
        ritval = 0;
        oled.setCursor(0, ROW2);
        oled.print("--> press menu  ");
        oled.setCursor(0, ROW3);
        oled.print("for RIT tuning  ");
        show_vfo(0, radiomode);
        ritmode = OFF;
      } else {
        ritmode = ON;
        stepsize = STEP_10;
        menumode = NOT_IN_MENU;
        show_banner();
        oled.clrR23();
        show_vfo(0, radiomode);
        update_freq = YES;
      }
      break;
    case CWCAL:
      if (menumode == SELECT_MENU) {
        event = NONE;
        oled.clrR23();
        oled.setCursor(0, ROW4);
        show_vfo(curfreq, radiomode);
      } else {
        show_banner();
        show_vfo(curfreq, radiomode);
        menumode = NOT_IN_MENU;
        cwdec_save = cwdec;
        cwdec = RXTX;
        radiomode = CW;
        event = NONE;
        while (1) {
          // play tone if dit/dah pressed to help zero-beat
          check_ditdah();
          // run goertzel if data is ready
          if (gdbrdy) rxcw_decode(YES);
          // check for encoder button click
          if (ENCO_PRESSED) {
            event = CLICK;
            t0 = millis();
            while (ENCO_PRESSED && (event != LONG)) {
              // until released or long press
              if ((millis() - t0) > MS_LONG) { event = LONG; break; }
              wait_ms(1);
            }
            if (event == CLICK) stepsize_change(); // update step
            else {
              while (ENCO_PRESSED) {
                if (encoder_val) {
                  int8_t i = volume + encoder_val;
                  if (i>15) i=15;
                  if (i<0)  i=0;
                  volume = i;
                  encoder_val = 0;
                  oled.setCursor(TWPMCOL, ROW1);
                  oled.print(volume);
                  oled.print(' ');
                }
                wdt_reset();
              }
              // restore the Tx WPM
              oled.setCursor(TWPMCOL, ROW1);
              oled.print(keyerspeed);
            }
            wait4release(ENCO);
          }
          // check for menu button click
          if (MENU_PRESSED) {
            // frequency scanning
            while (1) {
              encoder_val = 1;
              tune_freq();
              // check band CW frequency limits
              if (curfreq > cwupr[bandsel]) curfreq = bandlow[bandsel];
              set_freq();
              vfo[vfosel] = curfreq;
              show_vfo(curfreq, radiomode);
              encoder_val = 0;
              event = scan_wait_ms(500);
              if (event == CLICK) {
                wait4release(EXIT);
                break;    // stop scanning
              }
            }
            event = NONE;
            wait4release(MENU);
          }
          // check for exit button click
          if (EXIT_PRESSED) {
            wait4release(EXIT);
            break;  // exit cal loop
          }
          if (encoder_val) {
            tune_freq();
            set_freq();
            vfo[vfosel] = curfreq;
            show_vfo(curfreq, radiomode);
            encoder_val = 0;
          }
          wdt_reset();
        }
        // this is the cal loop exit point
        oled.clrR23();
        event = NONE;
        cwdec = cwdec_save;  // restore decoder settings
      }
      break;
    case BATT:
      oled.setCursor(0, ROW2);
      oled.print("    Voltage     ");
      if (menumode == SELECT_VALUE) {
        menumode = SELECT_MENU;
        encoder_val = 0;
        return;
      }
      oled.clrLine(ROW3);
      oled.setCursor(0, ROW4);
      oled.print(vbatt);
      oled.print('V');
      oled.clr2eol();
      break;
    case SWVER:
      if (menumode == SELECT_VALUE) {
        menumode = SELECT_MENU;
        encoder_val = 0;
        return;
      }
      oled.clrR23();
      oled.setCursor(0, ROW3);
      oled.print(VERSION);
      oled.clr2eol();
      oled.setCursor(0, ROW4);
      oled.print(TIMESTAMP);
      oled.clr2eol();
      break;
    default:
      break;
  }
}

uint8_t vox_tx = 0;
uint8_t vox_sample = 0;
uint16_t vox_adc = 0;

// remove trailing spaces
void clean_string(uint8_t size, char* str) {
  for (uint8_t i=size; i>0; i--) {
    if ((str[i-1] == ' ') || (str[i-1] == 0)) str[i-1] = 0;
    else break; // stop once content found
  }
}

#define EEPROM_OFFSET 0x00

// load user settings from eeprom
void loadAll() {
  uint16_t addr = EEPROM_OFFSET;
  oled.setCursor(0, ROW2);
  oled.print("Loading..       ");
  volume        = EEPROM.readByte(addr);  addr++;
  tonevol       = EEPROM.readByte(addr);  addr++;
  bwfilter      = EEPROM.readByte(addr);  addr++;
  cwfilter      = EEPROM.readByte(addr);  addr++;
  practice      = EEPROM.readByte(addr);  addr++;
  cwdec         = EEPROM.readByte(addr);  addr++;
  semiqsk       = EEPROM.readByte(addr);  addr++;
  keyermode     = EEPROM.readByte(addr);  addr++;
  keyerspeed    = EEPROM.readByte(addr);  addr++;
  keyswap       = EEPROM.readByte(addr);  addr++;
  xtimeout      = EEPROM.readByte(addr);  addr++;
  vfosel        = EEPROM.readByte(addr);  addr++;
  agc           = EEPROM.readByte(addr);  addr++;
  nrlevel       = EEPROM.readByte(addr);  addr++;
  attmic        = EEPROM.readByte(addr);  addr++;
  attrx         = EEPROM.readByte(addr);  addr++;
  att2          = EEPROM.readByte(addr);  addr++;
  smeter        = EEPROM.readByte(addr);  addr++;
  swrmeter      = EEPROM.readByte(addr);  addr++;
  vox           = EEPROM.readByte(addr);  addr++;
  voxlevel      = EEPROM.readByte(addr);  addr++;
  drive         = EEPROM.readByte(addr);  addr++;
  txdelay       = EEPROM.readByte(addr);  addr++;
  pwm_min       = EEPROM.readByte(addr);  addr++;
  pwm_max       = EEPROM.readByte(addr);  addr++;
  iqphase       = EEPROM.readByte(addr);  addr++;
  vfomode[VFOB] = EEPROM.readByte(addr);  addr++;
  vfomode[VFOA] = EEPROM.readByte(addr);  addr++;
  vfo[VFOA]     = EEPROM.readLong(addr);  addr+=4;
  vfo[VFOB]     = EEPROM.readLong(addr);  addr+=4;
  fxtal         = EEPROM.readLong(addr);  addr+=4;
  for (uint8_t i=1; i<=MAXIDX; i++) {
    sfmem[i] = EEPROM.readLong(addr); addr+=4;
    srmem[i] = EEPROM.readByte(addr); addr++;
  }
}

// save user settings to eeprom
void saveAll() {
  uint16_t addr = EEPROM_OFFSET;
  oled.setCursor(0, ROW2);
  oled.print("Saving..       ");
  EEPROM.updateByte(addr, volume);        addr++;
  EEPROM.updateByte(addr, tonevol);       addr++;
  EEPROM.updateByte(addr, bwfilter);      addr++;
  EEPROM.updateByte(addr, cwfilter);      addr++;
  EEPROM.updateByte(addr, practice);      addr++;
  EEPROM.updateByte(addr, cwdec);         addr++;
  EEPROM.updateByte(addr, semiqsk);       addr++;
  EEPROM.updateByte(addr, keyermode);     addr++;
  EEPROM.updateByte(addr, keyerspeed);    addr++;
  EEPROM.updateByte(addr, keyswap);       addr++;
  EEPROM.updateByte(addr, xtimeout);      addr++;
  EEPROM.updateByte(addr, vfosel);        addr++;
  EEPROM.updateByte(addr, agc);           addr++;
  EEPROM.updateByte(addr, nrlevel);       addr++;
  EEPROM.updateByte(addr, attmic);        addr++;
  EEPROM.updateByte(addr, attrx);         addr++;
  EEPROM.updateByte(addr, att2);          addr++;
  EEPROM.updateByte(addr, smeter);        addr++;
  EEPROM.updateByte(addr, swrmeter);      addr++;
  EEPROM.updateByte(addr, vox);           addr++;
  EEPROM.updateByte(addr, voxlevel);      addr++;
  EEPROM.updateByte(addr, drive);         addr++;
  EEPROM.updateByte(addr, txdelay);       addr++;
  EEPROM.updateByte(addr, pwm_min);       addr++;
  EEPROM.updateByte(addr, pwm_max);       addr++;
  EEPROM.updateByte(addr, iqphase);       addr++;
  EEPROM.updateByte(addr, vfomode[VFOB]); addr++;
  EEPROM.updateByte(addr, vfomode[VFOA]); addr++;
  EEPROM.updateLong(addr, vfo[VFOA]);     addr+=4;
  EEPROM.updateLong(addr, vfo[VFOB]);     addr+=4;
  EEPROM.updateLong(addr, fxtal);         addr+=4;
  for (uint8_t i=1; i<=MAXIDX; i++) {
    EEPROM.updateLong(addr, sfmem[i]); addr+=4;
    EEPROM.updateByte(addr, srmem[i]); addr++;
  }
  // wait a bit and then shut down
  delay(500);
  powerDown();
}

const char* offon_label[] = { "OFF", "ON"};
const char* cwdec_label[] = { "OFF", "Rx", "Tx", "Rx/Tx"};
const char* ssb_label[]   = { "Full", "3000Hz", "2400Hz", "1800Hz" };
const char* cwb_label[]   = { "Full", "500Hz", "250Hz", "100Hz", "50Hz" };
const char* band_label[]  = { "10m", "15m", "17m", "20m", "30m", "40m", "60m", "80m" };
const char* attmc_label[] = { "0dB", "-6dB", "-12dB" };
const char* attrx_label[] = { "0dB", "-13dB", "-20dB", "-33dB", "-40dB", "-53dB", "-60dB", "-73dB" };
const char* smtr_label[]  = { "OFF", "dBm", "S-Meter", "WPM"};
const char* swr_label[]   = { "OFF", "WATTS", "VOLTS" };
const char* key_label[]   = { "Straight", "Iambic A", "Iambic B"};
const char* agc_label[]   = { "OFF", "Fast", "Slow" };
const char* emty_label[]  = { "  " };
const char* menu_label[]  = { "Press Menu" };

int8_t menuAction(uint8_t id) {
  switch (id) {
    case VOLUME:    paramAction(id, 0, &volume,     "1.1 Volume",      NULL,         0, 15); break;
    case TONEVOL:   paramAction(id, 0, &tonevol,    "1.2 Tone Vol",    NULL,         0, 15); break;
    case RADIOMODE: paramAction(id, 1, &radiomode,  "1.3 Radio Mode",  rmode_label,  0,  4); break;
    case BWFILTER:  paramAction(id, 1, &bwfilter,   "1.4 SSB Filter",  ssb_label,    0,  3); break;
    case CWFILTER:  paramAction(id, 1, &cwfilter,   "1.5 CW Filter",   cwb_label,    0,  4); break;
    case NRLEVEL:   paramAction(id, 1, &nrlevel,    "1.6 Noise Avg",   NULL,         0,  3); break;
    case BAND:      paramAction(id, 1, &bandsel,    "1.7 Band",        band_label,   0,  7); break;
    case STEP:      paramAction(id, 1, &stepsize,   "1.8 Tune Rate",   step_label,   0,  6); break;

    case PRACTICE:  paramAction(id, 1, &practice,   "2.1 CW Practice", offon_label,  0,  1); break;
    case CWDEC:     paramAction(id, 1, &cwdec,      "2.2 CW Decoder",  cwdec_label,  0,  3); break;
    case KEYERMODE: paramAction(id, 1, &keyermode,  "2.3 Keyer Mode",  key_label,    0,  2); break;
    case KEYERWPM:  paramAction(id, 1, &keyerspeed, "2.4 Keyer Speed", NULL,        15, 40); break;
    case KEYSWAP:   paramAction(id, 1, &keyswap,    "2.5 Key Swap",    offon_label,  0,  1); break;
    case SEMIQSK:   paramAction(id, 1, &semiqsk,    "2.6 Semi QSK",    offon_label,  0,  1); break;

    case CWMSG1:    cwmsgAction(id,                 "3.1 CW Message 1");  break;
    case CWMSG2:    cwmsgAction(id,                 "3.2 CW Message 2");  break;
    case CWMSG3:    cwmsgAction(id,                 "3.3 CW Message 3");  break;
    case CWMSG4:    cwmsgAction(id,                 "3.4 CW Message 4");  break;
    case CWMSG5:    cwmsgAction(id,                 "3.5 CW Message 5");  break;
    case CWMSG6:    cwmsgAction(id,                 "3.6 CW Message 6");  break;

    case VFOSEL:    paramAction(id, 1, &vfosel,     "4.1 VFO Mode",    vfosel_label, 0,  1); break;
    case SMETER:    paramAction(id, 1, &smeter,     "4.2 S-meter",     smtr_label,   0,  3); break;
    case SWRMETER:  paramAction(id, 1, &swrmeter,   "4.3 SWR Meter",   swr_label,    0,  2); break;
    case VOX:       paramAction(id, 1, &vox,        "4.4 VOX",         offon_label,  0,  1); break;
    case VOXLEVEL:  paramAction(id, 1, &voxlevel,   "4.5 VOX Level",   NULL,         0, 99); break;
    case AGC:       paramAction(id, 1, &agc,        "4.6 AGC",         agc_label,    0,  2); break;
    
    case ATTMIC:    paramAction(id, 1, &attmic,     "5.1 ATT Mic",     attmc_label,  0,  2); break;
    case ATTRX:     paramAction(id, 1, &attrx,      "5.2 ATT Rx",      attrx_label,  0,  7); break;
    case ATT2:      paramAction(id, 1, &att2,       "5.3 ATT Dig",     NULL,         0,  2); break;
    case DRIVE:     paramAction(id, 1, &drive,      "5.4 TX Drive",    NULL,         2,  8); break;
    case TXDELAY:   paramAction(id, 1, &txdelay,    "5.5 TX Delay",    NULL,         0, 30); break;
    case PWM_MIN:   paramAction(id, 0, &pwm_min,    "5.6 PA Bias min", NULL,         0,  0); break;
    case PWM_MAX:   paramAction(id, 0, &pwm_max,    "5.7 PA Bias max", NULL,         0,160); break;
    case IQPHASE:   paramAction(id, 0, &iqphase,    "5.8 IQ Phase",    NULL,         0,180); break;
    case REFCAL:    paramAction(id, 0, &notused,    "5.9 Ref Freq",    NULL,         0,  0); break;

    case CALLSIGN:  editsAction(mycall,             "6.1 Call Sign");  break;
    case NAME:      editsAction(myname,             "6.2 Name");       break;
    case ANT:       editsAction(myant,              "6.3 Antenna");    break;
    case RIG:       editsAction(myrig,              "6.4 Rig");        break;
    case PWR:       editsAction(mypwr,              "6.5 Power");      break;
    case QTH:       editsAction(myqth,              "6.6 QTH");        break;

    case FREQMEM:   otherAction(id,                 "7.1 Freq Mems");  break;
    case DIAG:      otherAction(id,                 "7.2 Diagnostic"); break;
    case SAVE:      otherAction(id,                 "7.3 EEPROM");     break;
    case RIT:       otherAction(id,                 "7.4 RIT");        break;
    case CWCAL:     otherAction(id,                 "7.5 RX CW Cal");  break;
    case BATT:      otherAction(id,                 "7.6 Battery");    break;
    case BLANK:     paramAction(id, 1, &xtimeout,   "7.7 Blank OLED",  NULL,  0, 240); break;
    case SWVER:     otherAction(id,                 "7.8 SW Version"); break;
    case SHDN:      otherAction(id,                 "7.9 Shut Down");  break;

    default:        break;
  }
  return id;
}

// initialize pins
void initPins() {
  digitalWrite(RXEN,    HIGH);   // Rx is enabled
  digitalWrite(LED,     HIGH);   // LED is off
  digitalWrite(TXPWM,    LOW);   // disable pwm
  digitalWrite(SIDETONE, LOW);   // disable side tone
  digitalWrite(SDA0, LOW);       // I2C bus #0
  digitalWrite(SCL0, LOW);       // I2C bus #0
  pinMode(XP1,      INPUT_PULLUP);
  pinMode(XP2,      INPUT_PULLUP);
  pinMode(XP3,      INPUT_PULLUP);
  pinMode(XP4,      INPUT_PULLUP);
  pinMode(DIT,      INPUT_PULLUP);
  pinMode(DAH,      INPUT_PULLUP);
  pinMode(UI1,      INPUT_PULLUP);
  pinMode(UI2,      INPUT_PULLUP);
  pinMode(UI3,      INPUT_PULLUP);
  pinMode(ROTA,     INPUT_PULLUP);
  pinMode(ROTB,     INPUT_PULLUP);
  pinMode(BLINKY,   OUTPUT);
  pinMode(XPTT,     OUTPUT);
  pinMode(RXEN,     OUTPUT);
  pinMode(TXPWM,    OUTPUT);
  pinMode(SIDETONE, OUTPUT);
  pinMode(SDA0,     INPUT);
  pinMode(SCL0,     INPUT);
}

#ifdef CAT
#define CATCMD_SIZE   32
char CATcmd[CATCMD_SIZE];

void analyseCATcmd() {
  if ((CATcmd[0] == 'F') && (CATcmd[1] == 'A') && (CATcmd[2] == ';'))
    Command_GETFreqA();

  else if ((CATcmd[0] == 'F') && (CATcmd[1] == 'A') && (CATcmd[13] == ';'))
    Command_SETFreqA();

  else if ((CATcmd[0] == 'I') && (CATcmd[1] == 'F') && (CATcmd[2] == ';'))
    Command_IF ();

  else if ((CATcmd[0] == 'I') && (CATcmd[1] == 'D') && (CATcmd[2] == ';'))
    Command_ID();

  else if ((CATcmd[0] == 'P') && (CATcmd[1] == 'S') && (CATcmd[2] == ';'))
    Command_PS();

  else if ((CATcmd[0] == 'P') && (CATcmd[1] == 'S') && (CATcmd[2] == '1'))
    Command_PS1();

  else if ((CATcmd[0] == 'A') && (CATcmd[1] == 'I') && (CATcmd[2] == ';'))
    Command_AI();

  else if ((CATcmd[0] == 'A') && (CATcmd[1] == 'I') && (CATcmd[2] == '0'))
    Command_AI0();

  else if ((CATcmd[0] == 'M') && (CATcmd[1] == 'D') && (CATcmd[2] == ';'))
    Command_GetMD();

  else if ((CATcmd[0] == 'M') && (CATcmd[1] == 'D') && (CATcmd[3] == ';'))
    Command_SetMD();

  else if ((CATcmd[0] == 'R') && (CATcmd[1] == 'X') && (CATcmd[2] == ';'))
    Command_RX();

  else if ((CATcmd[0] == 'T') && (CATcmd[1] == 'X') && (CATcmd[2] == ';'))
    Command_TX0();

  else if ((CATcmd[0] == 'T') && (CATcmd[1] == 'X') && (CATcmd[2] == '0'))
    Command_TX0();

  else if ((CATcmd[0] == 'T') && (CATcmd[1] == 'X') && (CATcmd[2] == '1'))
    Command_TX1();

  else if ((CATcmd[0] == 'T') && (CATcmd[1] == 'X') && (CATcmd[2] == '2'))
    Command_TX2();

  else if ((CATcmd[0] == 'A') && (CATcmd[1] == 'G') && (CATcmd[2] == '0'))  // add
    Command_AG0();

  else if ((CATcmd[0] == 'X') && (CATcmd[1] == 'T') && (CATcmd[2] == '1'))  // add
    Command_XT1();

  else if ((CATcmd[0] == 'R') && (CATcmd[1] == 'T') && (CATcmd[2] == '1'))  // add
    Command_RT1();

  else if ((CATcmd[0] == 'R') && (CATcmd[1] == 'C') && (CATcmd[2] == ';'))  // add
    Command_RC();

  else if ((CATcmd[0] == 'F') && (CATcmd[1] == 'L') && (CATcmd[2] == '0'))  // need?
    Command_FL0();

  else if ((CATcmd[0] == 'R') && (CATcmd[1] == 'S') && (CATcmd[2] == ';'))
    Command_RS();

  else if ((CATcmd[0] == 'V') && (CATcmd[1] == 'X') && (CATcmd[2] != ';'))
    Command_VX(CATcmd[2]);

#ifdef CAT_EXT
  else if ((CATcmd[0] == 'U') && (CATcmd[1] == 'K') && (CATcmd[4] == ';'))  // remote key press
    Command_UK(CATcmd[2], CATcmd[3]);

  else if ((CATcmd[0] == 'U') && (CATcmd[1] == 'D') && (CATcmd[2] == ';'))  // display contents
    Command_UD();
#endif // CAT_EXT

#ifdef CAT_STREAMING
  else if ((CATcmd[0] == 'U') && (CATcmd[1] == 'A') && (CATcmd[3] == ';'))  // audio streaming enable/disable
    Command_UA(CATcmd[2]);
#endif // CAT_STREAMING

  else {
    Serial.print("?;");
  }
}

#ifdef CAT
uint8_t cat_ptr = 0;
void serialEvent() {
  if (Serial.available()) {
    // block display to prevent CAT cmds that update the display
    // from interfering with the next CAT cmd
    char data = Serial.read();
    CATcmd[cat_ptr++] = data;
    if (data == ';') {
      CATcmd[cat_ptr] = '\0'; // terminate the array
      cat_ptr = 0;            // reset for next CAT command

#ifdef CAT_STREAMING
      if (cat_streaming) {
        noInterrupts();
        cat_streaming = OFF;  // terminate CAT stream
        Serial.print(';');
      }
      analyseCATcmd();        // process CAT cmd
      if (_cat_streaming) {
        Serial.print("US");
        cat_streaming = ON;   // resume CAT stream
      }
      interrupts();
#else
      analyseCATcmd();
#endif // CAT_STREAMING
      wait_ms(10);
    } else if (cat_ptr > (CATCMD_SIZE - 1)) {
      Serial.print("E;");
      cat_ptr = 0;            // overrun
    }
  }
}
#endif // CAT

#ifdef CAT_EXT
void Command_UK(char k1, char k2) {
  cat_key = ((k1 - '0') << 4) | (k2 - '0');
  if (cat_key & 0x40) { encoder_val--; cat_key &= 0x3f; }
  if (cat_key & 0x80) { encoder_val++; cat_key &= 0x3f; }
  char Catbuffer[16];
  sprintf(Catbuffer, "UK%c%c;", k1, k2);
  Serial.print(Catbuffer);
}

void Command_UD() {
  char Catbuffer[40];
  sprintf(Catbuffer, "UD%02u%s;", oled.curs ? oled.y*16+oled.x : 16*2+1, oled.text);
  Serial.print(Catbuffer);
}

void Command_UA(char en) {
  char Catbuffer[16];
  sprintf(Catbuffer, "UA%01u;", (_cat_streaming = (en == '1')));
  Serial.print(Catbuffer);
  if (_cat_streaming) { Serial.print("US"); cat_streaming = ON; }
}
#endif // CAT_EXT

void Command_GETFreqA() {
  char Catbuffer[32];
  unsigned int g,m,k,h;
  uint32_t tf;

  tf=freq;
  g=(unsigned int)(tf/1000000000lu);
  tf-=g*1000000000lu;
  m=(unsigned int)(tf/1000000lu);
  tf-=m*1000000lu;
  k=(unsigned int)(tf/1000lu);
  tf-=k*1000lu;
  h=(unsigned int)tf;

  sprintf(Catbuffer,"FA%02u%03u",g,m);
  Serial.print(Catbuffer);
  sprintf(Catbuffer,"%03u%03u;",k,h);
  Serial.print(Catbuffer);
}

void Command_SETFreqA() {
  char Catbuffer[16];
  ncpy(Catbuffer,CATcmd+2,11);
  freq=(uint32_t)atol(Catbuffer);
  update_freq = YES;
}

void Command_IF () {
  char Catbuffer[32];
  unsigned int g,m,k,h;
  uint32_t tf;

  tf=freq;
  g=(unsigned int)(tf/1000000000lu);
  tf-=g*1000000000lu;
  m=(unsigned int)(tf/1000000lu);
  tf-=m*1000000lu;
  k=(unsigned int)(tf/1000lu);
  tf-=k*1000lu;
  h=(unsigned int)tf;

  sprintf(Catbuffer,"IF%02u%03u%03u%03u",g,m,k,h);
  Serial.print(Catbuffer);
  sprintf(Catbuffer,"00000+000000");
  Serial.print(Catbuffer);
  sprintf(Catbuffer,"0000");
  Serial.print(Catbuffer);
  Serial.print(mode + 1);
  sprintf(Catbuffer,"0000000;");
  Serial.print(Catbuffer);
}

void Command_AI() {
  Serial.print("AI0;");
}

void Command_AG0() {
  Serial.print("AG0;");
}

void Command_XT1() {
  Serial.print("XT1;");
}

void Command_RT1() {
  Serial.print("RT1;");
}

void Command_RC() {
  rit = 0;
  Serial.print("RC;");
}

void Command_FL0() {
  Serial.print("FL0;");
}

void Command_GetMD() {
  Serial.print("MD");
  Serial.print(mode + 1);
  Serial.print(';');
}

void Command_SetMD() {
  mode = CATcmd[2] - '1';
  vfomode[vfosel] = mode;
  update_freq = YES;
  si5351.iqmsa = 0;  // reset PLL
}

void Command_AI0() {
  Serial.print("AI0;");
}

void Command_RX() {
  switch_rxtx(OFF, SHOW);
  semiqsk_timeout = 0;  // hack: fix for multiple RX cmds
  Serial.print("RX0;");
}

void Command_TX0() {
  switch_rxtx(ON, SHOW);
}

void Command_TX1() {
  switch_rxtx(ON, SHOW);
}

void Command_TX2() {
  switch_rxtx(ON, SHOW);
}

void Command_RS() {
  Serial.print("RS0;");
}

void Command_VX(char mode) {
  char Catbuffer[16];
  sprintf(Catbuffer, "VX%c;",mode);
  Serial.print(Catbuffer);
}

void Command_ID() {
  Serial.print("ID020;");
}

void Command_PS() {
  Serial.print("PS1;");
}

void Command_PS1() {
}
#endif // CAT

#ifdef SWR_METER

#define REF_V  (5.0 * 1.15)

void read_SWR() {
  static float prev_PFWD;
  static float prev_VSWR;
  float v_FWD = 0;
  float v_REV = 0;
  for (uint8_t i=0; i<=7; i++) {
    v_FWD += (REF_V / 1023.0) * (float) analogRead(VFWD);
    v_REV += (REF_V / 1023.0) * (float) analogRead(VREV);
    delay(5);
  }
  v_FWD = v_FWD / 8.0;
  v_REV = v_REV / 8.0;

  float p_FWD  = sq(v_FWD);
  float p_REV  = sq(v_REV);
  float vRatio = v_REV / v_FWD;
  float VSWR   = (1 + vRatio) / (1 - vRatio);

  if ((VSWR > 9.99) || (VSWR < 1)) VSWR = 9.99;

  if (p_FWD != prev_PFWD || VSWR != prev_VSWR) {
    // display SWR
    oled.setCursor(0, ROW2);
    oled.print("SWR:");
    oled.print(floor(100*VSWR)/100);
    oled.setCursor(0, ROW3);
    if (swrmeter == VOLTS) {
      // display Volts
      oled.print("F:");
      oled.print(floor(100*v_FWD)/100);
      oled.print("V R:");
      oled.print(floor(100*v_REV)/100);
      oled.print("V");
    } else {
      // display Watts
      oled.print("F:");
      oled.print(floor(100*p_FWD)/100);
      oled.print("W R:");
      oled.print(floor(100*p_REV)/100);
      oled.print("W");
    }
    prev_PFWD = p_FWD;
    prev_VSWR = VSWR;
  }
}
#endif // SWR_METER

// iambic keyer state machine
void iambic_keyer() {
  static uint32_t ktimer;
  static uint32_t stimer;           // SWR timer
  // exit early if there is nothing to do
  if ((keyerstate == KEY_IDLE) && (!DIT_OR_DAH)) return;
  // else continue with the state machine
  switch (keyerstate) {
    case KEY_IDLE:
      read_paddles();
      if (GOTKEY) {
        keyerstate = CHK_KEY;
        #ifdef SWR_METER
        if (swrmeter) {
          cwdec = OFF;
          stimer = millis() + 500;
          switch_rxtx(ON, SHOW);  // key the transmitter
          while (GOTKEY) {
            keyerinfo = 0;
            read_paddles();
            wdt_reset();
            if ((radiomode == CW) && (millis() > stimer)) {
              read_SWR();
              stimer = millis() + 500;
            }
          }
          switch_rxtx(OFF, SHOW);
        }
        #endif
      } else {
        keyerinfo = 0;
      }
      break;
    case CHK_KEY:
      switch (keyerinfo & 0x07) {
        case 0x05:  // dit and prev dit
        case 0x03:  // dit and dah
        case 0x01:  // dit
          keyerinfo &= ~DIT_REG;
          keyerinfo |= WAS_DIT;
          ktimer = millis() + dittime;
          shift_txsym(DITX);  // record a dit
          switch_rxtx(ON, SHOW);
          keyerstate = KEY_WAIT;
          break;
        case 0x07:  // dit and dah and prev dit
        case 0x06:  // dah and prev dit
        case 0x02:  // dah
          keyerinfo &= ~DAH_REG;
          keyerinfo &= ~WAS_DIT;
          ktimer = millis() + dahtime;
          shift_txsym(DAHX);  // record a dah
          switch_rxtx(ON, SHOW);
          keyerstate = KEY_WAIT;
          break;
        default:
          // case 0x00:  // nothing
          // case 0x04:  // nothing and prev dit
          keyerstate = KEY_IDLE;
      }
      break;
    case KEY_WAIT:
      // wait dit/dah duration
      if (millis() > ktimer) {
        // done sending dit/dah
        switch_rxtx(OFF, SHOW);
        // inter-symbol time is 1 dit
        ktimer = millis() + dittime;
        keyerstate = IDD_WAIT;
      }
      break;
    case IDD_WAIT:
      // wait time between dit/dah
      if (millis() > ktimer) {
        // wait done
        keyerinfo &= ~KEY_REG;
        if (keyermode == IAMBIC_A) {
          // Iambic A
          // check for letter gap
          ktimer = millis() + lettergap;
          keyerstate = LTR_GAP;
        } else {
          // Iambic B
          read_paddles();
          if (NOKEY && (keyerinfo & BOTH_REG)) {
            // send opposite of last paddle sent
            if (keyerinfo & WAS_DIT) {
              // send a dah
              ktimer = millis() + dahtime;
              shift_txsym(DAHX);  // record a dah
            } else {
              // send a dit
              ktimer = millis() + dittime;
              shift_txsym(DITX);  // record a dit
            }
            switch_rxtx(ON, SHOW);
            keyerinfo = 0;
            keyerstate = KEY_WAIT;
          } else {
            // check for letter gap
            ktimer = millis() + lettergap;
            keyerstate = LTR_GAP;
          }
        }
      }
      break;
    case LTR_GAP:
      if (millis() > ktimer) {
        // letter gap found so print char
        print_cwsym(XMIT);
        txsym = 1;
        // check for word space
        ktimer = millis() + wordgap;
        keyerstate = WORD_GAP;
      }
      read_paddles();
      if (GOTKEY) {
        keyerstate = CHK_KEY;
      } else {
        keyerinfo = 0;
      }
      break;
    case WORD_GAP:
      if (millis() > ktimer) {
        // word gap found so print a space
        txsym = 1;
        print_cwsym(XMIT);
        keyerstate = KEY_IDLE;
      }
      read_paddles();
      if (GOTKEY) {
        keyerstate = CHK_KEY;
      } else {
        keyerinfo = 0;
      }
      break;
    default:
      break;
  }
}

// handle straight key mode
void straight_key() {
  static uint32_t dtimer;           // dit timer
  static uint32_t ltimer;           // letter-gap timer
  static uint32_t wtimer;           // word-gap timer
  static uint32_t stimer;           // SWR timer
  static uint8_t check_letter = 0;  // check letter timing
  static uint8_t check_word = 0;    // check word timing

  read_paddles();
  if (GOTKEY) {
    dtimer = millis() + dittime;
    stimer = millis() + 500;
    switch_rxtx(ON, SHOW);  // key the transmitter
    while (GOTKEY) {
      keyerinfo = 0;
      read_paddles();
      wdt_reset();
      #ifdef SWR_METER
      if (swrmeter && (radiomode == CW) && (millis() > stimer)) {
        read_SWR();
        stimer = millis() + 500;
      }
      #endif
    }
    if (millis() > dtimer) shift_txsym(DAHX);  // record a dah
    else shift_txsym(DITX);                    // record a dit
    switch_rxtx(OFF, SHOW);
    ltimer = millis() + lettergap;
    wtimer = millis() + wordgap;
    check_letter = 1;
    check_word = 1;
    wait_ms(10);  // debounce
  } else {
    if (check_letter) {
      if (millis() > ltimer) {
        // letter gap found so print char
        check_letter = 0;
        print_cwsym(XMIT);
        txsym = 1;
      }
    }
    if (check_word) {
      if (millis() > wtimer) {
        // word gap found so print space
        check_word = 0;
        txsym = 1;
        print_cwsym(XMIT);
      }
    }
    wait_ms(10);  // debounce
  }
}

// wait for any button press/release
void wait4button() {
  while (!BUTTON_PRESSED) wait_ms(1);
  wait4release();
}

void printV(char* msg, float val, char unit, uint8_t ok) {
  oled.setCursor(0, ROW3);
  oled.print(msg);
  oled.clr2eol();
  oled.setCursor(0, ROW4);
  oled.print(val);
  oled.print(unit);
  if (ok) oled.print(" OK");
  else oled.print(" XX");
  oled.clr2eol();
  wdt_reset();
}

void printS(char* msg, float val) {
  oled.setCursor(0, ROW3);
  oled.print(msg);
  oled.clr2eol();
  oled.setCursor(0, ROW4);
  oled.print(val);
  oled.print(" uSec");
  oled.clr2eol();
  wdt_reset();
}

// reset display timeout
void reset_xdt() {
  xtimer = 0;
  xdt = millis();
  if (display == OFF) {
    display = ON;
    oled.onDisplay();
  }
}

// run diagnostics
void run_diags() {
  float fpv;
  uint32_t t1;
  uint8_t ok;
  uint8_t diag_err = 0;

  // prepare radio to run diags
  MCUSR = ~(1<<WDRF);  // must be done before wdt_disable()
  wdt_disable();
  timer2_stop();
  timer1_stop();
  adc_stop();
  delay(100);

  // press any button to continue
  oled.setCursor(0, ROW3);
  oled.print("to continue..   ");
  wait4button();

  // Measure I input; should be ~2.5V
  si5351.SendRegister(SI_CLK_OE, TX0RX0);
  digitalWrite(TXPWM, LOW);
  digitalWrite(RXEN, LOW);
  delay(100);
  fpv = (float)analogRead(QSDI) * 5.0 / 1024.0;
  digitalWrite(RXEN, HIGH);
  ok = (fpv > 2.2 && fpv < 2.6);
  if (!ok) diag_err++;
  printV("Input I", fpv, 'V', ok);
  wait4button();

  // Measure Q input; should be ~2.5V
  si5351.SendRegister(SI_CLK_OE, TX0RX0);
  digitalWrite(TXPWM, LOW);
  digitalWrite(RXEN, LOW);
  delay(100);
  fpv = (float)analogRead(QSDQ) * 5.0 / 1024.0;
  digitalWrite(RXEN, HIGH);
  ok = (fpv > 2.2 && fpv < 2.6);
  if (!ok) diag_err++;
  printV("Input Q", fpv, 'V', ok);
  wait4button();

  // Measure MIC input; should be ~2.5V
  fpv = (float)analogRead(MIC) * 5.0 / 1024.0;
  ok = (fpv > 2.2 && fpv < 2.6);
  if (!ok) diag_err++;
  printV("MIC Input", fpv, 'V', ok);
  wait4button();

  // Measure the SSB bandwidth filters
  t0 = micros();
  nrlevel = 2;
  for (uint16_t i=0; i<1000; i++) {
    iirfilter(i);
    wdt_reset();
  }
  t1 = micros();
  fpv = (t1 - t0)/1000;
  ok = 1;
  printS("BW Filter", fpv);
  wait4button();

  // Measure the Goertzel filter
  t0 = micros();
  nrlevel  = 2;
  bwfilter = BW3000;
  for (uint16_t i=0; i<1000; i++) {
    goertzel();
    wdt_reset();
  }
  t1 = micros();
  fpv = (t1 - t0)/1000;
  ok = 1;
  printS("Goertzel", fpv);
  wait4button();

  // Measure the total Rx processing time
  t0 = micros();
  nrlevel  = 2;
  bwfilter = BW3000;
  for (uint16_t i=0; i<1000; i++) {
    process_rx(i, i+1);
    wdt_reset();
  }
  t1 = micros();
  fpv = (t1 - t0)/1000;
  ok = 1;
  printS("RX Processing", fpv);
  wait4button();

  // Measure I2C Bus speed for Bulk Transfers
  t0 = micros();
  for (uint16_t i=0; i<1000; i++) {
    si5351.SendPLLRegisterBulk();
    wdt_reset();
  }
  t1 = micros();
  fpv = 500000 / (t1 - t0);
  ok = 1;
  printV("I2C Bus Speed", fpv, 'K', ok);
  wait4button();

  // Measure I2C Bit-Error Rate (BER)
  si5351.freq(curfreq, 0, 90);
  uint16_t i2c_error = 0;
  #define SI_SYNTH_PLL_A 26
  for (uint16_t i=0; i<1000; i++) {
    si5351.freq_calc_fast(i);
    si5351.SendPLLRegisterBulk();
    for (uint8_t j=4; j<8; j++) {
      if (si5351.RecvRegister(SI_SYNTH_PLL_A + j) != si5351.pll_regs[j]) i2c_error++;
    }
    wdt_reset();
  }
  fpv = (float)i2c_error;
  ok = !i2c_error;
  if (!ok) diag_err++;
  printV("I2C Bus Errors", fpv, ' ', ok);
  wait4button();

  // end of diagnostic message
  oled.setCursor(0, ROW1);
  oled.print("Diagnostic");
  oled.clr2eol();
  oled.clrLine(ROW3);
  oled.setCursor(0, ROW4);
  if (diag_err) oled.print("Errors");
  else oled.print("Passed");
  oled.clr2eol();
  // wait for button press and then shut down
  wait4button();
  powerDown();
}

// startup initialization
void setup() {
  digitalWrite(TXPWM, LOW);  // prevent exploding PA MOSFETs
  si5351.powerDown();
  MCUSR = 0;
  wdt_disable();
  ADMUX = (1 << REFS0);  // reference voltage AREF=5V
  // disable external interrupts
  PCICR = 0;
  PCMSK0 = 0;
  PCMSK1 = 0;
  PCMSK2 = 0;
  encoder_init();
  initPins();
  Wire.init();
  delay(100);
  oled.begin(16, 4);
  #define BAUD   115200
  #define XBAUD  (16 * BAUD)/20
  Serial.begin(XBAUD);
  // check for button press during power-up
  if (BUTTON_PRESSED) {
    // use the default parameters
    oled.setCursor(0, ROW2);
    oled.print("Factory Reset.. ");
    while (BUTTON_PRESSED) delay(1);
    delay(20);  // debounce
  } else {
    // load parameters from the eeprom
    loadAll();
  }
  si5351.iqmsa = 0;  // reset PLL
  curfreq   = vfo[vfosel];
  radiomode = vfomode[vfosel];
  set_freq();
  set_lpf(bandsel);
  minsky_init();
  build_lut();
  show_banner();
  start_rx();
#ifdef CAT
  Command_IF ();
#endif // CAT
  loadWPM(keyerspeed);
  // wait until DIH/DAH/PTT is released to prevent TX on startup
  if (DIT_OR_DAH) {
    oled.setCursor(0, ROW2);
    oled.print("Check PTT/key!! ");
    while (DIT_OR_DAH) delay(1);
    delay(20);  // debounce
  }
  oled.clrR23();
  wdt_enable(WDTO_4S);  // enable watchdog
}

// main program loop
void loop() {
  uint8_t event;                // click long press
  uint8_t button;               // which button

  // if VOX is enabled in LSB/USB mode, then take mic samples and
  // derive the amplitude to detect a vox threshold crossing
  if (vox && ((radiomode == LSB) || (radiomode == USB))) {
    if (!vox_tx) {
      // take N mic samples, then process
      if (vox_sample++ == 16) {
        ssb(((int16_t)(vox_adc/16) - (512 - AF_BIAS)) >> attmic);
        vox_sample = 0;
        vox_adc = 0;
      } else {
        vox_adc += analogSampleMic();
      }
      if (tx) {
        // TX triggered by audio -> TX
        vox_tx = 1;
        switch_rxtx(ON, SHOW);
      }
    } else if (!tx) {
      // VOX activated, no audio detected -> RX
      switch_rxtx(OFF, SHOW);
      vox_tx = 0;
      wait_ms(32);
    }
  }

  // CW decoder active during RX
  if ((menumode == NOT_IN_MENU) && (radiomode == CW) &&
     ((cwdec == CWRX) || (cwdec == RXTX)) &&
       gcdone && gdbrdy && !tx && !semiqsk_timeout) rxcw_decode();

  // S-meter calculations
  if ((menumode == NOT_IN_MENU) && !semiqsk_timeout && !vox_tx) calc_smeter();

  // select straight key or iambic keyer
  if ((menumode == NOT_IN_MENU) && (radiomode == CW)) {
    if (keyermode == STRAIGHT) straight_key();
    else iambic_keyer();
  }

  // a long press of DIT or DAH will switch to CW mode
  if ((menumode == NOT_IN_MENU) && (radiomode != CW) &&  DIT_OR_DAH) {
    t0 = millis();
    while (DIT_OR_DAH) {
      if ((millis() - t0) > 700) {
        radiomode = CW;
        vfomode[vfosel] = radiomode;
        si5351.iqmsa = 0;
        update_freq = YES;
        break;
      }
      wait_ms(1);
      wdt_reset();
    }
  }

  // play side tone when in tone volume menu
  if ((menumode == SELECT_VALUE) && (menu == TONEVOL)) check_ditdah();

  // check for semi-qsk timeout
  if (semiqsk_timeout && (millis() > semiqsk_timeout)) switch_rxtx(OFF, SHOW);

  if (BUTTON_PRESSED) {

    if (ENCO_PRESSED) button = ENCO;
    if (MENU_PRESSED) button = MENU;
    if (EXIT_PRESSED) button = EXIT;

    reset_xdt();    // reset display timeout

    if (!(event & LONG)) {
      event = CLICK;
      t0 = millis();
      while (BUTTON_PRESSED) {
        if (EXIT_PRESSED) button |= EXIT;
        if (MENU_PRESSED) button |= MENU;
        if (ENCO_PRESSED) button |= ENCO;
        // until released or long press
        if ((millis() - t0) > MS_LONG) { event = LONG; break; }
        wait_ms(1);
        wdt_reset();
      }

#ifdef CAT_EXT
      if (cat_key) {
        if (cat_key&0x01) button = EXIT;
        if (cat_key&0x02) button = MENU;
        if (cat_key&0x04) button = ENCO;
      }
#endif // CAT_EXT

      event |= button;  // add button info
    }

    // enforce the UI lock
    if (ui_lock) {
      t0 = millis();
      while (EXIT_PRESSED) {
        // check for super-long press
        if ((millis() - t0) > MS_SLP) { event = SLP; break; }
        wait_ms(1);
        wdt_reset();
      }
      if (event == SLP) {
        // a super-long press will unlock the UI
        ui_lock = OFF;
        show_unlock();
      }
      wait4release(EXIT);
      event = NONE;
      goto end_of_loop;   // skip all UI processing
    }

    switch (event) {
      case EXIT|MENU|LONG:
        // a long press of both exit and menu will lock the UI
        if (menumode == NOT_IN_MENU) {
          ui_lock = ON;
          show_lock();
        }
        break;
      case MENU|CLICK:
        if ((menu >= CWMSG1) && (menu <= CWMSG6) &&
          (menumode == SELECT_MENU) && (radiomode == CW)) {
          // send a CW message
          send_cwmsg(menu-CWMSG1);
        }
        else switch (menumode) {
          case NOT_IN_MENU:
            menumode = SELECT_MENU;      // enter menu mode
            menuAction(menu);
            break;
          case SELECT_MENU:
            if ((menu >= CALLSIGN) && (menu <= QTH)) {
              menumode = EDIT_STRING;    // enter edit string mode
              oled.clrLine(ROW2);
              show_edit();
              get_spinpos(editstr[editpos]);
              show_cursor(ON);
            } else {
              menumode = SELECT_VALUE;   // enter value selection
              menuAction(menu);
            }
            break;
          case SELECT_VALUE:
            menumode = SELECT_MENU;      // back to menu mode
            menuAction(menu);
            break;
          case EDIT_STRING:
            editpos++;
            get_spinpos(editstr[editpos]);
            show_cursor(ON);
            break;
          default:
            break;
        }
        break;
      case MENU|LONG:
        switch (menumode) {
          case NOT_IN_MENU:
            // a long press of menu for keyer speed
            menumode = SELECT_VALUE;
            menu = KEYERWPM;
            menuAction(menu);
            t0 = millis();
            while (MENU_PRESSED) {
              // check for double-long press
              if ((millis() - t0) > MS_DLP) { event = DLP; break; }
              wait_ms(1);
              wdt_reset();
            }
            if (event == DLP) {
              // a double-long press for band selection
              menu = BAND;
              menuAction(menu);
            }
            break;
          case SELECT_MENU:
          case SELECT_VALUE:
          case EDIT_STRING:
          default:
            break;
        }
        break;
      case EXIT|CLICK:
        switch (menumode) {
          case NOT_IN_MENU:
            if (ritmode) {
              ritmode = OFF;  // disable RIT
              ritval = 0;
              update_freq = YES;
              break;
            }
            radiomode++;
            if (radiomode > CW) radiomode = LSB;  // only LSB, USB, CW
            vfomode[vfosel] = radiomode;
            si5351.iqmsa = 0;
            oled.clrR23();
            update_freq = YES;
            break;
          case SELECT_MENU:
          case SELECT_VALUE:
            menumode = NOT_IN_MENU;    // back to main screen
            oled.clrR23();
            update_freq = YES;
            show_banner();
            if (menu == KEYERWPM) beep_cw("V");  // acknowledge change
            break;
          case EDIT_STRING:
            menumode = SELECT_MENU;    // back to menu mode
            show_cursor(OFF);
            menuAction(menu);
            break;
          default:
            break;
        }
        break;
      case EXIT|LONG:
        if (menumode == NOT_IN_MENU) {
          // a long press of exit will open the frequency list
          menumode = SELECT_MENU;
          menu = FREQMEM;
          menuAction(menu);
          t0 = millis();
          while (EXIT_PRESSED) {
            // check for double-long press
            if ((millis() - t0) > MS_DLP) { event = DLP; break; }
            wait_ms(1);
            wdt_reset();
          }
          if (event == DLP) {
            // a double-long press will open the RIT menu
            menu = RIT;
            menuAction(menu);
            t0 = millis();
            while (EXIT_PRESSED) {
              // check for triple-long press
              if ((millis() - t0) > MS_SLP) { event = SLP; break; }
              wait_ms(1);
              wdt_reset();
            }
            if (event == SLP) {
              // a triple-long press will blank the screen
              display = OFF;
              ritmode = OFF;          // disable RIT
              ritval = 0;
              menumode = NOT_IN_MENU; // back to main screen
              oled.clrR23();
              update_freq = YES;
              show_banner();
              oled.setCursor(0, ROW2);
              oled.print("BLANK SCREEN");

              // wait for button release
              while (EXIT_PRESSED) {
                wait_ms(1);
                wdt_reset();
              }
              oled.clrLine(ROW2);
              oled.noDisplay();
            }
          }
        }
        break;
      case ENCO|CLICK:
        switch (menumode) {
          case NOT_IN_MENU:
            stepsize_change();
            break;
          case SELECT_VALUE:
            if (menu == REFCAL) ref_stepsize_change();
            break;
          case EDIT_STRING:
            insertchar();
            break;
          case SELECT_MENU:
          default:
            break;
        }
        break;
      case ENCO|LONG:
        switch (menumode) {
          case NOT_IN_MENU:
            // a long press of encoder for volume control
            menumode = SELECT_VALUE;
            menu = VOLUME;
            menuAction(menu);
            t0 = millis();
            while (ENCO_PRESSED) {
              // check for double-long press
              if ((millis() - t0) > MS_DLP) { event = DLP; break; }
              wait_ms(1);
              wdt_reset();
            }
            if (event == DLP) {
              // a double-long press for tone volume
              menu = TONEVOL;
              menuAction(menu);
              t0 = millis();
              while (ENCO_PRESSED) {
                // check for triple-long press
                if ((millis() - t0) > MS_SLP) { event = SLP; break; }
                wait_ms(1);
                wdt_reset();
              }
              if (event == SLP) {
                // a triple-long press for CW calibrate
                menu = CWCAL;
                menuAction(menu);
              }
            }
            break;
          case SELECT_MENU:
          case SELECT_VALUE:
          case EDIT_STRING:
          default:
            break;
        }
        break;
      default:
        break;
    }
    event = NONE;
  } else {
    // no button was pressed
    // check if 1 minute has passed
    if ((xtimeout > 0) && (display == ON) && ((millis() - xdt) > ONE_MINUTE)) {
      xdt = millis();
      xtimer++;
      if (xtimer > xtimeout) {
        display = OFF;
        oled.noDisplay();
      }
    }
  }

  if (encoder_val) {
    reset_xdt();    // reset display timeout
    switch (menumode) {
      case NOT_IN_MENU:  tune_freq();      break;
      case SELECT_MENU:  select_menu();    break;
      case SELECT_VALUE: menuAction(menu); break;
      case EDIT_STRING:  spinchar();       break;
      default: break;
    }
  }

  // only change band if tx is OFF
  if (update_freq && !tx && !vox_tx) {
    set_freq();
    update_freq = OFF;
    vfo[vfosel] = curfreq;
    if (menumode == NOT_IN_MENU) show_vfo(curfreq, radiomode);
  }

  end_of_loop:
  wdt_reset();
}

