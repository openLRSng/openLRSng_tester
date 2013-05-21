/*** Tester sketch for openLRS TX modules ***/

#include <Arduino.h>

//####### TX BOARD TYPE #######
// 2 = Original M2/M3 Tx Board or OrangeRx UHF TX
// 3 = OpenLRS Rx v2 Board works as TX
// 4 = OpenLRSngTX (tbd.)
#define HW 4

#if (HW==2)
#define PPM_IN 3
#define RF_OUT_INDICATOR A0
#define BUZZER 10
#define BTN 11
#define Red_LED 13
#define Green_LED 12

#define Red_LED_ON  PORTB |= _BV(5);
#define Red_LED_OFF  PORTB &= ~_BV(5);

#define Green_LED_ON   PORTB |= _BV(4);
#define Green_LED_OFF  PORTB &= ~_BV(4);

#define PPM_Pin_Interrupt_Setup  PCMSK2 = 0x08;PCICR|=(1<<PCIE2);
#define PPM_Signal_Interrupt PCINT2_vect
#define PPM_Signal_Edge_Check (PIND & 0x08)==0x08

void buzzerInit()
{
  pinMode(BUZZER, OUTPUT);
  digitalWrite(BUZZER, LOW);
}

void buzzerOn(uint16_t freq)
{
  if (freq) {
    digitalWrite(BUZZER,HIGH);
  } else {
    digitalWrite(BUZZER,LOW);
  }
}

#define buzzerOff(foo) buzzerOn(0)

//## RFM22B Pinouts for Public Edition (M2)
#define  nIRQ_1 (PIND & 0x04)==0x04 //D2
#define  nIRQ_0 (PIND & 0x04)==0x00 //D2

#define  nSEL_on PORTD |= (1<<4) //D4
#define  nSEL_off PORTD &= 0xEF //D4

#define  SCK_on PORTD |= (1<<7) //D7
#define  SCK_off PORTD &= 0x7F //D7

#define  SDI_on PORTB |= (1<<0) //B0
#define  SDI_off PORTB &= 0xFE //B0

#define  SDO_1 (PINB & 0x02) == 0x02 //B1
#define  SDO_0 (PINB & 0x02) == 0x00 //B1

#define SDO_pin 9
#define SDI_pin 8
#define SCLK_pin 7
#define IRQ_pin 2
#define nSel_pin 4

#define IRQ_interrupt 0

#elif (HW==3)

#define USE_ICP1 // use ICP1 for PPM input for less jitter

#ifdef USE_ICP1
#define PPM_IN 8 // ICP1
#else
#define PPM_IN 3
#define PPM_Pin_Interrupt_Setup  PCMSK2 = 0x08;PCICR|=(1<<PCIE2);
#define PPM_Signal_Interrupt PCINT2_vect
#define PPM_Signal_Edge_Check (PIND & 0x08)==0x08
#endif

#define BUZZER 6
#define BTN 7

void buzzerInit()
{
  pinMode(BUZZER, OUTPUT);
  digitalWrite(BUZZER, LOW);
}

void buzzerOn(uint16_t freq)
{
  if (freq) {
    digitalWrite(BUZZER,HIGH);
  } else {
    digitalWrite(BUZZER,LOW);
  }
}

#define buzzerOff(foo) buzzerOn(0)

#define Red_LED A3
#define Green_LED 13

#define Red_LED_ON  PORTC |= _BV(3);
#define Red_LED_OFF  PORTC &= ~_BV(3);    // Was originally #define Green_LED_OFF  PORTB |= _BV(5);   E.g turns it ON not OFF

#define Green_LED_ON  PORTB |= _BV(5);
#define Green_LED_OFF  PORTB &= ~_BV(5);

//## RFM22B Pinouts for Public Edition (Rx v2)
#define  nIRQ_1 (PIND & 0x04)==0x04 //D2
#define  nIRQ_0 (PIND & 0x04)==0x00 //D2

#define  nSEL_on PORTD |= (1<<4) //D4
#define  nSEL_off PORTD &= 0xEF //D4

#define  SCK_on PORTC |= (1<<2) //A2
#define  SCK_off PORTC &= 0xFB //A2

#define  SDI_on PORTC |= (1<<1) //A1
#define  SDI_off PORTC &= 0xFD //A1

#define  SDO_1 (PINC & 0x01) == 0x01 //A0
#define  SDO_0 (PINC & 0x01) == 0x00 //A0

#define SDO_pin A0
#define SDI_pin A1
#define SCLK_pin A2
#define IRQ_pin 2
#define nSel_pin 4

#define IRQ_interrupt 0

#elif (HW==4)

#define USE_ICP1 // use ICP1 for PPM input for less jitter
#define PPM_IN 8 // ICP1

#define BUZZER 3 // OCR2B
#define BTN A0
#define Red_LED 6
#define Green_LED 5

void buzzerInit()
{
  TCCR2A = (1<<WGM21); // mode=CTC
  TCCR2B = (1<<CS22) | (1<<CS20); // prescaler = 128
  pinMode(BUZZER, OUTPUT);
  digitalWrite(BUZZER, LOW);
}

void buzzerOn(uint16_t freq)
{
  if (freq) {
    uint32_t ocr = 125000L / freq;
    if (ocr>255) {
      ocr=255;
    }
    if (!ocr) {
      ocr=1;
    }
    OCR2A = ocr;
    TCCR2A |= (1<<COM2B0); // enable output
  } else {
    TCCR2A &= ~(1<<COM2B0); // disable output
  }
}

#define buzzerOff(foo) buzzerOn(0)

#define Red_LED_ON PORTD |= _BV(6);
#define Red_LED_OFF PORTD &= ~_BV(6);

#define Green_LED_ON PORTD |= _BV(5);
#define Green_LED_OFF PORTD &= ~_BV(5);

//## RFM22B Pinouts for Public Edition (M2)
#define nIRQ_1 (PIND & 0x04)==0x04 //D2
#define nIRQ_0 (PIND & 0x04)==0x00 //D2

#define nSEL_on PORTD |= (1<<4) //D4
#define nSEL_off PORTD &= 0xEF //D4

#define SCK_on PORTB |= _BV(5) //B5
#define SCK_off PORTB &= ~_BV(5) //B5

#define SDI_on PORTB |= _BV(3) //B3
#define SDI_off PORTB &= ~_BV(3) //B3

#define SDO_1 (PINB & _BV(4)) == _BV(4) //B4
#define SDO_0 (PINB & _BV(4)) == 0x00 //B4

#define SDO_pin 12
#define SDI_pin 11
#define SCLK_pin 13
#define IRQ_pin 2
#define nSel_pin 4

#define IRQ_interrupt 0

#else
#error invalid HW
#endif

#define TIMER1_FREQUENCY_HZ 50
#define TIMER1_PRESCALER 8
#define TIMER1_PERIOD (F_CPU/TIMER1_PRESCALER/TIMER1_FREQUENCY_HZ)
#define PPM_CHANNELS 8
volatile uint16_t PPM[PPM_CHANNELS] = { 512, 512, 512, 512, 512, 512, 512, 512 };
volatile uint16_t startPulse = 0;
volatile uint8_t ppmCounter = PPM_CHANNELS; // ignore data until first sync pulse

#ifdef USE_ICP1 // Use ICP1 in input capture mode
/****************************************************
* Interrupt Vector
****************************************************/
ISR(TIMER1_CAPT_vect)
{
  uint16_t stopPulse = ICR1;

  // Compensate for timer overflow if needed
  uint16_t pulseWidth = ((startPulse > stopPulse) ? TIMER1_PERIOD : 0) + stopPulse - startPulse;

  if (pulseWidth > 5000) { // Verify if this is the sync pulse (2.5ms)
    ppmCounter = 0; // -> restart the channel counter
  } else if ((pulseWidth > 1400) && (ppmCounter < PPM_CHANNELS)) { // extra channels will get ignored here
    PPM[ppmCounter] = (pulseWidth / 2); // Store measured pulse length in us
    ppmCounter++; // Advance to next channel
  } else {
    ppmCounter = PPM_CHANNELS; // glitch ignore rest of data
  }

  startPulse = stopPulse; // Save time at pulse start
}

void setupPPMinput()
{
  // Setup timer1 for input capture (PSC=8 -> 0.5ms precision, top at 20ms)
  TCCR1A = ((1 << WGM10) | (1 << WGM11));
  TCCR1B = ((1 << WGM12) | (1 << WGM13) | (1 << CS11) | (1 << ICES1));
  OCR1A = TIMER1_PERIOD;
  TIMSK1 |= (1 << ICIE1); // Enable timer1 input capture interrupt
}
#else // sample PPM using pinchange interrupt
ISR(PPM_Signal_Interrupt)
{
  uint16_t time_temp;

  if (PPM_Signal_Edge_Check) {   // Only works with rising edge of the signal
    time_temp = TCNT1; // read the timer1 value
    TCNT1 = 0; // reset the timer1 value for next

    if (time_temp > 5000) {   // new frame detection (>2.5ms)
      ppmCounter = 0;             // -> restart the channel counter
    } else if ((time_temp > 1400) && (ppmCounter < PPM_CHANNELS)) {
      PPM[ppmCounter] = (time_temp / 2);   // Store measured pulse length (converted)
      ppmCounter++;                     // Advance to next channel
    } else {
      ppmCounter = PPM_CHANNELS; // glitch ignore rest of data
    }
  }
}

void setupPPMinput(void)
{
  // Setup timer1 for input capture (PSC=8 -> 0.5ms precision, top at 20ms)
  TCCR1A = ((1 << WGM10) | (1 << WGM11));
  TCCR1B = ((1 << WGM12) | (1 << WGM13) | (1 << CS11));
  OCR1A = TIMER1_PERIOD;
  TIMSK1 = 0;
  PPM_Pin_Interrupt_Setup
}
#endif
// **** SPI bit banging functions
#define NOP() __asm__ __volatile__("nop")

#define RF22B_PWRSTATE_POWERDOWN 0x00
#define RF22B_PWRSTATE_READY 0x01
#define RF22B_PACKET_SENT_INTERRUPT 0x04
#define RF22B_PWRSTATE_RX 0x05
#define RF22B_PWRSTATE_TX 0x09

#define RF22B_Rx_packet_received_interrupt 0x02

uint8_t ItStatus1, ItStatus2;

void spiWriteBit(uint8_t b)
{
  if (b) {
    SCK_off;
    NOP();
    SDI_on;
    NOP();
    SCK_on;
    NOP();
  } else {
    SCK_off;
    NOP();
    SDI_off;
    NOP();
    SCK_on;
    NOP();
  }
}

uint8_t spiReadBit(void)
{
  uint8_t r = 0;
  SCK_on;
  NOP();

  if (SDO_1) {
    r = 1;
  }

  SCK_off;
  NOP();
  return r;
}

void spiSendCommand(uint8_t command)
{
  nSEL_on;
  SCK_off;
  nSEL_off;

  for (uint8_t n = 0; n < 8 ; n++) {
    spiWriteBit(command & 0x80);
    command = command << 1;
  }

  SCK_off;
}

void spiSendAddress(uint8_t i)
{
  spiSendCommand(i & 0x7f);
}

void spiWriteData(uint8_t i)
{
  for (uint8_t n = 0; n < 8; n++) {
    spiWriteBit(i & 0x80);
    i = i << 1;
  }

  SCK_off;
}

uint8_t spiReadData(void)
{
  uint8_t Result = 0;
  SCK_off;

  for (uint8_t i = 0; i < 8; i++) { //read fifo data byte
    Result = (Result << 1) + spiReadBit();
  }

  return(Result);
}

uint8_t spiReadRegister(uint8_t address)
{
  uint8_t result;
  spiSendAddress(address);
  result = spiReadData();
  nSEL_on;
  return(result);
}

void spiWriteRegister(uint8_t address, uint8_t data)
{
  address |= 0x80; //
  spiSendCommand(address);
  spiWriteData(data);
  nSEL_on;
}

void setup() {
  Serial.begin(115200); 
  buzzerInit();

  pinMode(Red_LED,OUTPUT);
  pinMode(Green_LED,OUTPUT);
   
  pinMode(BTN,INPUT);
  digitalWrite(BTN,HIGH); //enable pullup 
  
  pinMode(PPM_IN,INPUT);
  digitalWrite(PPM_IN,HIGH); //enable pullup 

  pinMode(SDO_pin, INPUT);
  pinMode(SDI_pin, OUTPUT);
  pinMode(SCLK_pin, OUTPUT);
  pinMode(nSel_pin, OUTPUT);
  pinMode(IRQ_pin, INPUT);
  nSEL_on;

  setupPPMinput();
}


void buzzerTest() {
  Serial.println("Buzzer test!");
  buzzerOn(2000);
  delay(500);
  buzzerOn(1000);  
  delay(500);
  buzzerOn(500);  
  delay(500);
  buzzerOff();  
  Serial.println("DONE");
}

void ledTest() {
  Serial.println("LED test!");
  Red_LED_ON;
  delay(500);
  Green_LED_ON;
  delay(500);
  Red_LED_OFF;
  delay(500);
  Green_LED_OFF;
  Serial.println("DONE");
}

void btnTest() {
  Serial.println("BUTTON test!");
  boolean oldbtn = !digitalRead(BTN);
  while(!Serial.available()) {
    boolean newbtn = digitalRead(BTN);
    if (oldbtn!=newbtn) {
      Serial.println(newbtn?"BTN UP":"BTN DOWN");
      oldbtn=newbtn;
    } 
  }
  Serial.println("DONE");
}

void ppmTest() {
  Serial.println("PPM input test!");
  while(!Serial.available()) {
    for (int c=0; c<PPM_CHANNELS;c++) {
      Serial.print(PPM[c]);
      Serial.print(',');
    }      
    Serial.println();
  }
  Serial.println("DONE");
}

void rfmcomTest() {
  Serial.println("RFMxx coms test!");
  uint8_t r0,r1;
  r0 = spiReadRegister(0);
  r1 = spiReadRegister(1);
  Serial.print("RFM id bytes:");
  Serial.print(r0);
  Serial.print(',');
  Serial.println(r1);
  spiWriteRegister(5,0x55);
  r0=0;
  if (0x55 == spiReadRegister(5)) {
    spiWriteRegister(5,0xaa);
    if (0xaa == spiReadRegister(5)) {
      r0=1;
    }
  }
  if (r0) {
    Serial.println("RFMxx coms test passed!");
  } else {
    Serial.println("RFMxx coms test FAILED!");
  }
  Serial.println("DONE");
} 

void rfmintTest() {
  uint8_t foo,status=0;
  Serial.println("RFMxx interrupt test!");
  // clear any
  spiWriteRegister(5,0);
  spiWriteRegister(6,0);
  foo=spiReadRegister(3);
  foo=spiReadRegister(4);
  delay(1);
  if (nIRQ_0) {
    Serial.println("Interrupt did not clear?");
  } else {
    status=1;
    spiWriteRegister(6,0x08); // enable WUT interrupt
    spiWriteRegister(7,0x21); // enable WUT
    delay(100);
    if (nIRQ_0) {
      Serial.println("Interrupt fired");
      status++;
    } else {
      Serial.println("Interrupt did not fire");
    }      
  }
  spiWriteRegister(5,0);
  spiWriteRegister(6,0);
  foo=spiReadRegister(3);
  foo=spiReadRegister(4);
  delay(1);
  if (nIRQ_0) {
    Serial.println("Interrupt did not clear?");
  } else {
    status++;
  }
  if (status==3) {
    Serial.println("Interrupt test passed");
  } else {
    Serial.println("Interrupt test FAILED");
  }    
  Serial.println("DONE");
} 

void beacon_tone(int16_t hz, int16_t len)
{
  int16_t d = 500 / hz; // somewhat limited resolution ;)

  if (d < 1) {
    d = 1;
  }

  int16_t cycles = (len * 1000 / d);

  for (int16_t i = 0; i < cycles; i++) {
    SDI_on;
    delay(d);
    SDI_off;
    delay(d);
  }
}

void rfmSetCarrierFrequency(uint32_t f)
{
  uint16_t fb, fc, hbsel;
  if (f < 480000000) {
    hbsel = 0;
    fb = f / 10000000 - 24;
    fc = (f - (fb + 24) * 10000000) * 4 / 625;
  } else {
    hbsel = 1;
    fb = f / 20000000 - 24;
    fc = (f - (fb + 24) * 20000000) * 2 / 625;
  }
  spiWriteRegister(0x75, 0x40 + (hbsel?0x20:0) + (fb & 0x1f));
  spiWriteRegister(0x76, (fc >> 8));
  spiWriteRegister(0x77, (fc & 0xff));
}

void TX_test(uint8_t power)
{
  Serial.print("TX ON at powerlevel ");
  Serial.println(power&7);
  Green_LED_ON
  ItStatus1 = spiReadRegister(0x03);   // read status, clear interrupt
  ItStatus2 = spiReadRegister(0x04);
  spiWriteRegister(0x06, 0x00);    // no wakeup up, lbd,
  spiWriteRegister(0x07, RF22B_PWRSTATE_READY);      // disable lbd, wakeup timer, use internal 32768,xton = 1; in ready mode
  spiWriteRegister(0x09, 0x7f);  // (default) c = 12.5p
  spiWriteRegister(0x0a, 0x05);
#if (HW==4)
  spiWriteRegister(0x0b, 0x15);    // gpio0 TX State
  spiWriteRegister(0x0c, 0x12);    // gpio1 RX State
#else
  spiWriteRegister(0x0b, 0x12);    // gpio0 TX State
  spiWriteRegister(0x0c, 0x15);    // gpio1 RX State
#endif
  spiWriteRegister(0x0d, 0xfd);    // gpio 2 micro-controller clk output
  spiWriteRegister(0x0e, 0x00);    // gpio    0, 1,2 NO OTHER FUNCTION.

  spiWriteRegister(0x70, 0x2C);    // disable manchest

  spiWriteRegister(0x30, 0x00);    //disable packet handling

  spiWriteRegister(0x79, 0);    // start channel

  spiWriteRegister(0x7a, 0x05);   // 50khz step size (10khz x value) // no hopping

  spiWriteRegister(0x71, 0x12);   // trclk=[00] no clock, dtmod=[01] direct using SPI, fd8=0 eninv=0 modtyp=[10] FSK
  spiWriteRegister(0x72, 0x02);   // fd (frequency deviation) 2*625Hz == 1.25kHz

  spiWriteRegister(0x73, 0x00);
  spiWriteRegister(0x74, 0x00);    // no offset

  rfmSetCarrierFrequency(435000000);

  spiWriteRegister(0x6d, power & 0x07);

  delay(10);
  spiWriteRegister(0x07, RF22B_PWRSTATE_TX);    // to tx mode
  delay(10);
  beacon_tone(500, 5);

  spiWriteRegister(0x07, RF22B_PWRSTATE_READY);
  Green_LED_OFF
  Serial.println("DONE");
}


void loop() {
  while (!Serial.available());
  char ch=Serial.read();
  delay(100);
  while (Serial.available()) Serial.read();
  switch (ch) {
  case '1':
      buzzerTest();
      break;
  case '2':
      ledTest();
      break;
  case '3':
      btnTest();
      break;
  case '4':
      ppmTest();
      break;
  case '5':
      rfmcomTest();
      break;
  case '6':
      rfmintTest();
      break;
  case '9':
      TX_test(7);
      break;
  default:
      break;
  }
}
