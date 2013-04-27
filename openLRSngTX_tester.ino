
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

#define TIMER1_FREQUENCY_HZ 50
#define TIMER1_PRESCALER 8
#define TIMER1_PERIOD (F_CPU/TIMER1_PRESCALER/TIMER1_FREQUENCY_HZ)
#define PPM_CHANNELS 8
volatile uint16_t PPM[PPM_CHANNELS] = { 512, 512, 512, 512, 512, 512, 512, 512 };
volatile uint16_t startPulse = 0;
volatile uint8_t ppmCounter = PPM_CHANNELS; // ignore data until first sync pulse
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
  default:
      break;
  }
}
