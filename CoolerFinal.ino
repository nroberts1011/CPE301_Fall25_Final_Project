// Names: Victor Rosales & Natalie Roberts
// Brief Description: Our Swamp Cooler
// Date: 12/12/25

// lcd for final proj

#include <LiquidCrystal.h>
#include <DHT.h>

#define DHTPIN    8 //pin 8 for humid/temp sensor
#define DHTTYPE   DHT11

#define THRESHOLD_WATER_LOW 30
//#define THRESHOLD_WATER_HIGH 100

//#define TEMP_THRESHOLD_LOW 71.5
#define TEMP_THRESHOLD_HIGH 75

#define START_BUTTON_PIN   2
#define RESTART_BUTTON_PIN 3  
#define STOP_BUTTON_PIN    18 


DHT dht(DHTPIN, DHTTYPE);

enum class State
 {
    Disabled,
    Idle,
    Running,
    Error
};


volatile State Cooler = State::Disabled;

float temp = 0;
float humid = 0;
unsigned int water_level = 0;


// LCD pins and Arduino pins
const int RS = 11, EN = 12, D4 = 6, D5 = 7, D6 = 4, D7 = 5;
LiquidCrystal lcd(RS, EN, D4, D5, D6, D7);

//Define port f register pointers 
volatile unsigned char* port_F = (unsigned char*) 0x31; 
volatile unsigned char* ddr_F = (unsigned char*) 0x30; 
volatile unsigned char* pin_F  = (unsigned char*) 0x2F;

//Define port f register pointers 
volatile unsigned char* pin_A  = (unsigned char*)0x20;  // PINA
volatile unsigned char* ddr_A  = (unsigned char*)0x21;  // DDRA
volatile unsigned char* port_A = (unsigned char*)0x22;  // PORTA

//define motor speed
int mSpeed= 150;
int speedPin=24;

//Define port k register pointers 
volatile unsigned char* port_K = (unsigned char*)0x4B;
volatile unsigned char* ddr_K  = (unsigned char*)0x4A;
volatile unsigned char* pin_K  = (unsigned char*)0x49;

//my millis
volatile unsigned char *myTCCR1A = (unsigned char *) 0x80;
volatile unsigned char *myTCCR1B = (unsigned char *) 0x81;
volatile unsigned char *myTCCR1C = (unsigned char *) 0x82;
volatile unsigned char *myTIMSK1 = (unsigned char *) 0x6F;
volatile unsigned int  *myTCNT1  = (unsigned  int *) 0x84;
volatile unsigned char *myTIFR1 =  (unsigned char *) 0x36;

// Port B
volatile unsigned char* port_B = (unsigned char*)0x25;  // PORTB
volatile unsigned char* ddr_B  = (unsigned char*)0x24;  // DDRB
volatile unsigned char* pin_B  = (unsigned char*)0x23;  // PINB
// Port D 
volatile unsigned char* ddr_D  = (unsigned char*) 0x2A; // DDRD
volatile unsigned char* port_D = (unsigned char*) 0x2B; // PORTD

//water sensor
volatile unsigned char* my_ADMUX = (unsigned char*) 0x7C;
volatile unsigned char* my_ADCSRB = (unsigned char*) 0x7B;
volatile unsigned char* my_ADCSRA = (unsigned char*) 0x7A;
volatile unsigned int* my_ADC_DATA = (unsigned int*) 0x78;

#define RDA 0x80
#define TBE 0x20


//pins A0-A7
#define WRITE_HIGH_PF(pin_num)  *port_F |= (0x01 << pin_num);
#define WRITE_LOW_PF(pin_num)  *port_F &= ~(0x01 << pin_num);

//pins A8-A15
#define WRITE_HIGH_PK(pin_num)  *port_K |= (0x01 << pin_num);
#define WRITE_LOW_PK(pin_num)  *port_K &= ~(0x01 << pin_num);

#define WRITE_HIGH_PA(pin_num)  *port_A |= (0x01 << pin_num);
#define WRITE_LOW_PA(pin_num)  *port_A &= ~(0x01 << pin_num);

const unsigned long LOOP_INTERVAL_MS = 2000;
unsigned long lastLoopMs = 0;

// for fan kickstart without delay()
bool fanKickActive = false;
unsigned long fanKickStartMs = 0;
const unsigned long FAN_KICK_MS = 25;

void setup()
{
  Serial.begin(9600);
  lcd.begin(16, 2); // set up number of columns and rows
  dht.begin();

  //water sensor  
  // setup the ADC
  adc_init();

  //TESTING STATE
  Cooler = State::Disabled;
  //*ddr_K |= (0x01 << 7); //A15 green light for idle output mode
  *ddr_F |= (0x01 << 7); //A7 green light for idle output mode
  *ddr_F |= (0x01 << 6); // A6 blue light for Running


  *ddr_F |= (0x01 << 5); // Set A5 (Yellow LED) as output

  // START on D2  -> PE4
  DDRE &= ~(1 << 4);   // input
  PORTE |= (1 << 4);   // pull-up

  // RESTART on D3 -> PE5
  DDRE &= ~(1 << 5);   // input
  PORTE |= (1 << 5);   // pull-up

  // STOP on D18 -> PD3
  DDRD &= ~(1 << 3);   // input
  PORTD |= (1 << 3);   // pull-up

  //LED set pin as output 
  *ddr_F |= (0x01 << 1);//pin A1 Error mode

  //motor pings
  *ddr_A |= (0x01 << 2);// pin digital 24
  *ddr_A |= (0x01 << 4); // pin digital d26
  *ddr_A |= (0x01 << 6); // Pin digital 28

  attachInterrupt(digitalPinToInterrupt(START_BUTTON_PIN), handleStart, FALLING);     
  attachInterrupt(digitalPinToInterrupt(RESTART_BUTTON_PIN), handleRestart, FALLING); 
  attachInterrupt(digitalPinToInterrupt(STOP_BUTTON_PIN), handleStop, FALLING);
}

void loop()
{
  unsigned long now = millis();

  if (fanKickActive && (now - fanKickStartMs >= FAN_KICK_MS))
  {
    fanKickActive = false;
    analogWrite(speedPin, mSpeed);   // drop to normal speed after kick
  }

  // delay(2000);

  // run main loop every 2000ms 
  if (now - lastLoopMs < LOOP_INTERVAL_MS)
  {
    return;  // non-blocking wait
  }
  lastLoopMs = now;

  humid = dht.readHumidity();
  temp = dht.readTemperature(true); // true for Fahrenheit
  //humid = 40.0;
  //temp  = 75.0;

  water_level = adc_read(0);

  Serial.print("Temp: ");
  Serial.print(temp);
  Serial.print("  Hum: ");
  Serial.println(humid);
  Serial.print("  Water lvl: "); Serial.println(water_level);
  Serial.print("Cooler state: ");
  Serial.println(stateName(Cooler));

  //RUN THE CURRENT STATE
  if(Cooler == State::Disabled)
  {
    TURN_OFF_FAN();
    lcd.noDisplay();
    WRITE_LOW_PF(6);
    WRITE_LOW_PF(7);
    WRITE_LOW_PF(1);

    WRITE_HIGH_PF(5);
  }
  else if(Cooler == State::Error)
  {
    TURN_OFF_FAN();
    errorMessage(lcd);
    WRITE_LOW_PF(7);
    WRITE_HIGH_PF(1);
    WRITE_LOW_PF(6);
    WRITE_LOW_PF(5);
  }
  else if(Cooler == State::Running)
  {
    TURN_ON_FAN();
    printStats(lcd,temp,humid);
    WRITE_LOW_PF(7);
    WRITE_LOW_PF(1);
    WRITE_HIGH_PF(6);
    WRITE_LOW_PF(5);
  }
  else if (Cooler == State::Idle)
  {
    TURN_OFF_FAN();
    printStats(lcd,temp,humid);
    WRITE_HIGH_PF(7);
    WRITE_LOW_PF(1);
    WRITE_LOW_PF(6);
    WRITE_LOW_PF(5);
  }

  //CHECK IF STATE NEEDS TO CHANGE
  if(Cooler != State::Disabled)
  {
    if(water_level <=  THRESHOLD_WATER_LOW)
    {
      Cooler = State::Error;
    }
    else if(temp > TEMP_THRESHOLD_HIGH && water_level > THRESHOLD_WATER_LOW)
    {
      Cooler = State::Running;
    }
    else if(temp <= TEMP_THRESHOLD_HIGH && water_level > THRESHOLD_WATER_LOW)
    {
      Cooler = State::Idle;
    }
  }

  // delay(2000);
}



// code from lab for millis
void my_delay(unsigned int freq)
{
  // calc period
  double period = 1.0/double(freq);
  // 50% duty cycle
  double half_period = period/ 2.0;
  // clock period def
  double clk_period = 0.0000000625;
  // calc ticks
  //ticks= (T wave/2)/Tclk 
  unsigned int ticks = (unsigned int)(half_period/clk_period);
  // stop the timer
  *myTCCR1B &= 0xF8;//modified
  // set the counts
  *myTCNT1 = (unsigned int) (65536 - ticks);
  //* myTCCR1A = 0x00;
  // start the timer
  * myTCCR1B |= 0x03;//modified
  // wait for overflow
  while((*myTIFR1 & 0x01)==0); 
  // stop the timer
  *myTCCR1B &= 0xF8;//modified   
  // reset TOV           
  *myTIFR1 |= 0x01;//modified
}

//debug thing
char* stateName(State s)
{
  switch(s) 
  {
    case State::Disabled: return "Disabled";
    case State::Idle:     return "Idle";
    case State::Running:  return "Running";
    case State::Error:    return "Error";
    default:              return "broken";
  }
}

// ISR
void handleStart()
 {
  if (Cooler == State::Disabled)
  {
    Serial.println("Start button pressed");
    Cooler = State::Idle;
  }
}

void handleRestart() 
{
  if (Cooler == State::Error) 
  {
    Serial.println("Restart button pressed");
    Cooler = State::Idle;
  }
}

void handleStop() 
{
  Serial.println("Stop button pressed");
  Cooler = State::Disabled;
}


void printStats(LiquidCrystal lcd, float t, float h)
{
  lcd.display();
  // lcd.clear();

  lcd.setCursor(0, 0);
  lcd.write("Temp(F):  ");
  lcd.setCursor(10, 0);
  lcd.print(t);

  lcd.setCursor(0, 1);
  lcd.write("Humidity:");
  lcd.setCursor(10, 1);
  lcd.print(h);
}

void errorMessage(LiquidCrystal lcd)
{
    lcd.display();
    lcd.clear();

    lcd.setCursor(0, 0);
    lcd.write("Err: Water level");
    lcd.setCursor(3, 1);
    lcd.write("is too low");
}
  
void TURN_OFF_FAN() {
    // turn off fan
    WRITE_LOW_PA(6);
    WRITE_LOW_PA(4);
    analogWrite(speedPin, 0);

    // stop any pending kick
    fanKickActive = false;
}


void TURN_ON_FAN() {
    // turn on fan
    WRITE_HIGH_PA(6);   // Pin A6 (digital 28)
    WRITE_LOW_PA(4);    // Pin A4 (digital 26)

    // non-blocking kick then settle 
    if (!fanKickActive)
    {
      analogWrite(speedPin, 255);        // kick
      fanKickActive = true;
      fanKickStartMs = millis();         // start timer
    }
}

void adc_init()
{
  // setup the A register
  // set bit   7 to 1 to enable the ADC
  *my_ADCSRA |= 0x80;
  
  // clear bit 6 to 0 to disable the ADC trigger mode
  *my_ADCSRA &= ~0x40;

  // clear bit 5 to 0 to disable the ADC interrupt
  *my_ADCSRA &= ~0x20;

  // clear bit 0-2 to 0 to set prescaler selection to slow reading
  *my_ADCSRA &= ~0x07;

  // setup the B register
  // clear bit 3 to 0 to reset the channel and gain bits
  *my_ADCSRB &= ~0x08;

  // clear bit 2-0 to 0 to set free running mode
  *my_ADCSRB &= ~0x07;

  // setup the MUX Register
  // clear bit 7 to 0 for AVCC analog reference
  *my_ADMUX &= ~0x80;

  // set bit 6 to 1 for AVCC analog reference
  *my_ADMUX |= 0x40;

  // clear bit 5 to 0 for right adjust result
  *my_ADMUX &= ~0x20;

  // clear bit 4-0 to 0 to reset the channel and gain bits
  *my_ADMUX &= ~0x1F;
}

unsigned int adc_read(unsigned char adc_channel_num) //work with channel 0
{
    // clear the channel selection bits (MUX 4:0)
  *my_ADMUX &= ~0x1F;

  // clear the channel selection bits (MUX 5) hint: it's not in the ADMUX register
  *my_ADCSRB &= ~0x08;

  // set the channel selection bits for channel 0
  *my_ADMUX |= 0x00;

  // set bit 6 of ADCSRA to 1 to start a conversion
  *my_ADCSRA |= 0x40;

  // wait for the conversion to complete
  while((*my_ADCSRA & 0x40) != 0);
  // return the result in the ADC data register and format the data based on right justification (check the lecture slide)
  
  unsigned int val = *my_ADC_DATA;
  return val;
}