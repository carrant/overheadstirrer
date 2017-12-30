/*
    Copyright 2017 Chris Arrant
    
    This file is part of OverheadStirrer
    .
    OverheadStirrer is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
    
    OverheadStirrer is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    
    You should have received a copy of the GNU General Public License
    along with OverheadStirrer.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <avr/io.h>
#include <avr/interrupt.h>

// Libraries
#include <AccelStepper.h>       // http://www.airspayce.com/mikem/arduino/AccelStepper/
#include <dht.h>                // https://github.com/adafruit/DHT-sensor-library
#include <ClickEncoder.h>       // https://github.com/0xPIT/encoder
#include <LiquidCrystal_I2C.h>  // https://bitbucket.org/fmalpartida/new-liquidcrystal/downloads

// datasheet                    http://www.atmel.com/Images/Atmel-42735-8-bit-AVR-Microcontroller-ATmega328-328P_Datasheet.pdf
// arduino nano pin out         http://www.pighixxx.com/test/pinouts/boards/nano.pdf
// rotary encoder intro         https://www.allaboutcircuits.com/projects/how-to-use-a-rotary-encoder-in-a-mcu-based-project/


/* Parts list
 *  
 *  Local stores (In Dallas, Texas, USA: BGMicro, Tanner Electronics, Microcenter, Home Depot)
 *  Arduino Nano
 *  Project box
 *  100uF caps
 *  100nF caps
 *  27K resistors
 *  12V power supply
 *  12V 40mm fan
 *  standoff, screw, nuts
 *  hookup wires
 *  jumper wires
 *  grommets
 *  NEMA 17 12V stepper         
 *  1/2" x 36" AL rod           https://www.homedepot.com/p/Everbilt-1-2-in-x-36-in-Aluminum-Round-Rod-801637/204273968
 *  Electrical brackets
 *  
 *  Online orders
 *  perfboard                   https://www.ebay.com/itm/10pcs-5cm-x-7cm-PCB-Prototyping-Perf-Boards-Breadboard-DIY-US/232379628629?hash=item361ae7bc55:g:JrUAAOSw8GtZSK-E
 *  16x2 LCD                    https://www.ebay.com/itm/New-Blue-IIC-I2C-TWI-1602-16x2-Serial-LCD-Module-Display-for-Arduino-SP-PL/222703433006?epid=592602913&hash=item33da28b52e:g:xB8AAOSw1DtXINYR
 *  rotary encoder              https://www.ebay.com/itm/10pcs-12mm-Rotary-Encoder-Push-Button-Switch-Keyswitch-Electronic-Components-US/161339817229?ssPageName=STRK%3AMEBIDX%3AIT&_trksid=p2057872.m2749.l2649
 *  solid state relay 1ch       https://www.ebay.com/itm/5V-DC-1-Channel-Solid-State-Relay-Board-module-High-Level-fuse-for-arduino/331599214885?hash=item4d34da7925:g:eHMAAOSwHixZo-0i
 *  drv8825                     https://www.ebay.com/itm/5-PCS-DRV8825-3D-PRINTER-STEPPER-MOTOR-DRIVER-MODULE-RAMPS-REPRAP-STEPSTICK-US/122017988734?hash=item1c68d6c07e:g:4lUAAOSw14xWK-RW
 *  DHT11                       https://www.ebay.com/itm/1PCS-Arduino-DHT11-Temperature-and-Relative-Humidity-Sensor-Module-NEW/181846902071?epid=1279768341&hash=item2a56eb8537:g:uXsAAOSwzrxUu0El
 *  NEMA 17 bracket             https://www.amazon.com/gp/product/B071NWWB7Z/ref=oh_aui_detailpage_o06_s00?ie=UTF8&psc=1
 *  5mm to 7mm shaft coupler    https://www.amazon.com/gp/product/B06X9WF7BG/ref=oh_aui_detailpage_o06_s00?ie=UTF8&psc=1 
 */


//***********************************************************************************************************
#define PIN_RELAY_FAN           9
#define PIN_RELAY_FAN_LOW       10

#define PIN_STEPPER_STEP        8
#define PIN_STEPPER_DIR         7

#define PIN_DHT11               2

#define PIN_LCD_SDA             A4
#define PIN_LCD_SCL             A5

#define PIN_ROTARY_A            A1
#define PIN_ROTARY_B            A2
#define PIN_ROTARY_BTN          A3

#define TIMER_TCCR_A            TCCR1A        // Timer/Counter Control Register A
#define TIMER_TCCR_B            TCCR1B        // Timer/Counter Control Register B
#define TIMER_OCRA              OCR1A         // timer compare match register
#define TIMER_TIMSK             TIMSK1        // timer mask interrupt


#define charCLOCKWISE           0             // LCD custom char index for clockwise symbol
#define charCOUNTERCLOCKWISE    1             // LCD custom char index for counter clockwise symbol
#define charFAN                 2             // LCD custom char index for fan

#define DEGREE_SYMBOL           (char)0xDF    // the degree symbol

#define MAX_INTERNAL_TEMP       55            // degree C (131F)
#define POST_OVERHEAT_TEMP      39            // degree C (102F) chilly summer in Texas
#define MIN_TEMP_FOR_FAN        38            // degree C (100F) Autumn in Texas
#define POST_TEMP_FOR_FAN       35            // degree C (95F)  winter in Texas

#define LCD_UPDATE_INTERVAL     1000          //  1sec
#define DHT_UPDATE_INTERVAL     60000         // 60sec

#define ONE_MILLISEC_COUNT      28860 / 1000  // how many does it take to get to 1ms?

#define ROTARY_MIN              0             // min speed for stepper
#define ROTARY_MAX              800           // max speed for stepper

#define MSEC_PER_SEC            (1000UL)
#define SECS_PER_MIN            (MSEC_PER_SEC * 60UL)
#define SECS_PER_HOUR           (SECS_PER_MIN * 60UL)
#define SECS_PER_DAY            (SECS_PER_HOUR * 24L)

#define FAN_ON                  LOW //HIGH
#define FAN_OFF                 HIGH //LOW

//***********************************************************************************************************
volatile int oneMilliSecondCounter  = 0;          //!< counter for 1ms using timer
volatile int DHTSecondCounter       = 0;          //!< counter for 20000ms

int currentSpeed                    = 0;          //!< current stepper speed

int speedDirection                  = 1;          //!< 1 = clockwise, -1 = counter clockwise

bool bUpdateLCD                     = false;      //!< Force LCD update (when RPM, fan state changes)
unsigned long tLCDUpate             = 0;          //!< Throttle LCD updates to 1sec (1sec is used because of clock)

int lastDHT                         = 0;
bool bUpdateDHT                     = true;       //!< Force update of temp sensor (when temp sensor read error occurs)
unsigned long tDHTUpate             = 0;          //!< Throttle temp updates to 20sec (reading pin causes issues with stepper)
int lastAirTemp                     = 0;          //!< Last internal temp reading
char sInsideAirTemp[4]              = "..";       //!< Internal temp as a string

bool bOverheating                   = false;      //!< Flag to indicate if the inside of the case is TOO hot
unsigned long tStartOverheating     = 0;          //!< Start time of overheating

bool bForceFanOn                    = false;      //!< User forced fan on/off
bool bFanOn                         = false;      //!< Flag to indicate if the fan is on

ClickEncoder *encoder               = NULL;       //!< Rotary encoder reader (read at 1ms intervals)
int16_t last                        = 0;          //!< Old speed before adjustment for rotary encoder

unsigned long stepperRunTime        = 0;

/* 
 Clockwise           Counterclockwise
     43210             43210
 16 0X               2 0   X
 24 1XX              6 1  XX 
 28 2XXX            14 2 XXX
 30 3XXXX           30 3XXXX 
 28 4XXX            14 4 XXX
 24 5XX              6 5  XX
 16 6X               2 6   X
  0 7                0 7
*/
const char g_Clockwise[]        PROGMEM = {16, 24, 28, 30, 28, 24, 16, 0};
const char g_CounterClockwise[] PROGMEM = {2, 6, 14, 30, 14, 6, 2, 0};
const uint8_t g_FanCharacter[]  PROGMEM = { 0b01100, 0b01100, 0b00101, 0b11011, 0b11011, 0b10100, 0b00110, 0b00110 }; // think I like the binary better...

//***********************************************************************************************************

// Set the pins on the I2C chip used for LCD connections:
//                    addr, en, rw, rs, d4, d5, d6, d7, bl, blpol
LiquidCrystal_I2C lcd(0x3f, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);  // Set the LCD I2C address
AccelStepper stepper(AccelStepper::DRIVER, PIN_STEPPER_STEP, PIN_STEPPER_DIR);
dht DHT;


//***********************************************************************************************************

String formatTimeLCD(unsigned long t);
void setupStepperClockTimer();
void updateLCD();
void readDHT11();
void readRotaryEncoder();
void setRelayFan(bool bOn);


//***********************************************************************************************************

//******************************************************
// formatTimeLCD
//! Convert a time in milliseconds to a string
//!
//! @param [in] t   Time in milliseconds to convert
//!
//! @return String of convert time
//******************************************************
String formatTimeLCD(unsigned long t)
{
  int iDays = t / SECS_PER_DAY;
  t -= iDays * SECS_PER_DAY;
  
  int iHours = t / SECS_PER_HOUR;
  t -= iHours * SECS_PER_HOUR;

  int iMin = t / SECS_PER_MIN;
  t -= iMin * SECS_PER_MIN;

  int iSec = t / MSEC_PER_SEC;
  t-= iSec * MSEC_PER_SEC;
  
  int iMs = t;

  char sTime[64];
  sprintf(sTime, "%02d:%02d:%02d", iHours, iMin, iSec);

  return sTime;
}

//******************************************************
// setupStepperClockTimer
//! Setup the clock timer for the stepper ISR
//******************************************************
void setupStepperClockTimer()
{
  // use CTC (clear time on compare match)
  // see timer_interrupt_test.ino for more info about clock timers
  cli();                        // disable global interrupts
  
  TIMER_TCCR_A = 0;             // set entire TCCR?A register to 0
  TIMER_TCCR_B = 0;             // set entire TCCR?B register to 0 

  TIMER_TCCR_B &= 0xF8;         // 11111000b - set CS12, CS11, and CS10 to zero
  TIMER_TCCR_B |= (1 << CS10);  // enable timer with no scaler

  TIMER_OCRA = 550;             // set compare match register to desired timer count

  TIMER_TCCR_B |= (1 << WGM12); // turn on timer compare on match (CTC) mode

  TIMER_TIMSK |= (1 << OCIE1A); // enable timer compare interrupt
  
  sei();                        // enable global interrupts
}


//******************************************************
// updateLCD
//! Update the LCD display
//! The LCD is only updated once every second (because of the elapsed time)
//! 
//! If the internal temp is too high, "Overheating" is printed along with
//! the duration of overheating
//!
//!             11111
//!  0123456789012345
//! 0Dir X   1234 RPM
//! 1xxoC  ? 00:00:00
//!        ^fan symbol
//!
//!             11111
//!  0123456789012345
//! 0Overheating xxoC
//! 1?   00:00:00    
//!  ^fan symbol
//******************************************************
void updateLCD()
{
  // only update every second
  if (!bUpdateLCD && (millis() - tLCDUpate) < LCD_UPDATE_INTERVAL)
    return;
    
  tLCDUpate = millis();
  bUpdateLCD = false;

  char sLine[32];
  
  lcd.setCursor(0,0);

  if (bOverheating)
  {
    //             11111
    //  0123456789012345
    // 0Overheating xxoC
    // 1?   00:00:00    
    //  ^fan symbol
    
                  //1234567890123456
    sprintf(sLine, "Overheating %02s%cC", sInsideAirTemp, DEGREE_SYMBOL);
    lcd.print(sLine);

    String sOverheatTime = formatTimeLCD(millis() - tStartOverheating);
    
    lcd.setCursor(0,1);
    sprintf(sLine, "%c   %8s    ", charFAN, sOverheatTime.c_str());
    //lcd.print("    Shutdown    ");
    lcd.print(sLine);
    
    return;
  }

  //             11111
  //  0123456789012345
  // 0Dir X   1234 RPM
  // 1xxoC  ? 00:00:00
  //        ^fan symbol
  
  lcd.print("Dir ");
  if (speedDirection < 0)
    lcd.print(char(charCOUNTERCLOCKWISE));
   else
    lcd.print(char(charCLOCKWISE));

  sprintf( sLine, "   %4d RPM", abs(currentSpeed) );
  lcd.print(sLine);

  
  lcd.setCursor(0,1);
  String sRunTime;
  if (currentSpeed > 0)
    sRunTime = formatTimeLCD(stepperRunTime);
  else
    sRunTime = "  IDLE  ";


  if (sInsideAirTemp[0] == 'E')
    sprintf(sLine,"%03s   %c %8s", sInsideAirTemp, bFanOn ? charFAN : ' ', sRunTime.c_str());
  else
    sprintf(sLine,"%02s%cC  %c %8s", sInsideAirTemp, DEGREE_SYMBOL, bFanOn ? charFAN : ' ', sRunTime.c_str());
   
  lcd.print(sLine);
}

//******************************************************
// readDHT11
//! Read the internal temp
//!
//! NOTE: Reading the temp is sensitve to being interrupted
//! If the temp read is interrupted then the library returns DHTLIB_ERROR_CHECKSUM
//! To avoid this interrupts are disabled while reading
//! However this presents a small problem by introducing jitter to the stepper
//!
//! To reduce the jitter, the temp is only read every 20seconds
//!
//! Overheating checking is done after reading the temperature
//******************************************************
void readDHT11()
{
  // only read every 20seconds
  if (!bUpdateDHT && (millis() - tDHTUpate) < DHT_UPDATE_INTERVAL)
    return;
  else 
  {
    // if we need to force an update, only check every when the LCD is updating    
    if (bUpdateDHT && (millis() - tDHTUpate) < LCD_UPDATE_INTERVAL)    
      return;
  }
    
  tDHTUpate = millis();
  bUpdateDHT = false;

  // reading the pin had problems with the timer interrupt for the stepper advance
  // so disable interrupts when reading
  // HOWEVER this might cause the stepper to skip steps :-(
//  cli();
//  int chk = DHT.read11(PIN_DHT11);
//  sei();

  bool bSuccess = false;
  switch (lastDHT)
  {
    case DHTLIB_ERROR_CHECKSUM: 
      sprintf(sInsideAirTemp, "%3s", "ER1");
      bUpdateDHT = true;
      break;
      
    case DHTLIB_ERROR_TIMEOUT: 
      sprintf(sInsideAirTemp, "%3s", "ER2");
      bUpdateDHT = true;
      break;

    case DHTLIB_ERROR_CONNECT: 
      sprintf(sInsideAirTemp, "%3s", "ER3");
      bUpdateDHT = true;
      break;
    
    case DHTLIB_ERROR_ACK_L: 
      sprintf(sInsideAirTemp, "%3s", "ER4");
      bUpdateDHT = true;
      break;
    
    case DHTLIB_ERROR_ACK_H: 
      sprintf(sInsideAirTemp, "%3s", "ER5");
      bUpdateDHT = true;
      break;

    case DHTLIB_OK:  
      dtostrf(DHT.temperature, 2, 0, sInsideAirTemp);
      lastAirTemp = atoi(sInsideAirTemp);
      //dtostrf(DHT.humidity, 6, 2, g_sHumidity);
      bSuccess = true;
      break;

    default: 
      sprintf(sInsideAirTemp, "%3s", "ER?");
      bUpdateDHT = true;
      break;
  }

  // if the temperature was read then check for overheating
  if (bSuccess)
  {
    // make sure the internal temp isn't too high
    if (lastAirTemp > MAX_INTERNAL_TEMP)
    {
      if (!bOverheating)
      {
        tStartOverheating = millis();
        bOverheating = true;
      }
    }
    else
    {
      // if the internal temp was too high check if the temp has cooled down to a managable level
      if (bOverheating)
      {
        if (lastAirTemp <= POST_OVERHEAT_TEMP)
          bOverheating = false;
      }
    }

    // check if the fan should be turned on
    checkTempForFan();    
  }  

  lastDHT = DHTLIB_ERROR_ACK_H;
}

//******************************************************
// readRotaryEncoder
//! Read rotary encoder state
//!
//! MUST be called every 1ms
//!
//! Turning the rotary encoder will increment or decrement the current RPM
//!
//! Pushing the rotary encoder will change direction
//! Holding the rotary encoder will stop stirring
//******************************************************
void readRotaryEncoder()
{
  // inc or dec the current speed based on the direction the rotary switch was rotated
  currentSpeed += encoder->getValue();

  // check agains min and max values
  if (currentSpeed < ROTARY_MIN)
    currentSpeed = ROTARY_MIN;
  else if (currentSpeed > ROTARY_MAX)
    currentSpeed = ROTARY_MAX;

  // if the speed has changed
  if (currentSpeed != last) 
  {
    // save speed
    last = currentSpeed;

    // updated stepper driver with new speed
    // positive speed = clockwise
    // negative speed = counter clockwise
    if (speedDirection == 1)
      stepper.setSpeed(currentSpeed);
    else
      stepper.setSpeed(-currentSpeed);

    // since the speed changed, force the LCD to update
    bUpdateLCD = true;
  }

  // check if the rotary button was clicked
  ClickEncoder::Button b = encoder->getButton();
  if (b != ClickEncoder::Open) 
  {
    switch(b)
    {
      // change direction
      case ClickEncoder::Clicked:
        speedDirection *= -1;
    
        if (speedDirection == 1)
          stepper.setSpeed(currentSpeed);
        else
          stepper.setSpeed(-currentSpeed);
    
        bUpdateLCD = true;
      break;

      // stop the stepper
      case ClickEncoder::Held:
        currentSpeed = 0;
        stepperRunTime = 0; // reset stepper runtime
        stepper.setSpeed(currentSpeed);
    
        bUpdateLCD = true;
      break;

      // force the fan on
      case ClickEncoder::DoubleClicked:
        bForceFanOn = !bForceFanOn;
        setRelayFan(bForceFanOn);
      break;
    }
  } 
 
}

//******************************************************
// setRelayFan
//! Enable/disable the fan
//!
//! @param [in] bOn   TRUE - enable fan; FALSE disable fan
//******************************************************
void setRelayFan(bool bOn)
{
  // same state? do nothing
  if (bOn == bFanOn)
    return;

  // toggle the fan
  digitalWrite(PIN_RELAY_FAN,bOn ? FAN_ON : FAN_OFF);

  bFanOn = bOn;
}

//******************************************************
// checkTempForFan
//! If the internal temp is above a threshold then turn on fan
//!
//! If the fan is running, wait for the temp to drop below a threshold before shutting off the fan
//******************************************************
void checkTempForFan()
{

  // Too HOT! We need the fan
  if (bOverheating)
  {
    setRelayFan(true);
    return;
  }

  // user forced fan on by double clicking on rotary encoder
  if (bForceFanOn)
    return;
  
  // if the fan is on, check if the temp has dropped below the shutoff threshold
  if (bFanOn)
  {
    if (lastAirTemp <= POST_TEMP_FOR_FAN)  
      setRelayFan(false);
  }
  // check if the internal temp is above the enable threshold
  else
  {
    if (lastAirTemp >= MIN_TEMP_FOR_FAN)  
      setRelayFan(true);
  }
}

//******************************************************
// setup
//! 
//! Create the rotary encoder handler
//! Setup the stepper driver
//! Add custom chars to LCD
//! Prep the LCD
//! 
//******************************************************
void setup()
{  
  //Serial.begin(115200);
  //Serial.println("Initializing");

  // stepper driver
  stepper.setMaxSpeed(ROTARY_MAX);
  stepper.setSpeed(currentSpeed);

  // display
  lcd.begin(16,2);         // initialize the lcd for 16 chars 2 lines, turn on backlight
  lcd.clear();

  // custom char map for counter and clockwise rotation
  lcd.createChar(charCLOCKWISE, g_Clockwise);
  lcd.createChar(charCOUNTERCLOCKWISE, g_CounterClockwise);
  lcd.createChar(charFAN, (char *)g_FanCharacter);

  lcd.setCursor(0,0);
  lcd.print("  Initializing  ");

  lastDHT = DHT.read11(PIN_DHT11);
  readDHT11();

  // fan relay
  pinMode(PIN_RELAY_FAN, OUTPUT);
  digitalWrite(PIN_RELAY_FAN,FAN_OFF);

  // rotary encoder
  encoder = new ClickEncoder(PIN_ROTARY_A, PIN_ROTARY_B, PIN_ROTARY_BTN);
  encoder->setAccelerationEnabled(true);
  
  setupStepperClockTimer();
}


//******************************************************
// loop
//! 
//! Read rotary encoder
//! Read current internal temp (also check for overheating) (every 20sec)
//! Update the LCD (every 1ms)
//******************************************************
void loop()
{  
  readRotaryEncoder();
  readDHT11(); // every 20s
  updateLCD(); // every  1s
}


//***********************************************************************************************************

//******************************************************
// ISR for timer 1, CTC (clear time on compare match)
//! If not overheating then step the stepper motor
//!
//! Also inc the milliseconds timer count and after 1ms
//! call the rotary encoder interval
//******************************************************
ISR(TIMER1_COMPA_vect)
{
  if (!bOverheating)
   stepper.runSpeed();


  // rotary encoder must be read every 1ms
  // so we glob on to this timer to count 1ms
  oneMilliSecondCounter++;
  if (oneMilliSecondCounter >= ONE_MILLISEC_COUNT)
  {
    if (stepper.speed() != 0.0)
      stepperRunTime++;
      
    oneMilliSecondCounter = 0;
    encoder->service();

    DHTSecondCounter++;
    if (DHTSecondCounter >= DHT_UPDATE_INTERVAL)
    {
      DHTSecondCounter = 0;
      lastDHT = DHT.read11(PIN_DHT11);
    }
    
  }
}
