//CLK100V12 12/12/2020
//We always have to include the library
#include "clock.h"
#include "LedControl.h"
#include <Wire.h>
#include "DS3231.h"
#include <TimerOne.h>
/* clock display variables */
#define NDIGITS         8
#define NDIGITS_MASK    7

// menu states
#define Menu_Off    0
#define Set_Time    1
#define Set_UTCOnOff    2
#define Set_UTC_Hours    3
#define Set_Alarm    4
#define Toggle_Alarm    5
#define Set_Local12_24    6
#define Set_Disp_Bright    7
// submenu states
#define Set_Hours    8
#define Set_Minutes    9
#define Set_Seconds    10
#define Inc_Hours    11
#define Dec_Hours    12
#define Inc_Minutes    13
#define Dec_Minutes    14
#define Inc_Seconds    15
#define Toggle_UTCOnOff    16
#define Inc_UTC_Hours    17
#define Set_Alarm_Hours    18
#define Set_Alarm_Minutes    19
#define Inc_Alarm_Hours    20
#define Dec_Alarm_Hours    21
#define Inc_Alarm_Minutes    22
#define Dec_Alarm_Minutes    23
#define Toggle_Alarm_OnOff    24
#define Toggle_Local_1224    25
#define Disp_Brightness    26
#define Inc_Brightness    27
#define Dec_Brightness    28
#define Alarm_Active    29

#define Menu_Button    3
#define Inc_Button    5
#define Dec_Button    6
//               Dpabcdefg
#define ltr_A    B01110111 // letter A
#define ltr_u    B00011100 // letter u
#define ltr_t    B00000111 // letter t
#define ltr_c    B00001101 // letter c
#define ltr_o    B00011101 // letter o
#define ltr_O    B01111110 // letter O
#define ltr_F    B01000111 // letter F
#define ltr_n    B00010101 // letter n
#define ltr_L    B00001110 // letter L
#define ltr_S    B01011011 // letter S
#define ltr_E    B01001111 // letter E
#define ltr_H    B00110111 // letter H
#define ltr_d    B00111101 // letter d
#define ltr_i    B00010000 // letter i
#define ltr_P    B01100111 // letter P
#define ltr_b    B00011111 // letter b

/*
 Now we need a LedControl to work with.
 ***** These pin numbers will probably not work with your hardware *****
 pin 12 is connected to the DataIn 
 pin 13 is connected to the CLK 
 pin 10 is connected to LOAD 
 We have only a single MAX72XX.
 */
LedControl lc=LedControl(12,13,10,1);

/* we always wait a bit between updates of the display */
unsigned long delaytime=500;
unsigned short adr = 0;
byte segbuf[NDIGITS] ;
int segcnt = 0;
bool UTC_led = false;
bool LOCAL_led = false;
bool twelve_hr_mode = true;
bool am_pm = false; // false with AM, true when PM
bool UTC_On = true;
bool alarm_On = false; // if true the alarm is set to trigger an alarm event on the time match
bool alarm_am_pm = false; // false with AM, true when PM
bool alarm_Event = false; // set when an alarm match occurs, cleared when any switch is pressed while on
bool time_menu = false; // if true we are setting the time
int disp_Bright = 8; // value that sets the display brightness
volatile bool refresh = false; // when true update the display with no delay
volatile bool fast_update = false; // when true update the display with no delay

int temp_secs;
int sec_count; // counts from 0 to 19 used for display of UTC or LOCAL hours
int temp_min;
int temp_hr;
int temp_utc_hr;
int sec_ones;
int sec_tens;
int min_ones;
int min_tens;
int hr_ones;
int hr_tens;
int utc_hr_ones;
int utc_hr_tens;
int utc_offset = 8;
int alarm_min = 45; // alarm variables
int alarm_hr = 6;
int alarm_min_ones = 8;
int alarm_min_tens = 0;
int alarm_hr_ones = 9;
int alarm_hr_tens = 0;
int temp_v = 0; // temporary variable used to set the time
int menu_sec_tens = 0; // used during the menu mode
volatile unsigned int local_timer_tick = 0; // used to time local events

// keypress temporary variables
unsigned long keypress_timer = 0; // variable to hold the current timer tick reading
unsigned long keypress_timer_delta = 0; // variable to hold the difference between timer tick readings 

// 0bDpGFEDCBA 0 segment on, 1 segment off
byte seg[] = {
    0b11000000,  // 0
    0b11111001,  // 1
    0b10100100,  // 2
    0b10110000,  // 3
    0b10011001,  // 4
    0b10010010,  // 5
    0b10000010,  // 6
    0b11111000,  // 7
    0b10000000,  // 8
    0b10010000,  // 9  
    0b10001000,  // A  
    0b10010000,  // b  
    0b10010000,  // C  
    0b10010000,  // d  
    0b10010000,  // E  
    0b10010000,  // F  
    0b10010000,  // H  
    0b10010000,  // I  
    0b10010000,  // J  
    0b10010000,  // L  
    0b10010000,  // n  
    0b10010000,  // o  
    0b10010000,  // P  
    0b10010000,  // r  
    0b10010000,  // S  
    0b10010000,  // U  
    0b10010000,  // Y  
    0b10010000,  // Z  
} ;

// switch pin assignements on clock main board
// Ref Function Pin num.  Ard Pin
// SW1 SET     6         4
// SW2 PLUS      5         3
// SW3 MINUS     4         2
// constants won't change. They're used here to set pin numbers:
const int Menu = 4;    // the number of the pushswitch pin
const int Set = 3;    // the number of the pushswitch pin
const int PlusSw = 2;    // the number of the pushswitch pin
const int Buzzer = 9;    // the number of the buzzer
const int TP1 = 5;    // the number of the test point

volatile int switchNumber = 0;             // identifies which switch input
volatile int TempswitchNumber = 0;     // temporary value for the switch input
volatile bool switchPress = false;     // flags that we have a new switch input
bool update_display =  false; // used to determine when to refresh the display
unsigned int marker = 0;
unsigned int last_read_time = 0;

// Variables will change:
int ledState = LOW;         // the current state of the output pin
int switchState;             // the current reading from the input pin
int lastButtonState = LOW;   // the previous reading from the input pin

int MenuState = Menu_Off; // variable used to determine what Menu state we are in


DS3231 clock;
RTCDateTime dt;

// pin change interrupt routine
ISR (PCINT2_vect) // handle pin change interrupt for D0 to D7 here
{
//digitalWrite(TP1, HIGH);
TempswitchNumber = ((PIND & B00011100) >> 2); // read the switch
//digitalWrite(TP1, LOW);
// Read and save the timer value
marker = millis();

if ((marker - last_read_time) < 200  )// then ignore input and record time
  {
  last_read_time = marker;  
  return;
  }

if (TempswitchNumber == 7)
  {
  return;
  }
  
switchNumber = TempswitchNumber;
last_read_time = marker;
switchPress = true; // set the new switch input flag    
}


int check_rollover_min(int value)
{
if (value >= 60)
  {
  value = 0;
  return(value);
  }
else
  {
  return(value);
  }
}

int check_rollunder_min(int value)
{
if (value < 0)
  {
  value = 59;
  return(value);
  }
else
  {
  return(value);
  }
}

int check_rollover_hr(int value)
{
  if (value >= 24)
    {
    value = 0;
    }
    return(value);
}

int check_rollunder_hr(int value)
{
if (value < 0)
  {
  value = 23;
  return(value);
  }
else
  {
  return(value);
  }
}

void turn_alarm_on(int duty_cycle)
{
int i,j;  
for (i=duty_cycle; i > 0; i--)
  {
  tone(Buzzer, 800); // turn on the buzzer
  for (j=duty_cycle; j > 0; j--)
    {
    delay(50);  
    }  
  noTone(Buzzer); // turn off the buzzer
  delay(100);  
  }
}

bool check_alarm(void) // routine to check if we have an alarm event
{
if (alarm_On)
  {
    if (alarm_hr == dt.hour)
    {
      if (alarm_min == dt.minute)
      {
       if (dt.second == 0)
       {
       return(true);
       }
       else return(false);        
      }
      else return(false);  
    }
  else return(false);  
  }
else return(false);  
}

// convert alarm_min to alarm_min_tens and alarm_min_ones
void conv_alarm_min(int alarm_min)
{
  alarm_min_tens=0;
  temp_min = alarm_min; 
  while (temp_min > 9)
    {
    alarm_min_tens++;
    temp_min = (temp_min - 10);
    }
  alarm_min_ones = temp_min;
}

void conv_to_two_digit(int value, int segbuftens, int segbufones, bool blanktens)
{
  int tens = 0; 
  int ones = 0;
  temp_v= value;
  // check if 24 hour mode and adjust
  if (twelve_hr_mode)
  {
    if (temp_v > 12)
    {
      temp_v = temp_v - 12;  
      }
    else
      {
      temp_v++;
      } 
  }  
  while (temp_v > 9) // adjust hour tens digit
  {
    tens++;
    temp_v = (temp_v - 10);
  }
  ones = temp_v;
  segbuf[segbufones] = ones;
  // blank tens if zero
  if (blanktens)
  {
    if (tens == 0) // blank the tens if it is zero
    {
      segbuf[segbuftens] = 16; // blank
    }
    else
     {
    segbuf[segbuftens] = tens; // tens 1 to 9    
     }
  }
}

void conv_to_two_digit_min(int value, int segbuftens, int segbufones)
{
  int tens = 0; 
  int ones = 0;
  temp_v= value;
  while (temp_v > 9) // adjust min tens digit
  {
    tens++;
    temp_v = (temp_v - 10);
  }
  ones = temp_v;
  segbuf[segbufones] = ones;
  segbuf[segbuftens] = tens; // tens 1 to 9    
  
}

void clear_display(void)
{
  lc.setRow(0,7,B00000000); // Clear utc tens hours
  lc.setRow(0,6,B00000000); // Clear utc ones hours
  lc.setRow(0,5,B00000000); // Clear tens hours
  lc.setRow(0,4,B00000000); // Clear ones hours
  lc.setRow(0,3,B00000000); // Clear tens minutess
  lc.setRow(0,2,B00000000); // Clear ones minutess
  lc.setRow(0,1,B00000000); // Clear seconds tens
  lc.setRow(0,0,B00000000); // Clear seconds ones
  delay(delaytime);
}

void set_clear_dp(void)
{
if (dt.hour >= 12 and twelve_hr_mode)
  {
  lc.setChar(0,4,segbuf[4],true);        
  }
  else
  { 
  lc.setChar(0,4,segbuf[4],false); 
  }
}

void pciSetup(byte pin)
{
    *digitalPinToPCMSK(pin) |= bit (digitalPinToPCMSKbit(pin));  // enable pin
    PCIFR  |= bit (digitalPinToPCICRbit(pin)); // clear any outstanding interrupt
    PCICR  |= bit (digitalPinToPCICRbit(pin)); // enable interrupt for the group
}


void LED_irq(void) // displays the time every interupt
{
// update the display every 500 mS
if (local_timer_tick == 100)
  {
  update_display = true;
  local_timer_tick = 0;
  }
else
  {
  local_timer_tick++;
  update_display = false;  
  }
if ((MenuState == Menu_Off) & update_display)
  {
//  digitalWrite(TP1, HIGH);
  if (LOCAL_led) // are we displaying local time?
    {
   lc.setChar(0,6,8,false); // Set LOCAL led
   lc.setRow(0,7,B00000000); // Clear UTC LED
    }
  else
    {
    lc.setChar(0,6,16,false); // Clear LOCAL led
    lc.setRow(0,7,B11111111); // Set UTC LED
    }
    
  lc.setChar(0,5,segbuf[5],false); // display tens hours

  if (am_pm and LOCAL_led) // if am_pm = true then PM and light decimal point
    {
  lc.setChar(0,4,segbuf[4],true); // display ones hours
    }
  else
    {
  lc.setChar(0,4,segbuf[4],false); // display ones hours with decimal point for PM
    }

  lc.setChar(0,3,segbuf[3],false); // display tens minutes
  lc.setChar(0,2,segbuf[2],false); // display ones minutes
  lc.setChar(0,1,segbuf[1],false); // display tens seconds
  lc.setChar(0,0,segbuf[0],false); // display ones seconds
//  digitalWrite(TP1, LOW);
  }

}  

void setup() {
// set up the input pins
  pinMode(Menu, INPUT);
  pinMode(Set, INPUT);
  pinMode(PlusSw, INPUT);
  pinMode(Buzzer, OUTPUT);
  pinMode(TP1, OUTPUT);

// set up the input pins for a change on input interrupt
  pciSetup(Menu);
  pciSetup(Set);
  pciSetup(PlusSw);

  /*
   The MAX72XX is in power-saving mode on startup,
   we have to do a wakeup call
   */
  lc.shutdown(0,false);
  /* Set the brightness to a medium values */
  //lc.setIntensity(0,8);
  lc.setIntensity(0,disp_Bright); // 0 to 15 0 for dim display
  /* and clear the display */
  lc.clearDisplay(0);
// Initialize the segment buffers...

    for (int i=0; i<8; i++)
        segbuf[i] = seg[i] ;         

    Serial.begin(9600) ;
    Timer1.initialize(10000);      // 10 ms
    Timer1.attachInterrupt(LED_irq) ;
    Timer1.start() ;

    // Should auto-run at this point... 

  // setup the clock
// Initialize DS3231
  Serial.println("CLK100V12 12/12/2020");
  Serial.println("Initialize DS3231");
/* example helpful debug routines
  Serial.println("MenuState is");
  Serial.println(MenuState, DEC);
*/
  clock.begin();

  // Set sketch compiling time
  //clock.setDateTime(__DATE__, __TIME__);
  //setDateTime(1954, 12, 23, 12, 10, 45);
  // Manual (YYYY, MM, DD, HH, II, SS
  //clock.setDateTime(2020, 12, 6, 7, 51, 10);
} // End of setup routine

/*
 This method will display the characters for the
 word "Arduino" one after the other on digit 0. 
 */
void writeArduinoOn7Segment() {
  lc.setChar(0,5,'1',false);
  lc.setChar(0,4,'2',false);
  lc.setChar(0,3,'3',false);
  lc.setChar(0,2,'4',false);
  lc.setChar(0,1,'5',false);
  lc.setChar(0,0,'6',false);
//  lc.setChar(0,1,'7',false);
//  lc.setChar(0,0,'8',false);
  delay(delaytime);
  lc.setRow(0,0,0x05);
  delay(delaytime);
  lc.setChar(0,0,'d',false);
  delay(delaytime);
  lc.setRow(0,0,0x1c);
  delay(delaytime);
  lc.setRow(0,0,B00010000);
  delay(delaytime);
  lc.setRow(0,0,0x15);
  delay(delaytime);
  lc.setRow(0,0,0x1D);
  delay(delaytime);
  lc.clearDisplay(0);
  delay(delaytime);
} 

/*
  This method will scroll all the hexa-decimal
 numbers and letters on the display. You will need at least
 four 7-Segment digits. otherwise it won't really look that good.

 //void LedControl::setDigit(int addr, int digit, byte value, boolean dp)
 */
void scrollDigits() {
  for(int i=0;i<13;i++) {
    lc.setDigit(0,7,i,false);
    lc.setDigit(0,6,i+1,false);
    lc.setDigit(0,5,i+2,false);
    lc.setDigit(0,4,i+3,false);
    lc.setDigit(0,3,i+4,false);
    lc.setDigit(0,2,i+5,false);
    delay(delaytime);
  }
  lc.clearDisplay(0);
  delay(delaytime);
}

/*
void loop() { 
  writeArduinoOn7Segment();
  scrollDigits();
}
*/

// ************************************* main routine *****************************
void loop()
{
if   (check_alarm())
  {
  MenuState = Alarm_Active;
  alarm_Event = true;
  }
if (MenuState == Menu_Off) // Check if we in the clock display or menu mode?
  {
    update_time(false);
    if (switchPress)
    {
    switchPress = false;  
    if (switchNumber == Menu_Button)
      {
       MenuState = Set_Hours;
       update_time(true);
       fast_update = false;
       menu_routine();
      }
    }
  }
else // we are now in the menu mode
  {
  menu_display();  
  menu_routine();
  }
} // end main loop


/* This routine reads the time from the RTC and updates the segment display buffer */
void update_time(bool menu) // the menu variable tells us if we are in the menu mode or not
{
  dt = clock.getDateTime();

// For leading zero look to DS3231_dateformat example
/* Debug only
  Serial.print(dt.hour);   Serial.print(":");
  Serial.print(dt.minute); Serial.print(":");
  Serial.print(dt.second); Serial.println("");
*/
  // handle the seconds
  if (menu) // are we in the menu mode?
  {
  sec_tens = menu_sec_tens;
  sec_ones = 0;
  }
  else
  {
    sec_tens=0;
    temp_secs = dt.second; 
    while (temp_secs > 9)
    {
      sec_tens++;
      temp_secs = (temp_secs - 10);
    }
    sec_ones = temp_secs;

    sec_count = dt.second;
  
    while (sec_count > 19)
    {
      sec_count = (sec_count - 20);
    }
   } 
 
 // handle the minutes
   min_tens=0;
  temp_min = dt.minute; 
  while (temp_min > 9)
  {
    min_tens++;
    temp_min = (temp_min - 10);
  }
  min_ones = temp_min;

 // handle the local hours
  hr_tens=0;
  temp_hr = dt.hour; // always 0 - 23

// if 12 hour mode adjust the display hours
if (twelve_hr_mode)
  {
    if (temp_hr == 0)
      {
        temp_hr = 12;
        am_pm = true;
      }    
    else if (temp_hr > 12)
      {
        temp_hr = temp_hr - 12;
        am_pm = true;  
      }
    else
      {
        am_pm = false;
      }
    }

  while (temp_hr > 9) // adjust hour tens digit
  {
    hr_tens++;
    temp_hr = (temp_hr - 10);
  }  
  hr_ones = temp_hr;

  utc_hr_tens=0; // handle the utc hours
  temp_utc_hr = dt.hour + utc_offset;
  if (temp_utc_hr > 23)
    {
    temp_utc_hr = temp_utc_hr - 24;
    }
  while (temp_utc_hr > 9) // adjust hour tens digit
    {
    utc_hr_tens++;
    temp_utc_hr = (temp_utc_hr - 10);
    }
  utc_hr_ones = temp_utc_hr;

  // Load the display buffer with the current time //
  segbuf[0] = sec_ones; // dt.second 0 to 59  
  segbuf[1] = sec_tens; // dt.second 0 to 59  
  segbuf[2] = min_ones; // dt.second 0 to 59  
  segbuf[3] = min_tens; // dt.second 0 to 59  
  segbuf[4] = hr_ones; // dt.second 0 to 59  
  // blank hr_tens if zero
  if (hr_tens == 0) // blank the tens of hours if it is zero
   {
    segbuf[5] = 16; // dt.second 0 to 59
    }
  else
    {
  segbuf[5] = hr_tens; // dt.second 0 to 59    
    }

    
//  segbuf[6] = seg[utc_hr_ones]; // dt.second 0 to 59  
//  segbuf[7] = seg[utc_hr_tens]; // dt.second 0 to 59

// turn on LOCAL led
//           void setLed(int addr, int row, int col, boolean state);
lc.setLed(0, 6, 0, 1);

if (sec_count & B00000010 )
  {
  LOCAL_led = false;
  UTC_led = true;
  segbuf[4] = utc_hr_ones;
  // blank hr_tens if zero
  if (utc_hr_tens == 0) // blank the tens of hours if it is zero
   {
    segbuf[5] = 16;
    }
  else
    {
  segbuf[5] = utc_hr_tens;
    }
  }
else
  {
  LOCAL_led = true;
  UTC_led = false;
  segbuf[4] = hr_ones; // dt.second 0 to 59  
  // blank hr_tens if zero
  if (hr_tens == 0) // blank the tens of hours if it is zero
    {
    segbuf[5] = 16; // dt.second 0 to 59
    }
  else
    {
    segbuf[5] = hr_tens; // dt.second 0 to 59    
    }
  }
  if (!fast_update)
    {
      delay(1000);
    }
}

/****************************** MAIN MENU ROUTINE ****************************/
void menu_routine(void)
{
 switch (MenuState)
  {
  case Menu_Off:
    {
//    Serial.println("Menu Routine Menu_Off");    
    }
    break;
    
  case Set_Hours:
  {
    if (switchPress)
    {
    switchPress = false;  
    if (switchNumber == Menu_Button)
      {
      MenuState = Set_Minutes;
      update_time(true);
      refresh = false;
      fast_update = false;
      }
    else if (switchNumber == Inc_Button)
      {
      MenuState = Set_Hours;
      // read the RTC hour
      temp_v = dt.hour;
      temp_v ++;
      temp_v = check_rollover_hr(temp_v); // check if > 23 then 0 
      dt.hour = temp_v;
//void conv_to_two_digit(int value, int segbuftens, int segbufones, bool blanktens)
/********************TEST CODE*************************************/
// convert hour setting to tens and ones
conv_to_two_digit(dt.hour, 5, 4, true);
/*********************************************************/
      clock.setDateTime(dt.year, dt.month, dt.day, dt.hour, dt.minute, dt.second);
      fast_update = true;
      time_menu = true;
      update_time(true);
// update the hour segment buffers
      }
      else if (switchNumber == Dec_Button)
      {
      MenuState = Set_Hours;
      // add decrement hours
      // read time
      temp_v = dt.hour;
      temp_v --;
      temp_v = check_rollunder_hr(temp_v); 
      dt.hour = temp_v;
      //clock.setDateTime(2018, 12, 8, 9, 04, 00);
      /********************TEST CODE*************************************/
// convert hour setting to tens and ones
conv_to_two_digit(dt.hour, 5, 4, true);
/*********************************************************/
      clock.setDateTime(dt.year, dt.month, dt.day, dt.hour, dt.minute, dt.second);
      fast_update = true;
      time_menu = true;
      update_time(true);
      }  
    }  
  }
  break;
    
    case Set_Minutes:
    {
    if (switchPress)
    {
    switchPress = false;  
    if (switchNumber == Menu_Button)
      {
        MenuState = Set_Seconds;
        update_time(true);
        refresh = false;
        fast_update = false;
      }
    else if (switchNumber == Inc_Button)
      {
      MenuState = Set_Minutes;
      // add increment minutes
      // read time
      temp_v = dt.minute;
      temp_v ++;
      temp_v = check_rollover_min(temp_v); 
      dt.minute = temp_v;
      // convert hour setting to tens and ones
      conv_to_two_digit_min(dt.minute, 3, 2);
 //     conv_to_two_digit(dt.minute, 3, 2, false);
      clock.setDateTime(dt.year, dt.month, dt.day, dt.hour, dt.minute, dt.second);
      time_menu = true;
      update_time(true);
//      refresh = true;
       fast_update = true;
      }  
    else if (switchNumber == Dec_Button)
      {
      MenuState = Set_Minutes;
      // add decrement minutes
      // read time
      temp_v = dt.minute;
      temp_v --;
      temp_v = check_rollunder_min(temp_v); 
      dt.minute = temp_v;
      conv_to_two_digit_min(dt.minute, 3, 2);
      clock.setDateTime(dt.year, dt.month, dt.day, dt.hour, dt.minute, dt.second);
      time_menu = true;
      update_time(true);
//      refresh = true;
       fast_update = true;
      }  
    }  
   }
    break;
    
    case Set_Seconds:
    {
    if (switchPress)
    {
    switchPress = false;  
    if (switchNumber == Menu_Button)
      {
        if (time_menu == true)
        {
        clock.setDateTime(dt.year, dt.month, dt.day, dt.hour, dt.minute, ((menu_sec_tens *10) + 1));
        MenuState = Menu_Off;
        time_menu = false;
        update_time(true);
        } 
        else
        MenuState = Set_UTCOnOff;
      }
    else if (switchNumber == Inc_Button)
      {
      MenuState = Set_Seconds;
      
      // add increment minutes
      menu_sec_tens ++;
      if (menu_sec_tens > 5)
      menu_sec_tens = 0;
      time_menu = true;
      update_time(true);
      fast_update = true;
      }  
    else if (switchNumber == Dec_Button)
      {
      MenuState = Set_Seconds;
      // add decrement minutes
      // read time
      menu_sec_tens --;
      if (menu_sec_tens == -1)
      menu_sec_tens = 5;
      time_menu = true;
      update_time(true);
      fast_update = true;

      }  
    }      
  }
  break;
    
  case Set_UTCOnOff:
    {
      if (switchPress)
      {
      switchPress = false;  
      if (switchNumber == Menu_Button)
        {
          MenuState = Set_UTC_Hours;
        }
      else if (switchNumber == Inc_Button)
        {
        if (UTC_On) UTC_On = false;
        else UTC_On = true;
        }
      else if (switchNumber == Dec_Button)
        {
        if (UTC_On) UTC_On = false;
        else UTC_On = true;
        }
      }        
    }
    break;
    
  case Set_UTC_Hours:
    {
      if (switchPress)
      {
      switchPress = false;  
      if (switchNumber == Menu_Button)
        {
          MenuState = Set_Alarm;
        }
      else if (switchNumber == Inc_Button)
        {
        MenuState = Set_UTC_Hours;
        refresh = true;
        temp_v = utc_offset;
        temp_v ++;
        // check for = 24, should be between 0 and 23 hours
          if (temp_v >= 24)
            {
            temp_v = 0;  
            }
         utc_offset = temp_v;  
         }
        else if (switchNumber == Dec_Button)
         {
          MenuState = Set_UTC_Hours;
          temp_v = utc_offset;
          temp_v --;
        // check for = -1, should be between 0 and 23 hours
          if (temp_v <= -1)
            {
            temp_v = 23;  
            }
          utc_offset = temp_v;     
          }  
        } 
    }
    break;

  case Set_Alarm:
    {
    if (switchPress)
      {
      switchPress = false;  
      if (switchNumber == Menu_Button)
        {
          MenuState = Toggle_Alarm;
        }
      else if (switchNumber == Inc_Button)
        {
        MenuState = Set_Alarm_Hours;
        }
      else if (switchNumber == Dec_Button)
        {
        MenuState = Set_Alarm_Hours;
        }
      }  
    }
    break;

    case Set_Alarm_Hours:
    {
    if (switchPress)
    {
    switchPress = false;  
    if (switchNumber == Menu_Button)
      {
        MenuState = Set_Alarm_Minutes;
      }
    else if (switchNumber == Inc_Button)
      {
      MenuState = Set_Alarm_Hours;
      fast_update = true;
//      refresh = true;
      temp_v = alarm_hr;
      temp_v ++;
      // check for = 24, should be between 0 and 23 hours
      if (temp_v > 24)
         {
         temp_v = 1;  
         }
      alarm_hr = temp_v;  
      }  
    else if (switchNumber == Dec_Button)
      {
      MenuState = Set_Alarm_Hours;
      fast_update = true;
      temp_v = alarm_hr; // 24 hour number, range 0 - 23
      temp_v --;
      // check for < 0 , should be between 0 and 23 hours
      if (temp_v < 1)
         {
         temp_v = 24;  
         }
      alarm_hr = temp_v;  
      }
      // check if 12 or 24 hour mode
/*   CHECK THIS ROUTINE  TO CONVERT 24 HR FORMAT TO 12 HOUR FORMAT  */
      temp_v =  alarm_hr;
        if (temp_v > 12 and twelve_hr_mode)
        {
          if (temp_v > 12)
          {
          temp_v = (temp_v - 12);
          }
          alarm_am_pm = true;  // use to turn on the display decimal point in the display routine if true
          temp_hr = temp_v;
        }
        else
        {
        temp_hr = temp_v; // temp_hr is in 24 hour format
        alarm_am_pm = false;
        }
/*   END OF ROUTINE  TO CONVERT 24 HR FORMAT TO 12 HOUR FORMAT  */
      // convert alarm_hr to alarm_hr_tens and alarm_hr_ones
      alarm_hr_tens=0;
      while (temp_hr > 9)
        {
        alarm_hr_tens++;
        temp_hr = (temp_hr - 10);
        }
      alarm_hr_ones = temp_hr;       
    }  
   }
    break;

    case Set_Alarm_Minutes:
    {
    if (switchPress)
    {
    switchPress = false;  
    if (switchNumber == Menu_Button)
      {
        MenuState = Toggle_Alarm;
      }
    else if (switchNumber == Inc_Button)
      {
      MenuState = Set_Alarm_Minutes;
      fast_update = true;
      temp_v = alarm_min;
      temp_v ++;
      alarm_min = check_rollover_min(temp_v);
      conv_alarm_min(alarm_min); // convert alarm_min to alarm_min_tens and alarm_min_ones
      }  
    else if (switchNumber == Dec_Button)
      {
      MenuState = Set_Alarm_Minutes;
      fast_update = true;
      temp_v = alarm_min;
      temp_v --;
      alarm_min = check_rollunder_min(temp_v);
      conv_alarm_min(alarm_min); // convert alarm_min to alarm_min_tens and alarm_min_ones
      }
    }  
   }
    break;

  case Toggle_Alarm:
    {
    noTone(Buzzer);
    if (switchPress)
      {
      switchPress = false;  
      if (switchNumber == Menu_Button)
        {
          MenuState = Set_Local12_24;
        }
      else if (switchNumber == Inc_Button)
        {
        if (alarm_On) alarm_On = false;
        else alarm_On = true;
        }
      }  
    }
    break;

  case Set_Local12_24:
    {
    if (switchPress)
      {
      switchPress = false;  
      if (switchNumber == Menu_Button)
        {
          MenuState = Set_Disp_Bright;
        }
      else if (switchNumber == Inc_Button)
        {
        if (twelve_hr_mode) twelve_hr_mode = false;
        else twelve_hr_mode = true;
        }
      }  
    }
    break;
    
  case Set_Disp_Bright:
    {
    if (switchPress)
      {
      switchPress = false;  
      if (switchNumber == Menu_Button)
        {
          MenuState = Menu_Off;
        }
      else if (switchNumber == Inc_Button)
        {
        MenuState = Disp_Brightness;
          if (disp_Bright == 15 ) // check if maximum brightness = 15 
            disp_Bright = 0;
          else
          {
           disp_Bright ++;
          }
          lc.setIntensity(0,disp_Bright); // 0 to 15 0 for dim display  

        }
      else if (switchNumber == Dec_Button)
        {
        MenuState = Disp_Brightness;
          if (disp_Bright == 0 ) // check if minimum brightness = 0 
            disp_Bright = 15;
          else
          {
           disp_Bright --;
          }
          lc.setIntensity(0,disp_Bright); // 0 to 15 0 for dim display
        }
      }  
    }
    break;

    case Disp_Brightness:
    {
    if (switchPress)
      {
      switchPress = false;  
      if (switchNumber == Menu_Button)
        {
          MenuState = Menu_Off;
        }
      else if (switchNumber == Inc_Button)
        {
          MenuState = Disp_Brightness;
          if (disp_Bright == 15 ) // check if maximum brightness = 15 
            disp_Bright = 0;
          else
          {
           disp_Bright ++;
          }
          lc.setIntensity(0,disp_Bright); // 0 to 15 0 for dim display  
        }
      else if (switchNumber == Dec_Button)
        {
          MenuState = Disp_Brightness;
          if (disp_Bright == 0 ) // check if minimum brightness = 0 
            disp_Bright = 15;
          else
          {
           disp_Bright --;
          }
          lc.setIntensity(0,disp_Bright); // 0 to 15 0 for dim display
  
        }
      }  
    }
    break;

    case Alarm_Active:
    {
      turn_alarm_on(2);
      update_time(false);
      if (switchPress)
      {
      switchPress = false;
      alarm_Event = false;
      MenuState = Menu_Off;
      }  
    }
    break;

    default:
    break;
  }      
} 

void menu_display(void)
{
//  Serial.println("Menu State is");
//  Serial.println(MenuState, DEC);
  switch (MenuState)
  {
  case Set_Time:
    {

    lc.clearDisplay(0);
    delay(500);
    lc.setChar(0,5,segbuf[5],false);
    lc.setChar(0,4,segbuf[4],false);
    lc.setChar(0,3,segbuf[3],false);
    lc.setChar(0,2,segbuf[2],false);
    lc.setChar(0,1,segbuf[1],false);
    lc.setChar(0,0,segbuf[0],false);
    delay(delaytime);

    }
    break;

    case Set_Hours:
    {
      if (!fast_update)
        {
        lc.setRow(0,5,B00000000); // Clear tens hours
        lc.setRow(0,4,B00000000); // Clear ones hours
        delay(500);
        }
      lc.setChar(0,5,segbuf[5],false);
      // check if 12 hour and PM
      set_clear_dp(); // sets or clears the hours decimal point depending on 12 or 24 hour mode
      if (!fast_update)
        {
        lc.setChar(0,3,segbuf[3],false);
        lc.setChar(0,2,segbuf[2],false);  
        lc.setChar(0,1,segbuf[1],false);
        lc.setChar(0,0,segbuf[0],false);
        delay(delaytime);
        fast_update = false; 
      }
//  digitalWrite(TP1, LOW);
    }
    break;

   case Set_Minutes:
   {
    if (!fast_update)
    {
    lc.setRow(0,3,B00000000); // Clear tens minutes
    lc.setRow(0,2,B00000000); // Clear ones minutes
    delay(500);
    }
    lc.setChar(0,3,segbuf[3],false);
    lc.setChar(0,2,segbuf[2],false);  
    
    if (!fast_update)
    {
    lc.setChar(0,5,segbuf[5],false);
    set_clear_dp(); // sets or clears the hours decimal point depending on 12 or 24 hour mode
    lc.setChar(0,1,segbuf[1],false);
    lc.setChar(0,0,segbuf[0],false);
    delay(delaytime);
    fast_update = false; 
    }
  }
    break;
    
   case Set_Seconds:
    {
    if (!fast_update)
      {
      lc.setRow(0,1,B00000000); // Clear tens seconds
      lc.setRow(0,0,B00000000); // Clear ones seconds
      lc.setChar(0,5,segbuf[5],false);
      set_clear_dp(); // sets or clears the hours decimal point depending on 12 or 24 hour mode
      lc.setChar(0,3,segbuf[3],false);
      lc.setChar(0,2,segbuf[2],false);  
      delay(delaytime);
      fast_update = false;
      }
    lc.setChar(0,1,segbuf[1],false);
    lc.setChar(0,0,segbuf[0],false);
    delay(delaytime);
    }
    break;
      
  case Set_UTCOnOff:
    {
    lc.clearDisplay(0);
    delay(500);
    if (UTC_On)
      {
      lc.setRow(0,7,B00100000); // UTC LED
      lc.setRow(0,5,ltr_u); // u   
      lc.setRow(0,4,ltr_t); // t 
      lc.setRow(0,3,ltr_c); // c 

      lc.setRow(0,1,ltr_o); // o 
      lc.setRow(0,0,ltr_n); // n
      }
     else
      {  
      lc.setRow(0,5,ltr_u); // u 
      lc.setRow(0,4,ltr_t); // t 
      lc.setRow(0,3,ltr_c); // c 
      lc.setRow(0,2,ltr_O); // O 
      lc.setRow(0,1,ltr_F); // F 
      lc.setRow(0,0,ltr_F); // F
      }
    delay(delaytime);  
    }
    break;

    case Set_UTC_Hours:
    {
      lc.clearDisplay(0);
      delay(500);  
      lc.setRow(0,5,ltr_u); // u   
      lc.setRow(0,4,ltr_t); // t 
      lc.setRow(0,3,ltr_c); // c 
    // Display the current UTC offset   
    // change offset from hex to decimal
      if (utc_offset < 10)
      {  
      lc.setChar(0,0,utc_offset,false);
      }
      else if (utc_offset < 20)
      {
      lc.setChar(0,1,1,false);
      lc.setChar(0,0,(utc_offset-10),false); 
      }
      else if (utc_offset < 24)
      {
      lc.setChar(0,1,2,false);
      lc.setChar(0,0,(utc_offset-20),false); 
      }
    delay(delaytime);
    }
    break;

    case Set_Alarm_Hours:
    {
    // Display the current alarm hour and minutes 
      lc.setChar(0,5,alarm_hr_tens,false);
      if (twelve_hr_mode and alarm_am_pm) 
      {
      lc.setChar(0,4,alarm_hr_ones,true);
      }
      else
      {
      lc.setChar(0,4,alarm_hr_ones,false);
      }
      if (!fast_update)
        {
        delay(delaytime);  
        }
      if (!fast_update)
        {
        clear_display();
        fast_update = false; 
        }
    }
    break;

    case Set_Alarm_Minutes:
    {
    // Display the current alarm hour and minutes 
    lc.setChar(0,3,alarm_min_tens,false);
    lc.setChar(0,2,alarm_min_ones,false);
//    lc.setRow(0,3,B00000000); // Clear tens minutes
//    lc.setRow(0,2,B00000000); // Clear ones minutes
//    lc.setRow(0,1,B00000000); // Clear seconds tens
//    lc.setRow(0,0,B00000000); // Clear seconds ones
      if (!fast_update)
        {
        delay(delaytime);  
        }
      if (!fast_update)
        {
        clear_display();
        fast_update = false; 
        }
    }
    break;

    case Set_Alarm:
    {
    lc.clearDisplay(0);
    delay(500);  
      lc.setRow(0,5,ltr_S); // S 
      lc.setRow(0,4,ltr_E); // E 
      lc.setRow(0,3,ltr_t); // t 

      lc.setRow(0,1,ltr_A); // A 
      lc.setRow(0,0,ltr_L); // L
    delay(delaytime);
    }
    break;

   case Toggle_Alarm:
    {
    lc.clearDisplay(0);
    delay(500);  
    if (alarm_On)
      {
      lc.setRow(0,5,ltr_A); // A 
      lc.setRow(0,4,ltr_L); // L 

      lc.setRow(0,1,ltr_o); // o 
      lc.setRow(0,0,ltr_n); // n
      }
     else
      {  
      lc.setRow(0,5,ltr_A); // A 
      lc.setRow(0,4,ltr_L); // L 

      lc.setRow(0,2,ltr_O); // O 
      lc.setRow(0,1,ltr_F); // F 
      lc.setRow(0,0,ltr_F); // F
      }
    delay(delaytime);
    }
    break;

    case Set_Local12_24:
    {
    lc.clearDisplay(0);
    delay(500);  
    if (twelve_hr_mode)
      {
      lc.setDigit(0,5,1,false); // 1  
      lc.setDigit(0,4,2,false); // 2 
      lc.setRow(0,3,ltr_H); // H 

      lc.setRow(0,1,ltr_o); // o 
      lc.setRow(0,0,ltr_n); // n
      }
     else
      {  
      lc.setDigit(0,5,1,false); // 1  
      lc.setDigit(0,4,2,false); // 2 
      lc.setRow(0,3,ltr_H); // H 
      lc.setRow(0,2,ltr_O); // O 
      lc.setRow(0,1,ltr_F); // F 
      lc.setRow(0,0,ltr_F); // F
      }
    delay(delaytime);
    }
    break;

    case Set_Disp_Bright:
    {
    lc.clearDisplay(0);
    delay(500);  
      lc.setRow(0,5,ltr_d); // d 
      lc.setRow(0,4,ltr_i); // i 
      lc.setRow(0,3,ltr_S); // S 
      lc.setRow(0,2,ltr_P); // P 

    // Display the current display brightness level   
    // change disp_Bright from hex to decimal
    if (disp_Bright < 10)
      {  
      lc.setChar(0,0,disp_Bright,false);
      }
      else
      {
      lc.setChar(0,1,1,false);
      lc.setChar(0,0,(disp_Bright-10),false); 
      }
    delay(delaytime);
    }
    break;

    case Disp_Brightness:
    {
    lc.clearDisplay(0);
    delay(500);
      lc.setRow(0,5,ltr_d); // d 
      lc.setRow(0,4,ltr_i); // i 
      lc.setRow(0,3,ltr_S); // S 
      lc.setRow(0,2,ltr_P); // P 
    // change disp_Bright from hex to decimal
    if (disp_Bright < 10)
      {  
      lc.setChar(0,0,disp_Bright,false);
      }
      else
      {
      lc.setChar(0,1,1,false);
      lc.setChar(0,0,(disp_Bright-10),false); 
      }    
    delay(delaytime);
    }
    break;

 case Alarm_Active:
    {

    lc.clearDisplay(0);
    delay(delaytime);
    lc.setChar(0,5,segbuf[5],false);
    lc.setChar(0,4,segbuf[4],false);
    lc.setChar(0,3,segbuf[3],false);
    lc.setChar(0,2,segbuf[2],false);
    lc.setChar(0,1,segbuf[1],false);
    lc.setChar(0,0,segbuf[0],false);
    }
    break;
    
    default:
    {
    lc.clearDisplay(0);
    }
    break;
  }  
}
