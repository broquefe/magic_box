#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Servo.h>
#include <Keypad.h>
#include <Wire.h>
#include "ds3231.h"
#include <HX711_ADC.h>

#define BUFF_MAX 128
#define LEAP_YEAR(Y)  ( (Y>0) && !(Y%4) && ( (Y%100) || !(Y%400) ))

uint8_t time[8];
char recv[BUFF_MAX];
unsigned int recv_size = 0;
unsigned long prev, interval = 500;

const char* daysOfTheWeek[] = {"sunday", "monday", "tuesday", "wednesday", "thursday", "friday", "saturday"};
char* dayOfWeek;
bool weekend = false;
 
//servos
Servo myservo;
Servo myservo2;

//button
const int buttonPin = 13;
bool isBoxOpened = false;
bool buttonGotPressed = false;
int buttonState = HIGH;
int lastButtonState = HIGH;

unsigned long lastDebounceTime = 0;
// Minimum time between button presses to consider it as a valid press (in milliseconds)
unsigned long debounceDelay = 50;

//lcd screen
LiquidCrystal_I2C lcd(0x27, 16, 2);

//relay
const int relayPin = 3;

//load cell
HX711_ADC LoadCell(6, 7);

void setup(){  
  //Servo part
  Serial.begin(9600);
  myservo.attach(9);
  myservo2.attach(8);

  //Real time sensor part
  Wire.begin();
  DS3231_init(DS3231_CONTROL_INTCN);
  memset(recv, 0, BUFF_MAX);

  //buton
  pinMode(buttonPin, INPUT_PULLUP);

  //lcd screen
  lcd.init();         // initialize the lcd
  lcd.backlight();

  //relay
  pinMode(relayPin, OUTPUT);
  digitalWrite(relayPin, HIGH);

  //load cell
  LoadCell.begin();
  LoadCell.start(2000);
  LoadCell.setCalFactor(1000.0);
  LoadCell.tare();
}

void loop(){

    char in;
    char buff[BUFF_MAX];
    unsigned long now = millis();
    struct ts t;

    LoadCell.update();
    float i = LoadCell.getData();

    // show time once in a while
    if ((now - prev > interval) && (Serial.available() <= 0)) 
    {
        DS3231_get(&t);
        snprintf(buff, BUFF_MAX, "%d.%02d.%02d %02d:%02d:%02d", t.year,t.mon, t.mday, t.hour, t.min, t.sec);
        //Serial.println(buff);
        dayOfWeek = daysOfTheWeek[getDayOfWeek(t.year,t.mon, t.mday)];
        weekend = isWeekend(dayOfWeek);
        
        prev = now;
    }

    buttonGotPressed = isButtonPressed(buttonPin);
    
    if (weekend == true) 
    {
        if (buttonGotPressed && !isBoxOpened)
        {
            isBoxOpened = true;
            //servo
            myservo.write(170);
            myservo2.write(0);
            //lcd screen
            lcd.clear();
            lcd.setCursor(2, 0);
            lcd.print("Box opened!");
            //relay
            Serial.println("yeah");
            digitalWrite(relayPin, LOW);
            delay(2000);
            digitalWrite(relayPin, HIGH);
            lcd.setCursor(0, 0);
            lcd.print("Weight (g)");
            lcd.setCursor(0, 1);
            lcd.print(i);
        }
        else if (buttonGotPressed && isBoxOpened)
        {
            myservo.write(70);//close
            myservo2.write(90);//close
            isBoxOpened = false;
            lcd.clear();
            lcd.setCursor(2, 0);
            lcd.print("Box closed!");
        } 
    }   
    else if (!weekend && buttonGotPressed)
    {
        myservo.write(70);//close
        myservo2.write(90);//close
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("It's not weekend");
        lcd.setCursor(0, 1);
        lcd.print("weedhead bastard");
    }
    else
    {
        myservo.write(70);//close
        myservo2.write(90);//close
    }

    delay(100);
}



int getDayOfWeek(uint16_t year, uint8_t month, uint8_t day)
{
  uint16_t months[] = 
  {
    0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334, 365 // days until 1st of month
  };   

  uint32_t days = year * 365;   // days until year 
  for (uint16_t i = 4; i < year; i+=4) 
  {
    if (LEAP_YEAR(i)) days++;  // adjust leap years
  }

  days += months[month-1] + day;
  if ((month > 2) && LEAP_YEAR(year)) days++;
  return days % 7;
}

bool isButtonPressed(uint16_t pinNumber)
{
  bool buttonPressed=false;
  int reading = digitalRead(pinNumber);
    // Check if the button state has changed
    if (reading != lastButtonState) 
    {
        lastDebounceTime = millis();// Reset the debounce time
    }
    if ((millis() - lastDebounceTime) > debounceDelay) 
    {
    // If the reading has been stable for debounceDelay milliseconds, accept it as valid
        if (reading != buttonState) 
        {
            buttonState = reading;
            // Check if the button is pressed (active-low)
            if (buttonState == LOW) 
            {
                buttonPressed = true;
                lastButtonState = reading;
                Serial.println("Button Pressed");
                return buttonPressed;
            }
        }
    }
    lastButtonState = reading;
    buttonPressed = false;
    return buttonPressed;
}

bool isWeekend(char* dayOfTheWeek)
{
  return ((dayOfTheWeek == "thursday") || (dayOfTheWeek == "friday") || (dayOfTheWeek == "saturday") || (dayOfTheWeek =="sunday")) ? true : false;
}
