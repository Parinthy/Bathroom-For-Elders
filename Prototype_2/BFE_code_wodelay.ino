#include <Keypad.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <DHT11.h>

enum Mode {
        Waiting = 0,
        Using,
        Cleaning,
    };

//flag integer for interruption
bool matChoose = LOW;
bool MInt = LOW;
bool toUsing = LOW;

//set up LCD Display
LiquidCrystal_I2C lcd(0x27, 16, 2);

//set up keypad
const byte ROWS = 4;
const byte COLS = 4;
char hexaKeys[ROWS][COLS] = {
  {'1','2','3','u'},
  {'4','5','6','w'},
  {'7','8','9','c'},
  {'-','0','-','-'},
};
byte rowPins[ROWS] = {51,50,53,52};
byte colPins[COLS] = {48,49,46,18};
Keypad customKeypad = Keypad(makeKeymap(hexaKeys), rowPins, colPins, ROWS, COLS);

//set up I/O pins
const byte LED_R {3};
const byte LED_G {4};
const byte LED_B {5};
const byte LED_l {6};
const byte BuzzerPin {13};
const byte IRO {12};
const byte ultTrig {8};
const byte ultEcho {9};
const byte micInt {2};
const byte TandH {7};
DHT11 dht(TandH);

void MicISR() {MInt = HIGH; Serial.println("Mic Interrupted");}

//written classes:

class TimeGod {
  private:
    uint32_t startTime = 0;
    uint32_t showerTime = 45000;

  public:
    //constructor
    TimeGod() {};
    
    //set the start time
    void start() {startTime = millis();}

    void stop() {startTime = 0;}

    //return the time passed after the start
    uint32_t timePassed() {return millis() - startTime;}

    //return true when more than 30 second has passed
    bool isMoreThan30() {return timePassed() >= 30000;}

    //return true when it is longer than the time set
    bool isMoreThanST() {
      /*Serial.print("Time passed: ");
      Serial.println(timePassed());
      Serial.print("Shower time: ");
      Serial.println(showerTime);
      Serial.print("isMoreThanST: ");
      Serial.println(isMoreThanST());*/

      return timePassed() >= showerTime;}
    
    //set the max shower time
    void setST(uint32_t ST) {showerTime = ST;}
};

TimeGod TimeGod;

class ModeControl {
  public:
    ModeControl(int LedPin, int RedPin, int GreenPin, int BluePin, LiquidCrystal_I2C &lcdRef) : 
    CurrentMode_(Waiting), LedPin_(LedPin), RedPin_(RedPin), GreenPin_(GreenPin), BluePin_(BluePin), lcd_ref(&lcdRef) {
      pinMode(LedPin_, OUTPUT);
      pinMode(RedPin_, OUTPUT);
      pinMode(GreenPin_, OUTPUT);
      pinMode(BluePin_, OUTPUT);
    };
    void setMode(Mode NewMode) {
      if (NewMode != CurrentMode_) {
        CurrentMode_ = NewMode;
      LedbyMode();
      LcdbyMode();
      attDetByMode();
    }
    }
    Mode getMode() const{return CurrentMode_;};
    bool isMode(Mode m) const{return CurrentMode_ == m;};
   
    //------------------------LED function-------------------------//
    void LedOn() {digitalWrite(LedPin_,HIGH);}
    void LedOff() {digitalWrite(LedPin_,LOW);}
    void RGBOff() {
      analogWrite(RedPin_,0);
      analogWrite(GreenPin_,0);
      analogWrite(BluePin_,0);
    }
    void RedOn() {analogWrite(RedPin_,255);}
    void GreenOn() {analogWrite(GreenPin_,255);}
    void BlueOn() {analogWrite(BluePin_,255);}
    void YellowOn() {RedOn(); GreenOn();}

    private:
      Mode CurrentMode_;
      int LedPin_;
      int RedPin_;
      int GreenPin_;
      int BluePin_;
      LiquidCrystal_I2C* lcd_ref;

      void LedbyMode() {
        switch (CurrentMode_) {
          case Waiting:
            RGBOff();
            GreenOn();
            LedOff();
            TimeGod.stop();
            break;
          case Using:
            RGBOff();
            RedOn();
            LedOn();
            TimeGod.start();
            break;
          case Cleaning:
            RGBOff();
            YellowOn();
            LedOn();
            TimeGod.stop();
            break;

        }
      }

      void LcdbyMode() {
        lcd_ref->clear();
        lcd_ref->setCursor(0,0);
        switch (CurrentMode_) {
          case 0:
            Serial.println("w");
            break;
          case 1:
            lcd_ref->print("Mode: Using");
            Serial.println("u");
            break;
          case 2:
            lcd_ref->print("Mode: Cleaning");
            Serial.println("c");
            break;
        }  
      }

      void attDetByMode() {
        switch (CurrentMode_) {
          case 0:
            detachInterrupt(digitalPinToInterrupt(micInt));
            break;
          case 1:
            attachInterrupt(digitalPinToInterrupt(micInt), MicISR, RISING);
            break;
          case 2:
            attachInterrupt(digitalPinToInterrupt(micInt), MicISR, RISING);
            break;
        }
      }
};

ModeControl mc(LED_l, LED_R, LED_G, LED_B, lcd);



class keypadChoose {
  private:
  char m[4] = {'-','-','-','-'};
  int i = 0;

  public:
  keypadChoose() {};

  bool isTimeInput(char k) {
    if (k == 'c' || k == 'w' || k == 'u') {
      return true;
    }
    else {
      return false;
    }
  }

  void change(char k) {
    if (k == 'c') {
      mc.setMode(Cleaning);
    }
    else if (k == 'w') {
      mc.setMode(Waiting);
    }
    else if (k == 'u') {
      mc.setMode(Using);
    }
  }

  bool isNum(char k) {
    return k>='0' && k<='9';
  }

  bool isOver() {return i == 4;}

  void charAdd(char k) {
      if (!isOver()) {
        m[i] = k;
        i++;
      }
  }

  void reset_i() {
    i = 0;
  }

  void reset_m() {
    for (int i = 0; i<4; i++) {
      m[i] = '-';
    }
  }

  void show(){  
    lcd.setCursor(0, 0);
    lcd.print("Set time:     ");
    lcd.setCursor(2,1);
    lcd.print(m[0]);
    lcd.print(m[1]);
    lcd.print(':');
    lcd.print(m[2]);
    lcd.print(m[3]);
  }

  bool valid() {
    int min = (m[0]-'0')*10 + (m[1]-'0');
    int sec = (m[2]-'0')*10 + (m[3]-'0');
    Serial.println(min);
    Serial.println(sec);
    Serial.println(min < 60 && sec < 60);
    return (min < 60 && sec < 60);

  }
  uint32_t cal() {
    if (valid()) {
    uint32_t mini = ((uint32_t)(m[0]-'0'))*600000 + ((uint32_t)(m[1]-'0'))*60000;
    uint32_t sec = ((uint32_t)(m[2]-'0'))*10000 + ((uint32_t)(m[3]-'0'))*1000;
    Serial.println(mini);
    Serial.println(sec);
    return ((uint32_t)(m[0]-'0')*10 + (m[1]-'0'))*60*1000 + ((uint32_t)(m[2]-'0')*10 + (m[3]-'0'))*1000;
    }
    else return 0;
  }
};

keypadChoose Key;

class Timer {
  private:
  uint32_t begin = 0;

  public:
  void setBegin() {
    if (begin == 0) begin = millis();
  }
  void resetBegin() {
    begin = 0;
  }
  uint32_t getBegin() {
    return begin;
  }
};

Timer Timer1;
Timer Timer2;

//written functions

void Buzzer(int BuzzerPin) {
  Timer1.setBegin();
  mc.RedOn();
  tone(BuzzerPin,1000);
  if (millis()-Timer1.getBegin() >= 500 && millis()-Timer1.getBegin() < 1000) {
    mc.RGBOff();
    noTone(BuzzerPin);
  }
  else if (millis() - Timer1.getBegin() >= 1000) {Timer1.resetBegin();} 
}

int DisUlt() {
  digitalWrite(ultTrig, LOW);  
	delayMicroseconds(2);  
	digitalWrite(ultTrig, HIGH);  
	delayMicroseconds(10);  
	digitalWrite(ultEcho, LOW);
  return pulseIn(ultEcho, HIGH)*.0343/2;
}

bool isOpen() {
  return digitalRead(IRO) && (DisUlt() > 20); 
}

void PrintTimePassed() {
  int minute = floor(TimeGod.timePassed()/60000);
  int second = floor(TimeGod.timePassed()/1000%60);
  lcd.setCursor(2,1);
  if (minute < 10) {lcd.print('0');}
  lcd.print(minute);
  lcd.print(':');
  if (second < 10) {lcd.print('0');}
  lcd.print(second);
}

void readTemphumid(LiquidCrystal_I2C &lcd) {
  Timer2.setBegin();
  if (millis() - Timer2.getBegin() < 500) {return;}
  int T = dht.readTemperature();
  int H = dht.readHumidity();
  lcd.setCursor(0,0);
  lcd.print("T:");
  lcd.print(T);
  lcd.print(" ");
  lcd.print("H:");
  lcd.print(H);
  lcd.print(" ");
  Timer2.resetBegin();
}

//start in flowchart
void setup() 
{
  Serial.begin(9600);
  //LCD
  lcd.begin();
  lcd.backlight();
  //I/O
  pinMode(IRO, INPUT);
  pinMode(ultTrig, OUTPUT);
  pinMode(ultEcho, INPUT);
  pinMode(BuzzerPin, OUTPUT);

  matChoose = LOW;
  MInt = LOW;
}

void loop()
{
  if (matChoose == HIGH) {
    Key.show();
    char l = customKeypad.getKey();
    if (l) {
      Serial.println(l);
      if (!Key.isOver() && Key.isNum(l)) {
        Key.charAdd(l);
        Key.show();
      }
      else if (Key.isOver()) {
        uint32_t ST = Key.cal();
        Serial.println(ST);
        if (ST >= 30000) {
          Serial.print("valid"); 
          TimeGod.setST(ST); 
          matChoose = LOW; 
          Key.reset_m(); Key.reset_i();
        }
        else {
          Serial.print("Not valid"); 
          Key.reset_i(); Key.reset_m();
        }
      }
    }
    return;
  }
  char k = customKeypad.getKey();
  if (k) {
    mc.RGBOff();
    noTone(BuzzerPin);
    MInt = LOW;
    Serial.println(k);
    if (Key.isTimeInput(k)) {Key.change(k);}
    else {matChoose = HIGH;}
  }
  if (mc.getMode() == Waiting) {
    if (isOpen()) {mc.setMode(Using);}
  }
  else {
    if (mc.getMode() == Cleaning) {return;}
    if (MInt == HIGH) {Buzzer(BuzzerPin); return;}
    readTemphumid(lcd);
    PrintTimePassed();
    if (TimeGod.isMoreThan30() && isOpen()) {
      delay(5000);
      mc.setMode(Waiting); 
      Timer2.resetBegin();
      }
    if (TimeGod.isMoreThanST()) {Serial.println(MInt); MInt = HIGH;}
  }
}