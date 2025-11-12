#include <Arduino.h>
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
volatile bool matChoose = false;
volatile bool MInt;
volatile bool matIrq = false;

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
const byte matInt {38};


//set up RGB LED
const byte LED_R {3};
const byte LED_G {4};
const byte LED_B {5};

//set up LED
const byte LED_l {6};

//set up buzzer
const byte BuzzerPin {13};

//set up IR Obstacle Track Sensor
const byte IRO {12};

//set up ultrasonic sensor
const byte ultTrig {8};
const byte ultEcho {9};

//set up microphone
const byte micInt {2};

//set up Temp & Humid

const byte TandH {7};
DHT11 dht(TandH);




//written classes:
class ModeControl {
  public:
    ModeControl(int LedPin, int RedPin, int GreenPin, int BluePin, LiquidCrystal_I2C &lcdRef) : 
    CurrentMode_(Waiting), LedPin_(LedPin), RedPin_(RedPin), GreenPin_(GreenPin), BluePin_(BluePin), lcd_ref(&lcdRef) {
      pinMode(LedPin_, OUTPUT);
      pinMode(RedPin_, OUTPUT);
      pinMode(GreenPin_, OUTPUT);
      pinMode(BluePin_, OUTPUT);
      LedbyMode();
    };
    void setMode(Mode NewMode) {
      if (NewMode != CurrentMode_) {
        CurrentMode_ = NewMode;
      LedbyMode();
      LcdbyMode();
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
            break;
          case Using:
            RGBOff();
            RedOn();
            LedOn();
            break;
          case Cleaning:
            RGBOff();
            YellowOn();
            LedOn();
            break;

        }
      }

      void LcdbyMode() {
        lcd_ref->clear();
        lcd_ref->setCursor(0,0);
        switch (CurrentMode_) {
          case 0:
            lcd_ref->clear();
            lcd_ref->print("Waiting...");
            Serial.println("w");
            break;
          case 1:
            lcd_ref->clear();
            lcd_ref->print("Mode: Using");
            Serial.println("u");
            break;
          case 2:
            lcd_ref->clear();
            lcd_ref->print("Mode: Cleaning");
            Serial.println("c");
            break;
        }  
      }
};

ModeControl mc(LED_l, LED_R, LED_G, LED_B, lcd);
 

class TimeGod {
  private:
    uint32_t startTime = 0;
    uint32_t showerTime = 15UL*60000;

  public:
    //constructor
    TimeGod() {};
    
    //set the start time
    void start() {if (startTime == 0) {startTime = millis();}}
    
    //stop
    void stop() {startTime = 0;}

    //return the time passed after the start
    uint32_t timePassed() {return startTime ? millis() - startTime : 0;}

    //return true when more than 30 second has passed
    bool isMoreThan30() {return timePassed() >= 30UL*1000;}

    //return true when it is longer than the time set
    bool isMoreThanST() {return timePassed() >= showerTime;}
    
    //set the max shower time
    void setST(uint32_t ST) {showerTime = ST;}
};

TimeGod TT;

class keypadChoose {
  private:
  char m[4] = {'0','0','0','0'};
  int i = 0;
  bool Skip = false;
  bool Setting = false;

  public:
  keypadChoose() {};
  bool getSkip() const{return Skip;};
  void clearSkip(){ Skip = false; }
  bool isSetting() const{return Setting;};

  void matIn() {
      noTone(BuzzerPin);
      char k = customKeypad.getKey();
      if (k) {
      if ( k == 'c' ) {
        mc.setMode(Cleaning);
        return;
      }
      else if ( k == 'w' ){ 
        mc.setMode(Waiting);
        Skip = true;  return;
      }
      else if ( k == 'u' ){
        mc.setMode(Using);
        Skip = true;  return;
      }
      else {
        Setting = true;
        LCD();
        Setting = false;

      }
      MInt  = false;
      }
      matChoose = false;
      
      //mc.setMode(Waiting);
      detachInterrupt(digitalPinToInterrupt(micInt));
  }

  void LCD() {
    int i = 0;
    while (i < 4) {
      Serial.print("i = ");  Serial.println(i);  
      lcd.clear();
      lcd.setCursor(0,0); lcd.print("set minutes: ");
      lcd.setCursor(11,1); lcd.print(m[0]); lcd.print(m[1]); lcd.print(':'); lcd.print(m[2]); lcd.print(m[3]);
      lcd.setCursor(i <= 1? i+10:i+11,1);
      //else {lcd.setCursor((2+i),1);}
      lcd.print('_');
      char k = customKeypad.waitForKey();
      //char k = customKeypad.getKey();
      Serial.print("k = "); Serial.println(k);
      if (k == 'w' || k == 'u' || k == 'c' || k == '-') {lcd.clear();break; }
      if (!k) continue;
      if (i%2 == 0) {
      if (k >= '0' && k <= '5' ) {
        m[i++] = k;
      }
      //else continue;
      } else {
        if (k >= '0' && k <= '9' ) {
        m[i++] = k;
      }
      //else continue;
      }
      /*if (k == '-') {i += 4;} 
      else if ( k == 'c' ) {mc.setMode(Cleaning);  return;}
      else if ( k == 'w' ) {mc.setMode(Waiting); Skip = true;  return;}
      else if ( k == 'u' ){mc.setMode(Using); Skip = true;  return;}
      else continue;*/
  }
  uint32_t sT = (m[0]-'0')*10+(m[1]-'0')*60000UL + (m[2]-'0')*10+(m[3]-'0')*1000UL;
  if ( sT > 30000UL ) {TT.stop(); TT.setST(sT);}
      //else { i = 0; };
  i = 0;
  Skip = true;
  //return;
  }
};

keypadChoose Key;

//written functions
void MicISR() {MInt = true; Serial.println(MInt);}

void MatISR() {
  
  matChoose = true;
  matIrq = true; 
  
  //digitalWrite(colPins[3], LOW); 
  //Serial.println("kk");
  //mc.setMode(Waiting); 
  //Serial.println("ควย");
  //detachInterrupt(digitalPinToInterrupt(colPins[3]));
  }

void Buzzer(int BuzzerPin) {
  mc.RedOn();
  tone(BuzzerPin,1000);
  delay(500);
  mc.RGBOff();
  noTone(BuzzerPin);
  delay(500);
}

int DisUlt() {
  digitalWrite(ultTrig, LOW);  
	delayMicroseconds(2);  
	digitalWrite(ultTrig, HIGH);  
	delayMicroseconds(10);  
	digitalWrite(ultEcho, LOW);

  unsigned long dur = pulseIn(ultEcho, HIGH, 30000UL);
  if (dur == 0) return 9999;
  return dur*.0343/2;
}

bool isOpen() {
  return (!digitalRead(IRO) && DisUlt() <= 20); 
}

void PrintTimePassed() {
  int minute = floor(TT.timePassed()/60000);
  int second = floor(TT.timePassed()/1000%60);
  lcd.setCursor(2,1);
  if (minute < 10) {lcd.print('0');}
  lcd.print(minute);
  lcd.print(':');
  if (second < 10) {lcd.print('0');}
  lcd.print(second);
}

void readTemphumid(int TempHumidPin, LiquidCrystal_I2C &lcd) {
  int T = dht.readTemperature();
  int H = dht.readHumidity();
  
  lcd.setCursor(0,0);
  lcd.print("T:");lcd.print(T);lcd.print(" H:");lcd.print(H); lcd.print("  ");
}


void setup() 
{
  //noInterrupts();
  Serial.begin(9600);
  //LCD
  lcd.init();
  lcd.backlight();
  //I/O
  pinMode(IRO, INPUT);
  pinMode(ultTrig, OUTPUT);
  pinMode(ultEcho, INPUT);
  pinMode(BuzzerPin, OUTPUT);

  mc.setMode(mc.getMode());

  //interrupts();
  /*for( int i=0; i< ROWS; ++i) pinMode(rowPins[i], OUTPUT);
  for( int i=0; i< COLS; ++i) pinMode(colPins[i], INPUT_PULLUP);
  for( int i=0; i< ROWS; ++i) digitalWrite(rowPins[i], LOW);
  *///interrupt
  attachInterrupt(digitalPinToInterrupt(colPins[3]),MatISR,FALLING);
}

void loop() {
  
  Serial.print("Choose ="); Serial.println(matChoose);
  Serial.print("Irq ="); Serial.println(matIrq);
  if (matChoose && matIrq) {
    
    matIrq = false;
    detachInterrupt(digitalPinToInterrupt(colPins[3]));
    Key.matIn();
    
    Key.clearSkip(); 
    //Serial.print(matIrq);
    delay(10);
    attachInterrupt(digitalPinToInterrupt(colPins[3]),MatISR,FALLING);
    //return;
    }
  if (mc.getMode() == Cleaning) {
    Key.matIn();
    attachInterrupt(digitalPinToInterrupt(colPins[3]),MatISR,FALLING);}
  else {
    if (mc.getMode() == Waiting) {
      if (isOpen()) {mc.setMode(Using);}
      //Serial.println("B");
      delay(1000);
      lcd.clear();
    }
    else {
      Serial.println(!(Key.getSkip()));
      Serial.println("!Set = ");
      Serial.println(!(Key.isSetting()));
      Serial.println(MInt);
      attachInterrupt(digitalPinToInterrupt(colPins[3]),MatISR,FALLING);
      if (!(Key.getSkip()) && !(Key.isSetting())) {
      attachInterrupt(digitalPinToInterrupt(micInt),MicISR,RISING);
      //if (mc.getMode() == Cleaning) {delay(10000); return;}
      Serial.println(isOpen());
      TT.start();
      if (MInt == true) { Serial.println("ควย"); Buzzer(BuzzerPin); return;}
      readTemphumid(TandH, lcd);
      PrintTimePassed();
      if (TT.isMoreThan30() && isOpen()) {
        mc.setMode(Waiting); 
        //Serial.println("D");
        detachInterrupt(digitalPinToInterrupt(micInt));
        TT.stop();
        delay(20000);
        return;
      }
      else if (TT.isMoreThanST()) {MInt = true; return;}
  }
  }
}
}