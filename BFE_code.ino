#include <Arduino.h>
#include <Keypad.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <DHT11.h>

//enum used in this code
enum Mode {
        Waiting = 0,
        Using,
        Cleaning,
    };

//flag boolean
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


//---set up pin I/O---//
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

//---------------written classes---------------//
class ModeControl {
  public:
    //constructor
    ModeControl(int LedPin, int RedPin, int GreenPin, int BluePin, LiquidCrystal_I2C &lcdRef) : 
    CurrentMode_(Waiting), LedPin_(LedPin), RedPin_(RedPin), GreenPin_(GreenPin), BluePin_(BluePin), lcd_ref(&lcdRef) {
      pinMode(LedPin_, OUTPUT);
      pinMode(RedPin_, OUTPUT);
      pinMode(GreenPin_, OUTPUT);
      pinMode(BluePin_, OUTPUT);
      LedbyMode();
    };
    
    //set currrent mode
    void setMode(Mode NewMode) {
      if (NewMode != CurrentMode_) {
        CurrentMode_ = NewMode;
        LedbyMode();
        LcdbyMode();
      }
    }

    //return current mode
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

//set up the class
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

//set up the class
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

  //check the key receive from keypad
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
      detachInterrupt(digitalPinToInterrupt(micInt));
  }

  //time selector
  void LCD() {
    int i = 0;
    while (i < 4) {
      Serial.print("i = ");  Serial.println(i);  
      lcd.clear();
      lcd.setCursor(0,0); lcd.print("set minutes: ");
      lcd.setCursor(11,1); lcd.print(m[0]); lcd.print(m[1]); lcd.print(':'); lcd.print(m[2]); lcd.print(m[3]);
      lcd.setCursor(i <= 1? i+10:i+11,1);
      lcd.print('_');
      char k = customKeypad.waitForKey();
      Serial.print("k = "); Serial.println(k);
      if (k == 'w' || k == 'u' || k == 'c' || k == '-') {lcd.clear();break; }
      if (!k) continue;
      if (i%2 == 0) {
        if (k >= '0' && k <= '5' ) {
          m[i++] = k;
        }
      } else {
        if (k >= '0' && k <= '9' ) {
          m[i++] = k;
        }
      }
    }
    uint32_t sT = (m[0]-'0')*10+(m[1]-'0')*60000UL + (m[2]-'0')*10+(m[3]-'0')*1000UL;
    if ( sT > 30000UL ) {TT.stop(); TT.setST(sT);}
    i = 0;
    Skip = true;
  }
};

//set up the class
keypadChoose Key;

//----------------written functions----------------//
void MicISR() {MInt = true; Serial.println(MInt);}

void MatISR() {
  matChoose = true;
  matIrq = true; 
  }

//alarm using buzzer and RGB
void Buzzer(int BuzzerPin) {
  mc.RedOn();
  tone(BuzzerPin,1000);
  delay(500);
  mc.RGBOff();
  noTone(BuzzerPin);
  delay(500);
}

//reading distance from ultrasonic sensor
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

//check if the door is opening
bool isOpen() {return (!digitalRead(IRO) && DisUlt() <= 20);}

//show the time passed during the using mode
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

//read and show the data from temp&humid sensor
void readTemphumid(int TempHumidPin, LiquidCrystal_I2C &lcd) {
  int T = dht.readTemperature();
  int H = dht.readHumidity();
  
  lcd.setCursor(0,0);
  lcd.print("T:");lcd.print(T);lcd.print(" H:");lcd.print(H); lcd.print("  ");
}

//---------------------------arduino start-----------------------------//
void setup() 
{
  Serial.begin(9600);
  lcd.init();
  lcd.backlight();
  pinMode(IRO, INPUT);
  pinMode(ultTrig, OUTPUT);
  pinMode(ultEcho, INPUT);
  pinMode(BuzzerPin, OUTPUT);
  mc.setMode(mc.getMode());
  attachInterrupt(digitalPinToInterrupt(colPins[3]),MatISR,FALLING);
}

void loop() {
  Serial.print("Choose ="); Serial.println(matChoose);
  Serial.print("Irq ="); Serial.println(matIrq);

  //after matrix interrupt
  if (matChoose && matIrq) {
    matIrq = false;
    detachInterrupt(digitalPinToInterrupt(colPins[3]));
    Key.matIn();
    Key.clearSkip(); 
    delay(10);
    attachInterrupt(digitalPinToInterrupt(colPins[3]),MatISR,FALLING);
  }

  //mode cleaning
  if (mc.getMode() == Cleaning) {
    Key.matIn();
    attachInterrupt(digitalPinToInterrupt(colPins[3]),MatISR,FALLING);
  }
  else {

    //mode waiting
    if (mc.getMode() == Waiting) {
      if (isOpen()) {mc.setMode(Using);}
      delay(1000);
      lcd.clear();
    }

    //mode using
    else {
      Serial.print("!Skip = "); Serial.println(!(Key.getSkip()));
      Serial.print("!Set = "); Serial.println(!(Key.isSetting()));
      Serial.print("MInt is "); Serial.println(MInt);
      attachInterrupt(digitalPinToInterrupt(colPins[3]),MatISR,FALLING);
      if (!(Key.getSkip()) && !(Key.isSetting())) {
        attachInterrupt(digitalPinToInterrupt(micInt),MicISR,RISING);
        Serial.println(isOpen());
        TT.start();

        //if mic was interrupted
        if (MInt == true) { Serial.println("Alarm is ringing!!"); Buzzer(BuzzerPin); return;}

        //continue mode using
        readTemphumid(TandH, lcd);
        PrintTimePassed();

        //openning the door
        if (TT.isMoreThan30() && isOpen()) {
          mc.setMode(Waiting); 
          detachInterrupt(digitalPinToInterrupt(micInt));
          TT.stop();
          delay(20000);
          return;
        }

        //if stay in mode using for too long, force mic interrupting flag
        else if (TT.isMoreThanST()) {MInt = true;}
      }
    }
  }
}