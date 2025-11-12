# Prototype 1 Code review
  ## Header
  The libraries we used are:
  - [Keypad.h](https://github.com/Chris--A/Keypad)
  - Wire.h
  - [LiquidCrystal_I2C.h](https://github.com/sstaub/LCD-I2C-HD44780)
  - [DHT11.h](https://github.com/dhrubasaha08/DHT11)
  ## I/O
  I/O are declared in the line `31` for keypad:
  ```
  byte rowPins[ROWS] = {51,50,53,52};
  byte colPins[COLS] = {48,49,46,47};
  ```
  and `37` for others:
  ```
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
  ```
  ## Classes
  We wrote 3 classes for this project: **ModeControl**, **TimeGod**, and **keypadChoose**
