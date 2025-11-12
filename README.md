# Smart Bathroom For Elderlies
  A **Smart Bathroom** design using Arduino with the purpose of being safe and accessible for elderlies. 
  The design follows **Goal 3** of Sustainable Development Goals (SDG): **"Good Health and Well-being"**.

## What motivated us
  The **bathroom** is one of the most common places for accidents among elderlies. According to the [_Thai Department of Disease Control 2023_](https://ddc.moph.go.th/dip/news.php?news=23567&deptcode=), approximately **86,000** elderlies are injured and around **1,500** die each year due to slips and falls in the bathroom.

  To address this issue, we developed **Smart Bathroom Prototype** featuring:
  - An **alarm system** for emergency detection
  - An **automatic light switch** for accessibility

## Components
  Here is the list of component we used for this prototype:
### Inputs
  1. Obstacle Track Sensor   -- one of the door sensors
  2. Ultrasonic Sensor       -- one of the door sensors
  3. Microphone Sensor       -- falling detector
  4. Temp & Humid sensor     -- read the temperater and humidity in the 
  5. Matrix Keypad Module    -- controller
### Outputs
  1. LED            -- represent the bathroom light
  2. Buzzer         -- alarm noise
  3. LCD Display    -- display
  4. RGB LED        -- shows the state and alarm light

## Function
  There are 3 states of working corresponding to the actual bathroom: **Waiting**, **Using**, and **Cleaning**

  ### Waiting state
  Waiting state corresponds to when the bathroom is empty. The function in this state are:
  - bathroom light is off
  - RGB light is green
  - door sensors are activated
  
  when the user opens the door, the system will change to **Using** state.

  ### Using state
  Using state corresponds to when the bathroom is being used. The function in this state are:
  - bathroom light is on
  - RGB light is red
  - LCD shows the temperater, humidity, and time used
  - Microphone sensor is activated as an interrupt
 
  When the timer exceed 30 seconds, the door sensors will be activated. So, when the door is opened after 30 seconds into using, the will change to **Waiting**.
  The microphone sensor will be talked about later.

  ### Cleaning state
  Cleaning state corresponds to when the bathroom is being cleaned. The function in this state are:
  - bathroom light is on
  - RGB light is yellow
  - LCD shows "Mode: Cleaning"

  This state can only be used or exited manually using the keypad.

  ### Matrix keypad
  Matrix keypad is used to:
  1. change the state simultenously using the right-most column as shown:
  ```
  [ 1 , 2 , 3 , u ]
  [ 4 , 5 , 6 , w ]
  [ 7 , 8 , 9 , c ]
  [ - , 0 , - , - ]
  ```
  **'u'** change to **Using** state,
  **'w'** change to **Watiting** state,
  **'c'** change to **Cleaning** state,
  and **'-'** has no function.

  2. change the limit time for shower using the numpad as shown above. As the time is being set, the LCD will show something like
  ```
  Set minutes:
  00:00
  ```
  The second line represent `minutes:seconds` and we can replace `0` with the number we want. And the range of the time set is between `00:30` and `59:59`.

  ### Alarm
  The alarm is made with RGB light and a buzzer where both will be turned on for `0.5 sec` and off for `0.5 sec` repeatedly until the right-most column of keypad is pushed.
  
  The alarm will be activated within the **Using** state when
  1. the time exeed the limit time set for shower (initially 45 seconds)
  2. microphone sensor detects the sound of falling

  # Code review
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
