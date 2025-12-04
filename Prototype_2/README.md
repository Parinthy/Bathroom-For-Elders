# Prototype 2 Code review
    This prototype works on the similar flow to protorype 1 without using delay and keypad is used entirely in poling. Creating a smoother, and reliable flow.
## Input difference
    The only difference is in keypad:
    ```
    byte rowPins[ROWS] = {51,50,53,52};
    byte colPins[COLS] = {48,49,46,18};
    ```