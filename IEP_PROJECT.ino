#include <RichShieldTM1637.h>
#include <RichShieldDHT.h>
#include <RichShieldPassiveBuzzer.h>

#define CLK 10
#define DIO 11

TM1637 disp(CLK, DIO);
DHT dht;
PassiveBuzzer buzzer;

const int buttonPin = 8;
const int buttonPin2 = 9;
const int yellowLedPin = 7;
const int blueLedPin = 6;
const int ldrPin = A2;
const int redPin = 4;
const int greenPin = 5;
const int knobPin = A0;
const int buzzerPin = 3;

const int buzzerTones[] = {1000, 1500, 2000}; 
const int numTones = sizeof(buzzerTones) / sizeof(buzzerTones[0]);
const int ledPins[] = {yellowLedPin, blueLedPin, greenPin, redPin}; 


const unsigned long debounceDelay = 50;
const unsigned long timerDuration = 10000;
int lastbuttonState = LOW;
int lastbutton2state = LOW;

bool ismanualmode = false;
bool isautomode = true;
bool isknob = false;
bool displayTimer = false;

int buttonState = LOW;
int button2state = LOW;
int ledState = 0;
int led2state = 0;
unsigned long previousMillis = 0;
unsigned long button2previousmillis = 0;
unsigned long timerstart1 = 0;
unsigned long displayStartMillis = 0;

void setup() {
  disp.init();
  disp.set(7);
  dht.begin();

  Serial.begin(9600);
  pinMode(redPin, OUTPUT);
  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(buttonPin2, INPUT_PULLUP);
  pinMode(yellowLedPin, OUTPUT);
  pinMode(blueLedPin, OUTPUT);
  pinMode(ldrPin, INPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(knobPin, INPUT);
  pinMode(buzzerPin, OUTPUT);
}

void ButtonK1() {
  int currentState = digitalRead(buttonPin);

  if (currentState != buttonState) {
    buttonState = currentState;

    if (buttonState == HIGH) {
      ledState = !ledState;

      if (ledState == 0) {
        digitalWrite(ledPins[0], HIGH);
        digitalWrite(ledPins[1], LOW);
        digitalWrite(ledPins[2], LOW);
        digitalWrite(ledPins[3], LOW);
        ismanualmode = true;
        isautomode = false;
        isknob = false;
        displayTimer = false; 
        Serial.println("now on manual mode");
        playTone(0,500);
      } else {
        digitalWrite(blueLedPin, HIGH);
        digitalWrite(yellowLedPin, LOW);
        ismanualmode = false;
        isautomode = true;
        displayTimer = false; 
        Serial.println("now on auto mode");
        playTone(0,500);
      }
    }
  }
}


void buttonk2() {
  if (!ismanualmode) return;
  int currentstate = digitalRead(buttonPin2);

  if (currentstate != lastbutton2state) {
    button2previousmillis = millis();
  }

  if ((millis() - button2previousmillis) > debounceDelay) {
    if (currentstate != button2state) {
      button2state = currentstate;

      if (button2state == HIGH) {
        led2state = !led2state;

        if (led2state == 0) {
          isknob = false;
          digitalWrite(greenPin, HIGH);
          digitalWrite(redPin, LOW);
          timerstart1 = millis();
          displayStartMillis = timerstart1; 
          displayTimer = true;
          playTone(1,100);
        } else {
          isknob = true;
          if (isknob) {
            knobcontrol();
          }
          digitalWrite(greenPin, LOW);
          digitalWrite(redPin, HIGH);
          timerstart1 = millis();
          displayStartMillis = timerstart1; 
          displayTimer = true;
          playTone(1,100);
        }
      }
    }
  }

  lastbutton2state = currentstate;
}
void manualmode() {
  segdisplay();
  Serial.println("now on manual mode");
  knobcontrol();
}

void automode() {
  segdisplay();
  Serial.println("now on auto mode");
}
void loop() {
  ButtonK1();
  buttonk2();
  if (isautomode) {
    segdisplay(); 
    ldr();
    float temperature = dht.readTemperature();
    AC(temperature);
  } else if (ismanualmode) {
    manualmode();
  }
  timer();
}

void segdisplay() {
  if (!ismanualmode && !displayTimer) {  
    float t = dht.readTemperature();
    displayTemperature((int8_t)t);
  }
}

void displayTemperature(int8_t temperature) {
  int8_t temp[4];
  if (temperature < 0) {
    temp[0] = INDEX_NEGATIVE_SIGN;
    temperature = abs(temperature);
  } else {
    if (temperature < 100) temp[0] = INDEX_BLANK;
    else temp[0] = temperature / 100;
  }
  temperature %= 100;
  temp[1] = temperature / 10;
  temp[2] = temperature % 10;
  temp[3] = 12;
  disp.display(temp);
}

void ldr() {
  int ldrValue = analogRead(ldrPin);
  int pwmValue = map(ldrValue, 0, 1023, 255, 0);
  pwmValue = constrain(pwmValue, 0, 255);

  analogWrite(redPin, pwmValue);
  delay(100);
}

void AC(float temperature) {
  if (temperature >= 25) {
    digitalWrite(greenPin, HIGH);
  } else {
    digitalWrite(greenPin, LOW);
  }
}

void knobcontrol() {
  int knobvalue = analogRead(knobPin);
  int pwmValue = map(knobvalue, 0, 1023, 0, 255);
  analogWrite(redPin, pwmValue);
}

void timer() {
  unsigned long currentMillis = millis();
  if (ismanualmode && (millis() - timerstart1 > timerDuration)) {
    digitalWrite(greenPin, LOW);
    digitalWrite(redPin, LOW);
    isknob = false;
    displayTimer = false;
  }
  if (ismanualmode && digitalRead(buttonPin2) == HIGH) {
    if (currentMillis - displayStartMillis >= 0) {
      displayStartMillis = currentMillis;
      displayTimer = !displayTimer;
    }
  }

  if (displayTimer) {
    timedisplay(currentMillis);
  } else {
    segdisplay();
  }
}

void timedisplay(unsigned long currentMillis) {
  unsigned long elapsedTime = currentMillis - timerstart1;
  int seconds = (elapsedTime / 1000) % 60;
  int minutes = (elapsedTime / 60000) % 60;
  int8_t timerDisplay[4] = {minutes / 10, minutes % 10, seconds / 10, seconds % 10};
  disp.display(timerDisplay);
}

void playTone(int tone, int duration) {
  if (tone >= 0 && tone < numTones) {
    buzzer.playTone(buzzerTones[tone], duration);
  }
}
