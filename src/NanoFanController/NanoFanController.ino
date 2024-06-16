#include <Wire.h>
#include <EEPROM.h>

//display
#include <SSD1803A_I2C.h>  //https://github.com/sstaub/SSD1803A_I2C

const uint8_t i2caddr = 0x3C;  //LCD i2c address
const uint8_t resetPin = 4;    //LCD reset pin Digital4

const uint8_t LEDg = 12;  //green background LED output pin
const uint8_t LEDr = 11;  //red background LED output pin
const uint8_t BEEP = 13;  //piezo active beeper output pin
const uint8_t tachoCh1 = 3; //channel 1 tacho input pin
const uint8_t tachoCh2 = 2; //channel 2 tacho input pin
const uint8_t tempPinCh1 = A0; //channel 1 ntc temp resistor input pin
const uint8_t tempPinCh2 = A1; //channel 2 ntc temp resistor input pin
const uint8_t tempPinCh3 = A3; //channel 2 ntc temp resistor input pin
const uint8_t pwmPinCh1 = 9; //channel 1 pwm output pin
const uint8_t pwmPinCh2 = 10; //channel 2 pwm output pin

//global timing variables
unsigned long processingTimer, channelSwitchTimer, buttonimpulse, alertTimer;
//tacho counter vars
volatile int tachocntPin2;
volatile int tachocntPin3;

#define PWM_DUTY_OFFSET 1.5  //to precise calibrate pwm duty cycle via oscilloscope

word magicValue_EEPROM = 1938476;  //detects new eeprom to load default values -> change to new value if eeprom data structure differs

//debounce for buttons in ms
#define debounceDelay 50

//lcd
SSD1803A_I2C lcd(i2caddr, resetPin);

//ssd1803a_2_0.pdf ROM C Charset Table
#define CHARgrad B11011111
#define CHARohm B00011110

//set custom chars
//arrow up
uint8_t CUSTOMup[8] = {
  0b00000,
  0b00100,
  0b01110,
  0b11111,
  0b00100,
  0b00000,
  0b00000,
  0b00000
};

//arrow down
uint8_t CUSTOMdown[8] = {
  0b00000,
  0b00000,
  0b00000,
  0b00100,
  0b11111,
  0b01110,
  0b00100,
  0b00000
};

//return char
uint8_t CUSTOMenter[8] = {
  0b00000,
  0b00001,
  0b00101,
  0b01101,
  0b11111,
  0b01100,
  0b00100,
  0b00000
};



//set interrupts for tacho pins
void tachoISRPin2() {
  tachocntPin2++;

};

void tachoISRPin3() {
  tachocntPin3++;
 
};

//software reset function// call with: resetFunc();
void (*resetFunc)(void) = 0;


//erases whole EEPROM
void EraseEEPROM() {
  for (int index = 0; index < EEPROM.length(); index++) {
    EEPROM[index] = 0;
  }
};

//struct to hold status, menu and display variables
struct statusVars {
  //menu
  byte displayMode = 0;  //0 = Status 1 = Setup
  byte level = 0;        //setup root = level 0
  byte menu0 = 0;        //menu level 0
  byte menu1 = 0;        //menu level 1
  bool edit = false;     //edit variable?
  int stringedit = -1;   //char of string to edit

  //status
  String infotxt;
  unsigned long Lastinfotxt = 0;
  bool showinfotxt = false;
  byte ch = 1;  //autoswitch channel index
  bool freshStart = true;
  bool beeperMuted = false;  //mutes the alert
  bool ALERT = false;        //alert aktive?
  bool ALERTHIGH = false;    //current alert beep high helper var
};
statusVars dv;

//struct to save general setup in EEPROM
struct generalEEPROM {
  int autoswitchtime;   //time in ms when it should switch to the next channel in autoswitch mode
  bool upsideDown;      //inverts display orientation and button layout
  byte statusMode = 0;  //auto,ch1,ch2,ch3
};

//start addr of general setup in EEPROM
int eeprom_addrGeneral = 0;

generalEEPROM generalSettings;

int ReadGeneralEEPROM(int StartAddr, struct generalEEPROM &settings) {
  EEPROM.get(StartAddr, settings);
  //set default values
  if (settings.autoswitchtime < 100) settings.autoswitchtime = 1000;

  return StartAddr + sizeof(generalEEPROM);  //return new addr offset
};

void WriteGeneralEEPROM(int StartAddr, struct generalEEPROM &settings) {
  EEPROM.put(StartAddr, settings);
};

//struct to save every channel instance in EEPROM
struct ChannelEEPROMSettings {
  bool initialized;  //true after first time we have written to EEPROM
  bool enabled;      //channel enabled?
  char chName[8];    //custom channel name

  word temp_THERMISTORNOMINAL;
  word temp_TEMPERATURENOMINAL;
  word temp_BCOEFFICIENT;
  word temp_SERIESRESISTOR;
  bool enablePWM, enableTacho, enableAutoPWM, enableMinTempAlert, enableMaxTempAlert, enableNoRPMAlert;
  byte minDuty, maxDuty, p1Duty, p2Duty, p3Duty, manualDuty, dutyHysteresis;
  word p1Temp, p2Temp, p3Temp, pmaxAlertTemp;
};

//channel class for every channel
class Channel {
public:
  word EEPROMaddr = 0;
  bool enabled;  //channel enabled?
  byte chNr;     //channel index 1,2,3
  char chName[8];

  //temperature
  byte temp_pin;
  word temp_THERMISTORNOMINAL;   // nominal resistance of NTC-Resistor in Ohm
  word temp_TEMPERATURENOMINAL;  // nominal temperature of NTC-Resistor in °C
  word temp_BCOEFFICIENT;        // material coefficient B of NTC-Resistor
  word temp_SERIESRESISTOR;      // series resistor in Ohm (~ 10k - should be measured carefully)
  int temp, oldtemp;

  //pwm
  byte pwm_pin;  // 0 = deactivated
  byte currentDuty = 50;
  int currentHystDutyMax = 0, currentHystDutyMin = 0;
  bool currentHystMax = true;
  bool enablePWM, enableAutoPWM;
  byte curveStatus = 0;     //which part of temp - to duty cycle is active
  byte curveStatusOld = 0;  //last value to compare and detect change

  //tacho
  byte tacho_pin;  // 0 = deactivated
  bool enableTacho;
  unsigned int rpm, oldrpm;

  //temp to pwm vars
  byte minDuty, maxDuty, p1Duty, p2Duty, p3Duty, manualDuty, dutyHysteresis;
  word p1Temp, p2Temp, p3Temp;

  //alerting
  bool enableMinTempAlert, enableMaxTempAlert, enableNoRPMAlert;
  word pmaxAlertTemp;

  float temperature_NTCB(float Tntc, float Rntc, float B, float RV, float VA_VB) {
    Tntc += 273.15;                       // convert celsius to absolute temperature
    float RN = RV * VA_VB / (1 - VA_VB);  // actual resistance of NTC

    return Tntc * B / (B + Tntc * log(RN / Rntc)) - 273.15;
  }

  int getTemp() {
    //https://forum.arduino.cc/t/ol-temperatur-messen-welcher-oltemperaturgeber-kfz/152055/12#msg1170722
    float temp = temperature_NTCB(this->temp_TEMPERATURENOMINAL, this->temp_THERMISTORNOMINAL, this->temp_BCOEFFICIENT, this->temp_SERIESRESISTOR, analogRead(temp_pin) / 1023.0);
    //if (temp < 0) temp = 0;   //prevent byte overflow later
    return round(temp * 10);  //integer is faster to compute
  }

  void initPWM() {
    if (!this->enablePWM) return;
    if (pwm_pin != 9 and pwm_pin != 10) return;  //deactivate PWM if both pwm function pins are deactivated
    //timer 1
    // Clear OC1A and OC1B on Compare Match / Set OC1A and OC1B at Bottom;
    // Wave Form Generator: Fast PWM 14, Top = ICR1
    TCCR1A = 0 + (1 << WGM11) + (1 << COM1A0) + (1 << COM1B0);  //(1 << COM1A0)+(1 << COM1B0) = invert
    TCCR1A = TCCR1A + (1 << COM1A1);                            //enable A
    TCCR1A = TCCR1A + (1 << COM1B1);                            //enable B
    //TCCR1A = (1<<COM1A1) + (1<<COM1B1) + (1<<WGM11);
    TCCR1B = (1 << WGM13) + (1 << WGM12) + (1 << CS10);  // prescaler = 1 (none)
    ICR1 = 1599;

    //set pwm output pins
    switch (this->pwm_pin) {
      case 9:
        DDRB |= (1 << PB1);
        break;
      case 10:
        DDRB |= (1 << PB2);
        break;
    }

    setPWM(currentDuty);  //default
  }

  void setPWM(byte duty) {
    this->currentDuty = duty;
    switch (this->pwm_pin) {
      case 9:
        if (duty > 0 and this->enablePWM) {
          OCR1A = ICR1 * 0.01 * (duty + PWM_DUTY_OFFSET);  //(A) digital pin 9
        } else {
          OCR1A = 0;
        }
        break;
      case 10:
        if (duty > 0 and this->enablePWM) {
          OCR1B = ICR1 * 0.01 * (duty + PWM_DUTY_OFFSET);  //(B) digital pin 10
        } else {
          OCR1B = 0;
        }
        break;
    }
  }

  //constructor sets channel number
  Channel(byte chNr) {
    this->chNr = chNr;
  }

  int init(int StartAddr, byte temp_pin, byte pwm_pin, byte tacho_pin) {
    int addr;
    this->temp_pin = temp_pin;
    this->pwm_pin = pwm_pin;
    this->tacho_pin = tacho_pin;

    addr = ReadEEPROM(StartAddr);
    initPWM();
    toggleTacho(this->enableTacho);
    return addr;
  }

  int ReadEEPROM(int StartAddr) {
    EEPROMaddr = StartAddr;
    ChannelEEPROMSettings settings;
    EEPROM.get(StartAddr, settings);
    this->temp_THERMISTORNOMINAL = settings.temp_THERMISTORNOMINAL;
    this->temp_TEMPERATURENOMINAL = settings.temp_TEMPERATURENOMINAL;
    this->temp_BCOEFFICIENT = settings.temp_BCOEFFICIENT;
    this->temp_SERIESRESISTOR = settings.temp_SERIESRESISTOR;
    memcpy(this->chName, settings.chName, 8);
    this->enablePWM = settings.enablePWM;
    this->enableTacho = settings.enableTacho;
    this->enabled = settings.enabled;
    this->enableAutoPWM = settings.enableAutoPWM;
    this->manualDuty = settings.manualDuty;
    this->minDuty = settings.minDuty;
    this->maxDuty = settings.maxDuty;
    this->p1Duty = settings.p1Duty;
    this->p2Duty = settings.p2Duty;
    this->p3Duty = settings.p3Duty;
    this->p1Temp = settings.p1Temp;
    this->p2Temp = settings.p2Temp;
    this->p3Temp = settings.p3Temp;
    this->dutyHysteresis = settings.dutyHysteresis;
    this->enableNoRPMAlert = settings.enableNoRPMAlert;
    this->enableMinTempAlert = settings.enableMinTempAlert;
    this->enableMaxTempAlert = settings.enableMaxTempAlert;
    this->pmaxAlertTemp = settings.pmaxAlertTemp;

    if (this->pwm_pin == 0) this->enablePWM = false;
    if (this->tacho_pin == 0) this->enableTacho = false;
    if (this->chNr == 1) this->enabled = true;  //ch1 always enabled

    if (!settings.initialized) {
      //defaults

      strncpy(this->chName, "ABC     ", 8);
      this->temp_THERMISTORNOMINAL = 10000;
      this->temp_TEMPERATURENOMINAL = 25;
      this->temp_BCOEFFICIENT = 3976;
      this->temp_SERIESRESISTOR = 10000;
      this->enableAutoPWM = true;
      this->enablePWM = true;
      this->enableTacho = true;
      this->manualDuty = 50;
      this->minDuty = 0;
      this->maxDuty = 100;
      this->p1Duty = 10;
      this->p2Duty = 40;
      this->p3Duty = 60;
      this->p1Temp = 220;
      this->p2Temp = 240;
      this->p3Temp = 260;
      this->dutyHysteresis = 5;
      this->enableNoRPMAlert = false;
      this->enableMinTempAlert = false;
      this->enableMaxTempAlert = true;
      this->pmaxAlertTemp = 450;
    }

    return StartAddr + sizeof(ChannelEEPROMSettings);  //return new addr offset
  }

  void WriteEEPROM() {
    ChannelEEPROMSettings settings;
    settings.temp_THERMISTORNOMINAL = this->temp_THERMISTORNOMINAL;
    settings.temp_TEMPERATURENOMINAL = this->temp_TEMPERATURENOMINAL;
    settings.temp_BCOEFFICIENT = this->temp_BCOEFFICIENT;
    settings.temp_SERIESRESISTOR = this->temp_SERIESRESISTOR;
    memcpy(settings.chName, this->chName, 8);
    settings.enablePWM = this->enablePWM;
    settings.enableTacho = this->enableTacho;
    settings.enabled = this->enabled;
    settings.initialized = true;
    settings.enableAutoPWM = this->enableAutoPWM;
    settings.manualDuty = this->manualDuty;
    settings.minDuty = this->minDuty;
    settings.maxDuty = this->maxDuty;
    settings.p1Duty = this->p1Duty;
    settings.p2Duty = this->p2Duty;
    settings.p3Duty = this->p3Duty;
    settings.p1Temp = this->p1Temp;
    settings.p2Temp = this->p2Temp;
    settings.p3Temp = this->p3Temp;
    settings.dutyHysteresis = this->dutyHysteresis;
    settings.enableNoRPMAlert = this->enableNoRPMAlert;
    settings.enableMinTempAlert = this->enableMinTempAlert;
    settings.enableMaxTempAlert = this->enableMaxTempAlert;
    settings.pmaxAlertTemp = this->pmaxAlertTemp;


    EEPROM.put(EEPROMaddr, settings);
  }

  bool tachoEnabled() {
    return this->enableTacho;
  }

  //enable/disable tacho interrupt handling
  void toggleTacho(bool enabled) {
    this->enableTacho = enabled;
    detachInterrupt(digitalPinToInterrupt(this->tacho_pin));
    if (enabled) {
      switch (this->tacho_pin) {
        case 2: attachInterrupt(digitalPinToInterrupt(this->tacho_pin), tachoISRPin2, RISING); break;
        case 3: attachInterrupt(digitalPinToInterrupt(this->tacho_pin), tachoISRPin3, RISING); break;
      }
    }
  }

  //calculates tacho value and resets isr counter variable
  int getTacho() {
    int ret = 0;
    switch (this->tacho_pin) {
      case 2:
        ret = tachocntPin2 * 30;
       
        tachocntPin2 = 0;
        return ret;
        break;

      case 3:
        ret = tachocntPin3 * 30;
        
        tachocntPin3 = 0;
        return ret;
        break;
    }
  }

  //process channel calculations and pwm control
  bool process() {
    bool changed = false;
    if (this->enabled) {
      this->oldtemp = this->temp;
      this->temp = getTemp();
      if (this->oldtemp != this->temp) changed = true;

      //check temps for alerts
      if (this->enableMaxTempAlert && this->temp > this->pmaxAlertTemp) DoAlert(this->chNr);
      if (this->enableMinTempAlert && this->temp < 0) DoAlert(this->chNr);

      if (this->enableAutoPWM) {
        //temperature controlled pwm regulation
        byte pwm = 0;

        //temperature between p1 and p2
        if (this->temp >= this->p1Temp && this->temp < this->p2Temp) {
          float deltaDuty = this->p2Duty - this->p1Duty;
          float deltaPT = this->p2Temp - this->p1Temp;
          float deltaT = this->temp - this->p1Temp;
          pwm = this->p1Duty;

          if (deltaPT > 0) {
            float factorT = deltaT / deltaPT;
            pwm = pwm + deltaDuty * factorT;
          }

          curveStatus = 1;
        }
        //temperature between p2 and p3
        if (this->temp >= this->p2Temp && this->temp < this->p3Temp) {
          float deltaDuty = this->p3Duty - this->p2Duty;
          float deltaPT = this->p3Temp - this->p2Temp;
          float deltaT = this->temp - this->p2Temp;
          pwm = this->p2Duty;

          if (deltaPT > 0) {
            float factorT = deltaT / deltaPT;
            pwm = pwm + deltaDuty * factorT;
          }

          curveStatus = 2;
        }

        //temperature > p3
        if (this->temp >= this->p3Temp) {
          pwm = this->maxDuty;

          curveStatus = 3;
        }

        if (pwm < this->minDuty) {
          pwm = this->minDuty;

          curveStatus = 0;
        }

        if (pwm > this->maxDuty) pwm = this->maxDuty;

        //calculate hysteresis for dutycycle
        byte hystpwm = pwm;
        if (this->dutyHysteresis > 1) {
          if (pwm >= this->currentHystDutyMax) {
            this->currentHystDutyMax = pwm;
            this->currentHystMax = true;
            this->currentHystDutyMin = pwm - this->dutyHysteresis;
            if (this->currentHystDutyMin < 0) this->currentHystDutyMin = 0;  //clip
          }
          if (pwm <= this->currentHystDutyMin) {
            this->currentHystDutyMin = pwm;
            this->currentHystMax = false;
            this->currentHystDutyMax = pwm + this->dutyHysteresis;
            if (this->currentHystDutyMax > 100) this->currentHystDutyMax = 100;  //clip
          }

          if (this->currentHystMax) hystpwm = this->currentHystDutyMax;
          else hystpwm = this->currentHystDutyMin;
        }

        setPWM(hystpwm);
      } else {
        //manual pwm settings
        setPWM(this->manualDuty);
      }

      this->oldrpm = this->rpm;
      this->rpm = getTacho();
      if (this->oldrpm != this->rpm) changed = true;
      //check no rpm alert
      if (this->enableNoRPMAlert && this->rpm == 0) DoAlert(this->chNr);
    }
    return changed;
  }

  //displays channel values
  bool display() {
    lcd.locate(0, 9);
    lcd.print(chNr);

    lcd.locate(0, 0);
    lcd.write(this->chName, 8);  //print needs null terminated string -> we dont have, so we write() bytes directly with specified length

    if (!enabled) return false;

    lcd.locate(1, 0);

    char buf[7];
    lcd.print(dtostrf((float)this->temp / 10, 2, 1, buf));

    lcd.print(" ");
    lcd.print((char)CHARgrad);
    lcd.print("C");

    if (this->enableTacho) {
      lcd.locate(2, 0);
      lcd.print(this->rpm);
      lcd.print(" RPM");
    }

    if (this->enablePWM) {
      lcd.locate(3, 0);
      lcd.print(this->currentDuty);

      lcd.print(" %");
      lcd.locate(3, 7);


      if (curveStatusOld != curveStatus) {
        digitalWrite(LEDr, LOW);
        digitalWrite(LEDg, LOW);
        curveStatusOld = curveStatus;
      }

      switch (curveStatus) {
        case 0:

          lcd.print("MIN");
          digitalWrite(LEDg, HIGH);  //green
          break;
        case 1:

          lcd.print(">P1");
          digitalWrite(LEDg, HIGH);  //green
          digitalWrite(LEDr, HIGH);  //red
          break;
        case 2:

          lcd.print(">P2");
          digitalWrite(LEDr, HIGH);  //red

          break;
        case 3:

          lcd.print("MAX");
          digitalWrite(LEDr, HIGH);  //red

          break;
      }
    }
    return true;
  }
};






class Button {
private:
  byte pin;
  byte state;
  byte lastReading;
  bool inverted;
  bool edge;
  unsigned long lastDebounceTime = 0;
  unsigned long startTimeHigh = 0;
public:

  void init(byte pin, bool inverted) {
    //Configue Port PA0 Output
  
    this->pin = pin;
    this->inverted = inverted;
    lastReading = LOW;
    pinMode(pin, INPUT_PULLUP);

    update();
  }
  void update() {
    byte newReading;
    if (inverted) {
      newReading = !digitalRead(pin);
    } else {
      newReading = digitalRead(pin);
    }

    if (newReading != lastReading) {
      lastDebounceTime = millis();
    }

    edge = false;
    if (millis() - lastDebounceTime > debounceDelay) {
      // Update the 'state' attribute only if debounce is checked
      edge = (newReading != state);
      state = newReading;
    }
    lastReading = newReading;
  }
  byte getState() {

    return state;
  }
  bool isPressed() {
    return (state == HIGH);
  }
  bool flHigh() {
    return ((state == HIGH) && edge);
  }
  bool flLow() {
    return ((state == LOW) && edge);
  }
  bool fl() {
    return edge;
  }
  int ms() {
    if (state == HIGH)
      return millis() - lastDebounceTime;
    else return 0;
  }
  bool longPressed() {
    return ms() > 500;
  }
};

Button up;
Button down;
Button left;
Button right;

Channel ch1(1);
Channel ch2(2);
Channel ch3(3);
int cnt = 0;

void SaveSettings() {

  ch1.WriteEEPROM();
  ch2.WriteEEPROM();
  ch3.WriteEEPROM();
  WriteGeneralEEPROM(eeprom_addrGeneral, generalSettings);
}

void setup() {

  //EraseEEPROM();  //uncomment to reset device EEPROM

  //LCD Background LED
  pinMode(LEDr, OUTPUT);  //red
  pinMode(LEDg, OUTPUT);  //green
  digitalWrite(LEDr, HIGH);
  digitalWrite(LEDg, LOW);

  //prepare i2c pin levels for DOGS104 i2c interface so lcd can boot up correct after powerloss - otherwise i2c will be pulled low by LCD for a longer time and i2c/lcd init wont work
  pinMode(resetPin, OUTPUT);
  digitalWrite(resetPin, LOW);
  pinMode(A4, OUTPUT);
  pinMode(A5, OUTPUT);
  digitalWrite(A4, LOW);
  digitalWrite(A5, LOW);
  delay(100);
  digitalWrite(A4, HIGH);
  digitalWrite(A5, HIGH);
  delay(100);



  Serial.begin(9600); //only for debugging needed

  //hardware bugfix for DOGS104 i2c interface after powerloss -> display sets i2c lines to low for some seconds - so we wait until we got "normal" pullup voltage
  //setting reset pin to low, i2c pins to high makes this propably obsolete but its ok to leave this here functional in case of non functional i2c levels
  /*
  float a4reading = analogRead(A4) * (5.0 / 1023.0);
  if (a4reading < 4.5) {
    digitalWrite(LEDr, LOW);
    uint8_t greenLED = LOW;
    while (a4reading < 4.5) {
      a4reading = analogRead(A4) * (5.0 / 1023.0);
       Serial.println(a4reading);

      digitalWrite(LEDg, greenLED);
      if (greenLED == HIGH) greenLED = LOW;
      else greenLED = HIGH;
      delay(200);
    }
    digitalWrite(LEDr, LOW);
    digitalWrite(LEDg, HIGH);
    delay(2000);
  }
*/
  digitalWrite(LEDr, LOW);
  digitalWrite(LEDg, HIGH);

  //LCD init
  lcd.begin(DOGS104);
  delay(200);  //needed for working LCD init otherwise i2c voltage level to low for communication
  lcd.display(DISPLAY_ON);

  lcd.create(0, CUSTOMup);
  lcd.create(1, CUSTOMdown);
  lcd.create(2, CUSTOMenter);
  lcd.cls();
  lcd.locate(0, 0);
  lcd.print("NANO PWM");


  word magicValue_EEPROM_SET;
  EEPROM.get(0, magicValue_EEPROM_SET);  //first int in eeprom is to check if its initialized
  if (magicValue_EEPROM_SET != magicValue_EEPROM) {
    lcd.locate(1, 0);
    lcd.print("Erasing");
    lcd.locate(2, 0);
    lcd.print("EEPROM..");
    EraseEEPROM();  //use once after first time flashed -> erase EEPROM to load defaults later
    delay(100);
    lcd.locate(3, 0);
    lcd.print("RESTART..");
    delay(1000);
    resetFunc();
  }

  //init channels (read EEPROM) -> we start at byte 3
  int addr = ch1.init(2, tempPinCh1, pwmPinCh1, tachoCh1);
  addr = ch2.init(addr, tempPinCh2, pwmPinCh2, tachoCh2);
  eeprom_addrGeneral = ch3.init(addr, tempPinCh3, 0, 0);

  //init general settings (read EEPROM)
  ReadGeneralEEPROM(eeprom_addrGeneral, generalSettings);

  //set display orientation
  if (generalSettings.upsideDown) {
    lcd.display(VIEW_TOP);
  } else {
    lcd.display(VIEW_BOTTOM);
  }

  //prevent further erases and save magic value in EEPROM
  if (magicValue_EEPROM_SET != magicValue_EEPROM) {
    EEPROM.put(0, magicValue_EEPROM);
  }

  //init buttons with specific orientation (ordering)
  if (generalSettings.upsideDown) {
    left.init(8, true);
    right.init(7, true);
    up.init(6, true);
    down.init(5, true);
  } else {
    left.init(5, true);
    right.init(6, true);
    up.init(7, true);
    down.init(8, true);
  }

  digitalWrite(LEDg, HIGH);

  lcd.locate(1, 0);
  lcd.print("V1.0");
  delay(300);
  dv.ch = 0;

  digitalWrite(LEDr, HIGH);
  delay(300);

  digitalWrite(LEDg, LOW);
  delay(300);

  //i2c clock
  Wire.setClock(400000);  //400khz instead of standard 100khz i2c clock

  //tacho counter init
  tachocntPin2 = 0;
  tachocntPin3 = 0;
};

//displays info text for a short time
void Infotxt(String txt) {
  dv.Lastinfotxt = millis();
  dv.showinfotxt = true;
  dv.infotxt = txt;
};

//enable alert if not already muted
void DoAlert(byte displayCH) {
  if (!dv.beeperMuted) {
    dv.ALERT = true;
    dv.ch = displayCH;
  }
}

void loop() {
  bool changed = false;
  if (dv.freshStart) {
    changed = true;  //display all for first time after startup
    dv.freshStart = false;
  }

  bool buttonpulse = false;
  unsigned long actms;
  actms = millis();
  Channel *ch;
  byte tempbyte;




  //processing
  if (actms - processingTimer > 1000) {
    //https://www.norwegiancreations.com/2018/10/arduino-tutorial-avoiding-the-overflow-issue-when-using-millis-and-micros/
    processingTimer = actms;

    if (ch1.process()) changed = true;
    if (ch2.process()) changed = true;
    if (ch3.process()) changed = true;
  }

  //display switch thru channels
  if ((generalSettings.statusMode == 0) && (actms - channelSwitchTimer > generalSettings.autoswitchtime)) {
    dv.ch++;
    if (dv.ch >= 4) dv.ch = 1;
    changed = true;
    channelSwitchTimer = actms;
  }

  //get button input state
  left.update();
  right.update();
  up.update();
  down.update();
  //get long press button state
  bool uplong = up.longPressed();
  bool downlong = down.longPressed();
  //refresh on every buttonchange edge
  if (left.fl() || right.fl() || up.fl() || down.fl()) {
    changed = true;
    //mute alert if any button has been pressed
    if (dv.ALERT) {
      dv.ALERT = false;
      dv.beeperMuted = true;
      digitalWrite(BEEP, LOW);
      digitalWrite(LEDg, LOW);  //green
      digitalWrite(LEDr, LOW);  //red
    }
  }

  //recognize long button press and simulate button press by pulses
  if (actms - buttonimpulse > 50) {
    buttonpulse = true;
    buttonimpulse = actms;
  }

  //alert beeping and background color flashing
  if (dv.ALERT) {
    if (actms - alertTimer > 1000) {

      if (dv.ALERTHIGH) {
        digitalWrite(BEEP, LOW);
        digitalWrite(LEDg, HIGH);  //green
        digitalWrite(LEDr, LOW);   //red
        dv.ALERTHIGH = false;
      } else {
        digitalWrite(BEEP, HIGH);
        digitalWrite(LEDg, LOW);   //green
        digitalWrite(LEDr, HIGH);  //red
        dv.ALERTHIGH = true;
      }

      alertTimer = actms;
    }
  }
  //control/menu logic
  switch (dv.displayMode) {
    case 0:  //status

      if (right.flHigh()) {
        dv.displayMode = 1;  //go setup
        dv.menu1 = 0;        //save
      }

      //toggle status
      if (up.flHigh()) generalSettings.statusMode++;
      if (down.flHigh()) generalSettings.statusMode--;
      if (generalSettings.statusMode > 3) generalSettings.statusMode = 3;
      if (generalSettings.statusMode < 0) generalSettings.statusMode = 0;
      if (up.fl() || down.fl()) {
        switch (generalSettings.statusMode) {
          case 0:
            Infotxt("DISP AUTO ");
            break;
          case 1:
            Infotxt("DISP CH1  ");
            break;
          case 2:
            Infotxt("DISP CH2  ");
            break;
          case 3:
            Infotxt("DISP CH3  ");
            break;
        }
      }
      break;
    case 1:                                                               //setup
      if (left.flHigh() && dv.level == 0) dv.displayMode = 0;             //go back
      if (left.flHigh() && dv.level > 0 && dv.edit == false) dv.level--;  //go back in setup menu
      if (left.flHigh() && dv.level > 0 && dv.edit == true && dv.stringedit <= 0) {
        dv.edit = false;     // exit edit
        dv.stringedit = -1;  //-1 because it increments +1 always after going into edit mode
        lcd.display(CURSOR_OFF);
        lcd.display(BLINK_OFF);
      }                                                     // go in
      if (right.flHigh() && dv.level == 1) dv.edit = true;  // go edit
      if (right.flHigh() && dv.level < 1) {
        dv.level++;
        dv.menu1 = 0;
        dv.edit = false;
      }

      switch (dv.level) {
        case 0:
          if ((dv.menu0 < 4) && up.flHigh()) dv.menu0++;    //up in menu0
          if ((dv.menu0 > 0) && down.flHigh()) dv.menu0--;  //down in menu0


          break;

        case 1:

          switch (dv.menu0) {
            case 0:  //save
              switch (dv.menu1) {
                case 0:  //save all
                  if (up.flHigh() && dv.edit) {
                    SaveSettings();
                    Infotxt("  SAVED!  ");
                  }
                  break;
              }
              break;
            case 1:                                                         //general
              if ((dv.menu1 < 1) && up.flHigh() && !dv.edit) dv.menu1++;    //up in menu1
              if ((dv.menu1 > 0) && down.flHigh() && !dv.edit) dv.menu1--;  //down in menu1
              switch (dv.menu1) {
                case 0:  //AutoSwitchTime
                  if ((up.flHigh() || (uplong && buttonpulse)) && dv.edit) {
                    if (uplong) generalSettings.autoswitchtime = generalSettings.autoswitchtime + 10;
                    else generalSettings.autoswitchtime++;
                    if (generalSettings.autoswitchtime > 10000) generalSettings.autoswitchtime = 10000;
                  }
                  if ((down.flHigh() || (downlong && buttonpulse)) && dv.edit) {
                    if (downlong) generalSettings.autoswitchtime = generalSettings.autoswitchtime - 10;
                    else generalSettings.autoswitchtime--;
                    if (generalSettings.autoswitchtime < 100) generalSettings.autoswitchtime = 100;
                  }
                  break;
                case 1:  //upsideDown
                  if (up.flHigh() && dv.edit) {
                    if (!generalSettings.upsideDown) {
                      generalSettings.upsideDown = true;
                      SaveSettings();
                      resetFunc();
                    }
                  }
                  if (down.flHigh() && dv.edit) {
                    if (generalSettings.upsideDown) {
                      generalSettings.upsideDown = false;
                      SaveSettings();
                      resetFunc();
                    }
                  }
                  break;
              }
              break;
            case 2 ... 4:                                                   //CH1-3
              if ((dv.menu1 < 22) && up.flHigh() && !dv.edit) dv.menu1++;   //up in menu1
              if ((dv.menu1 > 0) && down.flHigh() && !dv.edit) dv.menu1--;  //down in menu1
              switch (dv.menu0) {
                case 2: ch = &ch1; break;
                case 3: ch = &ch2; break;
                case 4: ch = &ch3; break;
              }
              switch (dv.menu1) {
                case 0:  //Ch Enabled
                  if (up.flHigh() && dv.edit) {
                    ch->enabled = true;
                  }
                  if (down.flHigh() && dv.edit) {
                    if (dv.menu0 > 2)  //ch1 should not be disabled
                      ch->enabled = false;
                  }
                  break;
                case 1:  //PWM Enabled
                  if (up.flHigh() && dv.edit) {
                    ch->enablePWM = true;
                  }
                  if (down.flHigh() && dv.edit) {
                    ch->enablePWM = false;
                  }
                  break;
                case 2:  //RPM Enabled
                  if (up.flHigh() && dv.edit) {
                    ch->toggleTacho(true);
                  }
                  if (down.flHigh() && dv.edit) {
                    ch->toggleTacho(false);
                  }
                  break;
                case 3:  //temp_THERMISTORNOMINAL
                  if ((up.flHigh() || (uplong && buttonpulse)) && dv.edit) {
                    if (uplong) ch->temp_THERMISTORNOMINAL = ch->temp_THERMISTORNOMINAL + 50;
                    else ch->temp_THERMISTORNOMINAL++;
                  }
                  if ((down.flHigh() || (downlong && buttonpulse)) && dv.edit) {
                    if (downlong) ch->temp_THERMISTORNOMINAL = ch->temp_THERMISTORNOMINAL - 50;
                    else ch->temp_THERMISTORNOMINAL--;
                    if (ch->temp_THERMISTORNOMINAL < 0) ch->temp_THERMISTORNOMINAL = 0;
                  }
                  break;
                case 4:  //temp_TEMPERATURENOMINAL
                  if ((up.flHigh() || (uplong && buttonpulse)) && dv.edit) {
                    if (uplong) ch->temp_TEMPERATURENOMINAL = ch->temp_TEMPERATURENOMINAL + 50;
                    else ch->temp_TEMPERATURENOMINAL++;
                  }
                  if ((down.flHigh() || (downlong && buttonpulse)) && dv.edit) {
                    if (downlong) ch->temp_TEMPERATURENOMINAL = ch->temp_TEMPERATURENOMINAL - 50;
                    else ch->temp_TEMPERATURENOMINAL--;
                    if (ch->temp_TEMPERATURENOMINAL < 0) ch->temp_TEMPERATURENOMINAL = 0;
                  }
                  break;
                case 5:  //temp_SERIESRESISTOR
                  if ((up.flHigh() || (uplong && buttonpulse)) && dv.edit) {
                    if (uplong) ch->temp_SERIESRESISTOR = ch->temp_SERIESRESISTOR + 50;
                    else ch->temp_SERIESRESISTOR++;
                  }
                  if ((down.flHigh() || (downlong && buttonpulse)) && dv.edit) {
                    if (downlong) ch->temp_SERIESRESISTOR = ch->temp_SERIESRESISTOR - 50;
                    else ch->temp_SERIESRESISTOR--;
                    if (ch->temp_SERIESRESISTOR < 0) ch->temp_SERIESRESISTOR = 0;
                  }
                  break;
                case 6:  //temp_BCOEFFICIENT
                  if ((up.flHigh() || (uplong && buttonpulse)) && dv.edit) {
                    if (uplong) ch->temp_BCOEFFICIENT = ch->temp_BCOEFFICIENT + 50;
                    else ch->temp_BCOEFFICIENT++;
                  }
                  if ((down.flHigh() || (downlong && buttonpulse)) && dv.edit) {
                    if (downlong) ch->temp_BCOEFFICIENT = ch->temp_BCOEFFICIENT - 50;
                    else ch->temp_BCOEFFICIENT--;
                    if (ch->temp_BCOEFFICIENT < 0) ch->temp_BCOEFFICIENT = 0;
                  }
                  break;
                case 7:  //channel name edit
                  if (dv.edit && dv.stringedit == -1) {
                    lcd.display(CURSOR_ON);
                    lcd.display(BLINK_ON);
                  }
                  if (dv.edit && (dv.stringedit < 7) && right.flHigh()) dv.stringedit++;
                  if (dv.edit && (dv.stringedit > 0) && left.flHigh()) dv.stringedit--;
                  if ((up.flHigh() || (uplong && buttonpulse)) && dv.edit) {
                    tempbyte = (byte)ch->chName[dv.stringedit];
                    tempbyte++;
                    ch->chName[dv.stringedit] = (char)tempbyte;
                  }
                  if ((down.flHigh() || (downlong && buttonpulse)) && dv.edit) {
                    tempbyte = (byte)ch->chName[dv.stringedit];
                    tempbyte--;
                    ch->chName[dv.stringedit] = (char)tempbyte;
                  }

                  break;
                case 8:  //enableAutoPWM
                  if (up.flHigh() && dv.edit) {
                    ch->enableAutoPWM = true;
                  }
                  if (down.flHigh() && dv.edit) {

                    ch->enableAutoPWM = false;
                  }
                  break;
                case 9:  //manual RPM
                  if ((up.flHigh() || (uplong && buttonpulse)) && dv.edit) {
                    ch->manualDuty++;
                    if (ch->manualDuty > 100) ch->manualDuty = 100;
                  }
                  if ((down.flHigh() || (downlong && buttonpulse)) && dv.edit && (ch->manualDuty > 0)) {
                    ch->manualDuty--;
                  }
                  break;
                case 10:  //min RPM
                  if ((up.flHigh() || (uplong && buttonpulse)) && dv.edit) {
                    ch->minDuty++;
                    if (ch->minDuty > 100) ch->minDuty = 100;
                    if (ch->maxDuty < ch->minDuty) ch->minDuty = ch->maxDuty;
                  }
                  if ((down.flHigh() || (downlong && buttonpulse)) && dv.edit && (ch->minDuty > 0)) {
                    ch->minDuty--;
                  }
                  break;
                case 11:  //max RPM
                  if ((up.flHigh() || (uplong && buttonpulse)) && dv.edit) {
                    if (uplong) ch->maxDuty = ch->maxDuty + 10;
                    else ch->maxDuty++;
                    if (ch->maxDuty > 100) ch->maxDuty = 100;
                    if (ch->maxDuty < ch->minDuty) ch->maxDuty = ch->minDuty;
                  }
                  if ((down.flHigh() || (downlong && buttonpulse)) && dv.edit && (ch->maxDuty > 0)) {
                    ch->maxDuty--;
                  }
                  break;

                case 12:  //p1Temp
                  if ((up.flHigh() || (uplong && buttonpulse)) && dv.edit) {
                    ch->p1Temp++;
                    if (ch->p1Temp > 1000) ch->p1Temp = 1000;
                    if (ch->p1Temp >= ch->p2Temp) ch->p1Temp = ch->p2Temp - 1;
                  }
                  if ((down.flHigh() || (downlong && buttonpulse)) && dv.edit && (ch->p1Temp > 0)) {
                    ch->p1Temp--;
                  }
                  break;
                case 13:  //p1Duty
                  if ((up.flHigh() || (uplong && buttonpulse)) && dv.edit) {
                    ch->p1Duty++;
                    if (ch->p1Duty > 100) ch->p1Duty = 100;
                    if (ch->p1Duty >= ch->p2Duty) ch->p1Duty = ch->p2Duty - 1;
                  }
                  if ((down.flHigh() || (downlong && buttonpulse)) && dv.edit && (ch->p1Duty > 0)) {
                    ch->p1Duty--;
                  }
                  break;

                case 14:  //p2Temp
                  if ((up.flHigh() || (uplong && buttonpulse)) && dv.edit) {
                    ch->p2Temp++;
                    if (ch->p2Temp > 1000) ch->p2Temp = 1000;
                    if (ch->p2Temp >= ch->p3Temp) ch->p2Temp = ch->p3Temp - 1;
                  }
                  if ((down.flHigh() || (downlong && buttonpulse)) && dv.edit && (ch->p2Temp > 0)) {
                    ch->p2Temp--;

                    if (ch->p2Temp <= ch->p1Temp) ch->p2Temp = ch->p1Temp + 1;
                  }
                  break;
                case 15:  //p2Duty
                  if ((up.flHigh() || (uplong && buttonpulse)) && dv.edit) {
                    ch->p2Duty++;
                    if (ch->p2Duty > 100) ch->p2Duty = 100;
                    if (ch->p2Duty >= ch->p3Duty) ch->p2Duty = ch->p3Duty - 1;
                  }
                  if ((down.flHigh() || (downlong && buttonpulse)) && dv.edit && (ch->p2Duty > 0)) {
                    ch->p2Duty--;

                    if (ch->p2Duty <= ch->p1Duty) ch->p2Duty = ch->p1Duty + 1;
                  }
                  break;

                case 16:  //p3Temp
                  if ((up.flHigh() || (uplong && buttonpulse)) && dv.edit) {
                    ch->p3Temp++;
                    if (ch->p3Temp > 1000) ch->p3Temp = 1000;
                  }
                  if ((down.flHigh() || (downlong && buttonpulse)) && dv.edit && (ch->p3Temp > 0)) {
                    ch->p3Temp--;

                    if (ch->p3Temp <= ch->p2Temp) ch->p3Temp = ch->p2Temp + 1;
                  }
                  break;
                case 17:  //p3Duty
                  if ((up.flHigh() || (uplong && buttonpulse)) && dv.edit) {
                    ch->p3Duty++;
                    if (ch->p3Duty > 100) ch->p3Duty = 100;
                  }
                  if ((down.flHigh() || (downlong && buttonpulse)) && dv.edit && (ch->p3Duty > 0)) {
                    ch->p3Duty--;

                    if (ch->p3Duty <= ch->p2Duty) ch->p3Duty = ch->p2Duty + 1;
                  }
                  break;
                case 18:  //Duty Hysteresis
                  if ((up.flHigh() || (uplong && buttonpulse)) && dv.edit) {
                    ch->dutyHysteresis++;
                    if (ch->dutyHysteresis > 30) ch->dutyHysteresis = 30;
                  }
                  if ((down.flHigh() || (downlong && buttonpulse)) && dv.edit && (ch->dutyHysteresis > 0)) {
                    ch->dutyHysteresis--;
                  }
                  break;
                case 19:  //pmaxAlertTemp
                  if ((up.flHigh() || (uplong && buttonpulse)) && dv.edit) {
                    ch->pmaxAlertTemp++;
                    if (ch->pmaxAlertTemp > 1000) ch->pmaxAlertTemp = 1000;
                  }
                  if ((down.flHigh() || (downlong && buttonpulse)) && dv.edit && (ch->pmaxAlertTemp > 0)) {
                    ch->pmaxAlertTemp--;
                  }
                  break;
                case 20:  //enableMaxTempAlert
                  if (up.flHigh() && dv.edit) {
                    ch->enableMaxTempAlert = true;
                  }
                  if (down.flHigh() && dv.edit) {

                    ch->enableMaxTempAlert = false;
                  }
                  break;
                case 21:  //enableMinTempAlert
                  if (up.flHigh() && dv.edit) {
                    ch->enableMinTempAlert = true;
                  }
                  if (down.flHigh() && dv.edit) {

                    ch->enableMinTempAlert = false;
                  }
                  break;
                case 22:  //enableNoRPMAlert
                  if (up.flHigh() && dv.edit) {
                    ch->enableNoRPMAlert = true;
                  }
                  if (down.flHigh() && dv.edit) {

                    ch->enableNoRPMAlert = false;
                  }
                  break;
              }
              break;
          }
          break;
      }

      if ((uplong || downlong) && buttonpulse) changed = true;

      break;
  }  //displayMode

  //detect if infomessage needs to disappear (refresh display)
  if (dv.showinfotxt) {
    if (millis() - dv.Lastinfotxt > 700) {
      changed = true;
      dv.showinfotxt = false;
    }
  }

  //menu/channel display logic
  if (changed) {
    char buf[5];
    lcd.cls();
    switch (dv.displayMode) {
      case 0:  //status

        switch (generalSettings.statusMode) {
          case 0:  //autoswitch
            switch (dv.ch) {
              case 1:
                if (!ch1.display()) dv.ch++;  //ch1 always enabled
                break;
              case 2:
                if (!ch2.display()) dv.ch++;
                break;
              case 3:
                if (!ch3.display()) {
                  dv.ch = 1;
                  ch1.display();
                }
                break;
            }
            break;
          case 1:  //manual ch1
            ch1.display();
            break;
          case 2:  //manual ch2
            ch2.display();
            break;
          case 3:  //manual ch3
            ch3.display();
            break;

        }  //statusMode
        break;
      case 1:  //setup
        lcd.locate(0, 0);
        lcd.print("SETUP");

        switch (dv.menu0) {
          case 0:
            lcd.locate(0, 9);
            lcd.write(0);  //CUSTOMup
            lcd.locate(1, 9);
            lcd.write(1);  //CUSTOMdown
            lcd.locate(1, 0);
            lcd.print("SAVE");
            lcd.locate(2, 9);
            lcd.print(">");
            lcd.locate(3, 9);
            lcd.print(" ");
            if (dv.level > 0) {

              switch (dv.menu1) {
                case 0:  //save cmd
                  lcd.locate(0, 9);
                  lcd.write(0);  //CUSTOMup
                  lcd.locate(1, 9);
                  lcd.write(2);  //CUSTOMenter
                  lcd.locate(2, 0);
                  lcd.print("ALL");
                  if (dv.edit) {
                    lcd.locate(0, 9);
                    lcd.write(0);  //CUSTOMup
                    lcd.locate(1, 9);
                    lcd.print(" ");
                    lcd.locate(2, 9);
                    lcd.print("!");

                    lcd.locate(3, 0);
                    lcd.print("DO SAVE!");
                  }
                  break;
              }
            }

            break;
          case 1:
            lcd.locate(0, 9);
            lcd.write(0);  //CUSTOMup
            lcd.locate(1, 9);
            lcd.write(1);  //CUSTOMdown
            lcd.locate(1, 0);
            lcd.print("GENERAL");
            lcd.locate(2, 9);
            lcd.print(">");
            lcd.locate(3, 9);
            lcd.print("<");
            if (dv.level > 0) {

              switch (dv.menu1) {

                case 0:  //AutoSwitchTime
                  lcd.locate(0, 9);
                  lcd.write(0);  //CUSTOMup
                  lcd.locate(1, 9);
                  lcd.write(2);  //CUSTOMenter
                  lcd.locate(2, 0);
                  lcd.print("AutoSwT");
                  lcd.locate(3, 0);
                  lcd.print(generalSettings.autoswitchtime);
                  lcd.print(" ms");

                  lcd.locate(2, 9);
                  lcd.print(">");
                  lcd.locate(3, 9);
                  lcd.print(" ");
                  if (dv.edit) {
                    lcd.locate(0, 9);
                    lcd.write(0);  //CUSTOMup
                    lcd.locate(1, 9);
                    lcd.print(" ");

                    lcd.locate(2, 9);
                    lcd.print("+");
                    lcd.locate(3, 9);
                    lcd.print("-");
                  }
                  break;

                case 1:  //upsideDown
                  lcd.locate(0, 9);
                  lcd.write(0);  //CUSTOMup
                  lcd.locate(1, 9);
                  lcd.write(2);  //CUSTOMenter
                  lcd.locate(2, 0);
                  lcd.print("upsideDwn");
                  lcd.locate(3, 0);

                  if (generalSettings.upsideDown) lcd.print("ON");
                  else lcd.print("OFF");
                  lcd.locate(2, 9);
                  lcd.print(" ");
                  lcd.locate(3, 9);
                  lcd.print("<");

                  if (dv.edit) {
                    lcd.locate(0, 9);
                    lcd.write(0);  //CUSTOMup
                    lcd.locate(1, 9);
                    lcd.print(" ");

                    lcd.locate(2, 9);
                    lcd.print("1");
                    lcd.locate(3, 9);
                    lcd.print("0");
                  }
                  break;
              }
            }

            break;
          case 2 ... 4:
            switch (dv.menu0) {
              case 2: ch = &ch1; break;
              case 3: ch = &ch2; break;
              case 4: ch = &ch3; break;
            }

            lcd.locate(0, 9);
            lcd.write(0);  //CUSTOMup
            lcd.locate(1, 9);
            lcd.write(1);  //CUSTOMdown

            lcd.locate(1, 0);
            lcd.print("CH ");
            lcd.locate(1, 3);
            lcd.print(ch->chNr);
            lcd.locate(2, 9);
            if (dv.menu0 < 4) {
              lcd.print(">");
            } else {
              lcd.print(" ");
            }
            lcd.locate(3, 9);
            lcd.print("<");
            if (dv.level > 0) {

              lcd.locate(1, 9);
              lcd.write(2);  //CUSTOMenter
              lcd.locate(2, 9);
              if (dv.menu1 < 22) lcd.print(">");
              else lcd.print(" ");
              lcd.locate(3, 9);
              if (dv.menu1 > 0) lcd.print("<");
              else lcd.print(" ");

              lcd.locate(2, 0);
              switch (dv.menu1) {

                case 0:  //Enable


                  lcd.print("EnableCH");
                  lcd.locate(3, 0);
                  if (ch->enabled) lcd.print("ON");
                  else lcd.print("OFF");
                  if (dv.edit) {
                    lcd.locate(1, 9);
                    lcd.print(" ");
                    lcd.locate(2, 9);
                    lcd.print("1");
                    lcd.locate(3, 9);
                    lcd.print("0");
                  }
                  break;
                case 1:  //Enable PWM

                  lcd.print("EnablePWM");
                  lcd.locate(3, 0);
                  if (ch->enablePWM) lcd.print("ON");
                  else lcd.print("OFF");
                  if (dv.edit) {
                    lcd.locate(1, 9);
                    lcd.print(" ");
                    lcd.locate(2, 9);
                    lcd.print("1");
                    lcd.locate(3, 9);
                    lcd.print("0");
                  }
                  break;
                case 2:  //Enable Tacho RPM Counter

                  lcd.print("EnableRPM");
                  lcd.locate(3, 0);
                  if (ch->enableTacho) lcd.print("ON");
                  else lcd.print("OFF");
                  if (dv.edit) {
                    lcd.locate(1, 9);
                    lcd.print(" ");
                    lcd.locate(2, 9);
                    lcd.print("1");
                    lcd.locate(3, 9);
                    lcd.print("0");
                  }
                  break;
                case 3:  //temp_THERMISTORNOMINAL

                  lcd.print("THERNOM");
                  lcd.locate(3, 0);
                  lcd.print(ch->temp_THERMISTORNOMINAL);
                  lcd.print(" ");
                  lcd.print((char)CHARohm);
                  if (dv.edit) {
                    lcd.locate(1, 9);
                    lcd.print(" ");
                    lcd.locate(2, 9);
                    lcd.print("+");
                    lcd.locate(3, 9);
                    lcd.print("-");
                  }
                  break;
                case 4:  //temp_TEMPERATURENOMINAL

                  lcd.print("TEMPNOM");
                  lcd.locate(3, 0);
                  lcd.print(ch->temp_TEMPERATURENOMINAL);
                  lcd.print(" ");
                  lcd.print((char)CHARgrad);

                  lcd.print("C");
                  if (dv.edit) {
                    lcd.locate(1, 9);
                    lcd.print(" ");
                    lcd.locate(2, 9);
                    lcd.print("+");
                    lcd.locate(3, 9);
                    lcd.print("-");
                  }
                  break;
                case 5:  //temp_SERIESRESISTOR

                  lcd.print("SERIES R");
                  lcd.locate(3, 0);
                  lcd.print(ch->temp_SERIESRESISTOR);
                  lcd.print(" ");
                  lcd.print((char)CHARohm);
                  if (dv.edit) {
                    lcd.locate(1, 9);
                    lcd.print(" ");
                    lcd.locate(2, 9);
                    lcd.print("+");
                    lcd.locate(3, 9);
                    lcd.print("-");
                  }
                  break;
                case 6:  //temp_BCOEFFICIENT

                  lcd.print("COEFF B");
                  lcd.locate(3, 0);
                  lcd.print(ch->temp_BCOEFFICIENT);

                  if (dv.edit) {
                    lcd.locate(1, 9);
                    lcd.print(" ");
                    lcd.locate(2, 9);
                    lcd.print("+");
                    lcd.locate(3, 9);
                    lcd.print("-");
                  }
                  break;
                case 7:  //channel name string edit

                  lcd.print("CH NAME");
                  lcd.locate(3, 0);
                  lcd.write(ch->chName, 8);
                  if (dv.edit) {
                    lcd.locate(1, 9);
                    lcd.print(" ");
                    lcd.locate(2, 9);
                    lcd.print("+");
                    lcd.locate(3, 9);
                    lcd.print("-");
                    lcd.locate(3, dv.stringedit);

                    //lcd.print("_");
                  }
                  break;
                case 8:  //enableAutoPWM

                  lcd.print("AutoPWM");
                  lcd.locate(3, 6);

                  if (ch->enableAutoPWM) lcd.print("ON");
                  else lcd.print("OFF");
                  if (dv.edit) {
                    lcd.locate(1, 9);
                    lcd.print(" ");
                    lcd.locate(2, 9);
                    lcd.print("1");
                    lcd.locate(3, 9);
                    lcd.print("0");
                  }
                  break;
                case 9:  //manualDuty

                  lcd.print("man.PWM");
                  lcd.locate(3, 0);
                  lcd.print(ch->manualDuty);

                  lcd.print(" %");
                  if (dv.edit) {
                    lcd.locate(1, 9);
                    lcd.print(" ");
                    lcd.locate(2, 9);
                    lcd.print("+");
                    lcd.locate(3, 9);
                    lcd.print("-");
                  }
                  break;
                case 10:  //minDuty

                  lcd.print("minPWM");
                  lcd.locate(3, 0);
                  lcd.print(ch->minDuty);

                  lcd.print(" %");
                  if (dv.edit) {
                    lcd.locate(1, 9);
                    lcd.print(" ");
                    lcd.locate(2, 9);
                    lcd.print("+");
                    lcd.locate(3, 9);
                    lcd.print("-");
                  }
                  break;
                case 11:  //maxDuty

                  lcd.print("maxPWM");
                  lcd.locate(3, 0);
                  lcd.print(ch->maxDuty);

                  lcd.print(" %");
                  if (dv.edit) {
                    lcd.locate(1, 9);
                    lcd.print(" ");
                    lcd.locate(2, 9);
                    lcd.print("+");
                    lcd.locate(3, 9);
                    lcd.print("-");
                  }
                  break;
                case 12:  //p1Temp

                  lcd.print("p1Temp");
                  lcd.locate(3, 0);

                  lcd.print(dtostrf((float)ch->p1Temp / 10, 2, 1, buf));
                  lcd.print(" ");
                  lcd.print((char)CHARgrad);
                  lcd.print("C");
                  if (dv.edit) {
                    lcd.locate(1, 9);
                    lcd.print(" ");
                    lcd.locate(2, 9);
                    lcd.print("+");
                    lcd.locate(3, 9);
                    lcd.print("-");
                  }
                  break;
                case 13:  //p1Duty

                  lcd.print("p1PWM");
                  lcd.locate(3, 0);
                  lcd.print(ch->p1Duty);

                  lcd.print(" %");
                  if (dv.edit) {
                    lcd.locate(1, 9);
                    lcd.print(" ");
                    lcd.locate(2, 9);
                    lcd.print("+");
                    lcd.locate(3, 9);
                    lcd.print("-");
                  }
                  break;

                case 14:  //p2Temp

                  lcd.print("p2Temp");
                  lcd.locate(3, 0);

                  lcd.print(dtostrf((float)ch->p2Temp / 10, 2, 1, buf));
                  lcd.print(" ");
                  lcd.print((char)CHARgrad);
                  lcd.print("C");
                  if (dv.edit) {
                    lcd.locate(1, 9);
                    lcd.print(" ");
                    lcd.locate(2, 9);
                    lcd.print("+");
                    lcd.locate(3, 9);
                    lcd.print("-");
                  }
                  break;
                case 15:  //p2Duty

                  lcd.print("p2PWM");
                  lcd.locate(3, 0);
                  lcd.print(ch->p2Duty);

                  lcd.print(" %");
                  if (dv.edit) {
                    lcd.locate(1, 9);
                    lcd.print(" ");
                    lcd.locate(2, 9);
                    lcd.print("+");
                    lcd.locate(3, 9);
                    lcd.print("-");
                  }
                  break;

                case 16:  //p3Temp

                  lcd.print("p3Temp");
                  lcd.locate(3, 0);

                  lcd.print(dtostrf((float)ch->p3Temp / 10, 2, 1, buf));
                  lcd.print(" ");
                  lcd.print((char)CHARgrad);
                  lcd.print("C");
                  if (dv.edit) {
                    lcd.locate(1, 9);
                    lcd.print(" ");
                    lcd.locate(2, 9);
                    lcd.print("+");
                    lcd.locate(3, 9);
                    lcd.print("-");
                  }
                  break;
                case 17:  //p3Duty

                  lcd.print("p3PWM");
                  lcd.locate(3, 0);
                  lcd.print(ch->p3Duty);

                  lcd.print(" %");
                  if (dv.edit) {
                    lcd.locate(1, 9);
                    lcd.print(" ");
                    lcd.locate(2, 9);
                    lcd.print("+");
                    lcd.locate(3, 9);
                    lcd.print("-");
                  }
                  break;
                case 18:  //duty hysteresis %

                  lcd.print("PWM HYST.");
                  lcd.locate(3, 0);
                  if (ch->dutyHysteresis > 1) {
                    lcd.print(ch->dutyHysteresis);
                    lcd.print(" %");
                  } else
                    lcd.print("OFF");

                  if (dv.edit) {
                    lcd.locate(1, 9);
                    lcd.print(" ");
                    lcd.locate(2, 9);
                    lcd.print("+");
                    lcd.locate(3, 9);
                    lcd.print("-");
                  }
                  break;
                case 19:  //pmaxAlertTemp

                  lcd.print("maxT.ALRT");
                  lcd.locate(3, 0);

                  lcd.print(dtostrf((float)ch->pmaxAlertTemp / 10, 2, 1, buf));
                  lcd.print(" ");
                  lcd.print((char)CHARgrad);
                  lcd.print("C");
                  if (dv.edit) {
                    lcd.locate(1, 9);
                    lcd.print(" ");
                    lcd.locate(2, 9);
                    lcd.print("+");
                    lcd.locate(3, 9);
                    lcd.print("-");
                  }
                  break;
                case 20:  //enableMaxTempAlert

                  lcd.print("maxT.ALRT");
                  lcd.locate(3, 6);

                  if (ch->enableMaxTempAlert) lcd.print("ON");
                  else lcd.print("OFF");
                  if (dv.edit) {
                    lcd.locate(1, 9);
                    lcd.print(" ");
                    lcd.locate(2, 9);
                    lcd.print("1");
                    lcd.locate(3, 9);
                    lcd.print("0");
                  }
                  break;
                case 21:  //enableMinTempAlert

                  lcd.print("minT.ALRT");
                  lcd.locate(3, 6);

                  if (ch->enableMinTempAlert) lcd.print("ON");
                  else lcd.print("OFF");
                  if (dv.edit) {
                    lcd.locate(1, 9);
                    lcd.print(" ");
                    lcd.locate(2, 9);
                    lcd.print("1");
                    lcd.locate(3, 9);
                    lcd.print("0");
                  }
                  break;
                case 22:  //enableMinTempAlert

                  lcd.print("RPM-ALERT");
                  lcd.locate(3, 6);

                  if (ch->enableNoRPMAlert) lcd.print("ON");
                  else lcd.print("OFF");
                  if (dv.edit) {
                    lcd.locate(1, 9);
                    lcd.print(" ");
                    lcd.locate(2, 9);
                    lcd.print("1");
                    lcd.locate(3, 9);
                    lcd.print("0");
                  }
                  break;
              }
            }

            break;
        }


        break;
    }  //displayMode


    //button debug output
    /*
    if (left.isPressed()) {
      lcd.print('L');
      if (left.longPressed()) lcd.print('+');
    }
    if (right.isPressed()) {
      lcd.print('R');
      if (right.longPressed()) lcd.print('+');
    }
    if (up.isPressed()) {
      lcd.print('U');
      if (up.longPressed()) lcd.print('+');
    }
    if (down.isPressed()) {
      lcd.print('D');
      if (down.longPressed()) lcd.print('+');
    }
    */


    //print infotext bottom
    if (dv.showinfotxt) {

      lcd.locate(3, 0);
      lcd.print(dv.infotxt);
    }


    changed = false;
  }
}