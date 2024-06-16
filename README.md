# Nano Fan Controller
 An Arduino Nano PWM fan controller project

Features:
 - 3 channel processing
 - 3 Temperature sensor inputs for NTC resistor
 - 2 PWM fan tacho inputs to count rpm
 - 2 PWM fan outputs to control speed
 - Menu system to edit all important parameters and save to EEPROM function
 - 4 Buttons to edit parameters and switch views
 - Support for DOGS104 4x10 display [link](https://www.mouser.com/pdfdocs/dogs104e.PDF)
 - Configurable hysteresis
 - 3 point temp/dutycycle curve per channel
 - Over/under temperature alert
 - 0 rpm alert
 - Upside down installation possible (button and display invert can be activated in settings)
 - Channel name can be user defined
 - Manual and auto RPM mode
 - Channel display mode static or automatic switching
 - Usage of green, orange and red display backlight LEDs to visualize status
 - Usage of active beeper for alarming
 - Channel 1 and 2 have full RPM/Tacho/Temp sensor functionality. Channel 3 has only temperature sensor functionality cause lack of pins on arduino nano and lack of additional pwm timers
   

Hardware:
- AZ Delivery Arduino Nano
- EA DOGS104W-A (display)
- EA LED36X28-GR (LED backlight green+red)
- Rest of electronic components - see schematics

Used library for SSD1803A controller of the display: https://github.com/sstaub/SSD1803A_I2C

View newest schematic at: https://crcit.net/c/6659b8724b3e4b67b652c3124bdcdf9e
Screenshot "Schematics.png":
![schematics](https://github.com/denis-1337/NanoFanController/blob/main/Schematics.png)

Menu structure:
![schematics](https://github.com/denis-1337/NanoFanController/blob/main/menu_structure.png)


Menu logic:

- If the device is in display mode (just after power on), the buttons left and right switch through displaying modes (channel1, channel2, channel3, autoswitching)
- If button down is pressed, the device goes into setup menu
- In setup menu, button up will exit setup, left and right buttons navigate through the first level of menu (general, ch1, ch2, ch3)
- Every parameter can be edited by going one step deeper and using left/right buttons to change value

"Save" menu: 
- saves all settings including actual display mode into EEPROM
- navigate with down button to the end of menu (DO SAVE!) and press up button to save (!)

"General" menu settings:
- AutoSwT (ms): autoswitch time in milliseconds
- UpsideDwn: (on/off): changes upside down mode for LCD and buttons

"CHx" menu settings (only CH1 and CH2 has full capability - CH3 only uses Temperature settings):
- EnableCH (on/off): enables channel functionality (display and processing)
- EnablePWM (on/off): enables PWM output and processing
- EnableRPM (on/off): enables Tacho input
- THERNOM (Ohm): Thermistor NTC nominal resistance (default 10k)
- TEMPNOM (°C): Thermistor NTC nominal temperature (default 25°C)
- SERIES R (Ohm): Series resistor resistance (default 10k) should be measured
- COEFF B: Thermistor coefficient b (default 3976) - of NTC specification
- CH NAME: (8 chars): Name of the channel
- AutoPWM (on/off): Use automatic PWM processing with 3 point temperature to dutycycle curve or use manual dutycycle
- man.PWM (%): Dutycycle in % for PWM output in manual mode
- minPWM (%): minimum dutycycle when autoPWM mode activated
- maxPWM (%): maximum dutycycle when autoPWM mode activated
- p1Temp (°C): Temperature for curve point 1
- p1PWM (%): Dutycycle for curve point 1
- p2Temp (°C): Temperature for curve point 2
- p2PWM (%): Dutycycle for curve point 2
- p3Temp (°C): Temperature for curve point 3
- p3PWM (%): Dutycycle for curve point 3
- PWM HYST (%): Hysteresis for calculated dutycycle (0%-1% = deactivated / 2%-30% = activated)
- maxT.ALRT (°C): Alerting on maximum temperature (alerts, if temperature is bigger than this value)
- maxT.ALRT (on/off): Activates alert on maximum temperature reached
- minT.ALRT (on/off): Activates alert on underflowing minimum temperature (0°C)
- RPM-ALERT: (on/off): Activates alert on 0 RPM on tacho input
  
