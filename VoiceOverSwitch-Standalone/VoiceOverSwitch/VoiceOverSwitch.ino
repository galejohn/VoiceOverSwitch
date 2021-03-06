/*********************************************************************
VoiceOver BLE Switch for nRF52 Feather and OLED wing
by John Gale
Reads switch inputs, sends iOS VoiceOver BLE keyboard shortcuts

based on:
AtariFruit 2600 Joystick
by John Park for Adafruit
https://learn.adafruit.com/atarifruit-2600-joystick/overview

*********************************************************************/
#include <bluefruit.h>

/* OLED wing */
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

Adafruit_SSD1306 display = Adafruit_SSD1306();

#if (SSD1306_LCDHEIGHT != 32)
 #error("Height incorrect, please fix Adafruit_SSD1306.h!");
#endif

#define BUTTON_A 31
#define BUTTON_B 30
#define BUTTON_C 27
#define LED_PIN 17

BLEDis bledis;
BLEHidAdafruit blehid;

#define VERSION "0.0"
#define LED_ON
#define DI_PIN_COUNT 6
#define MODE_COUNT 4

byte mode = 0;

byte diPins[DI_PIN_COUNT] =  {A0, A1, A2, A3, A4, A5};

byte m0KeyCodes[DI_PIN_COUNT] = {0x0B, 0x2C, 0x2D, 0x4F, 0x50, 0x04}; 

String m0KeyLabels[] = {"Home", "Tap", "2Tap", "Next", "Prev", "Read"};

String modeLabels[] = {"VO", "Test1", "Test2", "Test3"};

/* We want a very short debounce delay */
#define DI_DEBOUNCE_MS 10 // 100ms (1/10th second)

#define LEDBLINK_MS 1000


/* For I/O pin status and debounce. */
unsigned int  digitalStatus[DI_PIN_COUNT];          // Last set PIN mode.
unsigned long digitalDebounceTime[DI_PIN_COUNT];    // Debounce time.
//unsigned long digitalCounter[DI_PIN_COUNT];       // Times button pressed.
unsigned int  digitalDebounceRate = DI_DEBOUNCE_MS; // Debounce rate.

/* For the blinking LED (heartbeat). */
unsigned int  ledStatus = LOW;             // Last set LED mode.
unsigned long ledBlinkTime = 0;            // LED blink time.
unsigned int  ledBlinkRate = LEDBLINK_MS;  // LED blink rate.

unsigned int pinsOn = 0;


void setup()
{
  // generate OLED high voltage from 3.3v line internally! (neat!)
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3C (for the 128x32)
  display.display();
  display.setTextSize(1);
  display.setTextColor(WHITE);
    
  // Initialize the serial port.
  Serial.begin(115200);
  Serial.println("OLED begun");
  
  Serial.println("Go to your Bluetooth settings to pair device");
  // Clear the OLED
  display.clearDisplay();
  display.setCursor(0,0);
  display.println("Go to your");
  display.println("Bluetooth settings");
  display.println("to pair device");
  display.display();
  
  Bluefruit.begin();
  // Set max power. Accepted values are: -40, -30, -20, -16, -12, -8, -4, 0, 4
  Bluefruit.setTxPower(4);
  Bluefruit.setName("VoiceOverSwitch");

  // Configure and Start Device Information Service
  bledis.setManufacturer("SpokaneSchoolsAT");
  bledis.setModel("VoiceOverSwitch");
  bledis.begin();

  /* Start BLE HID
   * Note: Apple requires BLE device must have min connection interval >= 20m
   * ( The smaller the connection interval the faster we could send data).
   * However for HID and MIDI device, Apple could accept min connection interval
   * up to 11.25 ms. Therefore BLEHidAdafruit::begin() will try to set the min and max
   * connection interval to 11.25  ms and 15 ms respectively for best performance.
   */
  blehid.begin();

  /* Set connection interval (min, max) to your perferred value.
   * Note: It is already set by BLEHidAdafruit::begin() to 11.25ms - 15ms
   * min = 9*1.25=11.25 ms, max = 12*1.25= 15 ms
   */
  /* Bluefruit.setConnInterval(9, 12); */

  // Set up and start advertising
  startAdv();

  showHeader();

  // LOW POWER MODE!
  // Pins default to INPUT mode. To save power, turn them all to OUTPUT
  // initially, so only those being used will be turned on. See:
  // http://www.pjrc.com/teensy/low_power.html

  // Initialize the pins and digitalPin array.
  for (int thisPin=0; thisPin < DI_PIN_COUNT; thisPin++ )
  {
    // Set pin to be digital input using pullup resistor.
    pinMode(diPins[thisPin], INPUT_PULLUP);
    // Set the current initial pin status.
    digitalStatus[thisPin] = HIGH; //digitalRead(thisPin+DI_PIN_START);
    // Clear debounce time.
    digitalDebounceTime[thisPin] = 0;
    //digitalCounter[thisPin] = 0;
  }

  // Set LED pin to output, since it has an LED we can use.
  pinMode(LED_PIN, OUTPUT);
  
  Serial.println("Ready");
  display.clearDisplay();
  display.setCursor(0,0);
  display.println("Ready Mode: " + modeLabels[mode]);
  display.display();
  

}


void startAdv(void)
{
  // Advertising packet
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  Bluefruit.Advertising.addAppearance(BLE_APPEARANCE_HID_KEYBOARD);

  // Include BLE HID service
  Bluefruit.Advertising.addService(blehid);

  // There is enough room for the dev name in the advertising packet
  Bluefruit.Advertising.addName();

  /* Start Advertising
   * - Enable auto advertising if disconnected
   * - Interval:  fast mode = 20 ms, slow mode = 152.5 ms
   * - Timeout for fast mode is 30 seconds
   * - Start(timeout) with timeout = 0 will advertise forever (until connected)
   *
   * For recommended advertising interval
   * https://developer.apple.com/library/content/qa/qa1931/_index.html
   */
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);    // in unit of 0.625 ms
  Bluefruit.Advertising.setFastTimeout(30);      // number of seconds in fast mode
  Bluefruit.Advertising.start(0);                // 0 = Don't stop advertising after n seconds
}


void loop()
{
  // Tell the watchdog timer we are still alive.
  //wdt_reset();

#ifndef LED_OFF
  // LED blinking heartbeat. Yes, we are alive.
  if ( (long)(millis()-ledBlinkTime) >= 0 )
  {
    // Toggle LED.
    if (ledStatus==LOW)  // If LED is LOW...
    {
      ledStatus = HIGH;  // ...make it HIGH.
    } else {
      ledStatus = LOW;   // ...else, make it LOW.
    }
    // Set LED pin status.
    if (pinsOn==0) digitalWrite(LED_PIN, ledStatus);
    // Reset "next time to toggle" time.
    ledBlinkTime = millis()+ledBlinkRate;
  }
#endif

  // Check for serial data.
  if (Serial.available() > 0) {
    // If data ready, read a byte.
    int incomingByte = Serial.read();
    // Parse the byte we read.
    switch(incomingByte)
    {
      case '?':
        showStatus();
        break;
      default:
        break;
    }
  }


  // Loop through each Digital Input pin.
  for (int thisPin=0; thisPin < DI_PIN_COUNT; thisPin++ )
  {
    // Read the pin's current status.
    unsigned int status = digitalRead(diPins[thisPin]);

    // In pin status has changed from our last toggle...
    if (status != digitalStatus[thisPin])
    {
      blehid.keyRelease();
      // Remember when it changed, starting debounce mode.
      // If not currently in debounce mode,
      if (digitalDebounceTime[thisPin]==0)
      {
        // Set when we can accept this as valid (debounce is considered
        // done if the time gets to this point with the status still the same).
        digitalDebounceTime[thisPin] = millis()+digitalDebounceRate;
      }

      // Check to see if we are in debounce detect mode.
      if (digitalDebounceTime[thisPin]>0)
      {
        // Yes we are. Have we delayed long enough yet?
        if ( (long)(millis()-digitalDebounceTime[thisPin]) >= 0 )
        {
            // Yes, so consider it switched.
            // If pin is Active LOW,
            if (status==LOW)
            {
              // Send BUTTON PRESSED string.
              Serial.print(m0KeyLabels[thisPin]);
              Serial.print(" pressed");
              
              blehid.keyboardReport(0x05, m0KeyCodes[thisPin], 0, 0, 0, 0, 0);
              blehid.keyboardReport(0x05, 0, 0, 0, 0, 0, 0);
              //digitalCounter[thisPin]++;
              pinsOn++;
#ifndef LED_OFF
              digitalWrite(LED_PIN, HIGH);
#endif
            } else {
              // Emit BUTTON RELEASED string.
              Serial.print(m0KeyLabels[thisPin]);
              if (pinsOn>0) pinsOn--;
              if (pinsOn==0) digitalWrite(LED_PIN, LOW);
            }
            // Remember current (last set) status for this pin.
            digitalStatus[thisPin] = status;
            // Reset debounce time (disable, not looking any more).
            digitalDebounceTime[thisPin] = 0;
        } // End of if ( (long)(millis()-digitalDebounceTime[thisPin]) >= 0 )

      } // End of if (digitalDebounceTime[thisPin]>0)
    }
    else // No change? Flag no change.
    {
      // If we were debouncing, we are no longer debouncing.
      digitalDebounceTime[thisPin] = 0;
    }
  } // End of (int thisPin=0; thisPin < DI_PIN_COUNT; thisPin++ )

  // Request CPU to enter low-power mode until an event/interrupt occurs
  waitForEvent();
}

void showHeader()
{
  int i;
  // Emit some startup stuff to the serial port.
  Serial.println("VoiceOverSwitch by John Gale for Spokane Schools. ");
  Serial.println("Based on: AtariFruit by John Park for Adafruit");
  Serial.print(DI_PIN_COUNT);
  Serial.print(" DI Pins (");
  for (i=0; i<DI_PIN_COUNT; i++)
  {
    Serial.print(diPins[i]);
    Serial.print("=");
    Serial.print(m0KeyLabels[i]);
    Serial.print(" ");
  }
  Serial.print("), ");
  Serial.print(digitalDebounceRate);
  Serial.println("ms Debounce.");
}


void showStatus()
{
  showDigitalInputStatus();
}


void showDigitalInputStatus()
{
  Serial.print("DI: ");

  for (int thisPin=0; thisPin < DI_PIN_COUNT; thisPin++ )
  {
    // Read the pin's current status.
    Serial.print(m0KeyLabels[thisPin]);
    Serial.print("=");
    Serial.print(digitalRead(diPins[thisPin]));
    Serial.print(" ");
    //Serial.print(" (");
    //Serial.print(digitalCounter[thisPin]);
    //Serial.print(") ");
  }
  Serial.println("");
}

/**
 * RTOS Idle callback is automatically invoked by FreeRTOS
 * when there are no active threads. E.g when loop() calls delay() and
 * there is no bluetooth or hw event. This is the ideal place to handle
 * background data.
 *
 * NOTE: FreeRTOS is configured as tickless idle mode. After this callback
 * is executed, if there is time, freeRTOS kernel will go into low power mode.
 * Therefore waitForEvent() should not be called in this callback.
 * http://www.freertos.org/low-power-tickless-rtos.html
 *
 * WARNING: This function MUST NOT call any blocking FreeRTOS API
 * such as delay(), xSemaphoreTake() etc ... for more information
 * http://www.freertos.org/a00016.html
 */
void rtos_idle_callback(void)
{
  // Don't call any other FreeRTOS blocking API()
  // Perform background task(s) here
}
// End of file.
