
#include <EEPROM.h>

#include <Adafruit_NeoPixel.h>


#define DELAYVAL 500  // Time (in milliseconds) to pause between pixels


/*********************************************************************
 This is an example for our nRF51822 based Bluefruit LE modules

 Pick one up today in the adafruit shop!

 Adafruit invests time and resources providing this open source code,
 please support Adafruit and open-source hardware by purchasing
 products from Adafruit!

 MIT license, check LICENSE for more information
 All text above, and the splash screen below must be included in
 any redistribution
*********************************************************************/

#include <string.h>
#include <Arduino.h>
#include <SPI.h>
#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"

#include "BluefruitConfig.h"

#if SOFTWARE_SERIAL_AVAILABLE
#include <SoftwareSerial.h>
#endif

/*=========================================================================
    APPLICATION SETTINGS

    FACTORYRESET_ENABLE       Perform a factory reset when running this sketch
   
                              Enabling this will put your Bluefruit LE module
                              in a 'known good' state and clear any config
                              data set in previous sketches or projects, so
                              running this at least once is a good idea.
   
                              When deploying your project, however, you will
                              want to disable factory reset by setting this
                              value to 0.  If you are making changes to your
                              Bluefruit LE device via AT commands, and those
                              changes aren't persisting across resets, this
                              is the reason why.  Factory reset will erase
                              the non-volatile memory where config data is
                              stored, setting it back to factory default
                              values.
       
                              Some sketches that require you to bond to a
                              central device (HID mouse, keyboard, etc.)
                              won't work at all with this feature enabled
                              since the factory reset will clear all of the
                              bonding data stored on the chip, meaning the
                              central device won't be able to reconnect.
    MINIMUM_FIRMWARE_VERSION  Minimum firmware version to have some new features
    MODE_LED_BEHAVIOUR        LED activity, valid options are
                              "DISABLE" or "MODE" or "BLEUART" or
                              "HWUART"  or "SPI"  or "MANUAL"
    -----------------------------------------------------------------------*/
#define FACTORYRESET_ENABLE 0
#define MINIMUM_FIRMWARE_VERSION "0.6.6"
#define MODE_LED_BEHAVIOUR "MODE"
/*=========================================================================*/

// Create the bluefruit object, either software serial...uncomment these lines
/*
SoftwareSerial bluefruitSS = SoftwareSerial(BLUEFRUIT_SWUART_TXD_PIN, BLUEFRUIT_SWUART_RXD_PIN);

Adafruit_BluefruitLE_UART ble(bluefruitSS, BLUEFRUIT_UART_MODE_PIN,
                      BLUEFRUIT_UART_CTS_PIN, BLUEFRUIT_UART_RTS_PIN);
*/

/* ...or hardware serial, which does not need the RTS/CTS pins. Uncomment this line */
// Adafruit_BluefruitLE_UART ble(BLUEFRUIT_HWSERIAL_NAME, BLUEFRUIT_UART_MODE_PIN);

/* ...hardware SPI, using SCK/MOSI/MISO hardware SPI pins and then user selected CS/IRQ/RST */
Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

/* ...software SPI, using SCK/MOSI/MISO user-defined SPI pins and then user selected CS/IRQ/RST */
//Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_SCK, BLUEFRUIT_SPI_MISO,
//                             BLUEFRUIT_SPI_MOSI, BLUEFRUIT_SPI_CS,
//                             BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);


// A small helper
void error(const __FlashStringHelper *err) {
  Serial.println(err);
  while (1)
    ;
}

// function prototypes over in packetparser.cpp
uint8_t readPacket(Adafruit_BLE *ble, uint16_t timeout);
float parsefloat(uint8_t *buffer);
void printHex(const uint8_t *data, const uint32_t numBytes);

// the packet buffer
extern uint8_t packetbuffer[];

/**************************************************************************/
//                  VARIABLES FOR NEO-PIXEL
/**************************************************************************/

int pixelFormat = NEO_GRB + NEO_KHZ800;
uint8_t numPixels_a = 16;
uint8_t numPixels_b = 0;
int pin_a = 6;
int pin_b = 5;

const uint8_t SCENES = 2;
const uint8_t CHANNELS = 1;
const uint8_t COLOR_SETUPS = 3;
const uint8_t PIXEL_COUNT_MAX = 16;
const uint8_t CHANNEL_A = 0;
const uint8_t CHANNEL_B = 1;

uint32_t allPixelColors[SCENES][CHANNELS][COLOR_SETUPS][PIXEL_COUNT_MAX];

// Rather than declaring the whole NeoPixel object here, we just create
// a pointer for one, which we'll then allocate later...
Adafruit_NeoPixel *pixels_a;
Adafruit_NeoPixel *pixels_b;

uint8_t basicSceneMainColorGreen = 0;
uint8_t basicSceneMainColorRed = 0;
uint8_t basicSceneMainColorBlue = 150;
uint32_t basicSceneMainColor = 0;

/**************************************************************************/
//       SCENES
/**************************************************************************/

const uint8_t SCENE_NOT_SET = 255;
const uint8_t SCENE_ROLLING_FADE = 0;
const uint8_t SCENE_TEST = 1;

/**************************************************************************/
//                  VARIABLES FOR APP LOGIC
/**************************************************************************/


bool clearEeprom = false;
bool blueConnected = false;

//Color updating
bool waitingForColor = false;
uint8_t sceneToUpdate = 0;
uint8_t channelToUpdate = 0;
uint8_t colorTypeToUpdate = 0;
uint8_t beginPixelToUpdate = 0;
uint8_t endPixelToUpdate = 0;
uint32_t colorToUpdate = 0;

uint8_t currentScene = SCENE_NOT_SET;

const uint8_t SPEED_COUNT = 2;
uint16_t speedValues[SCENES][SPEED_COUNT];

/**************************************************************************/
/**************************************************************************/


/**************************************************************************/
//                  EPROM INDEXES
/**************************************************************************/

const uint16_t PIX_A = 0;
const uint16_t PIX_B = 1;




/**************************************************************************/
/*!
    @brief  Sets up the HW an the BLE module (this function is called
            automatically on startup)
*/
/**************************************************************************/
void setup(void) {
  //while (!Serial);  // required for Flora & Micro
  delay(500);

  Serial.begin(115200);
  Serial.println(F("Adafruit Bluefruit App Controller Example"));
  Serial.println(F("-----------------------------------------"));

  /* Initialise the module */
  Serial.print(F("Initialising the Bluefruit LE module: "));

  if (!ble.begin(VERBOSE_MODE)) {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }
  Serial.println(F("OK!"));

  if (FACTORYRESET_ENABLE) {
    /* Perform a factory reset to make sure everything is in a known state */
    Serial.println(F("Performing a factory reset: "));
    if (!ble.factoryReset()) {
      error(F("Couldn't factory reset"));
    }
  }


  /* Disable command echo from Bluefruit */
  ble.echo(false);

  Serial.println("Requesting Bluefruit info:");
  /* Print Bluefruit information */
  ble.info();

  Serial.println(F("Please use Adafruit Bluefruit LE app to connect in Controller mode"));
  Serial.println(F("Then activate/use the sensors, color picker, game controller, etc!"));
  Serial.println();

  ble.verbose(false);  // debug info is a little annoying after this point!



  Serial.println(F("******************************"));

  // LED Activity command is only supported from 0.6.6
  if (ble.isVersionAtLeast(MINIMUM_FIRMWARE_VERSION)) {
    // Change Mode LED Activity
    Serial.println(F("Change LED activity to " MODE_LED_BEHAVIOUR));
    ble.sendCommandCheckOK("AT+HWModeLED=" MODE_LED_BEHAVIOUR);
  }

  // Set Bluefruit to DATA mode
  Serial.println(F("Switching to DATA mode!"));
  ble.setMode(BLUEFRUIT_MODE_DATA);

  Serial.println(F("******************************"));


  Serial.print("EEPROM length: ");

  Serial.println(EEPROM.length());

  if (clearEeprom) {
    clearMem();
    setColorDefaults();
    setSpeedDefaults();
    setEpromFromVars();
  }



  setVarsFromEprom();

  // Then create a new NeoPixel object dynamically with these values:
  pixels_a = new Adafruit_NeoPixel(numPixels_a, pin_a, pixelFormat);
  pixels_b = new Adafruit_NeoPixel(numPixels_b, pin_b, pixelFormat);

  // Going forward from here, code works almost identically to any other
  // NeoPixel example, but instead of the dot operator on function calls
  // (e.g. pixels.begin()), we instead use pointer indirection (->) like so:
  pixels_a->begin();  // INITIALIZE NeoPixel strip object (REQUIRED)
  pixels_b->begin();
  // You'll see more of this in the loop() function below.
  pixels_a->setBrightness(50);

  uint8_t testConvert = charToHex('9');
  Serial.print("TestConvert: ");
  Serial.println(testConvert);
}

/**************************************************************************/
/*!
    @brief  Constantly poll for new command or response data
*/
/**************************************************************************/
void loop(void) {

  unsigned long timeNowInLoop = millis();

  blueConnected = ble.isConnected();

  if (blueConnected) {
    checkForBlue();
  }


  bool doReset = false;
  if (currentScene == SCENE_NOT_SET) {
    Serial.println("Scene not set");
    doReset = true;
    pixels_a->clear();  // Set all pixel colors to 'off'
    currentScene = SCENE_TEST;
  }

  processScene(timeNowInLoop, currentScene, doReset);
  doReset = false;

  delay(5);
}

void processScene(unsigned long timeNow, uint8_t scene, bool doReset) {
  //Serial.println("Process scene");
  if (scene == SCENE_TEST) {
    uint32_t testC = Adafruit_NeoPixel::Color(255, 0, 0);
    testScene(basicSceneMainColor, 50, timeNow, doReset);
  }
  if (scene == SCENE_ROLLING_FADE) {
    uint32_t startC = Adafruit_NeoPixel::Color(255, 0, 0);
    uint32_t endC = Adafruit_NeoPixel::Color(0, 255, 0);
    rollingCrossfade(startC, endC, 10, timeNow, doReset);
  }
}

void testScene(const uint32_t testColor, unsigned long speed, unsigned long timeNow, bool reset) {

  static unsigned long localTime;
  static uint8_t currentCounter;
  static uint8_t currentPixel;
  bool advancePixel;
  if (reset) {
    Serial.println("StartingTestScene");
    pixels_a->clear();  // Set all pixel colors to 'off'
    currentCounter = 0;
    currentPixel = 0;
  }
  if (timeNow - localTime > speed) {
    localTime = timeNow;
    if (currentPixel == numPixels_a) {
      currentPixel = 0;
      pixels_a->clear();
      //Serial.println("Back to start");
    }

    uint32_t colorToSet = allPixelColors[SCENE_TEST][CHANNEL_A][0][currentPixel];
    //Serial.print("TestColor: ");
    //Serial.println(colorToSet);
    //pixels_a->setPixelColor(currentPixel, testColor);
    //pixels_a->show();  // Send the updated pixel colors to the hardware.
    testSetColor(*pixels_a, currentPixel, colorToSet);
    currentPixel++;
  }
}

void testSetColor(Adafruit_NeoPixel &neo, uint8_t pix, uint32_t col) {
  neo.setPixelColor(pix, col);
  neo.show();
}

void checkForBlue() {

  /* Wait for new data to arrive */
  uint8_t len = readPacket(&ble, BLE_READPACKET_TIMEOUT);
  //if (len == 0) return;

  /* Got a packet! */
  // printHex(packetbuffer, len);


  // program mode command
  if (packetbuffer[1] == 'p') {
    Serial.print("Program mode command: ");
    uint8_t progcmd = packetbuffer[2];
    Serial.print((char)progcmd);
    if (packetbuffer[2] == 'c') {
      //Update Color
      // !pcsctbe program,  scene, channel, type, beginPixel, endPixel
      // !pc10055
      waitingForColor = true;
      sceneToUpdate = charToHex(packetbuffer[3]);
      channelToUpdate = charToHex(packetbuffer[4]);
      colorTypeToUpdate = charToHex(packetbuffer[5]);
      beginPixelToUpdate = charToHex(packetbuffer[6]);
      endPixelToUpdate = charToHex(packetbuffer[7]);
      Serial.print('-');
      Serial.print(sceneToUpdate, DEC);
      Serial.print('-');
      Serial.print(channelToUpdate, DEC);
      Serial.print('-');
      Serial.print(colorTypeToUpdate, DEC);
      Serial.print('-');
      Serial.print(beginPixelToUpdate, DEC);
      Serial.print('-');
      Serial.print(endPixelToUpdate, DEC);
    }
  }


  // Color
  if (packetbuffer[1] == 'C') {
    uint8_t red = packetbuffer[2];
    uint8_t green = packetbuffer[3];
    uint8_t blue = packetbuffer[4];
    Serial.print("RGB #");
    if (red < 0x10) Serial.print("0");
    Serial.print(red, HEX);
    if (green < 0x10) Serial.print("0");
    Serial.print(green, HEX);
    if (blue < 0x10) Serial.print("0");
    Serial.println(blue, HEX);
    if (waitingForColor) {
      for (uint8_t px = beginPixelToUpdate; px <= endPixelToUpdate; px++) {
        uint32_t newColor = Adafruit_NeoPixel::Color(red, green, blue);
        printColor(newColor);
        allPixelColors[sceneToUpdate][channelToUpdate][colorTypeToUpdate][px] = newColor;
      }
      waitingForColor = false;
      setEpromFromVars();
    }
  }

  // Buttons
  if (packetbuffer[1] == 'B') {
    uint8_t buttnum = packetbuffer[2] - '0';
    boolean pressed = packetbuffer[3] - '0';
    Serial.print("Button ");
    Serial.print(buttnum);
    if (pressed) {
      Serial.println(" pressed");
    } else {
      Serial.println(" released");
    }
  }
}

void clearMem() {
  for (int i = 0; i < EEPROM.length(); i++) {
    EEPROM.write(i, 0);
  }
}


void setColorDefaults() {
  Serial.println("");
  Serial.println("SET_COLOR_DEFAULTS *******");

  for (uint8_t scn = 0; scn < SCENES; scn++) {
    for (uint8_t ch = 0; ch < CHANNELS; ch++) {
      uint8_t pxCt = numPixels_a;
      if (ch == CHANNEL_B) {
        pxCt = numPixels_b;
      }
      for (uint8_t cs = 0; cs < COLOR_SETUPS; cs++) {
        uint32_t col = Adafruit_NeoPixel::Color(255, 0, 0);
        if (scn == SCENE_TEST) {
          col = Adafruit_NeoPixel::Color(0, 0, 255);
        }
        if (cs == 1) {
          col = Adafruit_NeoPixel::Color(0, 255, 0);
        }
        Serial.print("SetColor: ");
        printColor(col);
        Serial.print(" hexCol: ");
        Serial.println(col, HEX);
        Serial.println("");
        for (uint8_t px = 0; px < pxCt; px++) {
          if (scn == SCENE_TEST && px > 7) {
            col = 0xFF0000;
          }
          allPixelColors[scn][ch][cs][px] = col;
          if (scn == SCENE_TEST) {  // && ch == CHANNEL_A && cs == 0) {
            Serial.print("scene: ");
            Serial.print(scn, DEC);
            Serial.print("; channel: ");
            Serial.print(ch, DEC);
            Serial.print("; color type: ");
            Serial.print(cs, DEC);
            Serial.print("; pixel: ");
            Serial.print(px, DEC);
            Serial.print("; Color: ");
            printColor(allPixelColors[scn][ch][cs][px]);
            Serial.println("");
          }
        }
      }
    }
  }
}

void setSpeedDefaults() {
  speedValues[SCENE_TEST][0] = 50;
  speedValues[SCENE_ROLLING_FADE][0] = 10;
}


uint32_t crossFadeValue(const uint32_t startColor, const uint32_t endColor, int step) {
  uint32_t sc = startColor;
  uint32_t ec = endColor;
  int s = step;
  if (step > 127) {
    sc = endColor;
    ec = startColor;
    s = step - 128;
  }

  byte startRed = 0;
  byte startGreen = 0;
  byte startBlue = 0;
  convertHexToRgb(sc, startRed, startGreen, startBlue);

  byte endRed = 0;
  byte endGreen = 0;
  byte endBlue = 0;
  convertHexToRgb(ec, endRed, endGreen, endBlue);

  byte red = map(s, 0, 127, startRed, endRed);
  byte green = map(s, 0, 127, startGreen, endGreen);
  byte blue = map(s, 0, 127, startBlue, endBlue);

  // Serial.print("Step");
  // Serial.print(s);
  // Serial.print(" Red");
  // Serial.print(red);
  // Serial.print(" Green");
  // Serial.print(green);
  // Serial.print(" Blue");
  // Serial.println(blue);

  // Serial.print(" Red");
  // Serial.print(startRed);
  // Serial.print(" Green");
  // Serial.print(startGreen);
  // Serial.print(" Blue");
  // Serial.println(startBlue);

  unsigned long RGB = ((unsigned long)red << 16L) | ((unsigned long)green << 8L) | (unsigned long)blue;
  return RGB;
}


bool transitionSingle(Adafruit_NeoPixel &neo, uint16_t pixel, const uint32_t startColor, const uint32_t endColor, bool reset) {
  static uint8_t step;
  if (reset) {
    step = 0;
    Serial.println("Reset Transition");
  }
  uint32_t fadeVal = crossFadeValue(startColor, endColor, step);
  neo.setPixelColor(pixel, fadeVal);
  neo.show();
  if (step == 128) {
    step = 0;
    Serial.println("Transition return true");
    return true;
  }
  step++;
  return false;
}


void rollingCrossfade(const uint32_t startColor, const uint32_t endColor, unsigned long speed, unsigned long timeNow, bool reset) {

  static unsigned long localTime;
  static uint8_t currentCounter;
  static uint8_t currentPixel;
  static bool doReverse;
  bool advancePixel;
  uint32_t sc;
  uint32_t ec;

  if (reset) {
    Serial.println("rollingCrossfade");
    currentCounter = 0;
    currentPixel = 0;
    pixels_a->clear();
    doReverse = false;
  }

  sc = startColor;
  ec = endColor;
  if (doReverse) {
    sc = endColor;
    ec = startColor;
  }

  if (timeNow - localTime > speed) {
    localTime = timeNow;
    if (currentCounter == (numPixels_b + numPixels_a)) {
      currentPixel = 0;
      currentCounter = 0;
      doReverse = !doReverse;
      Serial.println("Reversing");
    }
    if (currentCounter < numPixels_a) {
      advancePixel = transitionSingle(*pixels_a, currentPixel, sc, ec, reset);
    } else {
      advancePixel = transitionSingle(*pixels_b, currentPixel, sc, ec, reset);
    }
    if (advancePixel) {
      if (currentCounter < numPixels_a) {
        currentCounter++;
        currentPixel = currentCounter;
      } else {
        currentPixel = currentCounter - numPixels_a;
        currentCounter++;
      }
    }
  }
}


void setVarsFromEprom() {
  numPixels_a = EEPROM.read(PIX_A);
  numPixels_b = EEPROM.read(PIX_B);

  uint16_t eepromIdx = 2;

  for (uint8_t scn = 0; scn < SCENES; scn++) {
    for (uint8_t spcnt = 0; spcnt < SPEED_COUNT; spcnt++) {
      speedValues[scn][spcnt] = EEPROM.read(eepromIdx);
      eepromIdx++;
    }
  }

  for (uint8_t scn = 0; scn < SCENES; scn++) {
    for (uint8_t ch = 0; ch < CHANNELS; ch++) {
      uint8_t pxCt = numPixels_a;
      if (ch == CHANNEL_B) {
        pxCt = numPixels_b;
      }
      for (uint8_t cs = 0; cs < COLOR_SETUPS; cs++) {
        for (uint8_t px = 0; px < pxCt; px++) {
          allPixelColors[scn][ch][cs][px] = getEepromColor(eepromIdx);

          // Serial.println("*******");
          // Serial.print("scene: ");
          // Serial.print(scn, DEC);
          // Serial.print(" channel: ");
          // Serial.print(ch, DEC);
          // Serial.print(" color type: ");
          // Serial.print(cs, DEC);
          // Serial.print(" pixel: ");
          // Serial.print(px, DEC);


          // Serial.print(" EEidx: ");
          // Serial.print(eepromIdx);
          // Serial.print("Color: ");
          // printColor(allPixelColors[scn][ch][cs][px]);
          //eepromIdx++;
        }
      }
    }
  }
}

void setEpromFromVars() {
  EEPROM.update(PIX_A, numPixels_a);
  EEPROM.update(PIX_B, numPixels_b);

  uint16_t eepromIdx = 2;

  for (uint8_t scn = 0; scn < SCENES; scn++) {
    for (uint8_t spcnt = 0; spcnt < SPEED_COUNT; spcnt++) {
      EEPROM.update(eepromIdx, speedValues[scn][spcnt]);
      eepromIdx++;
    }
  }

  //Serial.println("");
  //Serial.println("SET_EEPROM_FROM_VARS *******");

  for (uint8_t scn = 0; scn < SCENES; scn++) {
    for (uint8_t ch = 0; ch < CHANNELS; ch++) {
      uint8_t pxCt = numPixels_a;
      if (ch == CHANNEL_B) {
        pxCt = numPixels_b;
      }
      for (uint8_t cs = 0; cs < COLOR_SETUPS; cs++) {
        for (uint8_t px = 0; px < pxCt; px++) {
          setEepromColor(eepromIdx, allPixelColors[scn][ch][cs][px]);
          Serial.print("eePromIdx");
          Serial.println(eepromIdx, DEC);
          if (scn == SCENE_TEST && ch == CHANNEL_A && cs == 0) {

            // Serial.print("scene: ");
            // Serial.print(scn, DEC);
            // Serial.print("; channel: ");
            // Serial.print(ch, DEC);
            // Serial.print("; color type: ");
            // Serial.print(cs, DEC);
            // Serial.print("; pixel: ");
            // Serial.print(px, DEC);
            // Serial.print("; Color: ");
            // printColor(allPixelColors[scn][ch][cs][px]);
            // Serial.println("");
          }
          //EEPROM.update(eepromIdx, allPixelColors[scn][ch][cs][px]);
          //eepromIdx++;
        }
      }
    }
  }
}

void printColor(uint32_t colorVal) {
  byte red = 0;
  byte green = 0;
  byte blue = 0;
  convertHexToRgb(colorVal, red, green, blue);
  Serial.print(" Red: ");
  Serial.print(red, DEC);
  Serial.print("; Green: ");
  Serial.print(green, DEC);
  Serial.print("; Blue: ");
  Serial.print(blue, DEC);
}

void convertHexToRgb(uint32_t colorVal, uint8_t &r, uint8_t &g, uint8_t &b) {
  //r = (colorVal >> 16) & 0xff;
  //g = (colorVal >> 8) & 0xff;
  //b = colorVal & 0xff;
  r = colorVal >> 16;
  g = (colorVal & 0x00ff00) >> 8;
  b = (colorVal & 0x0000ff);
}

// int charToHex(unsigned char inputChar) {
//   if (inputChar >= '0' && inputChar <= '9')
//     return inputChar - '0';
//   if (inputChar >= 'A' && inputChar <= 'F')
//     return inputChar - 'A' + 10;
//   if (inputChar >= 'a' && inputChar <= 'f')
//     return inputChar - 'a' + 10;
// }

void setEepromColor(uint16_t &startingIndex, uint32_t colorToSet) {
  byte red = 0;
  byte green = 0;
  byte blue = 0;
  convertHexToRgb(colorToSet, red, green, blue);
  EEPROM.update(startingIndex, red);
  startingIndex++;
  EEPROM.update(startingIndex, green);
  startingIndex++;
  EEPROM.update(startingIndex, blue);
  startingIndex++;
}

uint32_t getEepromColor(uint16_t &startingIndex) {
  byte red = EEPROM.read(startingIndex);
  startingIndex++;
  byte green = EEPROM.read(startingIndex);
  startingIndex++;
  byte blue = EEPROM.read(startingIndex);
  startingIndex++;
  uint32_t returnColor = Adafruit_NeoPixel::Color(red, green, blue);
  return returnColor;
}

int charToHex(unsigned char inputChar) {
  if (inputChar >= '0' && inputChar <= '9')
    return inputChar - '0';
  if (inputChar >= 'A' && inputChar <= 'F')
    return inputChar - 'A' + 10;
  if (inputChar >= 'a' && inputChar <= 'f')
    return inputChar - 'a' + 10;
}
