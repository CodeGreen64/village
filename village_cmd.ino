
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

//int pixelFormat = NEO_GRB + NEO_KHZ800;
int pixelFormat = NEO_RGBW;
uint8_t numPixels_a = 9;
uint8_t numPixels_b = 6;
int pin_a = 6;
int pin_b = 5;

const uint8_t SCENES = 3;
const uint8_t CHANNELS = 2;
const uint8_t COLOR_SETUPS = 3;
const uint8_t PIXEL_COUNT_MAX = 16;
const uint8_t CHANNEL_A = 0;
const uint8_t CHANNEL_B = 1;

uint32_t allPixelColors[SCENES][CHANNELS][COLOR_SETUPS][PIXEL_COUNT_MAX];

// Rather than declaring the whole NeoPixel object here, we just create
// a pointer for one, which we'll then allocate later...
Adafruit_NeoPixel *pixels_a;
Adafruit_NeoPixel *pixels_b;

uint8_t brightness = 150;

uint8_t basicSceneMainColorGreen = 0;
uint8_t basicSceneMainColorRed = 0;
uint8_t basicSceneMainColorBlue = 150;
uint32_t basicSceneMainColor = 0;

/**************************************************************************/
//       SCENES
/**************************************************************************/

const uint8_t SCENE_NOT_SET = 255;
const uint8_t SCENE_ROLLING_FADE = 0;
const uint8_t SCENE_BACK_FORTH = 1;
const uint8_t SCENE_WATER_FALL = 2;
const uint8_t SCENE_TEST = 3;

//const uint8_t waterfallColorSetups[9][2] = { { 1, 0 }, { 0, 1 }, { 1, 0 }, { 0, 2 }, { 1, 2 }, { 0, 2 }, { 2, 1 }, { 2, 0 }, { 2, 1 } };

const uint8_t waterfallColorSetups[9][2] = { { 2, 0 }, { 0, 1 }, { 1, 0 }, { 0, 1 }, { 1, 2 }, { 0, 2 }, { 1, 2 }, { 2, 0 }, { 2, 1 } };



/**************************************************************************/
//                  VARIABLES FOR APP LOGIC
/**************************************************************************/


bool clearEeprom = false;
bool blueConnected = false;
bool doReset = true;
bool allSceneMode = true;

//Color updating
bool waitingForColor = false;
uint8_t sceneToUpdate = 0;
uint8_t channelToUpdate = 0;
uint8_t colorTypeToUpdate = 0;
uint8_t beginPixelToUpdate = 0;
uint8_t endPixelToUpdate = 0;
uint32_t colorToUpdate = 0;

uint8_t currentScene = SCENE_WATER_FALL;

const uint8_t SPEED_COUNT = 2;
uint16_t speedValues[SCENES][SPEED_COUNT];
uint8_t allSceneMinutes = 5;

/**************************************************************************/
/**************************************************************************/


/**************************************************************************/
//                  EPROM INDEXES
/**************************************************************************/

const uint16_t PIX_A = 0;
const uint16_t PIX_B = 1;
const uint16_t CURRENT_SCENE_EPROM = 2;
const uint16_t ALLSCENE_MINUTES_EPROM = 3;
const uint16_t ALLSCENE_MODE_EPROM = 4;


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
  pixels_a->setBrightness(brightness);
  pixels_b->setBrightness(brightness);


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

  if (doReset) {
    pixels_a->clear();  // Set all pixel colors to 'off'
    pixels_b->clear();
    pixels_a->show();
    pixels_b->show();
    Serial.print("CurrentScene: ");
    Serial.println(currentScene, DEC);
  }

  if (allSceneMode) {
    checkAllScene(timeNowInLoop, doReset);
  }
  processScene(timeNowInLoop, currentScene, doReset);
  doReset = false;

  delay(1);
}

void processScene(unsigned long timeNow, uint8_t scene, bool doResetHere) {
  //Serial.println("Process scene");
  if (scene == SCENE_TEST) {
    uint32_t testC = Adafruit_NeoPixel::Color(255, 0, 0);
    testScene(basicSceneMainColor, speedValues[SCENE_TEST][0], timeNow, doResetHere);
  }
  if (scene == SCENE_ROLLING_FADE) {
    uint32_t startC = Adafruit_NeoPixel::Color(255, 0, 0);
    uint32_t endC = Adafruit_NeoPixel::Color(0, 255, 0);
    //rollingCrossfade(startC, endC, 10, timeNow, doResetHere);
    rollingCrossfade(timeNow, doResetHere);
  }
  if (scene == SCENE_BACK_FORTH) {
    backAndForth(timeNow, doResetHere);
  }
  if (scene == SCENE_WATER_FALL) {
    waterFall(timeNow, doResetHere);
  }
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
      // !pcsctbe program, c for Color,  scene, channel, type, beginPixel, endPixel
      // !pc10055
      // !pc01000
      // !pc00048
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
    if (packetbuffer[2] == 's') {
      //Update Speed
      // !psnnnn program, s for Speed, scene, type, speed to set up to 5 digits
      // !ps0035
      Serial.println("SetSpeed");
      sceneToUpdate = charToHex(packetbuffer[3]);
      uint8_t speedTypeToUpdate = charToHex(packetbuffer[4]);
      char numBuf[6];
      Serial.print("len: ");
      Serial.println(len, DEC);
      uint8_t numChar = len - 4;
      memcpy(&numBuf[0], &packetbuffer[5], numChar * sizeof(char));
      uint16_t testVal = atoi(numBuf);
      Serial.print("inputVal: ");
      Serial.println(testVal, DEC);
      speedValues[sceneToUpdate][speedTypeToUpdate] = testVal;
      setEpromFromVars();
      //for (uint8_t i = 0; i < len; i++) {

      //}
      //speedValues[sceneToUpdate][speedTypeToUpdate]
    }
    if (packetbuffer[2] == 'x') {
      //Update NumPix
      // !pxcnnn program, x for numpiX, channel, number of pixels to 3 digits
      // !px016
      Serial.println("Set NumPixels");
      channelToUpdate = charToHex(packetbuffer[3]);

      char numBuf[4];
      Serial.print("len: ");
      Serial.println(len, DEC);
      uint8_t numChar = len - 3;
      memcpy(&numBuf[0], &packetbuffer[4], numChar * sizeof(char));
      uint16_t testVal = atoi(numBuf);
      Serial.print("inputVal: ");
      Serial.println(testVal, DEC);
      if (channelToUpdate == CHANNEL_A) {
        numPixels_a = testVal;
      } else {
        numPixels_b = testVal;
      }
      setEpromFromVars();
      //for (uint8_t i = 0; i < len; i++) {

      //}
      //speedValues[sceneToUpdate][speedTypeToUpdate]
    }
    if (packetbuffer[2] == 'a') {
      //Update allScene minutes
      // !pxann program, x for numpiX, channel, number of pixels to 2 digits
      // !pa5
      Serial.println("Set NumPixels");

      char numBuf[3];
      Serial.print("len: ");
      Serial.println(len, DEC);
      uint8_t numChar = len - 2;
      memcpy(&numBuf[0], &packetbuffer[3], numChar * sizeof(char));
      uint16_t testVal = atoi(numBuf);
      Serial.print("inputVal: ");
      Serial.println(testVal, DEC);
      allSceneMinutes = testVal;
      EEPROM.update(ALLSCENE_MINUTES_EPROM, allSceneMinutes);
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
      if (buttnum == 8) {
        if (currentScene == SCENES - 1) {
          currentScene = 0;
        } else {
          currentScene++;
        }
        EEPROM.update(CURRENT_SCENE_EPROM, currentScene);
        doReset = true;
      }
      if (buttnum == 5){
        if (!allSceneMode){
          allSceneMode = true;
          EEPROM.update(ALLSCENE_MODE_EPROM, allSceneMode);
        }
      }
      if (buttnum == 6){
        if (allSceneMode){
          allSceneMode = false;
          EEPROM.update(ALLSCENE_MODE_EPROM, allSceneMode);
        }
      }      
    }
  }
}

void clearMem() {
  for (int i = 0; i < EEPROM.length(); i++) {
    EEPROM.write(i, 0);
  }
}


void setColorDefaults() {

  bool reverse = false;
  Serial.println("");
  Serial.println("SET_COLOR_DEFAULTS *******");

  for (uint8_t scn = 0; scn < SCENES; scn++) {
    for (uint8_t ch = 0; ch < CHANNELS; ch++) {
      uint8_t pxCt = numPixels_a;
      if (ch == CHANNEL_B) {
        pxCt = numPixels_b;
      }
      reverse = false;
      for (uint8_t cs = 0; cs < COLOR_SETUPS; cs++) {

        reverse = !reverse;
        // Serial.print("SetColor: ");
        // printColor(col);
        // Serial.print(" hexCol: ");
        // Serial.println(col, HEX);
        // Serial.println("");
        for (uint8_t px = 0; px < pxCt; px++) {
          reverse = !reverse;
          uint32_t col = Adafruit_NeoPixel::Color(255, 0, 0);
          if (scn == SCENE_TEST) {
            col = Adafruit_NeoPixel::Color(0, 0, 255);
          }
          if (cs == 1) {
            col = Adafruit_NeoPixel::Color(0, 255, 0);
          }
          if (cs == 2) {
            col = Adafruit_NeoPixel::Color(0, 0, 255);
          }
          if (scn == SCENE_ROLLING_FADE && ch == CHANNEL_A && cs == 0 && px > 3 && px < 9) {
            col = Adafruit_NeoPixel::Color(0, 0, 255);
          }
          if (scn == SCENE_ROLLING_FADE && ch == CHANNEL_A && cs == 1 && px > 3 && px < 9) {
            col = Adafruit_NeoPixel::Color(255, 255, 75);
          }

          if (scn == SCENE_BACK_FORTH && (px % 2 == 0)) {
            if (cs == 0) {
              col = Adafruit_NeoPixel::Color(0, 0, 255);
            }
            if (cs == 1) {
              col = Adafruit_NeoPixel::Color(255, 255, 75);
            }
          }
          allPixelColors[scn][ch][cs][px] = col;
          // if (scn == SCENE_WATER_FALL) {  // && ch == CHANNEL_A && cs == 0) {
          //   Serial.print("scene: ");
          //   Serial.print(scn, DEC);
          //   Serial.print("; channel: ");
          //   Serial.print(ch, DEC);
          //   Serial.print("; color type: ");
          //   Serial.print(cs, DEC);
          //   Serial.print("; pixel: ");
          //   Serial.print(px, DEC);
          //   Serial.print("; Color: ");
          //   printColor(allPixelColors[scn][ch][cs][px]);
          //   Serial.println("");
          // }
        }
      }
    }
  }
}

void setSpeedDefaults() {
  //speedValues[SCENE_TEST][0] = 250;
  speedValues[SCENE_ROLLING_FADE][0] = 20;
  speedValues[SCENE_BACK_FORTH][0] = 1;  //seconds
  speedValues[SCENE_WATER_FALL][0] = 25;
}


uint32_t crossFadeValue(const uint32_t startColor, const uint32_t endColor, int step) {
  uint32_t sc = startColor;
  uint32_t ec = endColor;
  int s = step;
  // if (step > 127) {
  //   sc = endColor;
  //   ec = startColor;
  //   s = step - 128;
  // }

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
    //Serial.println("Reset Transition");
  }
  uint32_t fadeVal = crossFadeValue(startColor, endColor, step);
  neo.setPixelColor(pixel, fadeVal);
  neo.show();
  if (step == 127) {
    step = 0;
    // Serial.println("Transition return true");
    // Serial.print("FadeColor: ");
    // printColor(fadeVal);
    // Serial.println("");
    return true;
  }
  step++;
  return false;
}



void rollingCrossfade(unsigned long timeNow, bool reset) {

  static unsigned long localTime;
  static uint8_t currentCounter;
  static uint8_t currentPixel;
  static bool doReverse;
  static bool lastAdvance;
  bool advancePixel;
  uint32_t sc;
  uint32_t ec;
  uint8_t channel;
  unsigned long speed;

  if (reset) {
    Serial.println("rollingCrossfade");
    currentCounter = 0;
    currentPixel = 0;
    pixels_a->clear();
    pixels_b->clear();
    doReverse = false;
  }

  speed = speedValues[SCENE_ROLLING_FADE][0];

  if (timeNow - localTime > speed) {
    localTime = timeNow;
    if (currentCounter == (numPixels_b + numPixels_a)) {
      currentPixel = 0;
      currentCounter = 0;
      doReverse = !doReverse;
      //Serial.println("Reversing");
    }
    if (currentCounter < numPixels_a) {
      channel = CHANNEL_A;
    } else {
      channel = CHANNEL_B;
      //Serial.println("channel b");
    }
    uint32_t startColor = allPixelColors[SCENE_ROLLING_FADE][channel][0][currentPixel];
    uint32_t endColor = allPixelColors[SCENE_ROLLING_FADE][channel][1][currentPixel];
    sc = startColor;
    ec = endColor;
    if (doReverse) {
      sc = endColor;
      ec = startColor;
    }
    if (channel == CHANNEL_A) {
      advancePixel = transitionSingle(*pixels_a, currentPixel, sc, ec, reset);
    } else {
      advancePixel = transitionSingle(*pixels_b, currentPixel, sc, ec, reset);
    }
    // if (lastAdvance) {
    //   Serial.print("currentCounter: ");
    //   Serial.println(currentCounter, DEC);
    //   Serial.print("currentPixel: ");
    //   Serial.println(currentPixel, DEC);
    //   Serial.print("startColor: ");
    //   printColor(sc);
    //   Serial.println("");
    //   Serial.print("endColor: ");
    //   printColor(ec);
    //   Serial.println("");
    //   Serial.print("channel: ");
    //   Serial.println(channel, DEC);
    // }

    lastAdvance = advancePixel;
    if (advancePixel) {
      currentCounter++;
      if (currentCounter < numPixels_a) {
        currentPixel = currentCounter;
      } else {
        currentPixel = currentCounter - numPixels_a;
        //Serial.print("b pixel: ");
        //Serial.println(currentPixel, DEC);
      }
    }
  }
}


void waterFall(unsigned long timeNow, bool reset) {

  static unsigned long localTime;
  static uint8_t currentSetupConter;
  static uint8_t step;
  uint8_t currentPixel;
  uint8_t channel;
  unsigned long speed;
  uint8_t colorSetup1;
  uint8_t colorSetup2;
  uint8_t colorSetupIndex;

  static bool fadeFinished;

  if (reset) {
    Serial.println("WaterFall");
    currentSetupConter = 0;
    currentPixel = 0;
    step = 0;
    pixels_a->clear();
    pixels_b->clear();
    fadeFinished = true;
  }

  speed = speedValues[SCENE_WATER_FALL][0];

  if (timeNow - localTime > speed) {
    localTime = timeNow;
    uint8_t totalPixels = numPixels_a + numPixels_b;
    for (uint8_t currentCounter = 0; currentCounter < totalPixels; currentCounter++) {
      if (currentCounter < numPixels_a) {
        channel = CHANNEL_A;
        currentPixel = currentCounter;
      } else {
        channel = CHANNEL_B;
        currentPixel = currentCounter - numPixels_a;
      }
      if (currentCounter < 4) {
        colorSetupIndex = 0;
      } else if (currentCounter > 8) {
        colorSetupIndex = 2;
      } else {
        colorSetupIndex = 1;
      }
      colorSetupIndex = colorSetupIndex + (3 * currentSetupConter);
      colorSetup1 = waterfallColorSetups[colorSetupIndex][0];
      colorSetup2 = waterfallColorSetups[colorSetupIndex][1];
      uint32_t startColor = allPixelColors[SCENE_WATER_FALL][channel][colorSetup1][currentPixel];
      uint32_t endColor = allPixelColors[SCENE_WATER_FALL][channel][colorSetup2][currentPixel];
      uint32_t fadeVal = crossFadeValue(startColor, endColor, step);
      if (channel == CHANNEL_A) {
        pixels_a->setPixelColor(currentPixel, fadeVal);
        //Serial.println("channel A");
      } else {
        pixels_b->setPixelColor(currentPixel, fadeVal);
        //Serial.println("channel B");
      }
      // if (fadeFinished && channel == CHANNEL_B) {
      //   Serial.println("******************");
      //   Serial.print("currentCounter: ");
      //   Serial.println(currentCounter, DEC);
      //   Serial.print("colorSetupIndex: ");
      //   Serial.println(colorSetupIndex, DEC);
      //   Serial.print("currentSetupConter: ");
      //   Serial.println(currentSetupConter, DEC);
      //   Serial.print("currentPixel: ");
      //   Serial.println(currentPixel, DEC);
      //   Serial.print("colorSetup1: ");
      //   Serial.println(colorSetup1, DEC);
      //   Serial.print("colorSetup2: ");
      //   Serial.println(colorSetup2, DEC);
      //   Serial.print("step: ");
      //   Serial.println(step, DEC);
      //   Serial.print("startColor: ");
      //   printColor(startColor);
      //   Serial.println("");
      //   Serial.print("endColor: ");
      //   printColor(endColor);
      //   Serial.println("");
      //   Serial.print("fadeVal: ");
      //   printColor(fadeVal);
      //   Serial.println("");
      //   if (channel == CHANNEL_A) {
      //     Serial.println("channel A");
      //   } else {
      //     Serial.println("channel B");
      //   }
      //   //fadeFinished = false;
      // }
    }
    //fadeFinished = false;
    step++;
    if (step == 127) {
      step = 0;
      currentSetupConter++;
      if (currentSetupConter > 2) {
        currentSetupConter = 0;
        fadeFinished = false;
      }
      //fadeFinished = true;
    }
    pixels_a->show();
    pixels_b->show();
  }
}

void backAndForth(unsigned long timeNow, bool reset) {

  static unsigned long localTime;
  static bool channelAisFirst;


  static uint8_t currentCounter;
  static uint8_t currentPixel;
  static bool lastAdvance;
  bool advancePixel;
  uint32_t sc;
  uint32_t ec;
  uint8_t channel;
  unsigned long speed;

  if (reset) {
    Serial.println("backAndForth");
    channelAisFirst = true;


    currentCounter = 0;
    currentPixel = 0;
    pixels_a->clear();
    pixels_b->clear();
    pixels_a->show();
    pixels_b->show();
  }

  speed = speedValues[SCENE_BACK_FORTH][0];
  speed = speed * 1000;

  if (timeNow - localTime > speed) {
    //Serial.println("BackForth change");
    localTime = timeNow;
    uint8_t totalPixels = numPixels_a + numPixels_b;
    for (uint8_t currentCounter = 0; currentCounter < totalPixels; currentCounter++) {
      uint8_t colorSetup = 0;
      if (currentCounter < numPixels_a) {
        channel = CHANNEL_A;
        currentPixel = currentCounter;
        // if (!channelAisFirst){
        //   colorSetup = 1;
        // }
      } else {
        channel = CHANNEL_B;
        currentPixel = currentCounter - numPixels_a;
        //Serial.println("channel b");
        // if (channelAisFirst){
        //   colorSetup = 1;
        // }
      }
      colorSetup = (uint8_t)channelAisFirst;
      channelAisFirst = !channelAisFirst;

      uint32_t currentColor = allPixelColors[SCENE_BACK_FORTH][channel][colorSetup][currentPixel];

      if (channel == CHANNEL_A) {
        pixels_a->setPixelColor(currentPixel, currentColor);
        //Serial.println("channel A");
      } else {
        pixels_b->setPixelColor(currentPixel, currentColor);
        //Serial.println("channel B");
      }
      // Serial.print("currentPixel");
      // Serial.println(currentPixel, DEC);
      // Serial.print("color: ");
      // printColor(currentColor);
      // Serial.println("");
    }
    pixels_a->show();
    pixels_b->show();
  }
}

void checkAllScene(unsigned long timeNow, bool reset) {
  unsigned long speed;
  static unsigned long localTime;

  speed = (unsigned long)allSceneMinutes * 60000;

  if (timeNow - localTime > speed) {
    localTime = timeNow;
    if (currentScene == SCENES - 1) {
      currentScene = 0;
    } else {
      currentScene++;
    }
    doReset = true;
  }
}


void setEpromFromVars() {
  EEPROM.update(PIX_A, numPixels_a);
  EEPROM.update(PIX_B, numPixels_b);
  EEPROM.update(CURRENT_SCENE_EPROM, currentScene);
  EEPROM.update(ALLSCENE_MINUTES_EPROM, allSceneMinutes);
  EEPROM.update(ALLSCENE_MODE_EPROM, allSceneMode);

  uint16_t eepromIdx = 5;

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
          // Serial.print("eePromIdx");
          // Serial.println(eepromIdx, DEC);
          // if (scn == SCENE_WATER_FALL) {

          //   Serial.print("scene: ");
          //   Serial.print(scn, DEC);
          //   Serial.print("; channel: ");
          //   Serial.print(ch, DEC);
          //   Serial.print("; color type: ");
          //   Serial.print(cs, DEC);
          //   Serial.print("; pixel: ");
          //   Serial.print(px, DEC);
          //   Serial.print("; Color: ");
          //   printColor(allPixelColors[scn][ch][cs][px]);
          //   Serial.println("");
          // }
        }
      }
    }
  }
}

void setVarsFromEprom() {
  numPixels_a = EEPROM.read(PIX_A);
  numPixels_b = EEPROM.read(PIX_B);
  currentScene = EEPROM.read(CURRENT_SCENE_EPROM);
  allSceneMinutes = EEPROM.read(ALLSCENE_MINUTES_EPROM);
  allSceneMode = EEPROM.read(ALLSCENE_MODE_EPROM);

  uint16_t eepromIdx = 5;

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
        }
      }
    }
  }
}




void testScene(const uint32_t testColor, unsigned long speed, unsigned long timeNow, bool reset) {

  static unsigned long localTime;
  static uint8_t currentCounter;
  static uint8_t currentPixel;
  bool advancePixel;
  uint8_t channel;
  if (reset) {
    Serial.println("StartingTestScene");
    pixels_a->show();
    pixels_a->clear();  // Set all pixel colors to 'off'
    pixels_b->clear();
    pixels_b->show();
    currentCounter = 0;
    currentPixel = 0;
  }
  if (timeNow - localTime > speed) {
    localTime = timeNow;
    if (currentCounter == (numPixels_b + numPixels_a)) {
      currentPixel = 0;
      currentCounter = 0;
      pixels_a->clear();
      pixels_b->clear();
      pixels_a->show();
      pixels_b->show();
    }
    if (currentCounter < numPixels_a) {
      channel = CHANNEL_A;
      currentPixel = currentCounter;
    } else {
      channel = CHANNEL_B;
      currentPixel = currentCounter - numPixels_a;
      //Serial.println("channel b");
    }
    // if (currentPixel == numPixels_a) {
    //   currentPixel = 0;
    //   pixels_a->clear();
    //   //Serial.println("Back to start");
    // }

    uint32_t colorToSet = allPixelColors[SCENE_TEST][CHANNEL_A][0][currentPixel];
    //Serial.print("TestColor: ");
    //Serial.println(colorToSet);
    //pixels_a->setPixelColor(currentPixel, testColor);
    //pixels_a->show();  // Send the updated pixel colors to the hardware.
    if (channel == CHANNEL_A) {
      testSetColor(*pixels_a, currentPixel, colorToSet);
    } else {
      testSetColor(*pixels_b, currentPixel, colorToSet);
    }
    currentCounter++;
  }
}

void testSetColor(Adafruit_NeoPixel &neo, uint8_t pix, uint32_t col) {
  neo.setPixelColor(pix, col);
  neo.show();
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
