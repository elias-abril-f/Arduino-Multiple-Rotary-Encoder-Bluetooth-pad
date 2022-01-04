 /*********************************************************************
 /*********************************************************************
 This is an example of 3 rotary encoders sending a HID multiple keypress over bluetooth. 
 I am using it to control the volume of different sources in the same PC via Voicemeeter and Macro buttons.
 *********************************************************************/
 
 *********************************************************************/
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
#if not defined (_VARIANT_ARDUINO_DUE_X_) && not defined(ARDUINO_ARCH_SAMD)
  #include <SoftwareSerial.h>
#endif

#ifdef __AVR__
  #include <avr/power.h>
#endif

#include <Arduino.h>
#include <SPI.h>
#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"
#include "BluefruitConfig.h"
#include "keycode.h"
#include <SimpleRotary.h>

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
    -----------------------------------------------------------------------*/
    #define FACTORYRESET_ENABLE         0
/*=========================================================================*/


// Create the bluefruit object, hardware SPI, using SCK/MOSI/MISO hardware SPI pins and then user selected CS/IRQ/RST
Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

// A small helper
void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}


// Set up keyboard report variables:
typedef struct
{
  uint8_t modifier;   /**< Keyboard modifier keys  */
  uint8_t reserved;   /**< Reserved for OEM use, always set to 0. */
  uint8_t keycode[6]; /**< Key codes of the currently pressed keys. */
} hid_keyboard_report_t;

// Report that sends to Central every scanning period
hid_keyboard_report_t keyReport = { 0, 0, { 0 } };

// Report sent previously. This is used to prevent sending the same report over time.
// Notes: HID Central intepretes no new report as no changes, which is the same as
// sending very same report multiple times. This will help to reduce traffic especially
// when most of the time there is no keys pressed.
// - Init to different with keyReport
hid_keyboard_report_t previousReport = { 0, 0, { 1 } };

// Pin A, Pin B, Button Pin
SimpleRotary rotary1(2,3,A3);
SimpleRotary rotary2(5,6,A2);
SimpleRotary rotary3(9,10,A1);


/**************************************************************************/
/*!
    @brief  Sets up the HW an the BLE module (this function is called
            automatically on startup)
*/
/**************************************************************************/
void setup(void)
{
  //while (!Serial);  // required for Flora & Micro
  delay(500);

  Serial.begin(115200);

  /* Initialise the module */
  Serial.print(F("Initialising the Bluefruit LE module: "));

  if ( !ble.begin(VERBOSE_MODE) )
  {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }
  Serial.println( F("OK!") );

  if ( FACTORYRESET_ENABLE )
  {
    /* Perform a factory reset to make sure everything is in a known state */
    Serial.println(F("Performing a factory reset: "));
    ble.factoryReset();
  }

  /* Disable command echo from Bluefruit */
  ble.echo(false);

  Serial.println("Requesting Bluefruit info:");
  /* Print Bluefruit information */
  ble.info();

  /* Enable HID Service if not enabled */
  int32_t hid_en = 0;

  if (! ble.sendCommandCheckOK(F( "AT+GAPDEVNAME=Adafruit Custom Keyboard" )) ) {
    error(F("Could not set device name?"));
  }
  
  ble.sendCommandWithIntReply( F("AT+BleHIDEn"), &hid_en);

  if ( !hid_en )
  {
    Serial.println(F("Enable HID Service (including Keyboard): "));
    ble.sendCommandCheckOK(F( "AT+BleHIDEn=On" ));

    /* Add or remove service requires a reset */
    Serial.println(F("Performing a SW reset (service changes require a reset): "));
    !ble.reset();
  }
  
  Serial.println();
  Serial.println(F("Go to your phone's Bluetooth settings to pair your device"));
  Serial.println(F("then open an application that accepts keyboard input"));
  Serial.println();

}

/**************************************************************************/
/*!
    @brief  Constantly poll for new command or response data
*/
/**************************************************************************/
void loop(void)
{
  byte g;
  g = rotary1.rotate();

 byte f;
 f = rotary2.rotate(); 

 byte i;
 i = rotary3.rotate(); 
  
 
  if ( ble.isConnected() )
  {
    
    //Rotary encoder 1 // 0 = not turning, 1 = CW, 2 = CCW
    if ( g == 1 || g == 2) 
    {
      if ( g == 2 ) 
      {
        Serial.println("1Left");
        keyReport.modifier = KEY_MOD_LCTRL;
        keyReport.keycode[4] = HID_KEY_DELETE;
      }
      else if ( g == 1 ) 
      {
        Serial.println("1Right");
        keyReport.modifier = KEY_MOD_LCTRL;
        keyReport.keycode[4] = HID_KEY_INSERT;
      }
    } else {
      keyReport.modifier = 0;
      keyReport.keycode[4] = 0;
    }
    sendKeyReport();
    //Rotary encoder 1 end
    
    //Rotary encoder 2 // 0 = not turning, 1 = CW, 2 = CCW
     if ( f == 1 || f == 2) 
    {
      if ( f == 2 ) 
      {
        Serial.println("2Left");
        keyReport.modifier = KEY_MOD_LCTRL;
        keyReport.keycode[5] = HID_KEY_END;
      }
      else if ( f == 1 ) 
      {
        Serial.println("2Right");
        keyReport.modifier = KEY_MOD_LCTRL;
        keyReport.keycode[5] = HID_KEY_HOME;
      }
    } else {
      keyReport.modifier = 0;
      keyReport.keycode[5] = 0;
    }
    
     sendKeyReport();
//Rotary encoder 2 end

//Rotary encoder 3 // 0 = not turning, 1 = CW, 2 = CCW
    if ( i == 1 || i == 2) 
    {
      if ( i == 2 ) 
      {
        Serial.println("3Left");
        keyReport.modifier = KEY_MOD_LCTRL;
        keyReport.keycode[3] = HID_KEY_PAGE_DOWN;
      }
      else if ( i == 1 ) 
      {
        Serial.println("3Right");
        keyReport.modifier = KEY_MOD_LCTRL;
        keyReport.keycode[3] = HID_KEY_PAGE_UP;
      }
    } else {
      keyReport.modifier = 0;
      keyReport.keycode[3] = 0;
    }
    sendKeyReport();
    //Rotary encoder 3 end
    } 
}


void sendKeyReport() {
  // Only send if it is not the same as previous report
      if ( memcmp(&previousReport, &keyReport, 8) )
      {
        // Send keyboard report
        ble.atcommand("AT+BLEKEYBOARDCODE", (uint8_t*) &keyReport, 8);

        // copy to previousReport
        memcpy(&previousReport, &keyReport, 8);
      }
      
      if (keyReport.keycode[4] != 0) {
        ble.atcommand("AT+BLEKEYBOARDCODE", (uint8_t*) &keyReport, 8);
      }
      
      if (keyReport.keycode[5] != 0) {
        ble.atcommand("AT+BLEKEYBOARDCODE", (uint8_t*) &keyReport, 8);
      }

      if (keyReport.keycode[3] != 0) {
        ble.atcommand("AT+BLEKEYBOARDCODE", (uint8_t*) &keyReport, 8);
      }
}
