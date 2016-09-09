/**************************************************************************/
/*! 
    @file     FeliCa_nfcid2.pde
    @author   tomo mas
  @license  BSD (see license.txt)

    This example will attempt to connect to an FeliCa
    card or tag and retrieve some basic information such as IDm, PMm, and System Code.
   
    Note that you need the baud rate to be 115200 because we need to print
    out the data and read from the card at the same time!
*/
/**************************************************************************/
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_PN532.h>

// If using the breakout with SPI, define the pins for SPI communication.
#define PN532_SCK  (2)
#define PN532_MOSI (3)
#define PN532_SS   (4)
#define PN532_MISO (5)

// If using the breakout or shield with I2C, define just the pins connected
// to the IRQ and reset lines.  Use the values below (2, 3) for the shield!
#define PN532_IRQ   (2)
#define PN532_RESET (3)  // Not connected by default on the NFC Shield

// Uncomment just _one_ line below depending on how your breakout or shield
// is connected to the Arduino:

// Use this line for a breakout with a SPI connection:
Adafruit_PN532 nfc(PN532_SCK, PN532_MISO, PN532_MOSI, PN532_SS);

// Use this line for a breakout with a hardware SPI connection.  Note that
// the PN532 SCK, MOSI, and MISO pins need to be connected to the Arduino's
// hardware SPI SCK, MOSI, and MISO pins.  On an Arduino Uno these are
// SCK = 13, MOSI = 11, MISO = 12.  The SS line can be any digital IO pin.
//Adafruit_PN532 nfc(PN532_SS);

// Or use this line for a breakout or shield with an I2C connection:
//Adafruit_PN532 nfc(PN532_IRQ, PN532_RESET);

#if defined(ARDUINO_ARCH_SAMD)
// for Zero, output on USB Serial console, remove line below if using programming port to program the Zero!
// also change #define in Adafruit_PN532.cpp library file
   #define Serial SerialUSB
#endif

uint8_t prevIDm[8]; //IDm of the card previously detected

void setup(void) {
  #ifndef ESP8266
    while (!Serial); // for Leonardo/Micro/Zero
  #endif
  Serial.begin(115200);
  Serial.println("Hello!");

  nfc.begin();

  uint32_t versiondata = nfc.getFirmwareVersion();
  if (! versiondata) {
    Serial.print("Didn't find PN53x board");
    while (1); // halt
  }
  
  // Got ok data, print it out!
  Serial.print("Found chip PN5"); Serial.println((versiondata>>24) & 0xFF, HEX); 
  Serial.print("Firmware ver. "); Serial.print((versiondata>>16) & 0xFF, DEC); 
  Serial.print('.'); Serial.println((versiondata>>8) & 0xFF, DEC);
  
  // Set the max number of retry attempts to read from a card
  // This prevents us from waiting forever for a card, which is
  // the default behaviour of the PN532.
  nfc.setPassiveActivationRetries(0xFF);
  
  // configure board to read RFID tags
  nfc.SAMConfig();
  
  Serial.println("Waiting for an FeliCa card");
}

void loop(void) {
  boolean success;
  uint16_t systemCode = 0xFFFF;
  uint8_t requestCode = 0x01; // System Code request
  uint8_t idm[8];
  uint8_t pmm[8];
  uint16_t systemCodeResponse;
  
  // Wait for an FeliCa type cards.  
  // When one is found, some basic information such as IDm, PMm, and System Code are retrieved.
  success = nfc.felica_Polling(systemCode, requestCode, idm, pmm, systemCodeResponse, 1000);
  
  if (!success) {
    // PN532 probably timed out waiting for a card
    return;
  }
  if ( memcmp(idm, prevIDm, 8) == 0) {
    // the card previously detected is still on the reader.
    return;
  }
  memcpy(prevIDm, idm, 8);

  Serial.println("Found a card!");
  Serial.print("IDm: ");
  for (uint8_t i=0; i < 8; i++) {
    Serial.print(" 0x");Serial.print(idm[i], HEX); 
  }
  Serial.println("");

  Serial.print("PMm: ");
  for (uint8_t i=0; i < 8; i++) {
    Serial.print(" 0x");Serial.print(pmm[i], HEX); 
  }
  Serial.println("");
  // Wait 1 second before continuing
  delay(1000);
}
