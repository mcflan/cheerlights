/*********************************************************************
  Adafruit Flora + Bluefruit LE + Neopixels + Cheerlights

  By James L Macfarlane 2015.

  This code lives at https://github.com/mcflan/cheerlights

  Quick and dirty hack to make a string of Neopixels connected to
  an Adafruit Flora respond to messages over Bluetooth LE from
  the Flora Bluefruit module. Will set all "pixels" to any
  Cheerlights colour seen in Bluetooth UART data. Will also
  obey packets from Adafruit Bluefruit iOS/'Droid App Color Picker
  mode. If no characters arive on BLE within a 30s timeout, revert
  to updating each pixel in turn to a random Cheerlights colour.

  More about Cheerlights at: http://cheerlights.com/

  This code is based on heavily modifying Adafruit examples, their
  original comment header is included below.
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

#include <string.h>
#include <Arduino.h>
#include <SPI.h>


#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"

#include "BluefruitConfig.h"

#include <Adafruit_NeoPixel.h>


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
    PIN                       Which pin on the Arduino is connected to the NeoPixels?
    NUMPIXELS                 How many NeoPixels are attached to the Arduino?
    -----------------------------------------------------------------------*/
    #define FACTORYRESET_ENABLE     0

    #define PIN                     6
    #define NUMPIXELS               7
/*=========================================================================*/

Adafruit_NeoPixel pixel = Adafruit_NeoPixel(NUMPIXELS, PIN);

// Create the bluefruit object, either software serial...uncomment these lines
/*
SoftwareSerial bluefruitSS = SoftwareSerial(BLUEFRUIT_SWUART_TXD_PIN, BLUEFRUIT_SWUART_RXD_PIN);

Adafruit_BluefruitLE_UART ble(bluefruitSS, BLUEFRUIT_UART_MODE_PIN,
                      BLUEFRUIT_UART_CTS_PIN, BLUEFRUIT_UART_RTS_PIN);
*/

/* ...or hardware serial, which does not need the RTS/CTS pins. Uncomment this line */
Adafruit_BluefruitLE_UART ble(BLUEFRUIT_HWSERIAL_NAME, BLUEFRUIT_UART_MODE_PIN);

/* ...hardware SPI, using SCK/MOSI/MISO hardware SPI pins and then user selected CS/IRQ/RST */
//Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

/* ...software SPI, using SCK/MOSI/MISO user-defined SPI pins and then user selected CS/IRQ/RST */
//Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_SCK, BLUEFRUIT_SPI_MISO,
//                             BLUEFRUIT_SPI_MOSI, BLUEFRUIT_SPI_CS,
//                             BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);


// A small helper
void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}

// function prototypes over in packetparser.cpp
uint8_t readPacket(Adafruit_BLE *ble, uint16_t timeout);
float parsefloat(uint8_t *buffer);
void printHex(const uint8_t * data, const uint32_t numBytes);

// the packet buffer
extern uint8_t packetbuffer[];


/**************************************************************************/
/*!
    @brief  Sets up the HW an the BLE module (this function is called
            automatically on startup)
*/
/**************************************************************************/
void setup(void)
{
  // Only un-comment the line below if you want to debug using the 
  // serial monitor. It will completely stop the Flora from working
  // in stand-alone mode (no USB connected.)

  // while (!Serial);  // required for Flora & Micro

  delay(500);

  // turn off neopixel
  pixel.begin(); // This initializes the NeoPixel library.
  for(uint8_t i=0; i<NUMPIXELS; i++) {
    pixel.setPixelColor(i, pixel.Color(0,0,0)); // off
  }
  pixel.show();

  Serial.begin(115200);
  Serial.println(F("Adafruit Bluefruit Neopixel Color Picker Example"));
  Serial.println(F("------------------------------------------------"));

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
    if ( ! ble.factoryReset() ){
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

}

int check_ble_connection(void)
{
  /* Check for connection */
  if (!ble.isConnected()) {
    // Enter data mode.
    ble.setMode(BLUEFRUIT_MODE_DATA);
    return 1;
  } else {
    return 0;
  }
}


#define CL_BUFSIZE      (16)
#define CL_NCOLOURS     (12)

struct cheerlight_s {
  char *name;
  unsigned char red;
  unsigned char green;
  unsigned char blue;
} colourmap[CL_NCOLOURS] = {
  { "black",    0x00, 0x00, 0x00 },
  { "red",      0xff, 0x00, 0x00 },
  { "green",    0x00, 0x80, 0x00 },
  { "blue",     0x00, 0x00, 0xff },
  { "cyan",     0x00, 0xff, 0xff },
  { "white",    0xff, 0xff, 0xff },
  { "oldlace",  0xfd, 0xf5, 0xe6 },
  { "purple",   0x80, 0x00, 0x80 },
  { "magenta",  0xff, 0x00, 0xff },
  { "yellow",   0xff, 0xff, 0x00 },
  { "orange",   0xff, 0xa5, 0x00 },
  { "pink",     0xff, 0xc0, 0xcb },
};

char buf[CL_BUFSIZE];
int head = 0;

// Get a character from the ring buffer at
// a given +ve or -ve offset, wrapping as
// appropriate to ensure we don't overflow.
// Doesn't change the buffer contents or
// head index.
int ringbuf_get(int head, int offs)
{
  int bufptr = head + offs;

  // Wrap back into the span of the buffer. Yes,
  // could have used modulus but this is much
  // faster and simpler for small offsets.
  while (bufptr < 0) bufptr += CL_BUFSIZE;
  while (bufptr >= CL_BUFSIZE) bufptr -= CL_BUFSIZE;

  return buf[bufptr];
}

#define PACKET_COLOR_LEN                (6)

int nextpix = 0;

// Check for Color Picker command from Adafruit
// Bluefruit app. We don't check the checksum at
// present. Naughty. However, this is not a
// safety-critical app.
void check_bluefruit_color_cmd(int head)
{
  int len = PACKET_COLOR_LEN;

  if ( (ringbuf_get(head, 0 - len) == '!') &&
       (ringbuf_get(head, 1 - len) == 'C') ) {

    uint8_t red = ringbuf_get(head, 2 - len);
    uint8_t grn = ringbuf_get(head, 3 - len);
    uint8_t blu = ringbuf_get(head, 4 - len);

    Serial.print("\nColor Picker message: #");
    Serial.print(red, HEX);
    Serial.print(grn, HEX);
    Serial.print(blu, HEX);
    Serial.print('\n');

    // Set next Neopixel to this colour.
    pixel.setPixelColor(nextpix++, red, grn, blu);
    if (nextpix >= NUMPIXELS)
      nextpix = 0; // Wrap back to 1st pixel.
    pixel.show(); // This sends the updated pixel color to the hardware.
  }
}

void parse_ble_char(char c)
{
  // The Adafruit LE iOS app provides no framing on rx'd
  // mqtt messages so we need to get clever and push the
  // incoming characters through a ring buffer and check
  // for colour-name matches each time.
  
  buf[head++] = c;
  if (head >= CL_BUFSIZE)
    head = 0; // Wrap back to start of buffer.

  
  // Check for color-picker binary msg.
  check_bluefruit_color_cmd(head);

  // Scan the latest characters in the buffer against
  // all known cheerlights colour names.
  for (int col = 0; col < CL_NCOLOURS; col++) {
    int len = strlen(colourmap[col].name);
    int i;
    for (i = 0; i < len; i++) {
      if (colourmap[col].name[i] != ringbuf_get(head, i - len))
        break;
    }
    if (i == len) {
      // Success. We matched a colour in our table.
      // Set all neopixels to that colour.
      Serial.print("\ncheerlights ");
      Serial.print(colourmap[col].name);
      Serial.print('\n');
      for(uint8_t i = 0; i < NUMPIXELS; i++) {
        pixel.setPixelColor(i, pixel.Color(
          colourmap[col].red,
          colourmap[col].green,
          colourmap[col].blue
        ) );
      }
      pixel.show(); // This sends the updated pixel color to the hardware.
    }
  }
}

void set_random_colour(void)
{
  // Pick a random Cheerlights colour (not black.)
  int random_colour = 1+random(CL_NCOLOURS-1);

  // Set next Neopixel to this colour.
  pixel.setPixelColor(nextpix++,
    colourmap[random_colour].red,
    colourmap[random_colour].green,
    colourmap[random_colour].blue
  );
  if (nextpix >= NUMPIXELS)
    nextpix = 0; // Wrap back to 1st pixel.
  pixel.show(); // This sends the updated pixel color to the hardware.
}

#define BLE_TIMEOUT     (30000) // Half a minute. Ish.

uint16_t timer = 1000; // Start random seq after 1 second.     

// Linker should set such variables to zero anyway but I'm being
// paranoid as am unfamiliar with 'duino Wiring build system 
// internals.
int ble_is_connected = 0;

void loop(void)
{

  while ( ble_is_connected && ble.available() ) {
    int c = ble.read();
    Serial.print((char)c); // Echo incoming char to debug serial port
    timer = BLE_TIMEOUT;
    parse_ble_char(c); // Do something useful with the char.
  }

  if (!ble_is_connected) {
    if (check_ble_connection()) {
      ble_is_connected = 1;
      // Judgin from original Adafruit code, I think we only
      // need to check for connections and enter data mode once
      // on start-up, so we don't reset the ble_is_connected
      // flag anywhere except on start-up where it's 0.
    }
  }

  delay(1);
  if (timer == 0) {
    timer = 1000; // 1 second-ish to next random colour.
    set_random_colour();
  } else {
    timer--;
  }
}

