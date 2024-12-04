#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>

#define i2c_Address 0x3c //initialize with the I2C addr 0x3C Typically eBay OLED's

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET -1   //   QT-PY / XIAO
Adafruit_SH1106G display = Adafruit_SH1106G(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

void setup()   {

  Serial.begin(9600);

  delay(250); // wait for the OLED to power up
  display.begin(i2c_Address, true); // Address 0x3C default
 //display.setContrast (0); // dim display
 
  display.display();
  delay(2000);

  // Clear the buffer.
  display.clearDisplay();


  // text display tests
  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE);
  display.setCursor(0, 0);
  display.println("Battery level: 89%");
  display.setCursor(0, 9);
  display.println("Flight mode: CMD");
  display.setCursor(0, 18);
  display.println("Self-checks: OK");
  display.setCursor(0, 27);
  display.println("");
  display.setTextColor(SH110X_BLACK, SH110X_WHITE);
  display.setCursor(0, 36);
  display.println("CONFIRM THROTTLE");
  display.setCursor(0, 45);
  display.println("");
  display.setTextColor(SH110X_WHITE, SH110X_BLACK);
  display.setCursor(0, 54);
  display.println("Ready for operation...");
  display.display();
  delay(2000);
  display.clearDisplay();
}


void loop() {

}
