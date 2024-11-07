
#include <Adafruit_GFX.h>    // Core graphics library
// #include <Adafruit_ST7735.h> // Hardware-specific library for ST7735
#include <Adafruit_ST7789.h> // Hardware-specific library for ST7789
#include <SPI.h>


//PIN 21 IS THE INTERNAL LED --> ADD STATUS LED STUFF HERE

#define TFT_CS        D3
#define TFT_RST        D0 // Or set to -1 and connect to Arduino RESET pin
#define TFT_DC         D2
// #define TFT_MOSI       D10
// #define TFT_SCK        D8
//D1 for ADC input for pot
//CHANGE THIS LATER

Adafruit_ST7789 screen = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

void initScreen(){
    screen.init(240, 320);           // Init ST7789 320x240
    // SPI speed defaults to SPI_DEFAULT_FREQ defined in the library, you can override it here
    // Note that speed allowable depends on chip and quality of wiring, if you go too fast, you
    // may end up with a black screen some times, or all the time.
    screen.setSPISpeed(80000000);

    screen.fillScreen(ST77XX_BLACK);
}

void displayInfo(){
    screen.setCursor(0, 0);
    // screen.clear();
    // screen.clear
    screen.setTextSize(4);
    // screen.setTextColor(ST77XX_RED, ST77XX_BLACK);
    screen.print("HELLO");
}