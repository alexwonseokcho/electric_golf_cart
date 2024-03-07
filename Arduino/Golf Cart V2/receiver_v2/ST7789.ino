
#include <Adafruit_GFX.h>    // Core graphics library
// #include <Adafruit_ST7735.h> // Hardware-specific library for ST7735
#include <Adafruit_ST7789.h> // Hardware-specific library for ST7789
#include <SPI.h>


//PIN 21 IS THE INTERNAL LED --> ADD STATUS LED STUFF HERE

#define TFT_CS        D4
#define TFT_RST        D6 // Or set to -1 and connect to Arduino RESET pin
#define TFT_DC         D5
//CHANGE THIS LATER

Adafruit_ST7789 screen = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

void initScreen(){
    screen.init(240, 320);           // Init ST7789 320x240
    // SPI speed defaults to SPI_DEFAULT_FREQ defined in the library, you can override it here
    // Note that speed allowable depends on chip and quality of wiring, if you go too fast, you
    // may end up with a black screen some times, or all the time.
    //screen.setSPISpeed(40000000);

}

void displayInfo(){
    screen.fillScreen(ST77XX_BLACK);
    screen.setCursor(0, 0);
    
}