#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>

#define TFT_CS   39
#define TFT_MOSI 35
#define TFT_SCLK 36
#define TFT_MISO 37
#define TFT_DC   37
#define TFT_RST  38

Adafruit_ST7735 tft(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST);

void setup()
{
	Serial.begin(115200);
	Serial.println("Setup!");

	// Initialize the display
	tft.initR(INITR_MINI160x80_PLUGIN);

	for (int i = 0; i < 4; i++) {
		Serial.printf("Testing INITR=%u R=%d\n", INITR_MINI160x80_PLUGIN, i);
		tft.setRotation(i);
		tft.fillScreen(ST77XX_BLACK);
		tft.setCursor(0, 0);
		tft.setTextColor(ST77XX_WHITE);
		tft.setTextWrap(true);
		tft.printf("INITR=%u R=%d ", INITR_MINI160x80_PLUGIN, i);
		tft.print("Lorem ipsum dolor sit amet, consectetur adipiscing elit. Curabitur adipiscing ante sed nibh tincidunt feugiat. Maecenas enim massa, fringilla sed malesuada et, malesuada sit amet turpis. Sed porttitor neque ut ante pretium vitae malesuada nunc bibendum. Nullam aliquet ultrices massa eu hendrerit. Ut sed nisi lorem. In vestibulum purus a tortor imperdiet posuere. ");
		delay(5000);
	}
}

void loop()
{
	Serial.print("Looping!");
	Serial.print("uptime: ");
	Serial.println(millis());
	delay(1000);
}
