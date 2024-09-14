#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <nRF24L01.h>
#include <RF24.h>
#include "common/packets.hpp"

////////////////////////////////////////////////////////////////////////////////
// Hardware

SPIClass tft_spi(FSPI);
SPIClass radio_spi(HSPI);

#define TFT_SCLK 36
#define TFT_MISO -1 // unused, since only sending the display data
#define TFT_MOSI 35
#define TFT_CS   39
#define TFT_DC   37
#define TFT_RST  38

Adafruit_ST7735 tft(&tft_spi, TFT_CS, TFT_DC, TFT_RST);

#define RF24_SCLK 12
#define RF24_MISO 13
#define RF24_MOSI 11
#define RF24_CS   14
#define RF24_CE   10
#define RF24_CSN  RF24_CS

RF24 radio(RF24_CE, RF24_CSN);

const uint8_t transmitterOutputAddress[6] = "ctrl!";
const uint8_t transmitterInputAddress[6]  = "info?";

#define THROTTLE_PIN    4
#define RUDDER_PIN      5
#define ELEVATOR_PIN    1
#define AILERON_PIN     2
#define CHANNEL_5_PIN   6
#define AUX_1_PIN       43
#define AUX_2_PIN       44
#define AUX_3_PIN       42
#define F1_PIN          21
#define BUZZER_PIN      47

////////////////////////////////////////////////////////////////////////////////
// State

TransmitterSignal txSignal;

////////////////////////////////////////////////////////////////////////////////
// Setup

void setup()
{
	// Initialize the serial port
	//Serial.begin(115200); // unavailable AUX 1 & 2 taking RX/TX... 

	// Set pin modes
	pinMode(THROTTLE_PIN,   INPUT);
	pinMode(RUDDER_PIN,     INPUT);
	pinMode(ELEVATOR_PIN,   INPUT);
	pinMode(AILERON_PIN,    INPUT);
	pinMode(CHANNEL_5_PIN,  INPUT);
	pinMode(AUX_1_PIN,      INPUT_PULLUP);
	pinMode(AUX_2_PIN,      INPUT_PULLUP);
	pinMode(AUX_3_PIN,      INPUT_PULLUP);
	pinMode(F1_PIN,         INPUT_PULLUP);
	pinMode(BUZZER_PIN, OUTPUT);
	digitalWrite(BUZZER_PIN, LOW);

	// Initialize the SPI interfaces
	tft_spi.begin(TFT_SCLK, TFT_MISO, TFT_MOSI, TFT_CS);
	tft_spi.setFrequency(20'000'000);
	radio_spi.begin(RF24_SCLK, RF24_MISO, RF24_MOSI, RF24_CS);
	radio_spi.setFrequency(8'000'000);

	// Initialize the display
	tft.initR(INITR_MINI160x80_PLUGIN);
	tft.fillScreen(ST77XX_BLACK);
	tft.setRotation(1);

	// Initialize the radio
	radio.begin(&radio_spi, RF24_CE, RF24_CSN);
	radio.setDataRate(RF24_250KBPS);
	radio.setPALevel(RF24_PA_MAX);
	radio.setAutoAck(false);
	radio.setRetries(0, 0);
	radio.setPayloadSize(staticPayloadSize);
	radio.setCRCLength(RF24_CRC_8);
	radio.openReadingPipe(1, transmitterInputAddress);
	radio.openWritingPipe(transmitterOutputAddress);
	radio.stopListening();
}

////////////////////////////////////////////////////////////////////////////////
// Loop

void loop()
{
	txSignal.controlPacket.throttle = analogRead(THROTTLE_PIN);
	txSignal.controlPacket.rudder   = analogRead(RUDDER_PIN);
	txSignal.controlPacket.elevator = analogRead(ELEVATOR_PIN);
	txSignal.controlPacket.aileron  = analogRead(AILERON_PIN);
	txSignal.controlPacket.channel5 = analogRead(CHANNEL_5_PIN);
	txSignal.controlPacket.aux1     = digitalRead(AUX_1_PIN);
	txSignal.controlPacket.aux2     = digitalRead(AUX_2_PIN);
	txSignal.controlPacket.aux3     = digitalRead(AUX_3_PIN);
	radio.write(&txSignal, sizeof(txSignal));

	tft.fillScreen(ST77XX_BLACK);
	tft.setTextColor(ST77XX_WHITE);
	tft.setCursor(0, 0);
	tft.printf(
		"throttle=%hu\n"
		"rudder=%hu\n"
		"elevator=%hu\n"
		"aileron=%hu\n"
		"channel5=%hu\n"
		"aux1=%u\n"
		"aux2=%u\n"
		"aux3=%u\n"
		"f1=%u\n",
		txSignal.controlPacket.throttle,
		txSignal.controlPacket.rudder,
		txSignal.controlPacket.elevator,
		txSignal.controlPacket.aileron,
		txSignal.controlPacket.channel5,
		txSignal.controlPacket.aux1,
		txSignal.controlPacket.aux2,
		txSignal.controlPacket.aux3,
		digitalRead(F1_PIN)
	);
	delay(2);

	digitalWrite(BUZZER_PIN, txSignal.controlPacket.throttle > 1600 ? HIGH : LOW);
}
