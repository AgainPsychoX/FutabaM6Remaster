#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <EEPROM.h>
#include <rom/crc.h>
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
#define TRANSMITTER_BATTERY_PIN 8

////////////////////////////////////////////////////////////////////////////////
// Saved state (in EEPROM)

struct Settings
{
	////////////////////////////////////////
	// 0x000 - 0x010: Checksum

	uint8_t _emptyBeginPad[12];
	uint32_t checksum;

	uint32_t calculateChecksum() {
		constexpr uint16_t prefixLength = offsetof(Settings, checksum) + sizeof(checksum);
		return crc32_le(0, reinterpret_cast<uint8_t*>(this) + prefixLength, sizeof(Settings) - prefixLength);
	}

	bool prepareForSave() {
		uint32_t calculatedChecksum = calculateChecksum();
		bool changed = checksum != calculatedChecksum;
		checksum = calculatedChecksum;
		return changed;
	}

	////////////////////////////////////////
	// 0x010 - 0x040: Calibration

	struct Calibration
	{
		uint16_t throttleMin        = 685;
		uint16_t throttleCenter     = 1145;
		uint16_t throttleMax        = 1647;
		uint16_t rudderMin          = 663;
		uint16_t rudderCenter       = 1047;
		uint16_t rudderMax          = 1427;
		uint16_t elevatorMin        = 633;
		uint16_t elevatorCenter     = 1063;
		uint16_t elevatorMax        = 1494;
		uint16_t aileronMin         = 662;
		uint16_t aileronCenter      = 1101;
		uint16_t aileronMax         = 1548;
		uint16_t channel5Min        = 2779;
		uint16_t channel5Center     = 3207;
		uint16_t channel5Max        = 3793;
		uint8_t _pad[18];
	};
	Calibration calibration;

	////////////////////////////////////////

	void resetToDefault() {
		new (this) Settings(); // will apply all defaults
	}
};
static_assert(offsetof(Settings, calibration) == 0x10);
static_assert(sizeof(Settings::calibration) == 0x30);

Settings* settings;

////////////////////////////////////////////////////////////////////////////////
// State

enum class Page : unsigned int
{
	Info,       // Transmitter & receiver battery and signal strength.
	Raw,        // Raw analog values.
	Centered,   // Analog values with bias/offset, zero in configured position.
	Calibrate,  // Setup min/max values on each control, to be used for PPM gen.
	Reverse,    // Allow reversing of the channels.
	Count,      // Not a page, count of all the pages.
};
Page page = Page::Info;

void goNextPage()
{
	page = static_cast<Page>(
		(static_cast<unsigned int>(page) + 1) 
			% static_cast<unsigned int>(Page::Count)
	);
}

unsigned long f1ButtonPressed = 0; // 0 means not pressed

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

	// Initialize the EEPROM
	EEPROM.begin(sizeof(Settings));
	settings = reinterpret_cast<Settings*>(EEPROM.getDataPtr());
	if (settings->calculateChecksum() != settings->checksum) {
		settings->resetToDefault();
		settings->prepareForSave();
		EEPROM.commit();
		tft.fillScreen(ST77XX_BLUE);
		delay(1000);
	}

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

void sendSignal()
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
}

void loop()
{
	sendSignal();

	digitalWrite(BUZZER_PIN, txSignal.controlPacket.throttle > 1600 ? HIGH : LOW);

	unsigned long now = millis();

	bool wasLongPress = false;
	if (f1ButtonPressed) {
		if (digitalRead(F1_PIN) == LOW) /* still pressed */ {
			// Waiting for release
		}
		else /* released */ {
			if (now - f1ButtonPressed > 1023) /* long press finished */ {
				wasLongPress = true;
			}
			else /* short press finished */ {
				goNextPage();
			}
			f1ButtonPressed = 0;
		}
	}
	else {
		f1ButtonPressed = digitalRead(F1_PIN) == LOW ? now : 0;
	}

	static unsigned long lastDraw = 0;
	if (now - lastDraw < 100)
		return;
	tft.fillScreen(ST77XX_BLACK);
	tft.setTextColor(ST77XX_WHITE);
	tft.setCursor(0, 0);
	// TODO: prevent blinking of the screen when drawing stuff
	switch (page) {
		case Page::Info: {
			tft.printf(
				"Bateria nadajnika: 12.3V\n"
				"Bateria odbiornika: 12.3V\n"
				"Jakosc sygnalu: 40\n"
			);
			// TODO: make it actually work and look nice
			break;
		}
		case Page::Raw: {
			tft.printf(
				"Surowe wartosci:\n"
				" throttle=%hu\n"
				" rudder=%hu\n"
				" elevator=%hu\n"
				" aileron=%hu\n"
				" channel5=%hu\n"
				" aux1=%u\n"
				" aux2=%u\n"
				" aux3=%u\n",
				txSignal.controlPacket.throttle,
				txSignal.controlPacket.rudder,
				txSignal.controlPacket.elevator,
				txSignal.controlPacket.aileron,
				txSignal.controlPacket.channel5,
				txSignal.controlPacket.aux1,
				txSignal.controlPacket.aux2,
				txSignal.controlPacket.aux3
			);
			break;
		}
		case Page::Centered: {
			tft.printf(
				"Wartosci od srodka:\n"
				" throttle=%hd\n"
				" rudder=%hd\n"
				" elevator=%hd\n"
				" aileron=%hd\n"
				" channel5=%hd\n"
				" aux1=%u\n"
				" aux2=%u\n"
				" aux3=%u\n",
				settings->calibration.throttleCenter - txSignal.controlPacket.throttle,
				settings->calibration.rudderCenter   - txSignal.controlPacket.rudder,
				settings->calibration.elevatorCenter - txSignal.controlPacket.elevator,
				settings->calibration.aileronCenter  - txSignal.controlPacket.aileron,
				settings->calibration.channel5Center - txSignal.controlPacket.channel5,
				txSignal.controlPacket.aux1,
				txSignal.controlPacket.aux2,
				txSignal.controlPacket.aux3
			);
			break;
		}
		case Page::Calibrate: {
			tft.printf("Calibrate?");
			// TODO: ...
			break;
		}
		case Page::Reverse: {
			tft.printf("Reverse?");
			// TODO: ...
			break;
		}
		default:
			break;
	}
}
