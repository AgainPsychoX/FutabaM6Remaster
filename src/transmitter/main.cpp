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

#define F1_PIN          21
#define BUZZER_PIN      47
#define TRANSMITTER_BATTERY_PIN 8

#define THROTTLE_PIN    4
#define RUDDER_PIN      5
#define ELEVATOR_PIN    1
#define AILERON_PIN     2
#define CHANNEL_5_PIN   6
#define AUX_1_PIN       43
#define AUX_2_PIN       44
#define AUX_3_PIN       42

////////////////////////////////////////////////////////////////////////////////
// Saved state (in EEPROM)

struct Settings
{
	static constexpr uint32_t currentVersion = 2;

	////////////////////////////////////////
	// 0x000 - 0x010: Header

	uint8_t _emptyBeginPad[8];
	uint32_t version;
	uint32_t checksum;

	uint32_t calculateChecksum()
	{
		constexpr uint16_t prefixLength = offsetof(Settings, checksum) + sizeof(checksum);
		return crc32_le(0, reinterpret_cast<uint8_t*>(this) + prefixLength, sizeof(Settings) - prefixLength);
	}

	bool validate()
	{
		return checksum == calculateChecksum() && version == currentVersion;
	}

	bool prepareForSave()
	{
		uint32_t calculatedChecksum = calculateChecksum();
		bool changed = checksum != calculatedChecksum;
		version = currentVersion;
		checksum = calculatedChecksum;
		return changed;
	}

	////////////////////////////////////////
	// 0x010 - 0x060: Calibration values

	AnalogChannelsCalibration calibration = {
		/* Throttle */ { .rawMin =  685, .rawCenter = 1145, .rawMax = 1647, .usMin = 1000, .usCenter = 1500, .usMax = 2000 },
		/* Rudder   */ { .rawMin =  663, .rawCenter = 1047, .rawMax = 1427, .usMin = 1000, .usCenter = 1500, .usMax = 2000 },
		/* Elevator */ { .rawMin =  633, .rawCenter = 1063, .rawMax = 1494, .usMin = 1000, .usCenter = 1500, .usMax = 2000 },
		/* Aileron  */ { .rawMin =  662, .rawCenter = 1101, .rawMax = 1548, .usMin = 1000, .usCenter = 1500, .usMax = 2000 },
		/* Channel5 */ { .rawMin = 2779, .rawCenter = 3207, .rawMax = 3793, .usMin = 1000, .usCenter = 1500, .usMax = 2000 },
		/* Unused   */ { .rawMin = 1000, .rawCenter = 2000, .rawMax = 3000, .usMin = 1000, .usCenter = 1500, .usMax = 2000 },
	};
	uint8_t _padAfterCalibration[8];

	////////////////////////////////////////

	void resetToDefault() {
		new (this) Settings(); // will apply all defaults
	}
};
static_assert(offsetof(Settings, calibration) == 0x10);
static_assert(sizeof(Settings::calibration) <= 0x50);

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
ReceiverSignal rxSignal;

unsigned long lastTxSignalTime = 0;
unsigned long lastRxSignalTime = 0;
constexpr unsigned int rxSignalFetchInterval = 500; // ms
constexpr unsigned int rxSignalListenDuration = 20; // ms
unsigned long lastRxSignalLastLatency = 0;

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
	pinMode(TRANSMITTER_BATTERY_PIN, INPUT);
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
	if (!settings->validate()) {
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

void loop()
{
	unsigned long now = millis();

	// Send transmitter signal
	txSignal.packetType = PacketType::Control;
	txSignal.controlPacket.throttle = analogRead(THROTTLE_PIN);
	txSignal.controlPacket.rudder   = analogRead(RUDDER_PIN);
	txSignal.controlPacket.elevator = analogRead(ELEVATOR_PIN);
	txSignal.controlPacket.aileron  = analogRead(AILERON_PIN);
	txSignal.controlPacket.channel5 = analogRead(CHANNEL_5_PIN);
	txSignal.controlPacket.aux1     = digitalRead(AUX_1_PIN);
	txSignal.controlPacket.aux2     = digitalRead(AUX_2_PIN);
	txSignal.controlPacket.aux3     = digitalRead(AUX_3_PIN);
	if (now - lastRxSignalTime > rxSignalFetchInterval) {
		txSignal.controlPacket.request = TransmitterRequest::Status;
	}
	else {
		txSignal.controlPacket.request = TransmitterRequest::None;
	}
	radio.write(&txSignal, sizeof(txSignal));
	lastTxSignalTime = now;

	if (txSignal.controlPacket.request != TransmitterRequest::None) {
		radio.startListening();
		unsigned long listenStartTime = millis();
		do {
			now = millis();
			if (radio.available()) {
				radio.read(&rxSignal, sizeof(rxSignal));
				lastRxSignalTime = now;
				lastRxSignalLastLatency = now - listenStartTime;
				break;
			}
		}
		while (now - listenStartTime < rxSignalListenDuration);
		radio.stopListening();
	}

	// Buzzer testing, since it sounds weird...
	digitalWrite(BUZZER_PIN, analogRead(THROTTLE_PIN) > 1600 ? HIGH : LOW);

	// Transmitter battery uses 15V to 3.235V divider (12kOhm & 3.3kOhm),
	// ESP32S3 has 12-bit ADC.
	constexpr float txBatteryFactor = 3.235 / 4095.0 * (12000.0 + 3300.0) / 3300.0;
	uint16_t txBatteryRaw = analogRead(TRANSMITTER_BATTERY_PIN);

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
				"Bateria nadajnika: %.2fV\n"
				"Bateria odbiornika: %.2fV\n"
				"Jakosc sygnalu: %hhu\n",
				txBatteryFactor * txBatteryRaw,
				rxSignal.statusPacket.battery, // Note: assuming it's always status packet
				rxSignal.statusPacket.signalRating
			);
			// TODO: make it actually look nice
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
				settings->calibration[0].rawCenter - txSignal.controlPacket.throttle,
				settings->calibration[1].rawCenter - txSignal.controlPacket.rudder,
				settings->calibration[2].rawCenter - txSignal.controlPacket.elevator,
				settings->calibration[3].rawCenter - txSignal.controlPacket.aileron,
				settings->calibration[4].rawCenter - txSignal.controlPacket.channel5,
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
