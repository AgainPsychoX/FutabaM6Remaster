#include <tuple>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <Fonts/FreeSans12pt7b.h>
#include <Fonts/FreeSans9pt7b.h>
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

const char* channelNames[] = {
	"Throttle", "Rudder", "Elevator", "Aileron", "Channel 5", 
	"Aux 1", "Aux 2", "Aux 3",
};

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
	Calibrate,  // Setup analog min/center/max reference values on each control,
                // microseconds min/center/max for the servos for the receiver.
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
uint16_t rawAnalogValues[6];
uint16_t mappedValues[6];

TransmitterSignal txSignal;
ReceiverSignal rxSignal;

unsigned long lastTxSignalTime = 0;
unsigned long lastRxSignalTime = 0;
constexpr unsigned int rxSignalFetchInterval = 500; // ms
constexpr unsigned int rxSignalListenDuration = 20; // ms
constexpr unsigned int rxSignalLostDuration = 1200; // ms
unsigned long lastRxSignalLastLatency = 0;

AnalogChannel selectedChannel;
int8_t parameterSelected;
int16_t extraBias;

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

uint16_t mapAnalogValue(uint16_t value, AnalogChannelCalibrationData& calibration)
{
	// The safety constrain is applied in the receiver side, keeping servos in range 700-2300 us.
#if 1 // Two curves based on min & center and center & max values
	if (value < calibration.rawCenter)
		return map(value, calibration.rawMin, calibration.rawCenter, calibration.usMin, calibration.usCenter);
	else
		return map(value, calibration.rawCenter, calibration.rawMax, calibration.usCenter, calibration.usMax);
#else // Single linear curve based on min & max values
	return map(value, calibration.rawMin, calibration.rawMax, calibration.usMin, calibration.usMax);
#endif
}

AnalogChannel trySelectChannel()
{
	for (int8_t i = 0; i < 5; i++) {
		int delta = settings->calibration[i].rawCenter - rawAnalogValues[i];
		if (delta < 0) delta = -delta;
		if (delta > 100) {
			return static_cast<AnalogChannel>(i);
		}
	}
	return AnalogChannel::Unknown;
}

/// Returns pair of values representing deltas (from center) from the other 
/// than currently selected joystick: X/Y, with values always growing 
/// from left to right and top to bottom.
std::tuple<int16_t, int16_t> getOtherThanSelectedJoystickDeltas()
{
	switch (selectedChannel) {
		case AnalogChannel::Channel5:
		case AnalogChannel::Throttle:
		case AnalogChannel::Rudder: {
			// Using right joystick
			auto xAxisIdx = static_cast<int8_t>(AnalogChannel::Aileron);
			auto yAxisIdx = static_cast<int8_t>(AnalogChannel::Elevator);
			return {
				rawAnalogValues[xAxisIdx] - settings->calibration[xAxisIdx].rawCenter,
				rawAnalogValues[yAxisIdx] - settings->calibration[yAxisIdx].rawCenter,
			};
		}
		case AnalogChannel::Elevator:
		case AnalogChannel::Aileron: {
			// Using left joystick (throttle is inverted for some reason)
			auto xAxisIdx = static_cast<int8_t>(AnalogChannel::Rudder);
			auto yAxisIdx = static_cast<int8_t>(AnalogChannel::Throttle);
			return {
				rawAnalogValues[xAxisIdx] - settings->calibration[xAxisIdx].rawCenter,
				settings->calibration[yAxisIdx].rawCenter - rawAnalogValues[yAxisIdx],
			};
		}
		default:
			return { 0, 0 };
	}
}

void loop()
{
	unsigned long now = millis();

	// Read raw analog values
	rawAnalogValues[0] = analogRead(THROTTLE_PIN);
	rawAnalogValues[1] = analogRead(RUDDER_PIN);
	rawAnalogValues[2] = analogRead(ELEVATOR_PIN);
	rawAnalogValues[3] = analogRead(AILERON_PIN);
	rawAnalogValues[4] = analogRead(CHANNEL_5_PIN);
	rawAnalogValues[5] = 0;

	// Map the values to microseconds
	mappedValues[0] = mapAnalogValue(rawAnalogValues[0], settings->calibration[0]);
	mappedValues[1] = mapAnalogValue(rawAnalogValues[1], settings->calibration[1]);
	mappedValues[2] = mapAnalogValue(rawAnalogValues[2], settings->calibration[2]);
	mappedValues[3] = mapAnalogValue(rawAnalogValues[3], settings->calibration[3]);
	mappedValues[4] = mapAnalogValue(rawAnalogValues[4], settings->calibration[4]);
	mappedValues[5] = mapAnalogValue(rawAnalogValues[5], settings->calibration[5]);
	// TODO: clean it up somehow, feels very messy...
	
	// Send transmitter signal
	txSignal.packetType = PacketType::Control;
	txSignal.controlPacket.throttle = mappedValues[0];
	txSignal.controlPacket.rudder   = mappedValues[1];
	txSignal.controlPacket.elevator = mappedValues[2];
	txSignal.controlPacket.aileron  = mappedValues[3];
	txSignal.controlPacket.channel5 = mappedValues[4];
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
				tft.fillScreen(ST77XX_BLACK);
				switch (page) {
					case Page::Calibrate:
					case Page::Reverse: {
						selectedChannel = AnalogChannel::Unknown;
						parameterSelected = 0;
						extraBias = 0;
						break;
					}
					default: 
						break;
				}
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
	tft.setTextColor(ST77XX_WHITE);
	tft.setFont(); // to default
	tft.setCursor(0, 0);
	// TODO: prevent blinking of the screen when drawing stuff
	switch (page) {
		case Page::Info: {
			tft.setFont(&FreeSans9pt7b);
			tft.setCursor(0, 20);
			tft.printf("Nadajnik:");
			tft.setCursor(0, 40);
			tft.printf("Odbiornik:");
			tft.setCursor(0, 60);
			tft.printf("Sygnal:");

			tft.fillRect(96, 0, 160 - 96 - 1, 60, ST77XX_BLACK);
			tft.setFont(&FreeSans12pt7b);
			tft.setCursor(96, 20);
			tft.printf("%.2fV", txBatteryFactor * txBatteryRaw); // TODO: show only 1 digit after dot, if >10V
			tft.setCursor(96, 40);
			tft.printf("%.2fV", rxSignal.statusPacket.battery);
			tft.setCursor(96, 60);
			tft.printf("%hhu", rxSignal.statusPacket.signalRating);

			// TODO: show signal lost when no packet comes in for a while,
			//  maybe even include packets latency/missing in the signal rating.
			// tft.setFont(); // to default
			// tft.fillRect(0, 60, 160, 20, ST77XX_BLACK);
			// tft.setCursor(0, 80 - 12);
			// tft.printf("L=%lu", lastRxSignalLastLatency);
			break;
		}
		case Page::Raw: {
			tft.fillScreen(ST77XX_BLACK);
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
				rawAnalogValues[0],
				rawAnalogValues[1],
				rawAnalogValues[2],
				rawAnalogValues[3],
				rawAnalogValues[4],
				txSignal.controlPacket.aux1,
				txSignal.controlPacket.aux2,
				txSignal.controlPacket.aux3
			);
			break;
		}
		case Page::Centered: {
			tft.fillScreen(ST77XX_BLACK);
			tft.print("Wartosci od srodka:");

			tft.setFont(&FreeSans9pt7b);
			constexpr int div = 6; // losing some accuracy for easier displaying & reading
			tft.setCursor(0, 12 + 1 * 16);
			tft.printf("THR: %hd", (settings->calibration[0].usCenter - txSignal.controlPacket.throttle) / div);
			tft.setCursor(0, 12 + 2 * 16);
			tft.printf("RUD: %hd", (settings->calibration[1].usCenter - txSignal.controlPacket.rudder)   / div);
			tft.setCursor(80, 12 + 1 * 16);
			tft.printf("ELV: %hd", (settings->calibration[2].usCenter - txSignal.controlPacket.elevator) / div);
			tft.setCursor(80, 12 + 2 * 16);
			tft.printf("AIL: %hd", (settings->calibration[3].usCenter - txSignal.controlPacket.aileron)  / div);
			tft.setCursor(0, 12 + 3 * 16);
			tft.printf("CH5: %hd", (settings->calibration[4].usCenter - txSignal.controlPacket.channel5) / div);

			tft.setFont(); // to default
			tft.setCursor(0, 80 - 12);
			tft.printf(
				"AUX1: %u  AUX2: %u  AUX3: %u", 
				txSignal.controlPacket.aux1,
				txSignal.controlPacket.aux2,
				txSignal.controlPacket.aux3
			);

			if (wasLongPress) {
				settings->calibration[0].usCenter = txSignal.controlPacket.throttle;
				settings->calibration[1].usCenter = txSignal.controlPacket.rudder;
				settings->calibration[2].usCenter = txSignal.controlPacket.elevator;
				settings->calibration[3].usCenter = txSignal.controlPacket.aileron;
				settings->calibration[4].usCenter = txSignal.controlPacket.channel5;
				settings->prepareForSave();
				EEPROM.commit();
			}
			break;
		}
		case Page::Calibrate:
		case Page::Reverse: {
			tft.setCursor(0, 0);
			tft.printf(page == Page::Calibrate ? "Kalibracja" : "Odwracanie");
			if (selectedChannel == AnalogChannel::Unknown) {
				tft.setCursor(20, 20);
				tft.printf("Wybierz kanal");
				selectedChannel = trySelectChannel();
				if (selectedChannel != AnalogChannel::Unknown)
					tft.fillScreen(ST77XX_BLACK);
			}
			else /* any channel selected */ {
				auto& c = settings->calibration[static_cast<int8_t>(selectedChannel)];
				tft.setCursor(20, 12);
				tft.printf("Kanal: %s", channelNames[static_cast<int8_t>(selectedChannel)]);
				if (parameterSelected == -1) {
					tft.setCursor(20, 24);
					tft.print("Zapisano!");
				}
				else /* any parameter selected */ {
					const auto [x, y] = getOtherThanSelectedJoystickDeltas();
					if (x < -100 || 100 < x)
						extraBias += x / 100;

					// TODO: move parameterSelected with y, but still require long press to save
					// TODO: preview the calibration values below
					// TODO: allow extraBias for potentiometers too
					// TODO: warn if extraBias is invalid
					// TODO: make the extraBias raise even slower closer to center

					// Print current value (raw & mapped)
					tft.fillRect(2 + 24, 24, 32, 12, ST77XX_BLACK);
					tft.setCursor(2, 24);
					tft.printf("raw=%u\n", rawAnalogValues[static_cast<int8_t>(selectedChannel)]);
					tft.fillRect(82 + 18, 24, 32, 12, ST77XX_BLACK);
					tft.setCursor(82, 24);
					tft.printf("us=%u\n", mappedValues[static_cast<int8_t>(selectedChannel)] + extraBias);

					constexpr auto valuesStartY = 48;

					// When in calibration mode, print ">" to mark the selected parameter
					if (page == Page::Calibrate) {
						constexpr uint16_t markColor = 0x7BEF; // gray
						if (parameterSelected < 3)
							tft.drawRect(0, valuesStartY - 2 + parameterSelected * 12, 76, 12, markColor);
						else
							tft.drawRect(80, valuesStartY - 2 + (parameterSelected - 3) * 12, 76, 12, markColor);
					}

					// Print the calibration values
					tft.setCursor(2, valuesStartY + 0 * 12);
					tft.printf("rawMin=%u", c.rawMin);
					tft.setCursor(2, valuesStartY + 1 * 12);
					tft.printf("rawCntr=%u", c.rawCenter);
					tft.setCursor(2, valuesStartY + 2 * 12);
					tft.printf("rawMax=%u", c.rawMax);
					tft.setCursor(82, valuesStartY + 0 * 12);
					tft.printf("usMin=%u", c.usMin);
					tft.setCursor(82, valuesStartY + 1 * 12);
					tft.printf("usCntr=%u", c.usCenter);
					tft.setCursor(82, valuesStartY + 2 * 12);
					tft.printf("usMax=%u", c.usMax);
				}
				// On long press, save the current value as specific 
				if (wasLongPress) {
					if (page == Page::Calibrate) {
						switch (parameterSelected++) {
							case 0: c.rawMin    = rawAnalogValues[static_cast<int8_t>(selectedChannel)] + extraBias; break;
							case 1: c.rawCenter = rawAnalogValues[static_cast<int8_t>(selectedChannel)] + extraBias; break;
							case 2: c.rawMax    = rawAnalogValues[static_cast<int8_t>(selectedChannel)] + extraBias; break;
							case 3: c.usMin     = mappedValues[static_cast<int8_t>(selectedChannel)] + extraBias; break;
							case 4: c.usCenter  = mappedValues[static_cast<int8_t>(selectedChannel)] + extraBias; break;
							case 5: // On last one, commit to the EEPROM and show "Saved" message
								c.usMax = mappedValues[static_cast<int8_t>(selectedChannel)] + extraBias;
								parameterSelected = -1; // to show "Saved" message
								extraBias = 0;
								settings->prepareForSave();
								EEPROM.commit();
								break;
							default:
								break;
						}
					}
					else /* (page == Page::Reverse) */ {
						auto tmp = c.usMin;
						c.usMin = c.usMax;
						c.usMax = tmp;
						settings->prepareForSave();
						EEPROM.commit();
					}
					tft.fillScreen(ST77XX_BLACK);
				}
			}
			break;
		}
		default:
			break;
	}
}
