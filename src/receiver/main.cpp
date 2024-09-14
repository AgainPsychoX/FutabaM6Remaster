#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include "common/packets.hpp"

////////////////////////////////////////////////////////////////////////////////
// Hardware

RF24 radio(7, 8);

const uint8_t transmitterOutputAddress[6] = "ctrl!";
const uint8_t transmitterInputAddress[6]  = "info?";

#define PIN_BATTERY A7

////////////////////////////////////////////////////////////////////////////////
// State

struct SignalStabilityCounter
{
	static constexpr uint16_t updateInterval = 500;
	static constexpr uint16_t averageCountForInterval = 200;

	unsigned long lastUpdateTime = 0;
	
	unsigned long timeSinceLastTransmitterSignalSums = 0;
	
	uint16_t goodCount = 0;
	uint16_t weakCount = 0;

	uint8_t lastRating = 0;

	inline void probe()
	{
		if (radio.testRPD()) {
			goodCount += 1;
		}
		else {
			weakCount += 1;
		}
	}

	inline void update()
	{
		if (lastUpdateTime + updateInterval < millis()) {
			lastUpdateTime = millis();
			
			if (goodCount == 0 && weakCount == 0) {
				// No signal
				lastRating = 0;
			}
			else {
				lastRating = (
					constrain(33 * (goodCount + weakCount) / averageCountForInterval, 0, 33) +
					33 * (goodCount) / (goodCount + weakCount)
				);

#ifdef DEBUG
	// #ifdef ANTENNA_TESTING
				printf("signalStability::update() |||||||||||||||||| count: %3d   count/average ratio: %3d%%   good/weak ratio: %3d%%   average delta time: %3lu  --> RATING: %3u   fail? %u\n",
					(goodCount + weakCount),
					100 * (goodCount + weakCount) / averageCountForInterval,
					100 * (goodCount) / (goodCount + weakCount),
					timeSinceLastTransmitterSignalSums / (goodCount + weakCount),
					lastRating,
					radio.failureDetected
				);
	// #endif
#endif

				timeSinceLastTransmitterSignalSums = 0;
				goodCount = 0;
				weakCount = 0;
			}
		}
	}
};
SignalStabilityCounter signalStability;

unsigned long lastTransmitterSignalTime = 0;

TransmitterSignal txSignal;
ReceiverSignal receiverSignal;

////////////////////////////////////////////////////////////////////////////////
// Setup

int serial_putc(char c, FILE *) { Serial.write(c); return c; }

void setup()
{
	// Initialize the serial port
	Serial.begin(115200);
	Serial.println(F("Setup!"));
	fdevopen(&serial_putc, 0);

	// Initialize radio and start listening to allow read
	radio.begin();  
	radio.setDataRate(RF24_250KBPS);
	radio.setPALevel(RF24_PA_MAX);
	radio.setAutoAck(false);
	radio.setRetries(0, 0);
	radio.setPayloadSize(staticPayloadSize);
	radio.setCRCLength(RF24_CRC_8);
	radio.openReadingPipe(1, transmitterOutputAddress);
	radio.openWritingPipe(transmitterInputAddress);
	radio.startListening();
}

////////////////////////////////////////////////////////////////////////////////
// Loop

void loop()
{
	// Update signal stability counters
	signalStability.update();
	
	// Receive transmitter signal
	if (radio.available()) {
		unsigned long timeSinceLastTransmitterSignal = millis() - lastTransmitterSignalTime;
		lastTransmitterSignalTime = millis();

		radio.read(&txSignal, sizeof(txSignal));
		
		signalStability.probe();
		signalStability.timeSinceLastTransmitterSignalSums += timeSinceLastTransmitterSignal;

		printf(
			"time=%lu\t"
			"signalRating=%u\t"
			"testRPD=%u\t"
			"timeSinceLastTxSignal=%lu\t"
			"throttle=%hu\t"
			"rudder=%hu\t"
			"elevator=%hu\t"
			"aileron=%hu\t"
			"channel5=%hu\t"
			"aux1=%u\t"
			"aux2=%u\t"
			"aux3=%u\t"
			"battery=%u\n",
			millis(), 
			signalStability.lastRating,
			radio.testRPD(),
			timeSinceLastTransmitterSignal,
			txSignal.controlPacket.throttle,
			txSignal.controlPacket.rudder,
			txSignal.controlPacket.elevator,
			txSignal.controlPacket.aileron,
			txSignal.controlPacket.channel5,
			txSignal.controlPacket.aux1,
			txSignal.controlPacket.aux2,
			txSignal.controlPacket.aux3,
			analogRead(PIN_BATTERY)
		);
	}
}
