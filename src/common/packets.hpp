#include <stdint.h>

////////////////////////////////////////////////////////////////////////////////
// Other

enum class AnalogChannel : int8_t
{
	Throttle = 0,
	Rudder,
	Elevator,
	Aileron,
	Channel5,
	Unknown = -1,
};

struct AnalogChannelCalibrationData
{
	uint16_t rawMin;
	uint16_t rawCenter;
	uint16_t rawMax;
	uint16_t usMin;
	uint16_t usCenter;
	uint16_t usMax;
};

typedef AnalogChannelCalibrationData AnalogChannelsCalibration[6];

////////////////////////////////////////////////////////////////////////////////
// Common

#pragma pack(push)
#pragma pack(1)

constexpr uint8_t staticPayloadSize = 16;

enum class PacketType : uint8_t
{
	Unknown = 0,
	Control = 2,
	Status  = 3,
	SetServosCalibration = 4,
	GetServosCalibration = 5,
};

struct CalibrationPacket
{
	AnalogChannel channel;
	AnalogChannelCalibrationData data;
	uint8_t _pad[2];
};
static_assert(sizeof(CalibrationPacket) <= staticPayloadSize - 1);

////////////////////////////////////////////////////////////////////////////////
// Transmitter

enum class TransmitterRequest : uint8_t
{
	None = 0,
	Status = 3,
	AnalogCalibration = 5,
};

struct ControlPacket
{
	// Extra request from the receiver
	TransmitterRequest request;

	// Main data
	uint16_t throttle;
	uint16_t rudder;
	uint16_t elevator;
	uint16_t aileron;
	uint16_t channel5;
	uint8_t aux1;
	uint8_t aux2;
	uint8_t aux3;

	// Extra data
	union {
		AnalogChannel channel; // selection for analog calibration request
	};
};

struct TransmitterSignal
{
	PacketType packetType = PacketType::Control;

	union {
		ControlPacket controlPacket;
		CalibrationPacket calibrationPacket;
	};
};
static_assert(sizeof(TransmitterSignal) <= staticPayloadSize);

////////////////////////////////////////////////////////////////////////////////
// Receiver

struct StatusPacket
{
	union {
		struct {
			bool goodSignal : 1;
		};
		uint8_t flags;
	};
	uint8_t signalRating;
	float battery;
};

struct ReceiverSignal
{
	PacketType packetType = PacketType::Status;

	union {
		StatusPacket statusPacket;
		CalibrationPacket calibrationPacket;
	};
};
static_assert(sizeof(ReceiverSignal) <= staticPayloadSize);

////////////////////////////////////////////////////////////////////////////////

#pragma pack(pop)
