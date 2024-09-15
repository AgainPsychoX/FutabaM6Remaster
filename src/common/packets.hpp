#include <stdint.h>

#pragma pack(push)
#pragma pack(1)

enum class PacketType : uint8_t
{
	Unknown = 0,
	Control = 2,
	Status  = 3,
};

////////////////////////////////////////////////////////////////////////////////
// Transmitter

struct ControlPacket
{
	union {
		struct {
			bool requestingStatus : 1;
		};
		uint8_t flags;
	};
	uint16_t throttle;
	uint16_t rudder;
	uint16_t elevator;
	uint16_t aileron;
	uint16_t channel5;
	uint8_t aux1;
	uint8_t aux2;
	uint8_t aux3;
	uint8_t pad;
};

struct TransmitterSignal
{
	PacketType packetType = PacketType::Control;

	union {
		ControlPacket controlPacket;
	};
};

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
	};
};

////////////////////////////////////////////////////////////////////////////////

#pragma pack(pop)

static_assert(sizeof(TransmitterSignal) <= 16);
static_assert(sizeof(ReceiverSignal) <= 16);
constexpr uint8_t staticPayloadSize = 16;
