#ifndef LIBCAER_EVENTS_SPECIAL_HPP_
#define LIBCAER_EVENTS_SPECIAL_HPP_

#include <libcaer/events/special.h>
#include "common.hpp"

namespace libcaer {
namespace events {

using SpecialEvent = struct caer_special_event;

class SpecialEventPacket: public EventPacketHeader {
public:
	SpecialEventPacket(int32_t eventCapacity, int16_t eventSource, int32_t tsOverflow) {
		caerSpecialEventPacket packet = caerSpecialEventPacketAllocate(eventCapacity, eventSource, tsOverflow);
		if (packet == nullptr) {
			throw std::runtime_error("Failed to allocate special event packet.");
		}

		header = &packet->packetHeader;
	}

	// EventPacketHeader's destructor takes care of freeing above memory.
};

}
}

#endif /* LIBCAER_EVENTS_SPECIAL_HPP_ */
