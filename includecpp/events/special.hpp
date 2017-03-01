#ifndef LIBCAER_EVENTS_SPECIAL_HPP_
#define LIBCAER_EVENTS_SPECIAL_HPP_

#include <libcaer/events/special.h>
#include "common.hpp"

namespace libcaer {
namespace events {

using SpecialEvent = struct caer_special_event;

class SpecialEventProxy {
private:
	caerSpecialEvent event;
	caerSpecialEventPacket packet;

public:
	SpecialEventProxy(caerSpecialEvent e, caerSpecialEventPacket p) :
		event(e), packet(p) {
	}

	int32_t getTimestamp() const noexcept {
		return (caerSpecialEventGetTimestamp(event));
	}

	int64_t getTimestamp64() const noexcept {
		return (caerSpecialEventGetTimestamp64(event, packet));
	}

	void setTimestamp(int32_t timestamp) const noexcept {
		caerSpecialEventSetTimestamp(event, timestamp);
	}
};

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

	SpecialEventProxy getEvent(int32_t index) {
		return (SpecialEventProxy(internalGetEvent(index), reinterpret_cast<caerSpecialEventPacket>(header)));
	}

	const SpecialEventProxy getEvent(int32_t index) const {
		return (SpecialEventProxy(internalGetEvent(index), reinterpret_cast<caerSpecialEventPacket>(header)));
	}

	SpecialEventProxy operator[](size_t index) {
		return (getEvent(index));
	}

	const SpecialEventProxy operator[](size_t index) const {
		return (getEvent(index));
	}

private:
	caerSpecialEvent internalGetEvent(int32_t index) const {
		if (index < 0 || index >= size()) {
			throw std::out_of_range("Index out of range.");
		}

		return (caerSpecialEventPacketGetEvent(reinterpret_cast<caerSpecialEventPacket>(header), index));
	}
};

class SpecialEventIterator: public std::iterator<std::random_access_iterator_tag, SpecialEvent> {

};

}
}

#endif /* LIBCAER_EVENTS_SPECIAL_HPP_ */
