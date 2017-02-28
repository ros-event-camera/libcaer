#ifndef LIBCAER_EVENTS_COMMON_HPP_
#define LIBCAER_EVENTS_COMMON_HPP_

#include <libcaer/events/common.h>
#include <stdexcept>

namespace libcaer {
namespace events {

class EventPacketHeader {
protected:
	caerEventPacketHeader header;

	EventPacketHeader() {
		header = nullptr;
	}

	~EventPacketHeader() {
		// All EventPackets must have been allocated somewhere on the heap,
		// and can thus always be passed to free(). free(nullptr) does nothing.
		free(header);
	}

public:
	int16_t getEventType() {
		return (caerEventPacketHeaderGetEventType(header));
	}

	void setEventType(int16_t eventType) {
		return (caerEventPacketHeaderSetEventType(header, eventType));
	}

	int16_t getEventSource() {
		return (caerEventPacketHeaderGetEventSource(header));
	}

	void setEventSource(int16_t eventSource) {
		return (caerEventPacketHeaderSetEventSource(header, eventSource));
	}

	int32_t getEventSize() {
		return (caerEventPacketHeaderGetEventSize(header));
	}

	void setEventSize(int32_t eventSize) {
		return (caerEventPacketHeaderSetEventSize(header, eventSize));
	}

	int32_t getEventTSOffset() {
		return (caerEventPacketHeaderGetEventTSOffset(header));
	}

	void setEventTSOffset(int32_t eventTSOffset) {
		return (caerEventPacketHeaderSetEventTSOffset(header, eventTSOffset));
	}

	int32_t getEventTSOverflow() {
		return (caerEventPacketHeaderGetEventTSOverflow(header));
	}

	void setEventTSOverflow(int32_t eventTSOverflow) {
		return (caerEventPacketHeaderSetEventTSOverflow(header, eventTSOverflow));
	}

	int32_t getEventCapacity() {
		return (caerEventPacketHeaderGetEventCapacity(header));
	}

	void setEventCapacity(int32_t eventCapacity) {
		return (caerEventPacketHeaderSetEventCapacity(header, eventCapacity));
	}

	int32_t getEventNumber() {
		return (caerEventPacketHeaderGetEventNumber(header));
	}

	void setEventNumber(int32_t eventNumber) {
		return (caerEventPacketHeaderSetEventNumber(header, eventNumber));
	}

	int32_t getEventValid() {
		return (caerEventPacketHeaderGetEventValid(header));
	}

	void setEventValid(int32_t eventValid) {
		return (caerEventPacketHeaderSetEventValid(header, eventValid));
	}

	void grow(int32_t newEventCapacity) {
		caerEventPacketHeader enlargedPacket = caerGenericEventPacketGrow(header, newEventCapacity);
		if (enlargedPacket == nullptr) {
			throw std::runtime_error("Failed to grow this event packet.");
		}
		else {
			header = enlargedPacket;
		}
	}

	void append(const EventPacketHeader appendPacket) {
		caerEventPacketHeader mergedPacket = caerGenericEventPacketAppend(header, appendPacket.header);
		if (mergedPacket == nullptr) {
			throw std::runtime_error("Failed to append event packet to this one.");
		}
		else {
			header = mergedPacket;
		}
	}
};

}
}

#endif /* LIBCAER_EVENTS_COMMON_HPP_ */
