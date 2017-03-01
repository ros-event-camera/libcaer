#ifndef LIBCAER_EVENTS_COMMON_HPP_
#define LIBCAER_EVENTS_COMMON_HPP_

#include <libcaer/events/common.h>
#include <stdexcept>

namespace libcaer {
namespace events {

class EventPacketHeader {
protected:
	caerEventPacketHeader header;

	EventPacketHeader() :
		header(nullptr) {
	}

public:
	~EventPacketHeader() {
		// All EventPackets must have been allocated somewhere on the heap,
		// and can thus always be passed to free(). free(nullptr) does nothing.
		free(header);
	}

	int16_t getEventType() const noexcept {
		return (caerEventPacketHeaderGetEventType(header));
	}

	void setEventType(int16_t eventType) const noexcept {
		return (caerEventPacketHeaderSetEventType(header, eventType));
	}

	int16_t getEventSource() const noexcept {
		return (caerEventPacketHeaderGetEventSource(header));
	}

	void setEventSource(int16_t eventSource) const noexcept {
		return (caerEventPacketHeaderSetEventSource(header, eventSource));
	}

	int32_t getEventSize() const noexcept {
		return (caerEventPacketHeaderGetEventSize(header));
	}

	void setEventSize(int32_t eventSize) const noexcept {
		return (caerEventPacketHeaderSetEventSize(header, eventSize));
	}

	int32_t getEventTSOffset() const noexcept {
		return (caerEventPacketHeaderGetEventTSOffset(header));
	}

	void setEventTSOffset(int32_t eventTSOffset) const noexcept {
		return (caerEventPacketHeaderSetEventTSOffset(header, eventTSOffset));
	}

	int32_t getEventTSOverflow() const noexcept {
		return (caerEventPacketHeaderGetEventTSOverflow(header));
	}

	void setEventTSOverflow(int32_t eventTSOverflow) const noexcept {
		return (caerEventPacketHeaderSetEventTSOverflow(header, eventTSOverflow));
	}

	int32_t getEventCapacity() const noexcept {
		return (caerEventPacketHeaderGetEventCapacity(header));
	}

	void setEventCapacity(int32_t eventCapacity) const noexcept {
		return (caerEventPacketHeaderSetEventCapacity(header, eventCapacity));
	}

	int32_t getEventNumber() const noexcept {
		return (caerEventPacketHeaderGetEventNumber(header));
	}

	void setEventNumber(int32_t eventNumber) const noexcept {
		return (caerEventPacketHeaderSetEventNumber(header, eventNumber));
	}

	int32_t getEventValid() const noexcept {
		return (caerEventPacketHeaderGetEventValid(header));
	}

	void setEventValid(int32_t eventValid) const noexcept {
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

	// Generic event methods.
	struct GenericEvent {
		const void *event;
		caerEventPacketHeaderConst header;

		int32_t getTimestamp() const noexcept {
			return (caerGenericEventGetTimestamp(event, header));
		}

		int64_t getTimestamp64() const noexcept {
			return (caerGenericEventGetTimestamp64(event, header));
		}

		bool isValid() const noexcept {
			return (caerGenericEventIsValid(event));
		}
	};

	const GenericEvent genericGetEvent(int32_t index) const {
		if (index < 0 || index >= capacity()) {
			throw std::out_of_range("Index out of range.");
		}

		const void *evt = caerGenericEventGetEvent(header, index);

		return (GenericEvent { evt, header });
	}

	// Generic copy/clean methods.
	void *copy() {
		void *packetCopy = caerCopyEventPacket(header);
		if (packetCopy == nullptr) {
			throw std::bad_alloc();
		}

		return (packetCopy);
	}

	void *copyOnlyEvents() {
		void *packetCopy = caerCopyEventPacketOnlyEvents(header);
		if (packetCopy == nullptr) {
			throw std::bad_alloc();
		}

		return (packetCopy);
	}

	void *copyOnlyValidEvents() {
		void *packetCopy = caerCopyEventPacketOnlyValidEvents(header);
		if (packetCopy == nullptr) {
			throw std::bad_alloc();
		}

		return (packetCopy);
	}

	void clean() {
		caerCleanEventPacket(header);
	}

	// Convenience methods.
	int32_t capacity() const noexcept {
		return (getEventCapacity());
	}

	int32_t size() const noexcept {
		return (getEventNumber());
	}

	bool empty() const noexcept {
		return (getEventNumber() == 0);
	}
};

}
}

#endif /* LIBCAER_EVENTS_COMMON_HPP_ */
