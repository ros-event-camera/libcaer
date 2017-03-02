#ifndef LIBCAER_EVENTS_COMMON_HPP_
#define LIBCAER_EVENTS_COMMON_HPP_

#include <libcaer/events/common.h>
#include <stdexcept>
#include <cassert>

namespace libcaer {
namespace events {

class EventPacketHeader {
protected:
	caerEventPacketHeader header;

	// Constructors.
	EventPacketHeader() :
		header(nullptr) {
	}

private:
	EventPacketHeader(caerEventPacketHeader h) :
		header(h) {
	}

public:
	// Destructor.
	~EventPacketHeader() {
		// All EventPackets must have been allocated somewhere on the heap,
		// and can thus always be passed to free(). free(nullptr) does nothing.
		free(header);
		header = nullptr;
	}

	// Copy constructor.
	EventPacketHeader(const EventPacketHeader &rhs) {
		// Full copy.
		caerEventPacketHeader copy = static_cast<caerEventPacketHeader>(caerEventPacketCopy(rhs.header));
		if (copy == nullptr) {
			throw std::runtime_error("Failed to copy construct event packet.");
		}

		header = copy;
	}

	// Copy assignment.
	EventPacketHeader &operator=(const EventPacketHeader &rhs) {
		// If both the same, do nothing.
		if (this != &rhs) {
			// Different packets, so we need to check if they are the same type and size.
			if (getEventType() != rhs.getEventType()) {
				throw std::invalid_argument("Event type must be the same.");
			}
			if (getEventSize() != rhs.getEventSize()) {
				throw std::invalid_argument("Event size must be the same.");
			}

			// They are, so we can make a copy, and if successful, put it in place
			// of the old data.
			caerEventPacketHeader copy = static_cast<caerEventPacketHeader>(caerEventPacketCopy(rhs.header));
			if (copy == nullptr) {
				throw std::runtime_error("Failed to copy assign event packet.");
			}

			free(header);
			header = nullptr;

			header = copy;
		}

		return (*this);
	}

	// Move constructor.
	EventPacketHeader(EventPacketHeader &&rhs) {
		header = rhs.header;
		rhs.header = nullptr; // TODO: this is not a valid state!
	}

	// Move assignment.
	EventPacketHeader &operator=(EventPacketHeader &&rhs) {
		assert(this != &rhs);

		// Different packets, so we need to check if they are the same type and size.
		if (getEventType() != rhs.getEventType()) {
			throw std::invalid_argument("Event type must be the same.");
		}
		if (getEventSize() != rhs.getEventSize()) {
			throw std::invalid_argument("Event size must be the same.");
		}

		// They are, so we can move the data to this packet.
		free(header);
		header = nullptr;

		header = rhs.header;
		rhs.header = nullptr; // TODO: this is not a valid state!

		return (*this);
	}

	// Header data methods.
	int16_t getEventType() const noexcept {
		return (caerEventPacketHeaderGetEventType(header));
	}

	void setEventType(int16_t eventType) const {
		if (eventType < 0) {
			throw std::invalid_argument("Negative event type not allowed.");
		}

		return (caerEventPacketHeaderSetEventType(header, eventType));
	}

	int16_t getEventSource() const noexcept {
		return (caerEventPacketHeaderGetEventSource(header));
	}

	void setEventSource(int16_t eventSource) const {
		if (eventSource < 0) {
			throw std::invalid_argument("Negative event source not allowed.");
		}

		return (caerEventPacketHeaderSetEventSource(header, eventSource));
	}

	int32_t getEventSize() const noexcept {
		return (caerEventPacketHeaderGetEventSize(header));
	}

	void setEventSize(int32_t eventSize) const {
		if (eventSize < 0) {
			throw std::invalid_argument("Negative event size not allowed.");
		}

		return (caerEventPacketHeaderSetEventSize(header, eventSize));
	}

	int32_t getEventTSOffset() const noexcept {
		return (caerEventPacketHeaderGetEventTSOffset(header));
	}

	void setEventTSOffset(int32_t eventTSOffset) const {
		if (eventTSOffset < 0) {
			throw std::invalid_argument("Negative event TS offset not allowed.");
		}

		return (caerEventPacketHeaderSetEventTSOffset(header, eventTSOffset));
	}

	int32_t getEventTSOverflow() const noexcept {
		return (caerEventPacketHeaderGetEventTSOverflow(header));
	}

	void setEventTSOverflow(int32_t eventTSOverflow) const {
		if (eventTSOverflow < 0) {
			throw std::invalid_argument("Negative event TS overflow not allowed.");
		}

		return (caerEventPacketHeaderSetEventTSOverflow(header, eventTSOverflow));
	}

	int32_t getEventCapacity() const noexcept {
		return (caerEventPacketHeaderGetEventCapacity(header));
	}

	void setEventCapacity(int32_t eventCapacity) const {
		if (eventCapacity < 0) {
			throw std::invalid_argument("Negative event capacity not allowed.");
		}

		return (caerEventPacketHeaderSetEventCapacity(header, eventCapacity));
	}

	int32_t getEventNumber() const noexcept {
		return (caerEventPacketHeaderGetEventNumber(header));
	}

	void setEventNumber(int32_t eventNumber) const {
		if (eventNumber < 0) {
			throw std::invalid_argument("Negative event number not allowed.");
		}

		return (caerEventPacketHeaderSetEventNumber(header, eventNumber));
	}

	int32_t getEventValid() const noexcept {
		return (caerEventPacketHeaderGetEventValid(header));
	}

	void setEventValid(int32_t eventValid) const {
		if (eventValid < 0) {
			throw std::invalid_argument("Negative event valid not allowed.");
		}

		return (caerEventPacketHeaderSetEventValid(header, eventValid));
	}

	// Generic Event methods.
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

	// Generic Event Packet methods.
	void clean() noexcept {
		caerEventPacketClean(header);
	}

	void resize(int32_t newEventCapacity) {
		caerEventPacketHeader resizedPacket = caerEventPacketResize(header, newEventCapacity);
		if (resizedPacket == nullptr) {
			throw std::bad_alloc();
		}
		else {
			header = resizedPacket;
		}
	}

	void grow(int32_t newEventCapacity) {
		if (newEventCapacity <= getEventCapacity()) {
			throw std::invalid_argument("New event capacity must be strictly bigger than old one.");
		}

		caerEventPacketHeader enlargedPacket = caerEventPacketGrow(header, newEventCapacity);
		if (enlargedPacket == nullptr) {
			throw std::bad_alloc();
		}
		else {
			header = enlargedPacket;
		}
	}

	void append(const EventPacketHeader &appendPacket) {
		if (getEventType() != appendPacket.getEventType()) {
			throw std::invalid_argument("Event type must be the same.");
		}
		if (getEventSize() != appendPacket.getEventSize()) {
			throw std::invalid_argument("Event size must be the same.");
		}
		if (getEventTSOverflow() != appendPacket.getEventTSOverflow()) {
			throw std::invalid_argument("Event TS overflow must be the same.");
		}

		caerEventPacketHeader mergedPacket = caerEventPacketAppend(header, appendPacket.header);
		if (mergedPacket == nullptr) {
			throw std::bad_alloc();
		}
		else {
			header = mergedPacket;
		}
	}

	EventPacketHeader copy() {
		void *packetCopy = caerEventPacketCopy(header);
		if (packetCopy == nullptr) {
			throw std::bad_alloc();
		}

		return (EventPacketHeader(static_cast<caerEventPacketHeader>(packetCopy)));
	}

	EventPacketHeader copyOnlyEvents() {
		if (getEventNumber() == 0) {
			throw std::runtime_error("Copy would result in empty result.");
		}

		void *packetCopy = caerEventPacketCopyOnlyEvents(header);
		if (packetCopy == nullptr) {
			throw std::bad_alloc();
		}

		return (EventPacketHeader(static_cast<caerEventPacketHeader>(packetCopy)));
	}

	EventPacketHeader copyOnlyValidEvents() {
		if (getEventValid() == 0) {
			throw std::runtime_error("Copy would result in empty result.");
		}

		void *packetCopy = caerEventPacketCopyOnlyValidEvents(header);
		if (packetCopy == nullptr) {
			throw std::bad_alloc();
		}

		return (EventPacketHeader(static_cast<caerEventPacketHeader>(packetCopy)));
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
