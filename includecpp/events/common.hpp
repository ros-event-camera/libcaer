#ifndef LIBCAER_EVENTS_COMMON_HPP_
#define LIBCAER_EVENTS_COMMON_HPP_

#include <libcaer/events/common.h>
#include "../libcaer.hpp"
#include <cassert>

namespace libcaer {
namespace events {

template<class T>
class EventPacketIterator {
private:
	// Select proper pointer type (const or not) depending on template type.
	using eventPtrType = typename std::conditional<std::is_const<T>::value, const uint8_t *, uint8_t *>::type;

	eventPtrType eventPtr;
	size_t eventSize;

public:
	// Iterator traits.
	using iterator_category = std::random_access_iterator_tag;
	using value_type = T;
	using pointer = T *;
	using reference = T &;
	using difference_type = ptrdiff_t;
	using size_type = int32_t;

	// Constructors.
	EventPacketIterator() :
			eventPtr(nullptr),
			eventSize(0) {
	}

	EventPacketIterator(eventPtrType _eventPtr, size_t _eventSize) :
			eventPtr(_eventPtr),
			eventSize(_eventSize) {
	}

	// Data access operators.
	reference operator*() const noexcept {
		return (*static_cast<pointer>(eventPtr));
	}

	pointer operator->() const noexcept {
		return (static_cast<pointer>(eventPtr));
	}

	reference operator[](size_type index) const noexcept {
		return (*static_cast<pointer>(eventPtr + (index * eventSize)));
	}

	// Comparison operators.
	bool operator==(const EventPacketIterator &rhs) const noexcept {
		return (eventPtr == rhs.eventPtr);
	}

	bool operator!=(const EventPacketIterator &rhs) const noexcept {
		return (eventPtr != rhs.eventPtr);
	}

	bool operator<(const EventPacketIterator &rhs) const noexcept {
		return (eventPtr < rhs.eventPtr);
	}

	bool operator>(const EventPacketIterator &rhs) const noexcept {
		return (eventPtr > rhs.eventPtr);
	}

	bool operator<=(const EventPacketIterator &rhs) const noexcept {
		return (eventPtr <= rhs.eventPtr);
	}

	bool operator>=(const EventPacketIterator &rhs) const noexcept {
		return (eventPtr >= rhs.eventPtr);
	}

	// Prefix increment.
	EventPacketIterator& operator++() noexcept {
		eventPtr += eventSize;
		return (*this);
	}

	// Postfix increment.
	EventPacketIterator operator++(int) noexcept {
		eventPtrType currPtr = eventPtr;
		eventPtr += eventSize;
		return (EventPacketIterator(currPtr, eventSize));
	}

	// Prefix decrement.
	EventPacketIterator& operator--() noexcept {
		eventPtr -= eventSize;
		return (*this);
	}

	// Postfix decrement.
	EventPacketIterator operator--(int) noexcept {
		eventPtrType currPtr = eventPtr;
		eventPtr -= eventSize;
		return (EventPacketIterator(currPtr, eventSize));
	}

	// Iter += N.
	EventPacketIterator& operator+=(size_type add) noexcept {
		eventPtr += (eventSize * add);
		return (*this);
	}

	// Iter + N.
	EventPacketIterator operator+(size_type add) const noexcept {
		return (EventPacketIterator(eventPtr + (eventSize * add), eventSize));
	}

	// N + Iter. Must be friend as Iter is right-hand-side.
	friend EventPacketIterator operator+(size_type lhs, const EventPacketIterator &rhs) noexcept {
		return (EventPacketIterator(rhs.eventPtr + (rhs.eventSize * lhs), rhs.eventSize));
	}

	// Iter -= N.
	EventPacketIterator& operator-=(size_type sub) noexcept {
		eventPtr -= (eventSize * sub);
		return (*this);
	}

	// Iter - N. (N - Iter doesn't make sense!)
	EventPacketIterator operator-(size_type sub) const noexcept {
		return (EventPacketIterator(eventPtr - (eventSize * sub), eventSize));
	}

	// Iter - Iter. (Iter + Iter doesn't make sense!)
	difference_type operator-(const EventPacketIterator &rhs) const noexcept {
		// Distance in pointed-to-elements, so of eventSize size, hence
		// why the division by eventSize is necessary.
		return ((eventPtr - rhs.eventPtr) / eventSize);
	}

	// Swap two iterators.
	void swap(EventPacketIterator &rhs) noexcept {
		std::swap(eventPtr, rhs.eventPtr);
		std::swap(eventSize, rhs.eventSize);
	}
};

class EventPacketHeader {
protected:
	caerEventPacketHeader header;

	// Constructors.
	EventPacketHeader() :
			header(nullptr) {
	}

public:
	EventPacketHeader(caerEventPacketHeader packetHeader) {
		if (packetHeader == nullptr) {
			throw std::runtime_error(
				"Failed to initialize EventPacketHeader from existing C packet header: null pointer.");
		}

		if (caerEventPacketHeaderGetEventType(packetHeader) < CAER_DEFAULT_EVENT_TYPES_COUNT) {
			throw std::runtime_error(
				"Failed to initialize EventPacketHeader from existing C packet header: default event types are not allowed. "
					"Always call the proper specialized <Type>EventPacket constructor, to guarantee proper RTTI initialization.");
		}

		header = packetHeader;
	}

	// Destructor.
	virtual ~EventPacketHeader() {
		// All EventPackets must have been allocated somewhere on the heap,
		// and can thus always be passed to free(). free(nullptr) does nothing.
		free(header);
	}

	// Copy constructor.
	EventPacketHeader(const EventPacketHeader &rhs) {
		// Full copy.
		header = internalCopy(rhs.header);
	}

	// Copy assignment.
	EventPacketHeader &operator=(const EventPacketHeader &rhs) {
		// If both the same, do nothing.
		if (this != &rhs) {
			// Different packets, so we need to check if they are the same type.
			if (getEventType() != rhs.getEventType()) {
				throw std::invalid_argument("Event type must be the same.");
			}

			// They are, so we can make a copy, and if successful, put it in place
			// of the old data. internalCopy() checks for nullptr.
			caerEventPacketHeader copy = internalCopy(rhs.header);

			free(header);

			header = copy;
		}

		return (*this);
	}

	// Move constructor.
	EventPacketHeader(EventPacketHeader &&rhs) noexcept {
		// Moved-from object must remain in a valid state. We can define
		// valid-state-after-move to be nothing allowed but a destructor
		// call, which is what normally happens, and helps us a lot here.

		// Move data here.
		header = rhs.header;

		// Reset old data (ready for destruction).
		rhs.header = nullptr;
	}

	// Move assignment.
	EventPacketHeader &operator=(EventPacketHeader &&rhs) {
		assert(this != &rhs);

		// Different packets, so we need to check if they are the same type.
		if (getEventType() != rhs.getEventType()) {
			throw std::invalid_argument("Event type must be the same.");
		}

		// Moved-from object must remain in a valid state. We can define
		// valid-state-after-move to be nothing allowed but a destructor
		// call, which is what normally happens, and helps us a lot here.

		// Destroy current data.
		free(header);

		// Move data here.
		header = rhs.header;

		// Reset old data (ready for destruction).
		rhs.header = nullptr;

		return (*this);
	}

	// Comparison operators.
	bool operator==(const EventPacketHeader &rhs) const noexcept {
		return (caerEventPacketEquals(header, rhs.header));
	}

	bool operator!=(const EventPacketHeader &rhs) const noexcept {
		return (!caerEventPacketEquals(header, rhs.header));
	}

	// Header data methods.
	int16_t getEventType() const noexcept {
		return (caerEventPacketHeaderGetEventType(header));
	}

	void setEventType(int16_t eventType) {
		if (eventType < 0) {
			throw std::invalid_argument("Negative event type not allowed.");
		}

		return (caerEventPacketHeaderSetEventType(header, eventType));
	}

	int16_t getEventSource() const noexcept {
		return (caerEventPacketHeaderGetEventSource(header));
	}

	void setEventSource(int16_t eventSource) {
		if (eventSource < 0) {
			throw std::invalid_argument("Negative event source not allowed.");
		}

		return (caerEventPacketHeaderSetEventSource(header, eventSource));
	}

	int32_t getEventSize() const noexcept {
		return (caerEventPacketHeaderGetEventSize(header));
	}

	void setEventSize(int32_t eventSize) {
		if (eventSize < 0) {
			throw std::invalid_argument("Negative event size not allowed.");
		}

		return (caerEventPacketHeaderSetEventSize(header, eventSize));
	}

	int32_t getEventTSOffset() const noexcept {
		return (caerEventPacketHeaderGetEventTSOffset(header));
	}

	void setEventTSOffset(int32_t eventTSOffset) {
		if (eventTSOffset < 0) {
			throw std::invalid_argument("Negative event TS offset not allowed.");
		}

		return (caerEventPacketHeaderSetEventTSOffset(header, eventTSOffset));
	}

	int32_t getEventTSOverflow() const noexcept {
		return (caerEventPacketHeaderGetEventTSOverflow(header));
	}

	void setEventTSOverflow(int32_t eventTSOverflow) {
		if (eventTSOverflow < 0) {
			throw std::invalid_argument("Negative event TS overflow not allowed.");
		}

		return (caerEventPacketHeaderSetEventTSOverflow(header, eventTSOverflow));
	}

	int32_t getEventCapacity() const noexcept {
		return (caerEventPacketHeaderGetEventCapacity(header));
	}

	void setEventCapacity(int32_t eventCapacity) {
		if (eventCapacity < 0) {
			throw std::invalid_argument("Negative event capacity not allowed.");
		}

		return (caerEventPacketHeaderSetEventCapacity(header, eventCapacity));
	}

	int32_t getEventNumber() const noexcept {
		return (caerEventPacketHeaderGetEventNumber(header));
	}

	void setEventNumber(int32_t eventNumber) {
		if (eventNumber < 0) {
			throw std::invalid_argument("Negative event number not allowed.");
		}

		return (caerEventPacketHeaderSetEventNumber(header, eventNumber));
	}

	int32_t getEventValid() const noexcept {
		return (caerEventPacketHeaderGetEventValid(header));
	}

	void setEventValid(int32_t eventValid) {
		if (eventValid < 0) {
			throw std::invalid_argument("Negative event valid not allowed.");
		}

		return (caerEventPacketHeaderSetEventValid(header, eventValid));
	}

	// Generic Event definiton.
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

	// Container traits.
	using value_type = const GenericEvent;
	using pointer = const GenericEvent *;
	using const_pointer = const GenericEvent *;
	using reference = const GenericEvent &;
	using const_reference = const GenericEvent &;
	using size_type = int32_t;
	using difference_type = ptrdiff_t;

	// Generic Event access methods.
	value_type genericGetEvent(size_type index) const {
		if (index < 0 || index >= capacity()) {
			throw std::out_of_range("Index out of range.");
		}

		const void *evt = caerGenericEventGetEvent(header, index);

		return (GenericEvent { evt, header });
	}

	// Generic Event Packet methods.
	void clear() noexcept {
		caerEventPacketClear(header);
	}

	void clean() noexcept {
		caerEventPacketClean(header);
	}

	void resize(size_type newEventCapacity) {
		if (newEventCapacity <= 0) {
			throw std::invalid_argument("Negative or zero event capacity not allowed.");
		}

		caerEventPacketHeader resizedPacket = caerEventPacketResize(header, newEventCapacity);
		if (resizedPacket == nullptr) {
			throw std::bad_alloc();
		}
		else {
			header = resizedPacket;
		}
	}

	void shrink_to_fit() {
		resize(getEventValid());
	}

	void grow(size_type newEventCapacity) {
		if (newEventCapacity <= 0) {
			throw std::invalid_argument("Negative or zero event capacity not allowed.");
		}
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

	virtual EventPacketHeader *copy() const {
		return (new EventPacketHeader(internalCopy(header)));
	}

	virtual EventPacketHeader *copyOnlyEvents() const {
		return (new EventPacketHeader(internalCopyOnlyEvents(header)));
	}

	virtual EventPacketHeader *copyOnlyValidEvents() const {
		return (new EventPacketHeader(internalCopyOnlyValidEvents(header)));
	}

	// Swap two event packets.
	void swap(EventPacketHeader &rhs) {
		if (getEventType() != rhs.getEventType()) {
			throw std::invalid_argument("Event type must be the same.");
		}

		std::swap(header, rhs.header);
	}

	// Direct underlying pointer access.
	caerEventPacketHeader getHeaderPointer() noexcept {
		return (header);
	}

	caerEventPacketHeaderConst getHeaderPointer() const noexcept {
		return (header);
	}

	// Convenience methods.
	size_type capacity() const noexcept {
		return (getEventCapacity());
	}

	size_type size() const noexcept {
		return (getEventNumber());
	}

	bool empty() const noexcept {
		return (getEventNumber() == 0);
	}

protected:
	// Internal copy functions.
	static caerEventPacketHeader internalCopy(caerEventPacketHeaderConst header) {
		void *packetCopy = caerEventPacketCopy(header);
		if (packetCopy == nullptr) {
			throw std::bad_alloc();
		}

		return (static_cast<caerEventPacketHeader>(packetCopy));
	}

	static caerEventPacketHeader internalCopyOnlyEvents(caerEventPacketHeaderConst header) {
		if (caerEventPacketHeaderGetEventNumber(header) == 0) {
			throw std::runtime_error("Copy would result in empty result.");
		}

		void *packetCopy = caerEventPacketCopyOnlyEvents(header);
		if (packetCopy == nullptr) {
			throw std::bad_alloc();
		}

		return (static_cast<caerEventPacketHeader>(packetCopy));
	}

	static caerEventPacketHeader internalCopyOnlyValidEvents(caerEventPacketHeaderConst header) {
		if (caerEventPacketHeaderGetEventValid(header) == 0) {
			throw std::runtime_error("Copy would result in empty result.");
		}

		void *packetCopy = caerEventPacketCopyOnlyValidEvents(header);
		if (packetCopy == nullptr) {
			throw std::bad_alloc();
		}

		return (static_cast<caerEventPacketHeader>(packetCopy));
	}
};

}
}

#endif /* LIBCAER_EVENTS_COMMON_HPP_ */
