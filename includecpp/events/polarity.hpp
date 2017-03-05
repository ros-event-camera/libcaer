#ifndef LIBCAER_EVENTS_POLARITY_HPP_
#define LIBCAER_EVENTS_POLARITY_HPP_

#include <libcaer/events/polarity.h>
#include "common.hpp"

namespace libcaer {
namespace events {

class PolarityEventPacket: public EventPacketHeader {
public:
	using PolarityEventBase = struct caer_polarity_event;

	struct PolarityEvent: public PolarityEventBase {
		int32_t getTimestamp() const noexcept {
			return (caerPolarityEventGetTimestamp(this));
		}

		int64_t getTimestamp64(const PolarityEventPacket &packet) const noexcept {
			return (caerPolarityEventGetTimestamp64(this, reinterpret_cast<caerPolarityEventPacketConst>(packet.header)));
		}

		void setTimestamp(int32_t ts) {
			if (ts < 0) {
				throw std::invalid_argument("Negative timestamp not allowed.");
			}

			caerPolarityEventSetTimestamp(this, ts);
		}

		bool isValid() const noexcept {
			return (caerPolarityEventIsValid(this));
		}

		void validate(PolarityEventPacket &packet) noexcept {
			caerPolarityEventValidate(this, reinterpret_cast<caerPolarityEventPacket>(packet.header));
		}

		void invalidate(PolarityEventPacket &packet) noexcept {
			caerPolarityEventInvalidate(this, reinterpret_cast<caerPolarityEventPacket>(packet.header));
		}

		bool getPolarity() const noexcept {
			return (caerPolarityEventGetPolarity(this));
		}

		void setPolarity(bool pol) noexcept {
			caerPolarityEventSetPolarity(this, pol);
		}

		uint16_t getY() const noexcept {
			return (caerPolarityEventGetY(this));
		}

		void setY(uint16_t y) noexcept {
			caerPolarityEventSetY(this, y);
		}

		uint16_t getX() const noexcept {
			return (caerPolarityEventGetX(this));
		}

		void setX(uint16_t x) noexcept {
			caerPolarityEventSetX(this, x);
		}
	};

	// Constructors.
	PolarityEventPacket(int32_t eventCapacity, int16_t eventSource, int32_t tsOverflow) {
		if (eventCapacity <= 0) {
			throw std::invalid_argument("Negative or zero event capacity not allowed on construction.");
		}

		caerPolarityEventPacket packet = caerPolarityEventPacketAllocate(eventCapacity, eventSource, tsOverflow);
		if (packet == nullptr) {
			throw std::runtime_error("Failed to allocate polarity event packet.");
		}

		header = &packet->packetHeader;
	}

	PolarityEventPacket(caerPolarityEventPacket packet) {
		if (packet == nullptr) {
			throw std::runtime_error("Failed to initialize event packet from existing C packet: null pointer.");
		}

		// Check for proper event type too!
		if (caerEventPacketHeaderGetEventType(&packet->packetHeader) != POLARITY_EVENT) {
			throw std::runtime_error("Failed to initialize event packet from existing C packet: wrong type.");
		}

		header = &packet->packetHeader;
	}

	PolarityEventPacket(caerEventPacketHeader packetHeader) {
		if (packetHeader == nullptr) {
			throw std::runtime_error("Failed to initialize event packet from existing C packet header: null pointer.");
		}

		// Check for proper event type too!
		if (caerEventPacketHeaderGetEventType(packetHeader) != POLARITY_EVENT) {
			throw std::runtime_error("Failed to initialize event packet from existing C packet header: wrong type.");
		}

		header = packetHeader;
	}

	// EventPacketHeader's destructor takes care of freeing above memory.
	// Same for all copy/move constructor/assignment, use EventPacketHeader.

	PolarityEvent &getEvent(int32_t index) {
		if (index < 0 || index >= capacity()) {
			throw std::out_of_range("Index out of range.");
		}

		PolarityEventBase *evtBase = caerPolarityEventPacketGetEvent(reinterpret_cast<caerPolarityEventPacket>(header),
			index);
		PolarityEvent *evt = static_cast<PolarityEvent *>(evtBase);

		return (*evt);
	}

	const PolarityEvent &getEvent(int32_t index) const {
		if (index < 0 || index >= capacity()) {
			throw std::out_of_range("Index out of range.");
		}

		const PolarityEventBase *evtBase = caerPolarityEventPacketGetEventConst(
			reinterpret_cast<caerPolarityEventPacketConst>(header), index);
		const PolarityEvent *evt = static_cast<const PolarityEvent *>(evtBase);

		return (*evt);
	}

	PolarityEvent &operator[](size_t index) {
		return (getEvent(static_cast<int32_t>(index)));
	}

	const PolarityEvent &operator[](size_t index) const {
		return (getEvent(static_cast<int32_t>(index)));
	}

	PolarityEventPacket copy() const {
		return (PolarityEventPacket(internalCopy(header)));
	}

	PolarityEventPacket copyOnlyEvents() const {
		return (PolarityEventPacket(internalCopyOnlyEvents(header)));
	}

	PolarityEventPacket copyOnlyValidEvents() const {
		return (PolarityEventPacket(internalCopyOnlyValidEvents(header)));
	}
};

}
}

#endif /* LIBCAER_EVENTS_POLARITY_HPP_ */
