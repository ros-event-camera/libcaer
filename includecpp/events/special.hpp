#ifndef LIBCAER_EVENTS_SPECIAL_HPP_
#define LIBCAER_EVENTS_SPECIAL_HPP_

#include <libcaer/events/special.h>
#include "common.hpp"

namespace libcaer {
namespace events {

class SpecialEventPacket: public EventPacketHeader {
public:
	using SpecialEventBase = struct caer_special_event;

	struct SpecialEvent: public SpecialEventBase {
		int32_t getTimestamp() const noexcept {
			return (caerSpecialEventGetTimestamp(this));
		}

		int64_t getTimestamp64(const SpecialEventPacket &packet) const noexcept {
			return (caerSpecialEventGetTimestamp64(this, reinterpret_cast<caerSpecialEventPacketConst>(packet.header)));
		}

		void setTimestamp(int32_t ts) {
			if (ts < 0) {
				throw std::invalid_argument("Negative timestamp not allowed.");
			}

			caerSpecialEventSetTimestamp(this, ts);
		}

		bool isValid() const noexcept {
			return (caerSpecialEventIsValid(this));
		}

		void validate(SpecialEventPacket &packet) noexcept {
			caerSpecialEventValidate(this, reinterpret_cast<caerSpecialEventPacket>(packet.header));
		}

		void invalidate(SpecialEventPacket &packet) noexcept {
			caerSpecialEventInvalidate(this, reinterpret_cast<caerSpecialEventPacket>(packet.header));
		}

		uint8_t getType() const noexcept {
			return (caerSpecialEventGetType(this));
		}

		void setType(uint8_t t) noexcept {
			caerSpecialEventSetType(this, t);
		}

		uint32_t getData() const noexcept {
			return (caerSpecialEventGetData(this));
		}

		void setData(uint32_t d) noexcept {
			caerSpecialEventSetData(this, d);
		}
	};

	// Constructors.
	SpecialEventPacket(int32_t eventCapacity, int16_t eventSource, int32_t tsOverflow) {
		if (eventCapacity <= 0) {
			throw std::invalid_argument("Negative or zero event capacity not allowed on construction.");
		}

		caerSpecialEventPacket packet = caerSpecialEventPacketAllocate(eventCapacity, eventSource, tsOverflow);
		if (packet == nullptr) {
			throw std::runtime_error("Failed to allocate special event packet.");
		}

		header = &packet->packetHeader;
	}

	SpecialEventPacket(caerSpecialEventPacket packet) {
		if (packet == nullptr) {
			throw std::runtime_error("Failed to initialize event packet from existing C packet: null pointer.");
		}

		// Check for proper event type too!
		if (caerEventPacketHeaderGetEventType(&packet->packetHeader) != SPECIAL_EVENT) {
			throw std::runtime_error("Failed to initialize event packet from existing C packet: wrong type.");
		}

		header = &packet->packetHeader;
	}

	SpecialEventPacket(caerEventPacketHeader packetHeader) {
		if (packetHeader == nullptr) {
			throw std::runtime_error("Failed to initialize event packet from existing C packet header: null pointer.");
		}

		// Check for proper event type too!
		if (caerEventPacketHeaderGetEventType(packetHeader) != SPECIAL_EVENT) {
			throw std::runtime_error("Failed to initialize event packet from existing C packet header: wrong type.");
		}

		header = packetHeader;
	}

	// EventPacketHeader's destructor takes care of freeing above memory.
	// Same for all copy/move constructor/assignment, use EventPacketHeader.

	SpecialEvent &getEvent(int32_t index) {
		if (index < 0 || index >= capacity()) {
			throw std::out_of_range("Index out of range.");
		}

		SpecialEventBase *evtBase = caerSpecialEventPacketGetEvent(reinterpret_cast<caerSpecialEventPacket>(header),
			index);
		SpecialEvent *evt = static_cast<SpecialEvent *>(evtBase);

		return (*evt);
	}

	const SpecialEvent &getEvent(int32_t index) const {
		if (index < 0 || index >= capacity()) {
			throw std::out_of_range("Index out of range.");
		}

		const SpecialEventBase *evtBase = caerSpecialEventPacketGetEventConst(
			reinterpret_cast<caerSpecialEventPacketConst>(header), index);
		const SpecialEvent *evt = static_cast<const SpecialEvent *>(evtBase);

		return (*evt);
	}

	SpecialEvent &operator[](size_t index) {
		return (getEvent(static_cast<int32_t>(index)));
	}

	const SpecialEvent &operator[](size_t index) const {
		return (getEvent(static_cast<int32_t>(index)));
	}

	virtual SpecialEventPacket *copy() const override {
		return (new SpecialEventPacket(internalCopy(header)));
	}

	virtual SpecialEventPacket *copyOnlyEvents() const override {
		return (new SpecialEventPacket(internalCopyOnlyEvents(header)));
	}

	virtual SpecialEventPacket *copyOnlyValidEvents() const override {
		return (new SpecialEventPacket(internalCopyOnlyValidEvents(header)));
	}

	SpecialEvent &findEventByType(uint8_t type) {
		SpecialEventBase *evtBase = caerSpecialEventPacketFindEventByType(
			reinterpret_cast<caerSpecialEventPacket>(header), type);
		if (evtBase == nullptr) {
			throw std::range_error("Special Event of particular type not found.");
		}

		SpecialEvent *evt = static_cast<SpecialEvent *>(evtBase);

		return (*evt);
	}

	const SpecialEvent &findEventByType(uint8_t type) const {
		const SpecialEventBase *evtBase = caerSpecialEventPacketFindEventByTypeConst(
			reinterpret_cast<caerSpecialEventPacketConst>(header), type);
		if (evtBase == nullptr) {
			throw std::range_error("Special Event of particular type not found.");
		}

		const SpecialEvent *evt = static_cast<const SpecialEvent *>(evtBase);

		return (*evt);
	}

	SpecialEvent &findValidEventByType(uint8_t type) {
		SpecialEventBase *evtBase = caerSpecialEventPacketFindValidEventByType(
			reinterpret_cast<caerSpecialEventPacket>(header), type);
		if (evtBase == nullptr) {
			throw std::range_error("Valid Special Event of particular type not found.");
		}

		SpecialEvent *evt = static_cast<SpecialEvent *>(evtBase);

		return (*evt);
	}

	const SpecialEvent &findValidEventByType(uint8_t type) const {
		const SpecialEventBase *evtBase = caerSpecialEventPacketFindValidEventByTypeConst(
			reinterpret_cast<caerSpecialEventPacketConst>(header), type);
		if (evtBase == nullptr) {
			throw std::range_error("Valid Special Event of particular type not found.");
		}

		const SpecialEvent *evt = static_cast<const SpecialEvent *>(evtBase);

		return (*evt);
	}
};

}
}

#endif /* LIBCAER_EVENTS_SPECIAL_HPP_ */
