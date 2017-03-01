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
			return (caerSpecialEventGetTimestamp64(this, packet.packet));
		}

		void setTimestamp(int32_t timestamp) noexcept {
			caerSpecialEventSetTimestamp(this, timestamp);
		}

		bool isValid() const noexcept {
			return (caerSpecialEventIsValid(this));
		}

		void validate(SpecialEventPacket &packet) noexcept {
			caerSpecialEventValidate(this, packet.packet);
		}

		void invalidate(SpecialEventPacket &packet) noexcept {
			caerSpecialEventInvalidate(this, packet.packet);
		}

		uint8_t getType() const noexcept {
			return (caerSpecialEventGetType(this));
		}

		void setType(uint8_t type) noexcept {
			caerSpecialEventSetType(this, type);
		}

		uint32_t getData() const noexcept {
			return (caerSpecialEventGetData(this));
		}

		void setData(uint32_t data) noexcept {
			caerSpecialEventSetData(this, data);
		}
	};

	SpecialEventPacket(int32_t eventCapacity, int16_t eventSource, int32_t tsOverflow) {
		packet = caerSpecialEventPacketAllocate(eventCapacity, eventSource, tsOverflow);
		if (packet == nullptr) {
			throw std::runtime_error("Failed to allocate special event packet.");
		}

		header = &packet->packetHeader;
	}

	// EventPacketHeader's destructor takes care of freeing above memory.

	SpecialEvent &getEvent(int32_t index) {
		return (internalGetEvent(index));
	}

	const SpecialEvent &getEvent(int32_t index) const {
		return (internalGetEvent(index));
	}

	SpecialEvent &operator[](size_t index) {
		return (internalGetEvent(index));
	}

	const SpecialEvent &operator[](size_t index) const {
		return (internalGetEvent(index));
	}

	SpecialEvent &findEventByType(uint8_t type) {
		SpecialEventBase *evtBase = caerSpecialEventPacketFindEventByType(packet, type);
		if (evtBase == nullptr) {
			throw std::range_error("Special Event of particular type not found.");
		}

		SpecialEvent *evt = static_cast<SpecialEvent *>(evtBase);

		return (*evt);
	}

	SpecialEvent &findValidEventByType(uint8_t type) {
		SpecialEventBase *evtBase = caerSpecialEventPacketFindValidEventByType(packet, type);
		if (evtBase == nullptr) {
			throw std::range_error("Special Event of particular type not found.");
		}

		SpecialEvent *evt = static_cast<SpecialEvent *>(evtBase);

		return (*evt);
	}

private:
	caerSpecialEventPacket packet;

	SpecialEvent &internalGetEvent(int32_t index) {
		if (index < 0 || index >= size()) {
			throw std::out_of_range("Index out of range.");
		}

		SpecialEventBase *evtBase = caerSpecialEventPacketGetEvent(packet, index);
		SpecialEvent *evt = static_cast<SpecialEvent *>(evtBase);

		return (*evt);
	}

	const SpecialEvent &internalGetEvent(int32_t index) const {
		if (index < 0 || index >= size()) {
			throw std::out_of_range("Index out of range.");
		}

		const SpecialEventBase *evtBase = caerSpecialEventPacketGetEventConst(packet, index);
		const SpecialEvent *evt = static_cast<const SpecialEvent *>(evtBase);

		return (*evt);
	}
};

}
}

#endif /* LIBCAER_EVENTS_SPECIAL_HPP_ */
