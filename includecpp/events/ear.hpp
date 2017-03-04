#ifndef LIBCAER_EVENTS_EAR_HPP_
#define LIBCAER_EVENTS_EAR_HPP_

#include <libcaer/events/ear.h>
#include "common.hpp"

namespace libcaer {
namespace events {

class EarEventPacket: public EventPacketHeader {
public:
	using EarEventBase = struct caer_ear_event;

	struct EarEvent: public EarEventBase {
		int32_t getTimestamp() const noexcept {
			return (caerEarEventGetTimestamp(this));
		}

		int64_t getTimestamp64(const EarEventPacket &packet) const noexcept {
			return (caerEarEventGetTimestamp64(this, reinterpret_cast<caerEarEventPacketConst>(packet.header)));
		}

		void setTimestamp(int32_t ts) {
			if (ts < 0) {
				throw std::invalid_argument("Negative timestamp not allowed.");
			}

			caerEarEventSetTimestamp(this, ts);
		}

		bool isValid() const noexcept {
			return (caerEarEventIsValid(this));
		}

		void validate(EarEventPacket &packet) noexcept {
			caerEarEventValidate(this, reinterpret_cast<caerEarEventPacket>(packet.header));
		}

		void invalidate(EarEventPacket &packet) noexcept {
			caerEarEventInvalidate(this, reinterpret_cast<caerEarEventPacket>(packet.header));
		}

		uint8_t getEar() const noexcept {
			return (caerEarEventGetEar(this));
		}

		void setEar(uint8_t e) noexcept {
			caerEarEventSetEar(this, e);
		}

		uint16_t getChannel() const noexcept {
			return (caerEarEventGetChannel(this));
		}

		void setChannel(uint16_t c) noexcept {
			caerEarEventSetChannel(this, c);
		}

		uint8_t getNeuron() const noexcept {
			return (caerEarEventGetNeuron(this));
		}

		void setNeuron(uint8_t n) noexcept {
			caerEarEventSetNeuron(this, n);
		}

		uint8_t getFilter() const noexcept {
			return (caerEarEventGetFilter(this));
		}

		void setFilter(uint8_t f) noexcept {
			caerEarEventSetFilter(this, f);
		}
	};

	// Constructors.
	EarEventPacket(int32_t eventCapacity, int16_t eventSource, int32_t tsOverflow) {
		if (eventCapacity <= 0) {
			throw std::invalid_argument("Negative or zero event capacity not allowed on construction.");
		}

		caerEarEventPacket packet = caerEarEventPacketAllocate(eventCapacity, eventSource, tsOverflow);
		if (packet == nullptr) {
			throw std::runtime_error("Failed to allocate ear event packet.");
		}

		header = &packet->packetHeader;
	}

	EarEventPacket(caerEarEventPacket packet) {
		if (packet == nullptr) {
			throw std::runtime_error("Failed to initialize ear event packet from existing C struct.");
		}

		header = &packet->packetHeader;
	}

	// EventPacketHeader's destructor takes care of freeing above memory.
	// Same for all copy/move constructor/assignment, use EventPacketHeader.

	EarEvent &getEvent(int32_t index) {
		if (index < 0 || index >= capacity()) {
			throw std::out_of_range("Index out of range.");
		}

		EarEventBase *evtBase = caerEarEventPacketGetEvent(reinterpret_cast<caerEarEventPacket>(header), index);
		EarEvent *evt = static_cast<EarEvent *>(evtBase);

		return (*evt);
	}

	const EarEvent &getEvent(int32_t index) const {
		if (index < 0 || index >= capacity()) {
			throw std::out_of_range("Index out of range.");
		}

		const EarEventBase *evtBase = caerEarEventPacketGetEventConst(reinterpret_cast<caerEarEventPacketConst>(header),
			index);
		const EarEvent *evt = static_cast<const EarEvent *>(evtBase);

		return (*evt);
	}

	EarEvent &operator[](size_t index) {
		return (getEvent(static_cast<int32_t>(index)));
	}

	const EarEvent &operator[](size_t index) const {
		return (getEvent(static_cast<int32_t>(index)));
	}
};

}
}

#endif /* LIBCAER_EVENTS_EAR_HPP_ */
