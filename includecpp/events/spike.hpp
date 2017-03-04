#ifndef LIBCAER_EVENTS_SPIKE_HPP_
#define LIBCAER_EVENTS_SPIKE_HPP_

#include <libcaer/events/spike.h>
#include "common.hpp"

namespace libcaer {
namespace events {

class SpikeEventPacket: public EventPacketHeader {
public:
	using SpikeEventBase = struct caer_spike_event;

	struct SpikeEvent: public SpikeEventBase {
		int32_t getTimestamp() const noexcept {
			return (caerSpikeEventGetTimestamp(this));
		}

		int64_t getTimestamp64(const SpikeEventPacket &packet) const noexcept {
			return (caerSpikeEventGetTimestamp64(this, reinterpret_cast<caerSpikeEventPacketConst>(packet.header)));
		}

		void setTimestamp(int32_t ts) {
			if (ts < 0) {
				throw std::invalid_argument("Negative timestamp not allowed.");
			}

			caerSpikeEventSetTimestamp(this, ts);
		}

		bool isValid() const noexcept {
			return (caerSpikeEventIsValid(this));
		}

		void validate(SpikeEventPacket &packet) noexcept {
			caerSpikeEventValidate(this, reinterpret_cast<caerSpikeEventPacket>(packet.header));
		}

		void invalidate(SpikeEventPacket &packet) noexcept {
			caerSpikeEventInvalidate(this, reinterpret_cast<caerSpikeEventPacket>(packet.header));
		}

		uint8_t getSourceCoreID() const noexcept {
			return (caerSpikeEventGetSourceCoreID(this));
		}

		void setSourceCoreID(uint8_t coreID) noexcept {
			caerSpikeEventSetSourceCoreID(this, coreID);
		}

		uint8_t getChipID() const noexcept {
			return (caerSpikeEventGetChipID(this));
		}

		void setChipID(uint8_t chipID) noexcept {
			caerSpikeEventSetChipID(this, chipID);
		}

		uint32_t getNeuronID() const noexcept {
			return (caerSpikeEventGetNeuronID(this));
		}

		void setNeuronID(uint32_t neuronID) noexcept {
			caerSpikeEventSetNeuronID(this, neuronID);
		}

		uint16_t getX() const noexcept {
			return (caerSpikeEventGetX(this));
		}

		uint16_t getY() const noexcept {
			return (caerSpikeEventGetY(this));
		}
	};

	// Constructors.
	SpikeEventPacket(int32_t eventCapacity, int16_t eventSource, int32_t tsOverflow) {
		if (eventCapacity <= 0) {
			throw std::invalid_argument("Negative or zero event capacity not allowed on construction.");
		}

		caerSpikeEventPacket packet = caerSpikeEventPacketAllocate(eventCapacity, eventSource, tsOverflow);
		if (packet == nullptr) {
			throw std::runtime_error("Failed to allocate spike event packet.");
		}

		header = &packet->packetHeader;
	}

	// EventPacketHeader's destructor takes care of freeing above memory.
	// Same for all copy/move constructor/assignment, use EventPacketHeader.

	SpikeEvent &getEvent(int32_t index) {
		if (index < 0 || index >= capacity()) {
			throw std::out_of_range("Index out of range.");
		}

		SpikeEventBase *evtBase = caerSpikeEventPacketGetEvent(reinterpret_cast<caerSpikeEventPacket>(header), index);
		SpikeEvent *evt = static_cast<SpikeEvent *>(evtBase);

		return (*evt);
	}

	const SpikeEvent &getEvent(int32_t index) const {
		if (index < 0 || index >= capacity()) {
			throw std::out_of_range("Index out of range.");
		}

		const SpikeEventBase *evtBase = caerSpikeEventPacketGetEventConst(
			reinterpret_cast<caerSpikeEventPacketConst>(header), index);
		const SpikeEvent *evt = static_cast<const SpikeEvent *>(evtBase);

		return (*evt);
	}

	SpikeEvent &operator[](size_t index) {
		return (getEvent(static_cast<int32_t>(index)));
	}

	const SpikeEvent &operator[](size_t index) const {
		return (getEvent(static_cast<int32_t>(index)));
	}
};

}
}

#endif /* LIBCAER_EVENTS_SPIKE_HPP_ */
