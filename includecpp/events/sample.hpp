#ifndef LIBCAER_EVENTS_SAMPLE_HPP_
#define LIBCAER_EVENTS_SAMPLE_HPP_

#include <libcaer/events/sample.h>
#include "common.hpp"

namespace libcaer {
namespace events {

class SampleEventPacket: public EventPacketHeader {
public:
	using SampleEventBase = struct caer_sample_event;

	struct SampleEvent: public SampleEventBase {
		int32_t getTimestamp() const noexcept {
			return (caerSampleEventGetTimestamp(this));
		}

		int64_t getTimestamp64(const SampleEventPacket &packet) const noexcept {
			return (caerSampleEventGetTimestamp64(this, reinterpret_cast<caerSampleEventPacketConst>(packet.header)));
		}

		void setTimestamp(int32_t ts) {
			if (ts < 0) {
				throw std::invalid_argument("Negative timestamp not allowed.");
			}

			caerSampleEventSetTimestamp(this, ts);
		}

		bool isValid() const noexcept {
			return (caerSampleEventIsValid(this));
		}

		void validate(SampleEventPacket &packet) noexcept {
			caerSampleEventValidate(this, reinterpret_cast<caerSampleEventPacket>(packet.header));
		}

		void invalidate(SampleEventPacket &packet) noexcept {
			caerSampleEventInvalidate(this, reinterpret_cast<caerSampleEventPacket>(packet.header));
		}

		uint8_t getType() const noexcept {
			return (caerSampleEventGetType(this));
		}

		void setType(uint8_t t) noexcept {
			return (caerSampleEventSetType(this, t));
		}

		uint32_t getSample() const noexcept {
			return (caerSampleEventGetSample(this));
		}

		void setSample(uint32_t s) noexcept {
			return (caerSampleEventSetSample(this, s));
		}
	};

	// Constructors.
	SampleEventPacket(int32_t eventCapacity, int16_t eventSource, int32_t tsOverflow) {
		if (eventCapacity <= 0) {
			throw std::invalid_argument("Negative or zero event capacity not allowed on construction.");
		}

		caerSampleEventPacket packet = caerSampleEventPacketAllocate(eventCapacity, eventSource, tsOverflow);
		if (packet == nullptr) {
			throw std::runtime_error("Failed to allocate sample event packet.");
		}

		header = &packet->packetHeader;
	}

	SampleEventPacket(caerSampleEventPacket packet) {
		if (packet == nullptr) {
			throw std::runtime_error("Failed to initialize sample event packet from existing C struct.");
		}

		header = &packet->packetHeader;
	}

	// EventPacketHeader's destructor takes care of freeing above memory.
	// Same for all copy/move constructor/assignment, use EventPacketHeader.

	SampleEvent &getEvent(int32_t index) {
		if (index < 0 || index >= capacity()) {
			throw std::out_of_range("Index out of range.");
		}

		SampleEventBase *evtBase = caerSampleEventPacketGetEvent(reinterpret_cast<caerSampleEventPacket>(header),
			index);
		SampleEvent *evt = static_cast<SampleEvent *>(evtBase);

		return (*evt);
	}

	const SampleEvent &getEvent(int32_t index) const {
		if (index < 0 || index >= capacity()) {
			throw std::out_of_range("Index out of range.");
		}

		const SampleEventBase *evtBase = caerSampleEventPacketGetEventConst(
			reinterpret_cast<caerSampleEventPacketConst>(header), index);
		const SampleEvent *evt = static_cast<const SampleEvent *>(evtBase);

		return (*evt);
	}

	SampleEvent &operator[](size_t index) {
		return (getEvent(static_cast<int32_t>(index)));
	}

	const SampleEvent &operator[](size_t index) const {
		return (getEvent(static_cast<int32_t>(index)));
	}
};

}
}

#endif /* LIBCAER_EVENTS_SAMPLE_HPP_ */
