#ifndef LIBCAER_EVENTS_FRAME_HPP_
#define LIBCAER_EVENTS_FRAME_HPP_

#include <libcaer/events/frame.h>
#include "common.hpp"

namespace libcaer {
namespace events {

class FrameEventPacket: public EventPacketHeader {
public:
	using FrameEventBase = struct caer_frame_event;

	struct FrameEvent: public FrameEventBase {
		int32_t getTimestamp() const noexcept {
			return (caerFrameEventGetTimestamp(this));
		}

		int64_t getTimestamp64(const FrameEventPacket &packet) const noexcept {
			return (caerFrameEventGetTimestamp64(this, reinterpret_cast<caerFrameEventPacketConst>(packet.header)));
		}

		int32_t getTSStartOfFrame() const noexcept {
			return (caerFrameEventGetTSStartOfFrame(this));
		}

		int64_t getTSStartOfFrame64(const FrameEventPacket &packet) const noexcept {
			return (caerFrameEventGetTSStartOfFrame64(this, reinterpret_cast<caerFrameEventPacketConst>(packet.header)));
		}

		void setTSStartOfFrame(int32_t ts) {
			if (ts < 0) {
				throw std::invalid_argument("Negative timestamp not allowed.");
			}

			caerFrameEventSetTSStartOfFrame(this, ts);
		}

		int32_t getTSEndOfFrame() const noexcept {
			return (caerFrameEventGetTSEndOfFrame(this));
		}

		int64_t getTSEndOfFrame64(const FrameEventPacket &packet) const noexcept {
			return (caerFrameEventGetTSEndOfFrame64(this, reinterpret_cast<caerFrameEventPacketConst>(packet.header)));
		}

		void setTSEndOfFrame(int32_t ts) {
			if (ts < 0) {
				throw std::invalid_argument("Negative timestamp not allowed.");
			}

			caerFrameEventSetTSEndOfFrame(this, ts);
		}

		int32_t getTSStartOfExposure() const noexcept {
			return (caerFrameEventGetTSStartOfExposure(this));
		}

		int64_t getTSStartOfExposure64(const FrameEventPacket &packet) const noexcept {
			return (caerFrameEventGetTSStartOfExposure64(this,
				reinterpret_cast<caerFrameEventPacketConst>(packet.header)));
		}

		void setTSStartOfExposure(int32_t ts) {
			if (ts < 0) {
				throw std::invalid_argument("Negative timestamp not allowed.");
			}

			caerFrameEventSetTSStartOfExposure(this, ts);
		}

		int32_t getTSEndOfExposure() const noexcept {
			return (caerFrameEventGetTSEndOfExposure(this));
		}

		int64_t getTSEndOfExposure64(const FrameEventPacket &packet) const noexcept {
			return (caerFrameEventGetTSEndOfExposure64(this, reinterpret_cast<caerFrameEventPacketConst>(packet.header)));
		}

		void setTSEndOfExposure(int32_t ts) {
			if (ts < 0) {
				throw std::invalid_argument("Negative timestamp not allowed.");
			}

			caerFrameEventSetTSEndOfExposure(this, ts);
		}

		bool isValid() const noexcept {
			return (caerFrameEventIsValid(this));
		}

		void validate(FrameEventPacket &packet) noexcept {
			caerFrameEventValidate(this, reinterpret_cast<caerFrameEventPacket>(packet.header));
		}

		void invalidate(FrameEventPacket &packet) noexcept {
			caerFrameEventInvalidate(this, reinterpret_cast<caerFrameEventPacket>(packet.header));
		}
	};

	// Constructors.
	FrameEventPacket(int32_t eventCapacity, int16_t eventSource, int32_t tsOverflow, int32_t maxLengthX,
		int32_t maxLengthY, int16_t maxChannelNumber) {
		if (eventCapacity <= 0) {
			throw std::invalid_argument("Negative or zero event capacity not allowed on construction.");
		}

		caerFrameEventPacket packet = caerFrameEventPacketAllocate(eventCapacity, eventSource, tsOverflow, maxLengthX,
			maxLengthY, maxChannelNumber);
		if (packet == nullptr) {
			throw std::runtime_error("Failed to allocate frame event packet.");
		}

		header = &packet->packetHeader;
	}

	// EventPacketHeader's destructor takes care of freeing above memory.
	// Same for all copy/move constructor/assignment, use EventPacketHeader.

	FrameEvent &getEvent(int32_t index) {
		if (index < 0 || index >= capacity()) {
			throw std::out_of_range("Index out of range.");
		}

		FrameEventBase *evtBase = caerFrameEventPacketGetEvent(reinterpret_cast<caerFrameEventPacket>(header), index);
		FrameEvent *evt = static_cast<FrameEvent *>(evtBase);

		return (*evt);
	}

	const FrameEvent &getEvent(int32_t index) const {
		if (index < 0 || index >= capacity()) {
			throw std::out_of_range("Index out of range.");
		}

		const FrameEventBase *evtBase = caerFrameEventPacketGetEventConst(
			reinterpret_cast<caerFrameEventPacketConst>(header), index);
		const FrameEvent *evt = static_cast<const FrameEvent *>(evtBase);

		return (*evt);
	}

	FrameEvent &operator[](size_t index) {
		return (getEvent(static_cast<int32_t>(index)));
	}

	const FrameEvent &operator[](size_t index) const {
		return (getEvent(static_cast<int32_t>(index)));
	}
};

}
}

#endif /* LIBCAER_EVENTS_FRAME_HPP_ */
