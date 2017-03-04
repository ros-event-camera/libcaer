#ifndef LIBCAER_EVENTS_IMU6_HPP_
#define LIBCAER_EVENTS_IMU6_HPP_

#include <libcaer/events/imu6.h>
#include "common.hpp"

namespace libcaer {
namespace events {

class IMU6EventPacket: public EventPacketHeader {
public:
	using IMU6EventBase = struct caer_imu6_event;

	struct IMU6Event: public IMU6EventBase {
		int32_t getTimestamp() const noexcept {
			return (caerIMU6EventGetTimestamp(this));
		}

		int64_t getTimestamp64(const IMU6EventPacket &packet) const noexcept {
			return (caerIMU6EventGetTimestamp64(this, reinterpret_cast<caerIMU6EventPacketConst>(packet.header)));
		}

		void setTimestamp(int32_t ts) {
			if (ts < 0) {
				throw std::invalid_argument("Negative timestamp not allowed.");
			}

			caerIMU6EventSetTimestamp(this, ts);
		}

		bool isValid() const noexcept {
			return (caerIMU6EventIsValid(this));
		}

		void validate(IMU6EventPacket &packet) noexcept {
			caerIMU6EventValidate(this, reinterpret_cast<caerIMU6EventPacket>(packet.header));
		}

		void invalidate(IMU6EventPacket &packet) noexcept {
			caerIMU6EventInvalidate(this, reinterpret_cast<caerIMU6EventPacket>(packet.header));
		}
	};

	// Constructors.
	IMU6EventPacket(int32_t eventCapacity, int16_t eventSource, int32_t tsOverflow) {
		if (eventCapacity <= 0) {
			throw std::invalid_argument("Negative or zero event capacity not allowed on construction.");
		}

		caerIMU6EventPacket packet = caerIMU6EventPacketAllocate(eventCapacity, eventSource, tsOverflow);
		if (packet == nullptr) {
			throw std::runtime_error("Failed to allocate IMU6 event packet.");
		}

		header = &packet->packetHeader;
	}

	// EventPacketHeader's destructor takes care of freeing above memory.
	// Same for all copy/move constructor/assignment, use EventPacketHeader.

	IMU6Event &getEvent(int32_t index) {
		if (index < 0 || index >= capacity()) {
			throw std::out_of_range("Index out of range.");
		}

		IMU6EventBase *evtBase = caerIMU6EventPacketGetEvent(reinterpret_cast<caerIMU6EventPacket>(header), index);
		IMU6Event *evt = static_cast<IMU6Event *>(evtBase);

		return (*evt);
	}

	const IMU6Event &getEvent(int32_t index) const {
		if (index < 0 || index >= capacity()) {
			throw std::out_of_range("Index out of range.");
		}

		const IMU6EventBase *evtBase = caerIMU6EventPacketGetEventConst(
			reinterpret_cast<caerIMU6EventPacketConst>(header), index);
		const IMU6Event *evt = static_cast<const IMU6Event *>(evtBase);

		return (*evt);
	}

	IMU6Event &operator[](size_t index) {
		return (getEvent(static_cast<int32_t>(index)));
	}

	const IMU6Event &operator[](size_t index) const {
		return (getEvent(static_cast<int32_t>(index)));
	}
};

}
}

#endif /* LIBCAER_EVENTS_IMU6_HPP_ */
