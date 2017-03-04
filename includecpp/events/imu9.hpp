#ifndef LIBCAER_EVENTS_IMU9_HPP_
#define LIBCAER_EVENTS_IMU9_HPP_

#include <libcaer/events/imu9.h>
#include "common.hpp"

namespace libcaer {
namespace events {

class IMU9EventPacket: public EventPacketHeader {
public:
	using IMU9EventBase = struct caer_imu9_event;

	struct IMU9Event: public IMU9EventBase {
		int32_t getTimestamp() const noexcept {
			return (caerIMU9EventGetTimestamp(this));
		}

		int64_t getTimestamp64(const IMU9EventPacket &packet) const noexcept {
			return (caerIMU9EventGetTimestamp64(this, reinterpret_cast<caerIMU9EventPacketConst>(packet.header)));
		}

		void setTimestamp(int32_t ts) {
			if (ts < 0) {
				throw std::invalid_argument("Negative timestamp not allowed.");
			}

			caerIMU9EventSetTimestamp(this, ts);
		}

		bool isValid() const noexcept {
			return (caerIMU9EventIsValid(this));
		}

		void validate(IMU9EventPacket &packet) noexcept {
			caerIMU9EventValidate(this, reinterpret_cast<caerIMU9EventPacket>(packet.header));
		}

		void invalidate(IMU9EventPacket &packet) noexcept {
			caerIMU9EventInvalidate(this, reinterpret_cast<caerIMU9EventPacket>(packet.header));
		}
	};

	// Constructors.
	IMU9EventPacket(int32_t eventCapacity, int16_t eventSource, int32_t tsOverflow) {
		if (eventCapacity <= 0) {
			throw std::invalid_argument("Negative or zero event capacity not allowed on construction.");
		}

		caerIMU9EventPacket packet = caerIMU9EventPacketAllocate(eventCapacity, eventSource, tsOverflow);
		if (packet == nullptr) {
			throw std::runtime_error("Failed to allocate IMU9 event packet.");
		}

		header = &packet->packetHeader;
	}

	// EventPacketHeader's destructor takes care of freeing above memory.
	// Same for all copy/move constructor/assignment, use EventPacketHeader.

	IMU9Event &getEvent(int32_t index) {
		if (index < 0 || index >= capacity()) {
			throw std::out_of_range("Index out of range.");
		}

		IMU9EventBase *evtBase = caerIMU9EventPacketGetEvent(reinterpret_cast<caerIMU9EventPacket>(header), index);
		IMU9Event *evt = static_cast<IMU9Event *>(evtBase);

		return (*evt);
	}

	const IMU9Event &getEvent(int32_t index) const {
		if (index < 0 || index >= capacity()) {
			throw std::out_of_range("Index out of range.");
		}

		const IMU9EventBase *evtBase = caerIMU9EventPacketGetEventConst(
			reinterpret_cast<caerIMU9EventPacketConst>(header), index);
		const IMU9Event *evt = static_cast<const IMU9Event *>(evtBase);

		return (*evt);
	}

	IMU9Event &operator[](size_t index) {
		return (getEvent(static_cast<int32_t>(index)));
	}

	const IMU9Event &operator[](size_t index) const {
		return (getEvent(static_cast<int32_t>(index)));
	}
};

}
}

#endif /* LIBCAER_EVENTS_IMU9_HPP_ */
