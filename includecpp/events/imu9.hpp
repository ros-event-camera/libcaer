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

		float getAccelX() const noexcept {
			return (caerIMU9EventGetAccelX(this));
		}

		void setAccelX(float accelX) noexcept {
			caerIMU9EventSetAccelX(this, accelX);
		}

		float getAccelY() const noexcept {
			return (caerIMU9EventGetAccelY(this));
		}

		void setAccelY(float accelY) noexcept {
			caerIMU9EventSetAccelY(this, accelY);
		}

		float getAccelZ() const noexcept {
			return (caerIMU9EventGetAccelZ(this));
		}

		void setAccelZ(float accelZ) noexcept {
			caerIMU9EventSetAccelZ(this, accelZ);
		}

		float getGyroX() const noexcept {
			return (caerIMU9EventGetGyroX(this));
		}

		void setGyroX(float gyroX) noexcept {
			caerIMU9EventSetGyroX(this, gyroX);
		}

		float getGyroY() const noexcept {
			return (caerIMU9EventGetGyroY(this));
		}

		void setGyroY(float gyroY) noexcept {
			caerIMU9EventSetGyroY(this, gyroY);
		}

		float getGyroZ() const noexcept {
			return (caerIMU9EventGetGyroZ(this));
		}

		void setGyroZ(float gyroZ) noexcept {
			caerIMU9EventSetGyroZ(this, gyroZ);
		}

		float getTemp() const noexcept {
			return (caerIMU9EventGetTemp(this));
		}

		void setTemp(float t) noexcept {
			caerIMU9EventSetTemp(this, t);
		}

		float getCompX() const noexcept {
			return (caerIMU9EventGetCompX(this));
		}

		void setCompX(float compX) noexcept {
			caerIMU9EventSetCompX(this, compX);
		}

		float getCompY() const noexcept {
			return (caerIMU9EventGetCompY(this));
		}

		void setCompY(float compY) noexcept {
			caerIMU9EventSetCompY(this, compY);
		}

		float getCompZ() const noexcept {
			return (caerIMU9EventGetCompZ(this));
		}

		void setCompZ(float compZ) noexcept {
			caerIMU9EventSetCompZ(this, compZ);
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

	IMU9EventPacket(caerIMU9EventPacket packet) {
		if (packet == nullptr) {
			throw std::runtime_error("Failed to initialize event packet from existing C packet: null pointer.");
		}

		// Check for proper event type too!
		if (caerEventPacketHeaderGetEventType(&packet->packetHeader) != IMU9_EVENT) {
			throw std::runtime_error("Failed to initialize event packet from existing C packet: wrong type.");
		}

		header = &packet->packetHeader;
	}

	IMU9EventPacket(caerEventPacketHeader packetHeader) {
		if (packetHeader == nullptr) {
			throw std::runtime_error("Failed to initialize event packet from existing C packet header: null pointer.");
		}

		// Check for proper event type too!
		if (caerEventPacketHeaderGetEventType(packetHeader) != IMU9_EVENT) {
			throw std::runtime_error("Failed to initialize event packet from existing C packet header: wrong type.");
		}

		header = packetHeader;
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

	IMU9EventPacket copy() const {
		return (IMU9EventPacket(internalCopy(header)));
	}

	IMU9EventPacket copyOnlyEvents() const {
		return (IMU9EventPacket(internalCopyOnlyEvents(header)));
	}

	IMU9EventPacket copyOnlyValidEvents() const {
		return (IMU9EventPacket(internalCopyOnlyValidEvents(header)));
	}
};

}
}

#endif /* LIBCAER_EVENTS_IMU9_HPP_ */
