#ifndef LIBCAER_EVENTS_POINT3D_HPP_
#define LIBCAER_EVENTS_POINT3D_HPP_

#include <libcaer/events/point3d.h>
#include "common.hpp"

namespace libcaer {
namespace events {

class Point3DEventPacket: public EventPacketHeader {
public:
	using Point3DEventBase = struct caer_point3d_event;

	struct Point3DEvent: public Point3DEventBase {
		int32_t getTimestamp() const noexcept {
			return (caerPoint3DEventGetTimestamp(this));
		}

		int64_t getTimestamp64(const Point3DEventPacket &packet) const noexcept {
			return (caerPoint3DEventGetTimestamp64(this, reinterpret_cast<caerPoint3DEventPacketConst>(packet.header)));
		}

		void setTimestamp(int32_t ts) {
			if (ts < 0) {
				throw std::invalid_argument("Negative timestamp not allowed.");
			}

			caerPoint3DEventSetTimestamp(this, ts);
		}

		bool isValid() const noexcept {
			return (caerPoint3DEventIsValid(this));
		}

		void validate(Point3DEventPacket &packet) noexcept {
			caerPoint3DEventValidate(this, reinterpret_cast<caerPoint3DEventPacket>(packet.header));
		}

		void invalidate(Point3DEventPacket &packet) noexcept {
			caerPoint3DEventInvalidate(this, reinterpret_cast<caerPoint3DEventPacket>(packet.header));
		}

		uint8_t getType() const noexcept {
			return (caerPoint3DEventGetType(this));
		}

		void setType(uint8_t t) noexcept {
			return (caerPoint3DEventSetType(this, t));
		}

		int8_t getScale() const noexcept {
			return (caerPoint3DEventGetScale(this));
		}

		void setScale(int8_t s) noexcept {
			return (caerPoint3DEventSetScale(this, s));
		}

		float getX() const noexcept {
			return (caerPoint3DEventGetX(this));
		}

		void setX(float xVal) noexcept {
			return (caerPoint3DEventSetX(this, xVal));
		}

		float getY() const noexcept {
			return (caerPoint3DEventGetY(this));
		}

		void setY(float yVal) noexcept {
			return (caerPoint3DEventSetY(this, yVal));
		}

		float getZ() const noexcept {
			return (caerPoint3DEventGetZ(this));
		}

		void setZ(float zVal) noexcept {
			return (caerPoint3DEventSetZ(this, zVal));
		}
	};

	// Constructors.
	Point3DEventPacket(int32_t eventCapacity, int16_t eventSource, int32_t tsOverflow) {
		if (eventCapacity <= 0) {
			throw std::invalid_argument("Negative or zero event capacity not allowed on construction.");
		}

		caerPoint3DEventPacket packet = caerPoint3DEventPacketAllocate(eventCapacity, eventSource, tsOverflow);
		if (packet == nullptr) {
			throw std::runtime_error("Failed to allocate Point3D event packet.");
		}

		header = &packet->packetHeader;
	}

	Point3DEventPacket(caerPoint3DEventPacket packet) {
		if (packet == nullptr) {
			throw std::runtime_error("Failed to initialize Point3D event packet from existing C struct.");
		}

		header = &packet->packetHeader;
	}

	// EventPacketHeader's destructor takes care of freeing above memory.
	// Same for all copy/move constructor/assignment, use EventPacketHeader.

	Point3DEvent &getEvent(int32_t index) {
		if (index < 0 || index >= capacity()) {
			throw std::out_of_range("Index out of range.");
		}

		Point3DEventBase *evtBase = caerPoint3DEventPacketGetEvent(reinterpret_cast<caerPoint3DEventPacket>(header),
			index);
		Point3DEvent *evt = static_cast<Point3DEvent *>(evtBase);

		return (*evt);
	}

	const Point3DEvent &getEvent(int32_t index) const {
		if (index < 0 || index >= capacity()) {
			throw std::out_of_range("Index out of range.");
		}

		const Point3DEventBase *evtBase = caerPoint3DEventPacketGetEventConst(
			reinterpret_cast<caerPoint3DEventPacketConst>(header), index);
		const Point3DEvent *evt = static_cast<const Point3DEvent *>(evtBase);

		return (*evt);
	}

	Point3DEvent &operator[](size_t index) {
		return (getEvent(static_cast<int32_t>(index)));
	}

	const Point3DEvent &operator[](size_t index) const {
		return (getEvent(static_cast<int32_t>(index)));
	}
};

}
}

#endif /* LIBCAER_EVENTS_POINT3D_HPP_ */
