#ifndef LIBCAER_EVENTS_POINT1D_HPP_
#define LIBCAER_EVENTS_POINT1D_HPP_

#include <libcaer/events/point1d.h>
#include "common.hpp"

namespace libcaer {
namespace events {

class Point1DEventPacket: public EventPacketHeader {
public:
	using Point1DEventBase = struct caer_point1d_event;

	struct Point1DEvent: public Point1DEventBase {
		int32_t getTimestamp() const noexcept {
			return (caerPoint1DEventGetTimestamp(this));
		}

		int64_t getTimestamp64(const Point1DEventPacket &packet) const noexcept {
			return (caerPoint1DEventGetTimestamp64(this, reinterpret_cast<caerPoint1DEventPacketConst>(packet.header)));
		}

		void setTimestamp(int32_t ts) {
			if (ts < 0) {
				throw std::invalid_argument("Negative timestamp not allowed.");
			}

			caerPoint1DEventSetTimestamp(this, ts);
		}

		bool isValid() const noexcept {
			return (caerPoint1DEventIsValid(this));
		}

		void validate(Point1DEventPacket &packet) noexcept {
			caerPoint1DEventValidate(this, reinterpret_cast<caerPoint1DEventPacket>(packet.header));
		}

		void invalidate(Point1DEventPacket &packet) noexcept {
			caerPoint1DEventInvalidate(this, reinterpret_cast<caerPoint1DEventPacket>(packet.header));
		}

		uint8_t getType() const noexcept {
			return (caerPoint1DEventGetType(this));
		}

		void setType(uint8_t t) noexcept {
			return (caerPoint1DEventSetType(this, t));
		}

		int8_t getScale() const noexcept {
			return (caerPoint1DEventGetScale(this));
		}

		void setScale(int8_t s) noexcept {
			return (caerPoint1DEventSetScale(this, s));
		}

		float getX() const noexcept {
			return (caerPoint1DEventGetX(this));
		}

		void setX(float xVal) noexcept {
			return (caerPoint1DEventSetX(this, xVal));
		}
	};

	// Constructors.
	Point1DEventPacket(int32_t eventCapacity, int16_t eventSource, int32_t tsOverflow) {
		if (eventCapacity <= 0) {
			throw std::invalid_argument("Negative or zero event capacity not allowed on construction.");
		}

		caerPoint1DEventPacket packet = caerPoint1DEventPacketAllocate(eventCapacity, eventSource, tsOverflow);
		if (packet == nullptr) {
			throw std::runtime_error("Failed to allocate Point1D event packet.");
		}

		header = &packet->packetHeader;
	}

	Point1DEventPacket(caerPoint1DEventPacket packet) {
		if (packet == nullptr) {
			throw std::runtime_error("Failed to initialize event packet from existing C packet: null pointer.");
		}

		// Check for proper event type too!
		if (caerEventPacketHeaderGetEventType(&packet->packetHeader) != POINT1D_EVENT) {
			throw std::runtime_error("Failed to initialize event packet from existing C packet: wrong type.");
		}

		header = &packet->packetHeader;
	}

	Point1DEventPacket(caerEventPacketHeader packetHeader) {
		if (packetHeader == nullptr) {
			throw std::runtime_error("Failed to initialize event packet from existing C packet header: null pointer.");
		}

		// Check for proper event type too!
		if (caerEventPacketHeaderGetEventType(packetHeader) != POINT1D_EVENT) {
			throw std::runtime_error("Failed to initialize event packet from existing C packet header: wrong type.");
		}

		header = packetHeader;
	}

	// EventPacketHeader's destructor takes care of freeing above memory.
	// Same for all copy/move constructor/assignment, use EventPacketHeader.

	Point1DEvent &getEvent(int32_t index) {
		if (index < 0 || index >= capacity()) {
			throw std::out_of_range("Index out of range.");
		}

		Point1DEventBase *evtBase = caerPoint1DEventPacketGetEvent(reinterpret_cast<caerPoint1DEventPacket>(header),
			index);
		Point1DEvent *evt = static_cast<Point1DEvent *>(evtBase);

		return (*evt);
	}

	const Point1DEvent &getEvent(int32_t index) const {
		if (index < 0 || index >= capacity()) {
			throw std::out_of_range("Index out of range.");
		}

		const Point1DEventBase *evtBase = caerPoint1DEventPacketGetEventConst(
			reinterpret_cast<caerPoint1DEventPacketConst>(header), index);
		const Point1DEvent *evt = static_cast<const Point1DEvent *>(evtBase);

		return (*evt);
	}

	Point1DEvent &operator[](size_t index) {
		return (getEvent(static_cast<int32_t>(index)));
	}

	const Point1DEvent &operator[](size_t index) const {
		return (getEvent(static_cast<int32_t>(index)));
	}

	virtual Point1DEventPacket *copy() const override {
		return (new Point1DEventPacket(internalCopy(header)));
	}

	virtual Point1DEventPacket *copyOnlyEvents() const override {
		return (new Point1DEventPacket(internalCopyOnlyEvents(header)));
	}

	virtual Point1DEventPacket *copyOnlyValidEvents() const override {
		return (new Point1DEventPacket(internalCopyOnlyValidEvents(header)));
	}
};

}
}

#endif /* LIBCAER_EVENTS_POINT1D_HPP_ */
