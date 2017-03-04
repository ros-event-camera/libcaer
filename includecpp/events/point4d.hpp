#ifndef LIBCAER_EVENTS_POINT4D_HPP_
#define LIBCAER_EVENTS_POINT4D_HPP_

#include <libcaer/events/point4d.h>
#include "common.hpp"

namespace libcaer {
namespace events {

class Point4DEventPacket: public EventPacketHeader {
public:
	using Point4DEventBase = struct caer_point4d_event;

	struct Point4DEvent: public Point4DEventBase {
		int32_t getTimestamp() const noexcept {
			return (caerPoint4DEventGetTimestamp(this));
		}

		int64_t getTimestamp64(const Point4DEventPacket &packet) const noexcept {
			return (caerPoint4DEventGetTimestamp64(this, reinterpret_cast<caerPoint4DEventPacketConst>(packet.header)));
		}

		void setTimestamp(int32_t ts) {
			if (ts < 0) {
				throw std::invalid_argument("Negative timestamp not allowed.");
			}

			caerPoint4DEventSetTimestamp(this, ts);
		}

		bool isValid() const noexcept {
			return (caerPoint4DEventIsValid(this));
		}

		void validate(Point4DEventPacket &packet) noexcept {
			caerPoint4DEventValidate(this, reinterpret_cast<caerPoint4DEventPacket>(packet.header));
		}

		void invalidate(Point4DEventPacket &packet) noexcept {
			caerPoint4DEventInvalidate(this, reinterpret_cast<caerPoint4DEventPacket>(packet.header));
		}
	};

	// Constructors.
	Point4DEventPacket(int32_t eventCapacity, int16_t eventSource, int32_t tsOverflow) {
		if (eventCapacity <= 0) {
			throw std::invalid_argument("Negative or zero event capacity not allowed on construction.");
		}

		caerPoint4DEventPacket packet = caerPoint4DEventPacketAllocate(eventCapacity, eventSource, tsOverflow);
		if (packet == nullptr) {
			throw std::runtime_error("Failed to allocate Point4D event packet.");
		}

		header = &packet->packetHeader;
	}

	// EventPacketHeader's destructor takes care of freeing above memory.
	// Same for all copy/move constructor/assignment, use EventPacketHeader.

	Point4DEvent &getEvent(int32_t index) {
		if (index < 0 || index >= capacity()) {
			throw std::out_of_range("Index out of range.");
		}

		Point4DEventBase *evtBase = caerPoint4DEventPacketGetEvent(reinterpret_cast<caerPoint4DEventPacket>(header), index);
		Point4DEvent *evt = static_cast<Point4DEvent *>(evtBase);

		return (*evt);
	}

	const Point4DEvent &getEvent(int32_t index) const {
		if (index < 0 || index >= capacity()) {
			throw std::out_of_range("Index out of range.");
		}

		const Point4DEventBase *evtBase = caerPoint4DEventPacketGetEventConst(
			reinterpret_cast<caerPoint4DEventPacketConst>(header), index);
		const Point4DEvent *evt = static_cast<const Point4DEvent *>(evtBase);

		return (*evt);
	}

	Point4DEvent &operator[](size_t index) {
		return (getEvent(static_cast<int32_t>(index)));
	}

	const Point4DEvent &operator[](size_t index) const {
		return (getEvent(static_cast<int32_t>(index)));
	}
};

}
}

#endif /* LIBCAER_EVENTS_POINT4D_HPP_ */
