#ifndef LIBCAER_EVENTS_PACKETCONTAINER_HPP_
#define LIBCAER_EVENTS_PACKETCONTAINER_HPP_

#include "common.hpp"
#include <vector>
#include <memory>

namespace libcaer {
namespace events {

class EventPacketContainer {
private:
	/// Smallest event timestamp contained in this packet container.
	int64_t lowestEventTimestamp;
	/// Largest event timestamp contained in this packet container.
	int64_t highestEventTimestamp;
	/// Number of events contained within all the packets in this container.
	int32_t eventsNumber;
	/// Number of valid events contained within all the packets in this container.
	int32_t eventsValidNumber;
	/// Vector of pointers to the actual event packets.
	std::vector<std::shared_ptr<EventPacketHeader>> eventPackets;

public:
	/**
	 * Construct a new EventPacketContainer.
	 */
	EventPacketContainer() :
			lowestEventTimestamp(-1),
			highestEventTimestamp(-1),
			eventsNumber(0),
			eventsValidNumber(0) {
	}

	/**
	 * Construct a new EventPacketContainer with enough space to
	 * store up to the given number of event packet pointers.
	 * The pointers are present and initialized to nullptr.
	 *
	 * @param eventPacketsNumber the initial number of event packet pointers
	 *                           that can be stored in this container.
	 */
	EventPacketContainer(size_t eventPacketsNumber) :
			lowestEventTimestamp(-1),
			highestEventTimestamp(-1),
			eventsNumber(0),
			eventsValidNumber(0),
			eventPackets(eventPacketsNumber) {
		for (size_t i = 0; i < eventPacketsNumber; i++) {
			eventPackets.emplace_back();
		}
	}

	// The default destructor is fine here, as it will call the vector's
	// destructor, which will call all of its content's destructors; those
	// are shared_ptr, so if their count reaches zero it will then call the
	// EventPacketHeader destructor, and everything is fine.
	// Same for copy/move assignment/constructors, the defaults are fine,
	// as vector and shared_ptr take care of all the book-keeping.

	// EventPackets vector accessors.
	size_t capacity() const noexcept {
		return (eventPackets.capacity());
	}

	size_t size() const noexcept {
		return (eventPackets.size());
	}

	bool empty() const noexcept {
		return (eventPackets.empty());
	}

	void clear() noexcept {
		eventPackets.clear();
	}

	/**
	 * Get the pointer to the event packet stored in this container
	 * at the given index.
	 *
	 * @param index the index of the event packet to get.
	 *
	 * @return a pointer to an event packet.
	 *
	 * @exception std:out_of_range no packet exists at given index.
	 */
	std::shared_ptr<EventPacketHeader> getEventPacket(size_t index) {
		if (index >= eventPackets.size()) {
			throw std::out_of_range("Index out of range.");
		}

		return (eventPackets[index]);
	}

	std::shared_ptr<EventPacketHeader> operator[](size_t index) {
		return (getEventPacket(index));
	}

	/**
	 * Get the pointer to the event packet stored in this container
	 * at the given index.
	 * This is a read-only event packet, do not change its contents in any way!
	 *
	 * @param index the index of the event packet to get.
	 *
	 * @return a pointer to a read-only event packet.
	 *
	 * @exception std:out_of_range no packet exists at given index.
	 */
	std::shared_ptr<const EventPacketHeader> getEventPacket(size_t index) const {
		if (index >= eventPackets.size()) {
			throw std::out_of_range("Index out of range.");
		}

		return (eventPackets[index]);
	}

	std::shared_ptr<const EventPacketHeader> operator[](size_t index) const {
		return (getEventPacket(index));
	}

	/**
	 * Set the pointer to the event packet stored in this container
	 * at the given index.
	 *
	 * @param index the index of the event packet to set.
	 * @param packetHeader a pointer to an event packet. Can be a nullptr.
	 *
	 * @exception std:out_of_range no packet exists at given index.
	 */
	void setEventPacket(size_t index, std::shared_ptr<EventPacketHeader> packetHeader) {
		if (index >= eventPackets.size()) {
			throw std::out_of_range("Index out of range.");
		}

		eventPackets[index] = packetHeader;

		updateStatistics();
	}

	/**
	 * Add an event packet pointer at the end of this container.
	 *
	 * @param packetHeader a pointer to an event packet. Can be a nullptr.
	 */
	void add(std::shared_ptr<EventPacketHeader> packetHeader) {
		eventPackets.push_back(packetHeader);

		updateStatistics();
	}

	/**
	 * Get the lowest timestamp contained in this event packet container.
	 *
	 * @return the lowest timestamp (in µs) or -1 if not initialized.
	 */
	int64_t getLowestEventTimestamp() const noexcept {
		return (lowestEventTimestamp);
	}

	/**
	 * Get the highest timestamp contained in this event packet container.
	 *
	 * @return the highest timestamp (in µs) or -1 if not initialized.
	 */
	int64_t getHighestEventTimestamp() const noexcept {
		return (highestEventTimestamp);
	}

	/**
	 * Get the number of events contained in this event packet container.
	 *
	 * @return the number of events in this container.
	 */
	int32_t getEventsNumber() const noexcept {
		return (eventsNumber);
	}

	/**
	 * Get the number of valid events contained in this event packet container.
	 *
	 * @return the number of valid events in this container.
	 */
	int32_t getEventsValidNumber() const noexcept {
		return (eventsValidNumber);
	}

	/**
	 * Recalculates and updates all the packet-container level statistics (event
	 * counts and timestamps).
	 */
	void updateStatistics() noexcept {
		int64_t lowestTimestamp = -1;
		int64_t highestTimestamp = -1;
		int32_t events = 0;
		int32_t eventsValid = 0;

		for (const auto &packet : eventPackets) {
			if (packet == nullptr) {
				continue;
			}

			// If packet has no events, skip it, it contributes nothing to statistics.
			if (packet->getEventNumber() == 0) {
				continue;
			}

			// Get timestamps to update lowest/highest tracking.
			const auto firstEvent = packet->genericGetEvent(0);
			int64_t currLowestEventTimestamp = firstEvent.getTimestamp64();

			const auto lastEvent = packet->genericGetEvent(packet->getEventNumber() - 1);
			int64_t currHighestEventTimestamp = lastEvent.getTimestamp64();

			// Update tracked timestamps (or initialize if needed).
			if ((lowestTimestamp == -1) || (lowestTimestamp > currLowestEventTimestamp)) {
				lowestTimestamp = currLowestEventTimestamp;
			}

			if ((highestTimestamp == -1) || (highestTimestamp < currHighestEventTimestamp)) {
				highestTimestamp = currHighestEventTimestamp;
			}

			events += packet->getEventNumber();
			eventsValid += packet->getEventValid();
		}

		lowestEventTimestamp = lowestTimestamp;
		highestEventTimestamp = highestTimestamp;
		eventsNumber = events;
		eventsValidNumber = eventsValid;
	}

	/**
	 * Get the pointer to an event packet stored in this container
	 * with the given event type. This returns the first found event packet
	 * with that type ID, or nullptr if we get to the end without finding any
	 * such event packet.
	 *
	 * @param typeID the event type to search for.
	 *
	 * @return a pointer to an event packet with a certain type or nullptr if none found.
	 */
	std::shared_ptr<EventPacketHeader> findEventPacketByType(int16_t typeID) {
		for (const auto &packet : eventPackets) {
			if (packet == nullptr) {
				continue;
			}

			if (packet->getEventType() == typeID) {
				return (packet);
			}
		}

		return (nullptr);
	}

	/**
	 * Get the pointer to a read-only event packet stored in this container
	 * with the given event type. This returns the first found event packet
	 * with that type ID, or nullptr if we get to the end without finding any
	 * such event packet.
	 *
	 * @param typeID the event type to search for.
	 *
	 * @return a pointer to a read-only event packet with a certain type or nullptr if none found.
	 */
	std::shared_ptr<const EventPacketHeader> findEventPacketByType(int16_t typeID) const {
		for (const auto &packet : eventPackets) {
			if (packet == nullptr) {
				continue;
			}

			if (packet->getEventType() == typeID) {
				return (packet);
			}
		}

		return (nullptr);
	}

	/**
	 * Make a deep copy of this event packet container and all of its
	 * event packets and their current events.
	 * A normal copy (using copy constructor or assignment) copies all the
	 * container's internals, and correctly handles the pointers to the
	 * event packets held in it, but those will still point to the old
	 * event packets. To also copy the individual event packets and point
	 * to the new copies, use this function.
	 *
	 * @return a deep copy of this event packet container, containing all events.
	 */
	EventPacketContainer copyAllEvents() const {
		EventPacketContainer newContainer = EventPacketContainer();

		for (const auto &packet : eventPackets) {
			if (packet == nullptr) {
				newContainer.add(nullptr);
			}
			else {
				newContainer.add(std::make_shared<EventPacketHeader>(packet->copyOnlyEvents()));
			}
		}

		return (newContainer);
	}

	/**
	 * Make a deep copy of this event packet container, with its event packets
	 * sized down to only include the currently valid events (eventValid),
	 * and discarding everything else.
	 * A normal copy (using copy constructor or assignment) copies all the
	 * container's internals, and correctly handles the pointers to the
	 * event packets held in it, but those will still point to the old
	 * event packets. To also copy the individual event packets and point
	 * to the new copies, use this function.
	 *
	 * @return a deep copy of this event packet container, containing only valid events.
	 */
	EventPacketContainer copyValidEvents() const {
		EventPacketContainer newContainer = EventPacketContainer();

		for (const auto &packet : eventPackets) {
			if (packet == nullptr) {
				newContainer.add(nullptr);
			}
			else {
				newContainer.add(std::make_shared<EventPacketHeader>(packet->copyOnlyValidEvents()));
			}
		}

		return (newContainer);
	}
}
;

}
}

#endif /* LIBCAER_EVENTS_PACKETCONTAINER_HPP_ */
