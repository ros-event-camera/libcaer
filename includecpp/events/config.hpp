#ifndef LIBCAER_EVENTS_CONFIG_HPP_
#define LIBCAER_EVENTS_CONFIG_HPP_

#include <libcaer/events/config.h>
#include "common.hpp"

namespace libcaer {
namespace events {

class ConfigurationEventPacket: public EventPacketHeader {
public:
	using ConfigurationEventBase = struct caer_configuration_event;

	struct ConfigurationEvent: public ConfigurationEventBase {
		int32_t getTimestamp() const noexcept {
			return (caerConfigurationEventGetTimestamp(this));
		}

		int64_t getTimestamp64(const ConfigurationEventPacket &packet) const noexcept {
			return (caerConfigurationEventGetTimestamp64(this,
				reinterpret_cast<caerConfigurationEventPacketConst>(packet.header)));
		}

		void setTimestamp(int32_t ts) {
			if (ts < 0) {
				throw std::invalid_argument("Negative timestamp not allowed.");
			}

			caerConfigurationEventSetTimestamp(this, ts);
		}

		bool isValid() const noexcept {
			return (caerConfigurationEventIsValid(this));
		}

		void validate(ConfigurationEventPacket &packet) noexcept {
			caerConfigurationEventValidate(this, reinterpret_cast<caerConfigurationEventPacket>(packet.header));
		}

		void invalidate(ConfigurationEventPacket &packet) noexcept {
			caerConfigurationEventInvalidate(this, reinterpret_cast<caerConfigurationEventPacket>(packet.header));
		}

		uint8_t getModuleAddress() const noexcept {
			return (caerConfigurationEventGetModuleAddress(this));
		}

		void setModuleAddress(uint8_t modAddr) noexcept {
			caerConfigurationEventSetModuleAddress(this, modAddr);
		}

		uint8_t getParameterAddress() const noexcept {
			return (caerConfigurationEventGetParameterAddress(this));
		}

		void setParameterAddress(uint8_t paramAddr) noexcept {
			caerConfigurationEventSetParameterAddress(this, paramAddr);
		}

		uint32_t getParameter() const noexcept {
			return (caerConfigurationEventGetParameter(this));
		}

		void setParameter(uint32_t param) noexcept {
			caerConfigurationEventSetParameter(this, param);
		}
	};

	// Container traits.
	using value_type = ConfigurationEvent;
	using pointer = ConfigurationEvent *;
	using const_pointer = const ConfigurationEvent *;
	using reference = ConfigurationEvent &;
	using const_reference = const ConfigurationEvent &;
	using size_type = int32_t;
	using difference_type = ptrdiff_t;

	// Constructors.
	ConfigurationEventPacket(size_type eventCapacity, int16_t eventSource, int32_t tsOverflow) {
		if (eventCapacity <= 0) {
			throw std::invalid_argument("Negative or zero event capacity not allowed on construction.");
		}

		caerConfigurationEventPacket packet = caerConfigurationEventPacketAllocate(eventCapacity, eventSource,
			tsOverflow);
		if (packet == nullptr) {
			throw std::runtime_error("Failed to allocate configuration event packet.");
		}

		header = &packet->packetHeader;
	}

	ConfigurationEventPacket(caerConfigurationEventPacket packet) {
		if (packet == nullptr) {
			throw std::runtime_error("Failed to initialize event packet from existing C packet: null pointer.");
		}

		// Check for proper event type too!
		if (caerEventPacketHeaderGetEventType(&packet->packetHeader) != CONFIG_EVENT) {
			throw std::runtime_error("Failed to initialize event packet from existing C packet: wrong type.");
		}

		header = &packet->packetHeader;
	}

	ConfigurationEventPacket(caerEventPacketHeader packetHeader) {
		if (packetHeader == nullptr) {
			throw std::runtime_error("Failed to initialize event packet from existing C packet header: null pointer.");
		}

		// Check for proper event type too!
		if (caerEventPacketHeaderGetEventType(packetHeader) != CONFIG_EVENT) {
			throw std::runtime_error("Failed to initialize event packet from existing C packet header: wrong type.");
		}

		header = packetHeader;
	}

	// EventPacketHeader's destructor takes care of freeing above memory.
	// Same for all copy/move constructor/assignment, use EventPacketHeader's.

	// Event access methods.
	reference getEvent(size_type index) {
		if (index < 0 || index >= capacity()) {
			throw std::out_of_range("Index out of range.");
		}

		ConfigurationEventBase *evtBase = caerConfigurationEventPacketGetEvent(
			reinterpret_cast<caerConfigurationEventPacket>(header), index);
		ConfigurationEvent *evt = static_cast<ConfigurationEvent *>(evtBase);

		return (*evt);
	}

	const_reference getEvent(size_type index) const {
		if (index < 0 || index >= capacity()) {
			throw std::out_of_range("Index out of range.");
		}

		const ConfigurationEventBase *evtBase = caerConfigurationEventPacketGetEventConst(
			reinterpret_cast<caerConfigurationEventPacketConst>(header), index);
		const ConfigurationEvent *evt = static_cast<const ConfigurationEvent *>(evtBase);

		return (*evt);
	}

	reference operator[](size_type index) {
		return (getEvent(index));
	}

	const_reference operator[](size_type index) const {
		return (getEvent(index));
	}

	reference front() {
		return (getEvent(0));
	}

	const_reference front() const {
		return (getEvent(0));
	}

	reference back() {
		return (getEvent(size() - 1));
	}

	const_reference back() const {
		return (getEvent(size() - 1));
	}

	virtual ConfigurationEventPacket *copy() const override {
		return (new ConfigurationEventPacket(internalCopy(header)));
	}

	virtual ConfigurationEventPacket *copyOnlyEvents() const override {
		return (new ConfigurationEventPacket(internalCopyOnlyEvents(header)));
	}

	virtual ConfigurationEventPacket *copyOnlyValidEvents() const override {
		return (new ConfigurationEventPacket(internalCopyOnlyValidEvents(header)));
	}

	// Iterator support.
	using iterator = EventPacketIterator<value_type>;
	using const_iterator = EventPacketIterator<const value_type>;
	using reverse_iterator = std::reverse_iterator<iterator>;
	using const_reverse_iterator = std::reverse_iterator<const_iterator>;

	iterator begin() noexcept {
		return (iterator(reinterpret_cast<uint8_t *>(&front()), static_cast<size_t>(getEventSize())));
	}

	iterator end() noexcept {
		return (iterator(reinterpret_cast<uint8_t *>(&front()) + (size() * getEventSize()),
			static_cast<size_t>(getEventSize())));
	}

	const_iterator begin() const noexcept {
		return (cbegin());
	}

	const_iterator end() const noexcept {
		return (cend());
	}

	const_iterator cbegin() const noexcept {
		return (const_iterator(reinterpret_cast<const uint8_t *>(&front()), static_cast<size_t>(getEventSize())));
	}

	const_iterator cend() const noexcept {
		return (const_iterator(reinterpret_cast<const uint8_t *>(&front()) + (size() * getEventSize()),
			static_cast<size_t>(getEventSize())));
	}

	reverse_iterator rbegin() noexcept {
		return (reverse_iterator(end()));
	}

	reverse_iterator rend() noexcept {
		return (reverse_iterator(begin()));
	}

	const_reverse_iterator rbegin() const noexcept {
		return (crbegin());
	}

	const_reverse_iterator rend() const noexcept {
		return (crend());
	}

	const_reverse_iterator crbegin() const noexcept {
		return (const_reverse_iterator(cend()));
	}

	const_reverse_iterator crend() const noexcept {
		return (const_reverse_iterator(cbegin()));
	}
};

}
}

#endif /* LIBCAER_EVENTS_CONFIG_HPP_ */
