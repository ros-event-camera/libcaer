#ifndef LIBCAER_EVENTS_FRAME_HPP_
#define LIBCAER_EVENTS_FRAME_HPP_

#include <libcaer/events/frame.h>
#include <libcaer/frame_utils.h>
#include "common.hpp"

namespace libcaer {
namespace events {

class FrameEventPacket: public EventPacketHeader {
public:
	enum class colorChannels {
		GRAYSCALE = 1, //!< Grayscale, one channel only.
		RGB = 3,       //!< Red Green Blue, 3 color channels.
		RGBA = 4,      //!< Red Green Blue Alpha, 3 color channels plus transparency.
	};

	enum class colorFilter {
		MONO = 0,    //!< No color filter present, all light passes.
		RGBG = 1,    //!< Standard Bayer color filter, 1 red 2 green 1 blue. Variation 1.
		GRGB = 2,    //!< Standard Bayer color filter, 1 red 2 green 1 blue. Variation 2.
		GBGR = 3,    //!< Standard Bayer color filter, 1 red 2 green 1 blue. Variation 3.
		BGRG = 4,    //!< Standard Bayer color filter, 1 red 2 green 1 blue. Variation 4.
		RGBW = 5,    //!< Modified Bayer color filter, with white (pass all light) instead of extra green. Variation 1.
		GRWB = 6,    //!< Modified Bayer color filter, with white (pass all light) instead of extra green. Variation 2.
		WBGR = 7,    //!< Modified Bayer color filter, with white (pass all light) instead of extra green. Variation 3.
		BWRG = 8,    //!< Modified Bayer color filter, with white (pass all light) instead of extra green. Variation 4.
	};

	using FrameEventBase = struct caer_frame_event;

	struct FrameEvent: public FrameEventBase {
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

		int32_t getTimestamp() const noexcept {
			return (caerFrameEventGetTimestamp(this));
		}

		int64_t getTimestamp64(const FrameEventPacket &packet) const noexcept {
			return (caerFrameEventGetTimestamp64(this, reinterpret_cast<caerFrameEventPacketConst>(packet.header)));
		}

		int32_t getExposureLength() const noexcept {
			return (caerFrameEventGetExposureLength(this));
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

		uint8_t getROIIdentifier() const noexcept {
			return (caerFrameEventGetROIIdentifier(this));
		}

		void setROIIdentifier(uint8_t roiIdent) noexcept {
			caerFrameEventSetROIIdentifier(this, roiIdent);
		}

		colorFilter getColorFilter() const noexcept {
			return (static_cast<colorFilter>(caerFrameEventGetColorFilter(this)));
		}

		void setColorFilter(colorFilter cFilter) noexcept {
			caerFrameEventSetColorFilter(this,
				static_cast<enum caer_frame_event_color_filter>(static_cast<typename std::underlying_type<colorFilter>::type>(cFilter)));
		}

		int32_t getLengthX() const noexcept {
			return (caerFrameEventGetLengthX(this));
		}

		int32_t getLengthY() const noexcept {
			return (caerFrameEventGetLengthY(this));
		}

		colorChannels getChannelNumber() const noexcept {
			return (static_cast<colorChannels>(caerFrameEventGetChannelNumber(this)));
		}

		void setLengthXLengthYChannelNumber(int32_t lenX, int32_t lenY, colorChannels cNumber,
			const FrameEventPacket &packet) {
			// Verify lengths and color channels number don't exceed allocated space.
			enum caer_frame_event_color_channels cNumberEnum =
				static_cast<enum caer_frame_event_color_channels>(static_cast<typename std::underlying_type<
					colorChannels>::type>(cNumber));

			size_t neededMemory = (sizeof(uint16_t) * static_cast<size_t>(lenX) * static_cast<size_t>(lenY)
				* cNumberEnum);

			if (neededMemory > packet.getPixelsSize()) {
				throw std::invalid_argument(
					"Given values result in memory usage higher than allocated frame event size.");
			}

			caerFrameEventSetLengthXLengthYChannelNumber(this, lenX, lenY, cNumberEnum,
				reinterpret_cast<caerFrameEventPacketConst>(packet.header));
		}

		size_t getPixelsMaxIndex() const noexcept {
			return (caerFrameEventGetPixelsMaxIndex(this));
		}

		size_t getPixelsSize() const noexcept {
			return (caerFrameEventGetPixelsSize(this));
		}

		int32_t getPositionX() const noexcept {
			return (caerFrameEventGetPositionX(this));
		}

		void setPositionX(int32_t posX) noexcept {
			caerFrameEventSetPositionX(this, posX);
		}

		int32_t getPositionY() const noexcept {
			return (caerFrameEventGetPositionY(this));
		}

		void setPositionY(int32_t posY) noexcept {
			caerFrameEventSetPositionY(this, posY);
		}

		uint16_t getPixel(int32_t xAddress, int32_t yAddress) const {
			// Check frame bounds first.
			if (yAddress < 0 || yAddress >= caerFrameEventGetLengthY(this)) {
				throw std::invalid_argument("Invalid Y address.");
			}

			int32_t xLength = caerFrameEventGetLengthX(this);

			if (xAddress < 0 || xAddress >= xLength) {
				throw std::invalid_argument("Invalid X address.");
			}

			// Get pixel value at specified position.
			return (le16toh(this->pixels[(yAddress * xLength) + xAddress]));
		}

		void setPixel(int32_t xAddress, int32_t yAddress, uint16_t pixelValue) {
			// Check frame bounds first.
			if (yAddress < 0 || yAddress >= caerFrameEventGetLengthY(this)) {
				throw std::invalid_argument("Invalid Y address.");
			}

			int32_t xLength = caerFrameEventGetLengthX(this);

			if (xAddress < 0 || xAddress >= xLength) {
				throw std::invalid_argument("Invalid X address.");
			}

			// Set pixel value at specified position.
			this->pixels[(yAddress * xLength) + xAddress] = htole16(pixelValue);
		}

		uint16_t getPixel(int32_t xAddress, int32_t yAddress, uint8_t channel) const {
			// Check frame bounds first.
			if (yAddress < 0 || yAddress >= caerFrameEventGetLengthY(this)) {
				throw std::invalid_argument("Invalid Y address.");
			}

			int32_t xLength = caerFrameEventGetLengthX(this);

			if (xAddress < 0 || xAddress >= xLength) {
				throw std::invalid_argument("Invalid X address.");
			}

			uint8_t channelNumber = caerFrameEventGetChannelNumber(this);

			if (channel >= channelNumber) {
				throw std::invalid_argument("Invalid channel number.");
			}

			// Get pixel value at specified position.
			return (le16toh(this->pixels[(((yAddress * xLength) + xAddress) * channelNumber) + channel]));
		}

		void setPixel(int32_t xAddress, int32_t yAddress, uint8_t channel, uint16_t pixelValue) {
			// Check frame bounds first.
			if (yAddress < 0 || yAddress >= caerFrameEventGetLengthY(this)) {
				throw std::invalid_argument("Invalid Y address.");
			}

			int32_t xLength = caerFrameEventGetLengthX(this);

			if (xAddress < 0 || xAddress >= xLength) {
				throw std::invalid_argument("Invalid X address.");
			}

			uint8_t channelNumber = caerFrameEventGetChannelNumber(this);

			if (channel >= channelNumber) {
				throw std::invalid_argument("Invalid channel number.");
			}

			// Set pixel value at specified position.
			this->pixels[(((yAddress * xLength) + xAddress) * channelNumber) + channel] = htole16(pixelValue);
		}

		uint16_t getPixelUnsafe(int32_t xAddress, int32_t yAddress) const noexcept {
			// Get pixel value at specified position.
			return (le16toh(this->pixels[(yAddress * caerFrameEventGetLengthX(this)) + xAddress]));
		}

		void setPixelUnsafe(int32_t xAddress, int32_t yAddress, uint16_t pixelValue) noexcept {
			// Set pixel value at specified position.
			this->pixels[(yAddress * caerFrameEventGetLengthX(this)) + xAddress] = htole16(pixelValue);
		}

		uint16_t getPixelUnsafe(int32_t xAddress, int32_t yAddress, uint8_t channel) const noexcept {
			uint8_t channelNumber = caerFrameEventGetChannelNumber(this);
			// Get pixel value at specified position.
			return (le16toh(
				this->pixels[(((yAddress * caerFrameEventGetLengthX(this)) + xAddress) * channelNumber) + channel]));
		}

		void setPixelUnsafe(int32_t xAddress, int32_t yAddress, uint8_t channel, uint16_t pixelValue) noexcept {
			uint8_t channelNumber = caerFrameEventGetChannelNumber(this);
			// Set pixel value at specified position.
			this->pixels[(((yAddress * caerFrameEventGetLengthX(this)) + xAddress) * channelNumber) + channel] =
				htole16(pixelValue);
		}

		uint16_t *getPixelArrayUnsafe() noexcept {
			return (this->pixels);
		}

		const uint16_t *getPixelArrayUnsafe() const noexcept {
			return (this->pixels);
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

	FrameEventPacket(caerFrameEventPacket packet) {
		if (packet == nullptr) {
			throw std::runtime_error("Failed to initialize event packet from existing C packet: null pointer.");
		}

		// Check for proper event type too!
		if (caerEventPacketHeaderGetEventType(&packet->packetHeader) != FRAME_EVENT) {
			throw std::runtime_error("Failed to initialize event packet from existing C packet: wrong type.");
		}

		header = &packet->packetHeader;
	}

	FrameEventPacket(caerEventPacketHeader packetHeader) {
		if (packetHeader == nullptr) {
			throw std::runtime_error("Failed to initialize event packet from existing C packet header: null pointer.");
		}

		// Check for proper event type too!
		if (caerEventPacketHeaderGetEventType(packetHeader) != FRAME_EVENT) {
			throw std::runtime_error("Failed to initialize event packet from existing C packet header: wrong type.");
		}

		header = packetHeader;
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

	FrameEventPacket copy() const {
		return (FrameEventPacket(internalCopy(header)));
	}

	FrameEventPacket copyOnlyEvents() const {
		return (FrameEventPacket(internalCopyOnlyEvents(header)));
	}

	FrameEventPacket copyOnlyValidEvents() const {
		return (FrameEventPacket(internalCopyOnlyValidEvents(header)));
	}

	size_t getPixelsSize() const noexcept {
		return (caerFrameEventPacketGetPixelsSize(reinterpret_cast<caerFrameEventPacketConst>(header)));
	}

	size_t getPixelsMaxIndex() const noexcept {
		return (caerFrameEventPacketGetPixelsMaxIndex(reinterpret_cast<caerFrameEventPacketConst>(header)));
	}

	FrameEventPacket demosaic() const {
		caerFrameEventPacket colorPacket = caerFrameUtilsDemosaic(reinterpret_cast<caerFrameEventPacketConst>(header));
		if (colorPacket == nullptr) {
			throw std::runtime_error("Failed to generate a demosaiced frame event packet.");
		}

		return (FrameEventPacket(colorPacket));
	}

	void contrast() noexcept {
		caerFrameUtilsContrast(reinterpret_cast<caerFrameEventPacket>(header));
	}

#if defined(LIBCAER_HAVE_OPENCV)

	// DEMOSAIC_VARIABLE_NUMBER_OF_GRADIENTS not supported on 16bit images currently.
	enum class opencvDemosaic {
		NORMAL = 0,
		EDGE_AWARE = 1,
	};

	enum class opencvContrast {
		NORMALIZATION = 0,
		HISTOGRAM_EQUALIZATION = 1,
		CLAHE = 2,
	};

	FrameEventPacket demosaic(opencvDemosaic demosaicType) const {
		caerFrameEventPacket colorPacket =
			caerFrameUtilsOpenCVDemosaic(reinterpret_cast<caerFrameEventPacketConst>(header),
				static_cast<enum caer_frame_utils_opencv_demosaic>(static_cast<typename std::underlying_type<
					opencvDemosaic>::type>(demosaicType)));
		if (colorPacket == nullptr) {
			throw std::runtime_error("Failed to generate a demosaiced frame event packet using OpenCV.");
		}

		return (FrameEventPacket(colorPacket));
	}

	void contrast(opencvContrast contrastType) noexcept {
		caerFrameUtilsOpenCVContrast(reinterpret_cast<caerFrameEventPacket>(header),
			static_cast<enum caer_frame_utils_opencv_contrast>(static_cast<typename std::underlying_type<opencvContrast>::type>(contrastType)));
	}

#endif
};

}
}

#endif /* LIBCAER_EVENTS_FRAME_HPP_ */
