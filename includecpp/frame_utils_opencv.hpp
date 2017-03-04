#ifndef LIBCAER_FRAME_UTILS_OPENCV_HPP_
#define LIBCAER_FRAME_UTILS_OPENCV_HPP_

namespace libcaer {
namespace frame_utils {

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

caerFrameEventPacket caerFrameUtilsOpenCVDemosaic(caerFrameEventPacketConst framePacket,
	enum caer_frame_utils_opencv_demosaic demosaicType);
void caerFrameUtilsOpenCVContrast(caerFrameEventPacket framePacket, enum caer_frame_utils_opencv_contrast contrastType);

}
}

#endif /* LIBCAER_FRAME_UTILS_OPENCV_HPP_ */
