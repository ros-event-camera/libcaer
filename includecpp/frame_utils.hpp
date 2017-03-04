#ifndef LIBCAER_FRAME_UTILS_HPP_
#define LIBCAER_FRAME_UTILS_HPP_

#include <libcaer/frame_utils.h>

namespace libcaer {
namespace frame_utils {

caerFrameEventPacket caerFrameUtilsDemosaic(caerFrameEventPacketConst framePacket);
void caerFrameUtilsContrast(caerFrameEventPacket framePacket);

}
}

#endif /* LIBCAER_FRAME_UTILS_HPP_ */
