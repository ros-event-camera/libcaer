# Python example for interfacing with DVS128
# basic example for python and libcaer

import _libcaer_wrap as libcaer
import numpy
import cv2


class DVS128:

    def __init__(self, busRestriction=0, devAddressRestriction=0, serialNumber=""):
        """ init DVS128, display info, and start data transfer """
        self.handle = libcaer.caerDeviceOpen(1, libcaer.CAER_DEVICE_DVS128, busRestriction, devAddressRestriction, serialNumber)
        self.info = libcaer.caerDVS128InfoGet(self.handle)

        print("device ID: " + str(libcaer.caer_dvs128_info_deviceID_get(self.info)))

        if (libcaer.caer_dvs128_info_deviceIsMaster_get(self.info)):
            print("device is Master")
        else:
            print("device is Slave")

        print("device Serial Number: " + str(libcaer.caer_dvs128_info_deviceSerialNumber_get(self.info)))
        print(libcaer.caer_dvs128_info_deviceString_get(self.info))

        self.dvsSizeX = libcaer.caer_dvs128_info_dvsSizeX_get(self.info)
        self.dvsSizeY = libcaer.caer_dvs128_info_dvsSizeY_get(self.info)

        # init default biases
        ret = libcaer.caerDeviceSendDefaultConfig(self.handle)
        if(ret == True):
            print("Default biases loaded")
        else:
            print("Error while loading default biases")
            raise Exception

         # set blocking data exchange
        ret = libcaer.caerDeviceConfigSet(self.handle, libcaer.CAER_HOST_CONFIG_DATAEXCHANGE, libcaer.CAER_HOST_CONFIG_DATAEXCHANGE_BLOCKING, True)
        if(ret == True):
            print("Data exchange set to blocking mode")
        else:
            print("Error in communicating with the device, please check your setup")
            raise Exception

        # start data transfer from device
        ret = libcaer.caerDeviceDataStart(self.handle, None, None, None, None, None)
        if(ret == True):
            print("Data transfer started")
        else:
            print("Error in starting data transfer")
            raise Exception

    def read_events(self):
        """ A simple function that reads events from DVS128 sensors: polarity, special"""
        polarity = None
        special = None

        packetContainer = libcaer.caerDeviceDataGet(self.handle)

        if packetContainer != None:
            packetNum = libcaer.caerEventPacketContainerGetEventPacketsNumber(packetContainer)

            for i in range(packetNum):
                packetHeader = libcaer.caerEventPacketContainerGetEventPacketConst(packetContainer, i)

                if packetHeader == None:
                    continue

                packetType = libcaer.caerEventPacketHeaderGetEventType(packetHeader)
                eventNum = libcaer.caerEventPacketHeaderGetEventNumber(packetHeader)

                if packetType == libcaer.POLARITY_EVENT:
                    # loop over all polarity events
                    polarityPacket = libcaer.caerPolarityEventPacketFromPacketHeaderConst(packetHeader)

                    polarity_ts = numpy.empty(eventNum, dtype=numpy.int32)
                    polarity_x = numpy.empty(eventNum, dtype=numpy.uint16)
                    polarity_y = numpy.empty(eventNum, dtype=numpy.uint16)
                    polarity_pol = numpy.empty(eventNum, dtype=numpy.bool)

                    for e in range(eventNum):
                        polarityEvent = libcaer.caerPolarityEventPacketGetEventConst(polarityPacket, e)

                        polarity_ts[e] = libcaer.caerPolarityEventGetTimestamp(polarityEvent)
                        polarity_x[e] = libcaer.caerPolarityEventGetX(polarityEvent)
                        polarity_y[e] = libcaer.caerPolarityEventGetY(polarityEvent)
                        polarity_pol[e] = libcaer.caerPolarityEventGetPolarity(polarityEvent)

                    polarity = (polarity_ts, polarity_x, polarity_y, polarity_pol)

                elif packetType == libcaer.SPECIAL_EVENT:
                    # loop over all special events
                    specialPacket = libcaer.caerSpecialEventPacketFromPacketHeaderConst(packetHeader)

                    special_ts = numpy.empty(eventNum, dtype=numpy.int32)
                    special_type = numpy.empty(eventNum, dtype=numpy.uint8)
                    special_data = numpy.empty(eventNum, dtype=numpy.uint32)

                    for e in range(eventNum):
                        specialEvent = libcaer.caerSpecialEventPacketGetEventConst(specialPacket, e)

                        special_ts[e] = libcaer.caerSpecialEventGetTimestamp(specialEvent)
                        special_type[e] = libcaer.caerSpecialEventGetType(specialEvent)
                        special_data[e] = libcaer.caerSpecialEventGetData(specialEvent)

                    special = (special_ts, special_type, special_data)

        return polarity, special


if __name__ == "__main__":
    camera = DVS128()

    try:
        while True:
            polarity, special = camera.read_events()

            # if there are polarity events, accumulate them into a numpy array
            # and display it (black/white coding)
            if polarity != None:
                (polarity_ts, polarity_x, polarity_y, polarity_pol) = polarity

                matrix_events = numpy.full((camera.dvsSizeY, camera.dvsSizeX), 0.5)

                for e in range(len(polarity_ts)):
                    matrix_events[polarity_y[e], polarity_x[e]] = polarity_pol[e]

                cv2.imshow("polarity", matrix_events)
                cv2.waitKey(1)

    except KeyboardInterrupt:
        # close camera on CTRL+C
        libcaer.caerDeviceDataStop(camera.handle)
        libcaer.caerDeviceClose(camera.handle)
