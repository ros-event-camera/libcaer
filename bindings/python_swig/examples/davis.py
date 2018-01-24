# Python example for interfacing with DAVIS
# Authors: federico.corradi@inilabs.com, luca.longinotti@inivation.com
# basic example for python and libcaer
# doc is here: https://inilabs.github.io/libcaer/

import _libcaer_wrap as libcaer
import numpy
import cv2


class DAVIS:

    def __init__(self, busRestriction=0, devAddressRestriction=0, serialNumber=""):
        """ init DAVIS, display info, and start data transfer """
        self.handle = libcaer.caerDeviceOpen(1, libcaer.CAER_DEVICE_DAVIS, busRestriction, devAddressRestriction, serialNumber)
        self.info = libcaer.caerDavisInfoGet(self.handle)

        print("device ID: " + str(libcaer.caer_davis_info_deviceID_get(self.info)))

        if (libcaer.caer_davis_info_deviceIsMaster_get(self.info)):
            print("device is Master")
        else:
            print("device is Slave")

        print("device Serial Number: " + str(libcaer.caer_davis_info_deviceSerialNumber_get(self.info)))
        print(libcaer.caer_davis_info_deviceString_get(self.info))

        self.dvsSizeX = libcaer.caer_davis_info_dvsSizeX_get(self.info)
        self.dvsSizeY = libcaer.caer_davis_info_dvsSizeY_get(self.info)

        self.apsSizeX = libcaer.caer_davis_info_apsSizeX_get(self.info)
        self.apsSizeY = libcaer.caer_davis_info_apsSizeY_get(self.info)

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
        """ A simple function that reads events from DAVIS sensors: polarity, frame, imu, special"""
        polarity = None
        frame = None
        imu = None
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

                elif packetType == libcaer.FRAME_EVENT:
                    # only get first frame event in packet
                    framePacket = libcaer.caerFrameEventPacketFromPacketHeaderConst(packetHeader)

                    frameEvent = libcaer.caerFrameEventPacketGetEventConst(framePacket, 0)

                    frame_numpy = numpy.empty((self.apsSizeY, self.apsSizeX), dtype=numpy.uint16)

                    # read pixels values
                    for y in range(libcaer.caerFrameEventGetLengthY(frameEvent)):
                        for x in range(libcaer.caerFrameEventGetLengthX(frameEvent)):
                            frame_numpy[y, x] = libcaer.caerFrameEventGetPixel(frameEvent, x, y)

                    frame_ts = libcaer.caerFrameEventGetTimestamp(frameEvent)

                    frame = (frame_ts, frame_numpy)

                elif packetType == libcaer.IMU6_EVENT:
                    # loop over all IMU 6-axis events
                    imuPacket = libcaer.caerIMU6EventPacketFromPacketHeaderConst(packetHeader)

                    imu_ts = numpy.empty(eventNum, dtype=numpy.int32)
                    imu_acc = numpy.empty((eventNum, 3), dtype=numpy.float32)
                    imu_gyro = numpy.empty((eventNum, 3), dtype=numpy.float32)
                    imu_temp = numpy.empty(eventNum, dtype=numpy.float32)

                    for e in range(eventNum):
                        imuEvent = libcaer.caerIMU6EventPacketGetEventConst(imuPacket, e)

                        imu_ts[e] = libcaer.caerIMU6EventGetTimestamp(imuEvent)
                        imu_acc[e, 0] = libcaer.caerIMU6EventGetAccelX(imuEvent)
                        imu_acc[e, 1] = libcaer.caerIMU6EventGetAccelY(imuEvent)
                        imu_acc[e, 2] = libcaer.caerIMU6EventGetAccelZ(imuEvent)
                        imu_gyro[e, 0] = libcaer.caerIMU6EventGetGyroX(imuEvent)
                        imu_gyro[e, 1] = libcaer.caerIMU6EventGetGyroY(imuEvent)
                        imu_gyro[e, 2] = libcaer.caerIMU6EventGetGyroZ(imuEvent)
                        imu_temp[e] = libcaer.caerIMU6EventGetTemp(imuEvent)

                    imu = (imu_ts, imu_acc, imu_gyro, imu_temp)

        return polarity, frame, imu, special


if __name__ == "__main__":
    camera = DAVIS()

    try:
        while True:
            polarity, frame, imu, special = camera.read_events()

            # if there are polarity events, accumulate them into a numpy array
            # and display it (black/white coding)
            if polarity != None:
                (polarity_ts, polarity_x, polarity_y, polarity_pol) = polarity

                matrix_events = numpy.full((camera.dvsSizeY, camera.dvsSizeX), 0.5)

                for e in range(len(polarity_ts)):
                    matrix_events[polarity_y[e], polarity_x[e]] = polarity_pol[e]

                cv2.imshow("polarity", matrix_events)

            # if a standard frame exists, show it
            if frame != None:
                (frame_ts, frame_numpy) = frame

                cv2.imshow("frame", frame_numpy)

            # wait 1ms on user input, required for OpenCV imshow()
            if polarity != None or frame != None:
                cv2.waitKey(1)

    except KeyboardInterrupt:
        # close camera on CTRL+C
        libcaer.caerDeviceDataStop(camera.handle)
        libcaer.caerDeviceClose(camera.handle)
