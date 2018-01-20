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
        polarity = []
        frame = []
        imu = []
        special = []

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

                    for e in range(eventNum):
                        polarityEvent = libcaer.caerPolarityEventPacketGetEventConst(polarityPacket, e)

                        polarity_ts = libcaer.caerPolarityEventGetTimestamp(polarityEvent)
                        polarity_x = libcaer.caerPolarityEventGetX(polarityEvent)
                        polarity_y = libcaer.caerPolarityEventGetY(polarityEvent)
                        polarity_pol = libcaer.caerPolarityEventGetPolarity(polarityEvent)

                        polarity.append((polarity_ts, polarity_x, polarity_y, polarity_pol))

                elif packetType == libcaer.SPECIAL_EVENT:
                    # loop over all special events
                    specialPacket = libcaer.caerSpecialEventPacketFromPacketHeaderConst(packetHeader)

                    for e in range(eventNum):
                        specialEvent = libcaer.caerSpecialEventPacketGetEventConst(specialPacket, e)

                        special_ts = libcaer.caerSpecialEventGetTimestamp(specialEvent)
                        special_type = libcaer.caerSpecialEventGetType(specialEvent)

                        special.append((special_ts, special_type))

                elif packetType == libcaer.FRAME_EVENT:
                    # only get first frame event in packet
                    framePacket = libcaer.caerFrameEventPacketFromPacketHeaderConst(packetHeader)

                    frameEvent = libcaer.caerFrameEventPacketGetEventConst(framePacket, 0)

                    frame_numpy = numpy.zeros((self.apsSizeY, self.apsSizeX), dtype=numpy.uint16)

                    # read pixels values
                    for y in range(libcaer.caerFrameEventGetLengthY(frameEvent)):
                        for x in range(libcaer.caerFrameEventGetLengthX(frameEvent)):
                            frame_numpy[y, x] = libcaer.caerFrameEventGetPixel(frameEvent, x, y)

                    frame_ts = libcaer.caerFrameEventGetTimestamp(frameEvent)

                    frame.append((frame_ts, frame_numpy))

                elif packetType == libcaer.IMU6_EVENT:
                    # loop over all IMU 6-axis events
                    imuPacket = libcaer.caerIMU6EventPacketFromPacketHeaderConst(packetHeader)

                    for e in range(eventNum):
                        imuEvent = libcaer.caerIMU6EventPacketGetEventConst(imuPacket, e)

                        x_acc = libcaer.caerIMU6EventGetAccelX(imuEvent)
                        y_acc = libcaer.caerIMU6EventGetAccelY(imuEvent)
                        z_acc = libcaer.caerIMU6EventGetAccelZ(imuEvent)
                        x_gyro = libcaer.caerIMU6EventGetGyroX(imuEvent)
                        y_gyro = libcaer.caerIMU6EventGetGyroY(imuEvent)
                        z_gyro = libcaer.caerIMU6EventGetGyroZ(imuEvent)

                        imu_ts = libcaer.caerIMU6EventGetTimestamp(imuEvent)
                        imu_acc = (x_acc, y_acc, z_acc)
                        imu_gyro = (x_gyro, y_gyro, z_gyro)
                        imu_temp = libcaer.caerIMU6EventGetTemp(imuEvent)

                        imu.append((imu_ts, imu_acc, imu_gyro, imu_temp))

        return polarity, frame, imu, special


if __name__ == "__main__":
    camera = DAVIS()

    try:
        while True:
            polarity, frame, imu, special = camera.read_events()

            # if there are polarity events, accumulate them into a numpy array
            # and display it (black/white coding)
            if len(polarity) > 0:
                matrix_events = numpy.full((camera.dvsSizeY, camera.dvsSizeX), 0.5)

                for evt in polarity:
                    matrix_events[evt[2], evt[1]] = evt[3]

                cv2.imshow("polarity", matrix_events)

            # if a standard frame exists, show it
            if len(frame) > 0:
                cv2.imshow("frame", frame[0][1])

            # wait 1ms on user input, required for OpenCV imshow()
            if len(polarity) > 0 or len(frame) > 0:
                cv2.waitKey(1)

    except KeyboardInterrupt:
        # close camera on CTRL+C
        libcaer.caerDeviceDataStop(camera.handle)
        libcaer.caerDeviceClose(camera.handle)
