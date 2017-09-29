# Python example for interfacing with DAVIS
# Author: federico.corradi@inilabs.com
# basic example for python and libcaer
# doc is here: https://inilabs.github.io/libcaer/

import _libcaer_wrap as libcaer
import numpy as np

class DAVIS:
    def __init__(self, busRestriction = 0, devAddressRestriction = 0, serialNumber = ""):
        """ init DAVIS, display info, and start data transfer """   
        self.handle = libcaer.caerDeviceOpen(1, libcaer.CAER_DEVICE_DAVIS, busRestriction, devAddressRestriction, serialNumber)
        self.info = libcaer.caerDavisInfoGet(self.handle)
        print("device ID: "+str(libcaer.caer_davis_info_deviceID_get(self.info)))
        if(libcaer.caer_davis_info_deviceIsMaster_get(self.info)):
            print("device is Master: ")
        else:
            print("device is Slave")
        print("device Serial Number"+str(libcaer.caer_davis_info_deviceSerialNumber_get(self.info)))	
        print(libcaer.caer_davis_info_deviceString_get(self.info))
        
        self.sizeX = libcaer.caer_davis_info_dvsSizeX_get(self.info)
        self.sizeY = libcaer.caer_davis_info_dvsSizeY_get(self.info)
        
        # data containers
        self.x = []
        self.y = []
        self.ts = []
        self.pol = []
        self.special = []
        self.special_ts = []
        self.frame = []
        self.frame_ts = []
        self.imu_gyro = []
        self.imu_acc = []
        self.imu_ts = []
        self.imu_temp = []
        
        # init default biases
        ret = libcaer.caerDeviceSendDefaultConfig(self.handle)
        if(ret == True):
            print("Default biases loaded")
        else:
            print("Error while loading default biases")
            raise Exception
            
        # start data transfer from device
        ret = libcaer.caerDeviceDataStart(self.handle, None, None, None, None, None)
        if(ret == True):
            print("Data transfer started")
        else:
            print("Error in data transfer")
            raise Exception 
            
        # set data exchange blocking    
        ret = libcaer.caerDeviceConfigSet(self.handle, libcaer.CAER_HOST_CONFIG_DATAEXCHANGE, libcaer.CAER_HOST_CONFIG_DATAEXCHANGE_BLOCKING, True)
        if(ret == True):
            print("Data exchange set to blocking mode")
        else:
            print("Error in communicating with the device, please check your setup")
            raise Exception 
        
    def read_events(self):
        """ A simple function that reads events from DAVIS sensors: polarity, imu6, special, frames"""
        packetContainer = libcaer.caerDeviceDataGet(self.handle)
        if packetContainer == None:
            return False
        else:
            packetNum = libcaer.caerEventPacketContainerGetEventPacketsNumber(packetContainer)
            for i in range(packetNum):
                packetHeader = libcaer.caerEventPacketContainerGetEventPacket(packetContainer, i)
                if packetHeader == None:
                    continue
                packetType = libcaer.caerEventPacketHeaderGetEventType(packetHeader)
                
                #get only polarity and special events
                if packetType == libcaer.POLARITY_EVENT:
                    #loop over all polarity events
                    nEvents = libcaer.caerEventPacketHeaderGetEventNumber(packetHeader)
                    for this_e in range(nEvents):
                        polarity = libcaer.caerPolarityEventPacketFromPacketHeader(packetHeader)
                        event = libcaer.caerPolarityEventPacketGetEvent(polarity, this_e)
                        self.ts.append(libcaer.caerPolarityEventGetTimestamp(event))
                        self.x.append(libcaer.caerPolarityEventGetX(event))
                        self.y.append(libcaer.caerPolarityEventGetY(event))
                        self.pol.append(libcaer.caerPolarityEventGetPolarity(event))
                        
                if packetType == libcaer.SPECIAL_EVENT:
                    #loop over all special events
                    nEvents = libcaer.caerEventPacketHeaderGetEventNumber(packetHeader)
                    for this_e in range(nEvents):
                        special = libcaer.caerSpecialEventPacketFromPacketHeader(packetHeader)
                        event = libcaer.caerSpecialEventPacketGetEvent(special, this_e)
                        self.special_ts.append(libcaer.caerSpecialEventGetTimestamp(event))
                        self.special.append(libcaer.caerSpecialEventGetData(event))
                        
                if packetType == libcaer.FRAME_EVENT:
                    frame = libcaer.caerFrameEventPacketFromPacketHeader(packetHeader)
                    # only get first frame event in packet
                    firstEvent = libcaer.caerFrameEventPacketGetEventConst(frame, 0)
                    self.frame_ts = libcaer.caerFrameEventGetTimestamp(firstEvent)
                    matrix_frame = np.zeros([libcaer.caer_davis_info_dvsSizeX_get(self.info), libcaer.caer_davis_info_dvsSizeY_get(self.info)])
                    #read pixels values
                    for y in range(libcaer.caerFrameEventGetLengthY(firstEvent)):
                        for x in range(libcaer.caerFrameEventGetLengthX(firstEvent)):
                            matrix_frame[x,y] = libcaer.caerFrameEventGetPixel(firstEvent, x, y)
                    self.frame.append(matrix_frame)
                    
                if packetType == libcaer.IMU6_EVENT:    
                    nEvents = libcaer.caerEventPacketHeaderGetEventNumber(packetHeader)
                    imu = libcaer.caerIMU6EventPacketFromPacketHeader(packetHeader)
                    for this_e in range(nEvents):
                        imu6 = libcaer.caerIMU6EventPacketGetEvent(imu, this_e)
                        x_acc = libcaer.caerIMU6EventGetAccelX(imu6)
                        y_acc = libcaer.caerIMU6EventGetAccelY(imu6)
                        z_acc = libcaer.caerIMU6EventGetAccelZ(imu6)
                        x_gyro = libcaer.caerIMU6EventGetGyroX(imu6)
                        y_gyro = libcaer.caerIMU6EventGetGyroY(imu6)
                        z_gyro = libcaer.caerIMU6EventGetGyroZ(imu6)
                        self.imu_acc.append([x_acc, y_acc, z_acc])
                        self.imu_gyro.append([x_gyro, y_gyro, z_gyro])
                        self.imu_ts.append(libcaer.caerIMU6EventGetTimestamp(imu6))
                        self.imu_temp.append(libcaer.caerIMU6EventGetTemp(imu6))                        
        return True
        
if __name__ == "__main__":

    import davis

    camera = davis.DAVIS()
    if(camera.read_events() == True):
        print("succesfully read events from the device")
        print("pixels x:"+str(camera.x))
        print("pixels y:"+str(camera.y)) 
        print("pixels ts:"+str(camera.ts))  
        print("pixels pol:"+str(camera.pol)) 
        print("camera frames:"+str(len(camera.frame))) 
    else:
        print("nothing read from the device")     
    
