# Python example for interfacing with DVS128
# Author: federico.corradi@inilabs.com
# basic example for python and libcaer
# online doc is here : https://inilabs.github.io/libcaer/files.html

import _libcaer_wrap as libcaer

class DVS128:
    def __init__(self):
        """ init DVS128, display info, and start data transfer """   
        self.handle = libcaer.caerDeviceOpen(1, libcaer.CAER_DEVICE_DVS128, 0, 0, "")
        self.info = libcaer.caerDVS128InfoGet(self.handle)
        print("device ID: "+str(libcaer.caer_dvs128_info_deviceID_get(self.info)))
        if(libcaer.caer_dvs128_info_deviceIsMaster_get(self.info)):
            print("device is Master: ")
        else:
            print("device is Slave")
        print("device Serial Number"+str(libcaer.caer_dvs128_info_deviceSerialNumber_get(self.info)))	
        print(libcaer.caer_dvs128_info_deviceString_get(self.info))
        
        # containers
        self.x = []
        self.y = []
        self.ts = []
        self.pol = []
        self.special = []
        self.special_ts = []
        
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
        """ A simple function that reads events from dvs128 """
        packetContainer = libcaer.caerDeviceDataGet(self.handle)
        packetNum = libcaer.caerEventPacketContainerGetEventPacketsNumber(packetContainer)
        for i in range(packetNum):
            packetHeader = libcaer.caerEventPacketContainerGetEventPacket(packetContainer, i)
            if packetHeader == None:
                continue
            packetType = libcaer.caerEventPacketHeaderGetEventType(packetHeader)
            
            #get only polarity and special events
            if packetType == libcaer.POLARITY_EVENT:
                #loop over all events
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
                    polarity = libcaer.caerSpecialEventPacketFromPacketHeader(packetHeader)
                    event = libcaer.caerPolarityEventPacketGetEvent(polarity, this_e)
                    self.special_ts.append(libcaer.caerSpecialEventGetTimestamp(event))
                    self.special.append(libcaer.caerSpecialEventGetData(event))
             
        
if __name__ == "__main__":

    import dvs128

    camera = dvs128.DVS128()
    camera.read_events()
    print("pixels x:"+str(camera.x))
    print("pixels y:"+str(camera.y)) 
    print("pixels ts:"+str(camera.ts))  
    print("pixels pol:"+str(camera.pol))  
    
