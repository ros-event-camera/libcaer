# Python example for interfacing with Dynap-se
# Author: federico.corradi@inilabs.com
# basic example for python and libcaer
# doc is here: https://inilabs.github.io/libcaer/

import _libcaer_wrap as libcaer


class DYNAPSE:
    def __init__(self):
        """ init Dynapse, display info, and start data transfer """   
        self.handle = libcaer.caerDeviceOpen(1, libcaer.CAER_DEVICE_DYNAPSE, 0, 0, "")
        self.info = libcaer.caerDynapseInfoGet(self.handle)
        print("device ID: "+str(libcaer.caer_dynapse_info_deviceID_get(self.info)))
        print("device Logic Version: "+str(libcaer.caer_dynapse_info_logicVersion_get(self.info)))	
        print(libcaer.caer_dynapse_info_deviceString_get(self.info))
        
        # containers
        self.neuid = []
        self.coreid = []
        self.chipid = []
        self.ts = []
        
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
        """ A simple function that reads spikes from dynapse """
        packetContainer = libcaer.caerDeviceDataGet(self.handle)
        packetNum = libcaer.caerEventPacketContainerGetEventPacketsNumber(packetContainer)
        for i in range(packetNum):
            packetHeader = libcaer.caerEventPacketContainerGetEventPacket(packetContainer, i)
            if packetHeader == None:
                continue
            packetType = libcaer.caerEventPacketHeaderGetEventType(packetHeader)
            
            #get only polarity and special events
            if packetType == libcaer.SPIKE_EVENT:
                #loop over all events
                nEvents = libcaer.caerEventPacketHeaderGetEventNumber(packetHeader)
                for this_e in range(nEvents):
                    spike = libcaer.caerSpikeEventPacketFromPacketHeader(packetHeader)
                    event = libcaer.caerSpikeEventPacketGetEvent(spike, this_e)
                    self.neuid.append(libcaer.caerSpikeEventGetNeuronID(event))
                    self.coreid.append(libcaer.caerSpikeEventGetSourceCoreID(event))
                    self.chipid.append(libcaer.caerSpikeEventGetChipID(event))
                    self.ts.append(libcaer.caerSpikeEventGetTimestamp(event))


        
if __name__ == "__main__":

    import dynapse

    board = dynapse.DYNAPSE()
    board.read_events()
    print(" We print some events... ")
    print("neu: "+str(board.neuid))
    print("core: "+str(board.coreid)) 
    print("chip: "+str(board.chipid))  
    print("ts: "+str(board.ts))  
