
import os, sys, time
import serial
import struct
import queue

from .radar_parseFrame import parseStandardFrame
from .constants import *


class uartParser():
    def __init__(self):

        self.parserType = "DoubleCOMPort"

    def connectComPorts(self, cliCom, dataCom):
        self.cliCom = serial.Serial(cliCom, 115200, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, timeout=0.6)
        self.dataCom = serial.Serial(dataCom, 921600, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, timeout=0.6)
        self.dataCom.reset_output_buffer()
        print('Connected')

    def readAndParseUartDoubleCOMPort(self):
        
        self.fail = 0
    
        # Find magic word, and therefore the start of the frame
        index = 0
        magicByte = self.dataCom.read(1)
        frameData = bytearray(b'')
        while (1):
            # If the device doesnt transmit any data, the COMPort read function will eventually timeout
            # Which means magicByte will hold no data, and the call to magicByte[0] will produce an error
            # This check ensures we can give a meaningful error
            if (len(magicByte) < 1):
                print ("ERROR: No data detected on COM Port, read timed out")
                print("\tBe sure that the device is in the proper mode, and that the cfg you are sending is valid")
                magicByte = self.dataCom.read(1)
                
            # Found matching byte
            elif (magicByte[0] == UART_MAGIC_WORD[index]):
                index += 1
                frameData.append(magicByte[0])
                if (index == 8): # Found the full magic word
                    break
                magicByte = self.dataCom.read(1)
                
            else:
                # When you fail, you need to compare your byte against that byte (ie the 4th) AS WELL AS compare it to the first byte of sequence
                # Therefore, we should only read a new byte if we are sure the current byte does not match the 1st byte of the magic word sequence
                if (index == 0): 
                    magicByte = self.dataCom.read(1)
                index = 0 # Reset index
                frameData = bytearray(b'') # Reset current frame data
        
        # Read in version from the header
        versionBytes = self.dataCom.read(4)
        
        frameData += bytearray(versionBytes)

        # Read in length from header
        lengthBytes = self.dataCom.read(4)
        frameData += bytearray(lengthBytes)
        frameLength = int.from_bytes(lengthBytes, byteorder='little')
        
        # Subtract bytes that have already been read, IE magic word, version, and length
        # This ensures that we only read the part of the frame in that we are lacking
        frameLength -= 16 

        # Read in rest of the frame
        frameData += bytearray(self.dataCom.read(frameLength))

        # # If save binary is enabled
        # if(self.saveBinary == 1):
        #     self.binData += frameData
        #     # Save data every framesPerFile frames
        #     self.uartCounter += 1
        #     if (self.uartCounter % self.framesPerFile == 0):
        #         # First file requires the path to be set up
        #         if(self.first_file is True): 
        #             if(os.path.exists('binData/') == False):
        #                 # Note that this will create the folder in the caller's path, not necessarily in the Industrial Viz Folder                        
        #                 os.mkdir('binData/')
        #             os.mkdir('binData/'+self.filepath)
        #             self.first_file = False
        #         toSave = bytes(self.binData)
        #         fileName = 'binData/' + self.filepath + '/pHistBytes_' + str(math.floor(self.uartCounter/self.framesPerFile)) + '.bin'
        #         bfile = open(fileName, 'wb')
        #         bfile.write(toSave)
        #         bfile.close()
        #         # Reset binData and missed frames
        #         self.binData = []
 
        # frameData now contains an entire frame, send it to parser
        if (self.parserType == "DoubleCOMPort"):
            outputDict = parseStandardFrame(frameData)
        else:
            print ('FAILURE: Bad parserType')
        
        return outputDict
    
        #send cfg over uart
    def sendCfg(self, cfg):
        # Ensure each line ends in \n for proper parsing
        for i, line in enumerate(cfg):
            # Remove empty lines from cfg
            if(line == '\n'):
                cfg.remove(line)
            # add a newline to the end of every line (protects against last line not having a newline at the end of it)
            elif(line[-1] != '\n'):
                cfg[i] = cfg[i] + '\n'

        for line in cfg:
            time.sleep(.03) # Line delay

            if(self.cliCom.baudrate == 1250000):
                for char in [*line]:
                    time.sleep(.001) # Character delay. Required for demos which are 1250000 baud by default else characters are skipped
                    self.cliCom.write(char.encode())
            else:
                self.cliCom.write(line.encode())
                
            ack = self.cliCom.readline()
            # print(ack)
            ack = self.cliCom.readline()
            # print(ack)
            splitLine = line.split()
            if(splitLine[0] == "baudRate"): # The baudrate CLI line changes the CLI baud rate on the next cfg line to enable greater data streaming off the IWRL device.
                try:
                    self.cliCom.baudrate = int(splitLine[1])
                except:
                    print("Error - Invalid baud rate")
                    sys.exit(1)
        # Give a short amount of time for the buffer to clear
        time.sleep(0.03)
        self.cliCom.reset_input_buffer()
        # NOTE - Do NOT close the CLI port because 6432 will use it after configuration


    #send single command to device over UART Com.
    def sendLine(self, line):
        self.cliCom.write(line.encode())
        ack = self.cliCom.readline()
        print(ack)
        ack = self.cliCom.readline()
        print(ack)


def main():
    cfg_file = './IWR6843AOP-configs/baseline-AOP-people-detection.cfg'
    cfg = open(cfg_file, 'r').readlines()
    radar = uartParser()
    radar.connectComPorts('/dev/ttyUSB0', '/dev/ttyUSB1')
    radar.sendCfg(cfg)
    while(1):
        print('next frame')
        radar_dict = radar.readAndParseUartDoubleCOMPort()
        print(radar_dict.get('pointCloud'))
        # time.sleep(1)

def run_radar(output_queue:queue.Queue, cli_com_port='/dev/ttyUSB0', data_com_port='/dev/ttyUSB1', cfg_file='./IWR6843AOP-configs/baseline-AOP-people-detection.cfg'):

    working_dir = os.path.dirname(os.path.realpath(__file__))
    cfg_file = os.path.join(working_dir, cfg_file)

    cfg = open(cfg_file, 'r').readlines()
    radar = uartParser()
    radar.connectComPorts(cli_com_port, data_com_port) #defaults for linux
    radar.sendCfg(cfg)
    print('radar config sent')
    while True:
        print('next radar frame')
        radar_dict = radar.readAndParseUartDoubleCOMPort()
        # print(radar_dict.get('pointCloud'))
        # time.sleep(1)
        try:
            output_queue.put_nowait(radar_dict['pointCloud'])
        except queue.Full:
            output_queue.get()
            output_queue.put_nowait(radar_dict['pointCloud'])
        except Exception as e: 
            print(e)
            print('Exit run_radar main loop')
            break

    
if __name__ == '__main__':
    print('main in %s' % __file__)
    main()
