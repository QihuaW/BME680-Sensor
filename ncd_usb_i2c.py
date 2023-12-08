"""
Sends hex packet over serial port to NCD USB to I2C converter in NCD API Frame format
Read/write 1 byte

Formatting (NCD API Frame):
0xAA is NCD header byte
0x## is number of bytes
0xBE is write command, 0xBF is read command
0x## I2C address
0x## byte to write/ 0x## bytes to read
...
0x## Check sum

Must install FTDI Drivers to use
"""

#Imports
import time
import serial as s

class usb_i2c:
    def __init__(self, port):
        # Set up serial port
        self.handle = s.Serial()
        self.handle.port = port
        self.handle.baudrate = 115200
        self.handle.bytesize = 8
        self.handle.stopbits = 1
        self.handle.parity = 'N'
        self.handle.timeout = 0.05

    def send_receive(self, packet):
        num_bytes = len(packet) # Determine number of bytes to send
        summation = 170 + num_bytes + sum(packet) # Sum all bytes for check sum
        check_sum = summation % 256 # Determine last two bytes (check sum)
        packet.insert(0, 170) # Insert NCD header to packet (0xAA = 170)
        packet.insert(1, num_bytes) # Insert byte count after header
        packet.append(check_sum) # Append check sum to end of packet
        packet_to_send = bytearray(packet) # Convert packet to hex bytes
        self.handle.open() # Open serial port
        self.handle.write(packet_to_send) # Write to serial port
        time.sleep(0.01) # Wait for response
        response = []
        #response += self.handle.readline()
        while True:
            line = self.handle.read()
            if line == b'':
                break
            else:
                response.append(ord(line))
        self.handle.close() # Close serial port
        return response # Raise response

    def write(self, i2c_address, register, msg = None):
        if isinstance(register, int) == True:
            register = [register]
        else: pass
        
        if msg == None:
            packet = [190, i2c_address] + register
        else:
            if isinstance(msg, int) == True:
                msg = [msg]
            else: pass
            packet = [190, i2c_address] + register + msg # Assemble packet to send, (0xBE = 190 is write command)
        response = self.send_receive(packet)
        return response

    def read(self, i2c_address, register, length):
        write_response = self.write(i2c_address, register)
        packet = [191, i2c_address, length] # Assemble packet to send, (0xBF = 191 is read command), read 1 byte
        read_response = self.send_receive(packet)
        return read_response

if __name__ == "__main__":        
    port = 'COM24'
    i2c = usb_i2c(port)
    while True:
        i2c.write(0x70, 0x0C, [0, 0, 0, 0, 0])
        time.sleep(0.1)
        response = i2c.read(0x70, 0x0C, 7)
        print(response)
        time.sleep(0.1)
