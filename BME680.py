"""
Back end for communicating with the BME680 environmental sensor

Qihua Wu. June 1, 2021
"""
#Imports
import time
import numpy as np
import ncd_usb_i2c as ncd

class bme680:
    def __init__(self, comm, port):
        self.comm = comm
        # Utilizing the NCD USB to I2C converter circuit board
        if comm == 'ncd':
            self.handle = ncd.usb_i2c(port)

        # BME680 i2c address is 0x77
        self.address = 0x77

        # Get calibration registers
        calibration = self.read([137], 25) + self.read([225], 16)

        # Temperature calibration constants
        self.par_t1 = (calibration[34] << 8) + calibration[33]
        self.par_t2 = self.signed_byte((calibration[2] << 8) + calibration[1], 16)
        self.par_t3 = self.signed_byte(calibration[3], 8)

        # Humidity calibration constants
        self.par_h1 = (calibration[27] << 4) + (calibration[26] & 0x0F)
        self.par_h2 = (calibration[25] << 4) + (calibration[26] >> 4)
        self.par_h3 = self.signed_byte(calibration[28], 8)
        self.par_h4 = self.signed_byte(calibration[29], 8)
        self.par_h5 = self.signed_byte(calibration[30], 8)
        self.par_h6 = calibration[31]
        self.par_h7 = self.signed_byte(calibration[32], 8)

        # Pressure calibration constants
        self.par_p1 = (calibration[6] << 8) + calibration[5]
        self.par_p2 = self.signed_byte((calibration[8] << 8) + calibration[7], 16)
        self.par_p3 = self.signed_byte(calibration[9], 8)
        self.par_p4 = self.signed_byte((calibration[12] << 8) + calibration[11], 16)
        self.par_p5 = self.signed_byte((calibration[14] << 8) + calibration[13], 16)
        self.par_p6 = self.signed_byte(calibration[16], 8)
        self.par_p7 = self.signed_byte(calibration[15], 8)
        self.par_p8 = self.signed_byte((calibration[20] << 8) + calibration[19], 16)
        self.par_p9 = self.signed_byte((calibration[22] << 8) + calibration[21], 16)
        self.par_p10 = calibration[23]

        # Gas sensor calibration constants
        self.par_g1 = self.signed_byte(calibration[37], 8)
        self.par_g2 = self.signed_byte((calibration[36] << 8) + calibration[35], 16)
        self.par_g3 = self.signed_byte(calibration[38],8)
        self.res_heat_range = int(format(self.read([0x02], 1)[0], '08b')[2:4], 2) # 0x02 <5:4>
        self.res_heat_val = self.signed_byte(self.read([0x00], 1)[0], 8) # 0x00 <all>, signed value from -128 to 127
        self.range_switching_error = self.signed_byte(int(format(self.read([0x04], 1)[0], '08b')[:4], 2), 4) # 0x04 <7:4> (signed 4 bit)

        # Setting initial variables
        self.set_oversampling(8, 8, 8)
        self.set_IIR_filter(1)
        self.amb_temp = 24
        self.set_gas_heater(400)
        self.set_gas_wait(30, 16)
        self.set_gas_profile(0)
        self.set_gas_meas('off')
        self.amb_temp = self.take_measurement()[0]
        self.set_gas_heater(400)
        self.set_gas_meas('on')
        
    def signed_byte(self, value, length):
        # Converts byte to signed byte
        bit_string = format(value, ('0' + str(length) + 'b'))
        if bit_string[0] == '1':
            if bit_string == '1' + '0' * (length - 1):
                return -1 * (2 ** (length - 1))
            else:
                value = value - 1
                bit_string = format(value, '0' + str(length) + 'b')
                inverse_bit_string = ''.join('1' if x =='0' else '0' for x in bit_string)
                signed_value = -1 * int(inverse_bit_string,2)
                return signed_value
        else:
            signed_value = int(bit_string[1:], 2)
        return signed_value
        
    
    def write(self, register, msg):
        # Write msg to register
        if self.comm == 'ncd':
            self.handle.write(self.address, register, msg)
        elif self.comm == 'pi':
            self.handle.write_i2c_block_data(self.address, register, msg)
        else:
            pass

    def read(self, register, length):
        # Read data from register
        if self.comm == 'ncd':
            line = self.handle.read(self.address, register, length)
        elif self.comm == 'pi':
            line = self.handle.read_i2c_block_data(self.address, register, length)[0]
        else:
            pass
        return line[2:-1]
    
    
    def reset(self):
        # Soft reset similar to a power on cycle
        self.write([0xE0], 0xB6)

    def set_oversampling(self, tx, hx, px):
        # Specify the oversampling rate for temperature, humidity, pressure
        # Oversampling must be 1, 2, 4, 8, 16
        def convert_oversampling(num):
            if num == 1: return '001'
            elif num == 2: return '010'
            elif num == 4: return '011'
            elif num == 8: return '100'
            elif num == 16: return '101'
            else: print('Over sampingling must be 1, 2, 4, 8, or 16')
            
        self.osrs_t = convert_oversampling(tx)
        self.osrs_h = convert_oversampling(hx)
        self.osrs_p = convert_oversampling(px)

    def set_IIR_filter(self, IIR_coef):
        # Specify IIR Filter Coefficient
        # Higher filter reduces environmental fluctuations but lowers response time
        # Filters: 0, 1, 3, 7, 15, 31, 63, 127
        if IIR_coef == 0: self.IIR_filter = '000'
        elif IIR_coef == 1: self.IIR_filter = '001'
        elif IIR_coef == 3: self.IIR_filter = '010'
        elif IIR_coef == 7: self.IIR_filter = '011'
        elif IIR_coef == 15: self.IIR_filter = '100'
        elif IIR_coef == 31: self.IIR_filter = '101'
        elif IIR_coef == 63: self.IIR_filter = '110'
        elif IIR_coef == 127: self.IIR_filter = '111'
        else: print('IIR_coef must be 0, 1, 3, 7, 15, 31, 63, or 127')

    def set_gas_heater(self, temperature):
        # Store gas heater temperature
        self.heater_temperature = temperature

    def update_gas_heater(self):
        # Specify temperature of heater for gas sensor in Celcius
        var1 = (self.par_g1 / 16) + 49
        var2 = ((self.par_g2 / 32768) * 0.0005) + 0.00235
        var3 = self.par_g3 / 1024
        var4 = var1 * (1 + (var2 * self.heater_temperature))
        var5 = var4 + (var3 * self.amb_temp)
        res_heat = (3.4 * ((var5 * (4 / (4 + self.res_heat_range)) * \
                            (1 / (1 + (self.res_heat_val * 0.002)))) - 25))
        self.res_heat = format(round(res_heat), '08b')
        self.write([0x5A], int(self.res_heat, 2))

    def set_gas_wait(self, time, multi):
        # Specify amount of time for heater to stabilize before gas measurement
        # Time must be less than or equal to 63
        # Multiplication can be 1, 4, 16, 64
        if time < 64:
            gas_wait_time = format(time, '06b')
        else: print('Gas wait time must be less than 64')
        
        if multi == 1: gas_wait_multi = '00'
        elif multi == 4: gas_wait_multi = '01'
        elif multi == 16: gas_wait_multi = '10'
        elif multi == 64: gas_wait_multi = '11'
        else: print('Gas wait multiplication must be 1, 4, 16, or 64')
        
        self.gas_wait = gas_wait_multi + gas_wait_time

    def set_gas_profile(self, profile):
        # Profile must be 0 to 9
        if profile < 10: self.gas_profile = format(profile, '04b')
        else: print('Profile must be less 10')

    def set_gas_meas(self, on_off = 'on'):
        # Gas runs if set to 1
        if on_off == 'off': self.run_gas = '0'
        else: self.run_gas = '1'

    def read_data(self):
        data_package = self.read([0x1F],13) # Read pressure, temp, hum and gas MSB, (XLSB) and LSB
        return data_package
      
    def calculate_temperature(self, MSB, LSB, XLSB):
        # Collect ADC bytes from registers
        #MSB = self.read([0x22], 1) # MSB 0x22 <all>
        #LSB = self.read([0x23], 1) # LSB 0x23 <all>
        #XLSB = int(format(self.read([0x24], 1), '08b')[:4], 2) # XLSB 0x24 <7:4>

        # Combine register bytes to ADC value
        temperature_adc = (MSB << 12) + (LSB << 4) + XLSB

        # Convert ADC value to temperature
        var1 = (((temperature_adc / 16384) - (self.par_t1 / 1024)) * (self.par_t2))
        var2 = ((((temperature_adc / 131072) - (self.par_t1 / 8192)) * \
                 ((temperature_adc / 131072) - (self.par_t1 / 8192))) * \
                (self.par_t3 * 16))
        self.t_fine = var1 + var2
        temperature = self.t_fine / 5120
        self.amb_temp = temperature
        
        return temperature

    def calculate_pressure(self, MSB, LSB, XLSB):
        # Collect ADC bytes from registers
        #MSB = self.read([0x1F], 1) # MSB 0x1F <all>
        #LSB = self.read([0x20], 1) # LSB 0x20 <all>
        #XLSB = int(format(self.read([0x21], 1), '08b')[:4], 2) # XLSB 0x21 <7:4>
        
        # Combine register bytes to ADC value
        pressure_adc = (MSB << 12) + (LSB << 4) + XLSB

        # Convert ADC value to pressure
        var1 = (self.t_fine / 2) - 64000
        var2 = var1 * var1 * (self.par_p6 / 131072)
        var2 = var2 + (var1 * self.par_p5 * 2)
        var2 = (var2 / 4) + (self.par_p4 * 65536)
        var1 = (((self.par_p3 * var1 * var1) / 524288) + \
                 (self.par_p2 * var1)) / 524288
        var1 = (1 + (var1 / 32768)) * self.par_p1
        pressure = 1048576 - pressure_adc

        if var1 != 0:
            pressure = ((pressure - (var2 / 4096)) * 6250) / var1
            var1 = (self.par_p9 * pressure * pressure) / 2147483648
            var2 = pressure * (self.par_p8 / 32768)
            var3 = ((pressure / 256) * (pressure / 256) * (pressure / 256) * \
                    (self.par_p10 / 131072))
            pressure = pressure + (var1 + var2 + var3 + (self.par_p7 * 128)) / 16
        else: pressure = 0
        
        return pressure

    def calculate_humidity(self,MSB,LSB):
        # Collect ADC bytes from registers
        #MSB = self.read([0x25], 1) # 0x25 <all>
        #LSB = self.read([0x26], 1) # 0x26 <all>

        # Combine register bytes to ADC value
        humidity_adc = (MSB << 8) + LSB

        # Convert ADC value to humidity
        temp_comp =(self.t_fine / 5120)
        var1 = humidity_adc - ((self.par_h1 * 16) + ((self.par_h3 / 2) * temp_comp))
        var2 = var1 * (((self.par_h2 / 262144) * (1 + ((self.par_h4 / 16384) * temp_comp) \
                                                  + ((self.par_h5 / 1048576) * temp_comp * temp_comp))))
        var3 = self.par_h6 / 16384
        var4 = self.par_h7 / 2097152
        humidity = var2 + ((var3 + (var4 * temp_comp)) * var2 * var2)
        
        if humidity > 100: humidity = 100
        elif humidity < 0: humidity = 0
        else: pass

        return humidity

    def calculate_gas(self, MSB, LSB, gas_range):
        # Collect ADC bytes from registers
        #MSB = self.read([0x2A], 1) # gas_r_msb 0x2A <all>
        #LSB = int(format(self.read([0x2B], 1), '08b')[:2], 2) # gas_r_lsb 0x2B <7:6>
        gas_r = (MSB << 2) + LSB
        #gas_range = int(format(self.read([0x2B], 1), '08b')[4:], 2) # gas_range_r 0x2B <3:0>

        # Convert gas range value to constant values
        if gas_range == 0: const_1 = 1; const_2 = 8e6
        elif gas_range == 1: const_1 = 1; const_2 = 4e6
        elif gas_range == 2: const_1 = 1; const_2 = 2e6
        elif gas_range == 3: const_1 = 1; const_2 = 1e6
        elif gas_range == 4: const_1 = 1; const_2 = 499500.4995
        elif gas_range == 5: const_1 = 0.99; const_2 = 248262.1648
        elif gas_range == 6: const_1 = 1; const_2 = 125000
        elif gas_range == 7: const_1 = 0.992; const_2 = 63004.03226
        elif gas_range == 8: const_1 = 1; const_2 = 31281.28128
        elif gas_range == 9: const_1 = 1; const_2 = 15625
        elif gas_range == 10: const_1 = 0.998; const_2 = 7812.5
        elif gas_range == 11: const_1 = 0.995; const_2 = 3906.25
        elif gas_range == 12: const_1 = 1; const_2 = 1953.125
        elif gas_range == 13: const_1 = 0.99; const_2 = 976.5625
        elif gas_range == 14: const_1 = 1; const_2 = 488.28125
        else: const_1 = 1; const_2 = 244.140625

        # Convert ADC value to gas resistance
        var1 = (1340 + 5 * self.range_switching_error) * const_1
        gas_res = var1 * const_2 / (gas_r - 512 + var1)

        return gas_res

    def check_measurement(self):
        # Measurement bit is 1 when measurement is running
        bit = format(self.read([0x1D], 1)[0], '08b')[2]
        if bit == '1': measuring = True
        else: measuring = False
        return measuring

    def check_gas_valid(self,gas_lsb):
        # gas valid bit is 1 for real measurements
        bit = format(gas_lsb, '08b')[2] # gas_valid 0x2B <5>
        if bit == '1': valid = True
        else: valid = False
        return valid

    def check_heat_stability(self, gas_lsb):
        # Target heater resistance reached if bit is 0
        bit = format(gas_lsb, '08b')[3] # heat_stab 0x2B <4>
        if bit == '1': valid = True
        else: valid = False
        return valid

    def initialize(self):
        # Set parameter registers
        self.write([0x64], int(self.gas_wait, 2))
        self.write([0x71], int('000' + self.run_gas + self.gas_profile, 2))
        self.write([0x72], int('00000' + self.osrs_h, 2))
        self.write([0x75], int('000'+self.IIR_filter+'00', 2))
        self.write([0x74], int(self.osrs_t + self.osrs_p + '00', 2))

    def take_measurement(self):
        # Update gas heater temperature
        self.update_gas_heater()
        
        # Set control registers
        self.write([0x74], int(self.osrs_t + self.osrs_p + '01', 2)) # '01' starts measurement
        time.sleep (0.01)
        # Wait until measurement bit goes low
        while self.check_measurement() == True:
            time.sleep(0.01)
            
        # Read all data into a package
        data_package = self.read_data()
        
        p_msb = data_package[0]
        p_lsb = data_package[1]
        p_xlsb = int(format(data_package[2], '08b')[:4], 2) 
        t_msb = data_package[3]
        t_lsb = data_package[4]
        t_xlsb = int(format(data_package[5], '08b')[:4], 2) 
        h_msb = data_package[6]
        h_lsb = data_package[7]
        gas_r_msb = data_package[11]
        gas_lsb = data_package[12]
        #print(format(gas_lsb, '08b'))
        gas_r_lsb = int(format(gas_lsb, '08b')[:2], 2) # gas_r_lsb 0x2B <7:6>
        gas_range = int(format(gas_lsb, '08b')[4:], 2) # gas_range_r 0x2B <3:0>
        
        # Check if gas measurement is valid and heater was stable
        if self.check_gas_valid(gas_lsb) == True: gas_valid = 'Gas Meas Valid'
        else: gas_valid = 'Gas Meas Failed'
        if self.check_heat_stability(gas_lsb) == True: heater_valid = 'Heater Stable'
        else: heater_valid = 'Heater Unstable'

        # Read THPG values from registers
        temperature = round(self.calculate_temperature(t_msb, t_lsb, t_xlsb),2)
        humidity = round(self.calculate_humidity(h_msb, h_lsb),2)
        pressure = round(self.calculate_pressure(p_msb,p_lsb, p_xlsb),2)
        gas_res = round(self.calculate_gas(gas_r_msb, gas_r_lsb, gas_range),2)
        
        return [temperature, humidity, pressure, gas_res, gas_valid, heater_valid]
    
### Test via ncd usb-i2c connector ###
if __name__ == "__main__":
    sensor = bme680('ncd','COM6')
    line = sensor.read([0xD0],1)[0]
    print(hex(line)) # Sanity check, returns sensor id=0x61
    sensor.initialize()
    while True:
        sensor.set_gas_heater(150)
        results = sensor.take_measurement()
        print(results)
        time.sleep(1.0)
        sensor.set_gas_heater(350)
        results = sensor.take_measurement()
        print(results)
        time.sleep(1.0)
