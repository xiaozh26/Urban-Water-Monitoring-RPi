import RPi.GPIO as GPIO
import time
import serial
import crcmod
import struct
import datetime
import matplotlib.pyplot as plt
import random
import math
import sys
import smbus


'''
 The function "water_level" is aim to measure the water level by measuring the distance between the water surface and the top plate
 it use hc-sr04 ultrasonic transmitter to measure the distance between water level and the top bracket
 The minimum value of this distance is 20mm,the maximum value of this distance is 4000mm
 The return value of this function is the distance you measured (unit:cm)
 The power supply is connected to the 5v pin, the output pin is connected to the physical pin 38 (BCM20), and the input pin is connected to the physical pin 40 (BCM21)
 the measurement error is about 1 mm
 ''' 
def water_level(GPIO_TRIGGER,GPIO_ECHO,m):

    #Set the GPIO mode to BCM
    GPIO.setmode(m)
  
    #set up the roles for each pin
    GPIO.setup(GPIO_TRIGGER, GPIO.OUT)
    GPIO.setup(GPIO_ECHO, GPIO.IN)
    
    #Set up the high level on pin Tri (send out the ultrasonic)
    GPIO.output(GPIO_TRIGGER, True)
  
    # last for 10 microsecond, since the datasheet ask for a dely time >= 10 microsecond
    time.sleep(0.00001)
    #Turn to low level on pin Tri (stop sending the ultrasonic)
    GPIO.output(GPIO_TRIGGER, False)
    
    start = time.time()
    stop = time.time()
    # record the time when the ultrasonic was sent out
    while GPIO.input(GPIO_ECHO) == 0:
        start = time.time()
    
    # record the time when the ultrasonic was recieved
    while GPIO.input(GPIO_ECHO) == 1:
        stop = time.time()
  
    # calculate the travel time
    t = stop - start
    # The ultrasonic's velocity when travels in the air is about 343 m/s
    distance = (t * 34300) / 2
    #There are some errors in the measurement due to the distance between the transmitter and the the front end of the hardware and python's operation time. Then according to my real measurement, the error is about fixed, and its averge is about 3.3%, so we need to make up the error.
    distance = distance / 0.967
    return distance

'''
--END--
'''


'''
This function is used to measure the huminity, temperature and pressure
it contains 3 parts, one for initialize the address, one for getting data, one for calculate
 it use AHT20 and BMP280
 The temperature can be measured is between -40 to 85 C, the huminity is between 0% to 100%, the pressure is between 300 to 1100 hpa
 The return value of this function is the temperature, huminity and pressure (unit: Celsius, % and pa)
 The power supply is connected to the 3.3v pin, the SCL pin is connected to the physical pin 5, and the SDA pin is connected to the physical pin 3, the GND pin is connected to physical pin 9
 the measurement error is about 0.3 C, 2 % and 1 hpa
'''
class AHT10:
    
    CONFIG = [0x00, 0x00] #Used to configure the sensor.
    MEASURE = [0x00, 0x00] #For starting measurement commands
    
    def __init__(self, bus, addr=0x38): #"bus" indicates the I2C bus number, "addr" indicates the address of the sensor shows in the datasheet
        self.bus = smbus.SMBus(bus) #An object is created to communicate with the specified I2C bus
        self.addr = addr
        self.bus.write_i2c_block_data(self.addr, 0xE1, self.CONFIG)#Send configuration commands to the sensor to set the sampling mode and calibration. 0xE1 command
        time.sleep(0.2)#Delay 0.2 seconds according to the datasheet to ensure the sensor completes initialization.

    
    def THP_get(self):
        self.bus.write_i2c_block_data(self.addr, 0xAC, self.MEASURE) #Send measurement commands to the sensor to initiate temperature and humidity measurements 0xAC command
        time.sleep(0.5) #0.5 seconds delay according to datasheet to ensure measurement completion
        data = self.bus.read_i2c_block_data(self.addr, 0x00) #Read the temperature and humidity data returned by the sensor
        '''
        Extracts the temperature value from the "data". By bit manipulation and shift operations on the data bytes,
        the high and low bytes are combined into a 20-bit temperature value. According to the datasheet
        '''
        temp = ((data[3] & 0x0F) << 16) | (data[4] << 8) | data[5]
        ctemp = ((temp*200) / 1048576) - 50
        hum = ((data[1] << 16) | (data[2] << 8) | data[3]) >> 4
        chum = int(hum * 100 / 1048576) 
        return (ctemp, chum)
    
    
    def THP_cal(self):
        self.bus = smbus.SMBus(1) #Create an object to communicate with I2C bus 1

        #Reads 24 bytes of data from the device at address 0x77, which are the calibration coefficients for the sensor
        b1 = self.bus.read_i2c_block_data(0x77, 0x88, 24) 

        #Resolution calibration factor
        dig_T1 = b1[1] * 256 + b1[0]
        dig_T2 = b1[3] * 256 + b1[2]

        if dig_T2 > 32767 :
            dig_T2 -= 65536
        dig_T3 = b1[5] * 256 + b1[4]
        if dig_T3 > 32767 :
            dig_T3 -= 65536

        
        dig_P1 = b1[7] * 256 + b1[6]
        dig_P2 = b1[9] * 256 + b1[8]

        if dig_P2 > 32767 :
            dig_P2 -= 65536
        dig_P3 = b1[11] * 256 + b1[10]
        if dig_P3 > 32767 :
            dig_P3 -= 65536
        dig_P4 = b1[13] * 256 + b1[12]
        if dig_P4 > 32767 :
            dig_P4 -= 65536
        dig_P5 = b1[15] * 256 + b1[14]
        if dig_P5 > 32767 :
            dig_P5 -= 65536
        dig_P6 = b1[17] * 256 + b1[16]
        if dig_P6 > 32767 :
            dig_P6 -= 65536
        dig_P7 = b1[19] * 256 + b1[18]
        if dig_P7 > 32767 :
            dig_P7 -= 65536
        dig_P8 = b1[21] * 256 + b1[20]
        if dig_P8 > 32767 :
            dig_P8 -= 65536
        dig_P9 = b1[23] * 256 + b1[22]
        if dig_P9 > 32767 :
            dig_P9 -= 65536

        
        self.bus.write_byte_data(0x77, 0xF4, 0x27) #set up the pressure and Temperature Oversampling rate = 1, read once every time to save time
        
       
        #bus.write_byte_data(0x77, 0xF5, 0xA0) #set up the stand_by time = 1000 ms (I put the time delay in the main function)

        
        data = self.bus.read_i2c_block_data(0x77, 0xF7, 8) #Reads raw temperature and pressure data

        #Analyze raw temperature and pressure data( given in the datasheet)
        adc_p = ((data[0] * 65536) + (data[1] * 256) + (data[2] & 0xF0)) / 16 #Extract the raw temperature data from "data" and convert it to a 16-bit integer value "adc_t"
        adc_t = ((data[3] * 65536) + (data[4] * 256) + (data[5] & 0xF0)) / 16 #Extract the raw pressure data from "data" and convert it to the 16-bit integer value "adc_p"

        var1 = ((adc_t) / 16384.0 - (dig_T1) / 1024.0) * (dig_T2)
        var2 = (((adc_t) / 131072.0 - (dig_T1) / 8192.0) * ((adc_t)/131072.0 - (dig_T1)/8192.0)) * (dig_T3)
        t_fine = (var1 + var2)

        var1 = (t_fine / 2.0) - 64000.0
        var2 = var1 * var1 * (dig_P6) / 32768.0
        var2 = var2 + var1 * (dig_P5) * 2.0
        var2 = (var2 / 4.0) + ((dig_P4) * 65536.0)
        var1 = ((dig_P3) * var1 * var1 / 524288.0 + ( dig_P2) * var1) / 524288.0
        var1 = (1.0 + var1 / 32768.0) * (dig_P1)
        p = 1048576.0 - adc_p
        p = (p - (var2 / 4096.0)) * 6250.0 / var1
        var1 = (dig_P9) * p * p / 2147483648.0
        var2 = p * (dig_P8) / 32768.0
        pressure = (p + (var1 + var2 + (dig_P7)) / 16.0) / 100
        return pressure
    
'''
--END--
'''
    
    
'''
The following functions is to measure the speed of the wind
This use ZTS-3000-FSJT-N01 to attain the goal
This use RS485 Serial port connection
The power supply should between 10V to 30V
Operating temperature: -20℃ ~ 70℃
According to the datasheet: standard Modbus-RTU protocol, baud rate: 4800; parity bits: none; data bits: 8; stop bits: 1
'''

# Calculating CRC-16 Checksum
def crc16(veritydata):
    if not veritydata:
        return
    crc16 = crcmod.mkCrcFun(0x18005, rev=True, initCrc=0xFFFF, xorOut=0x0000)
    #0x18005 means the command for verification of crc16-IBM
    #the rev=true means the inversion of data bit sequence according to modbus-RTU
    #initCrc=0xFFFF means to set the initial value of calibration to 1 for better detection capability (default is 0)
    #Last is defult means no more extra operation after rhe verification
    return crc16(veritydata)


# verify wether the CRC16 code is correct
def checkcrc(data):
    if not data:#If data is 0, then there is no checksum and the data is transmitted incorrectly
        return False 
    if len(data) <= 2:#Check the length of the data to be taught, as the length of the data to be calibrated is at least 3 bytes
        return False
    nocrcdata = data[:-2]#Get the part of data excluding the last two bytes, which is the original data without the checksum
    oldcrc16 = data[-2:]#Get the last two bytes of data, i.e. the check digit part
    oldcrclist = list(oldcrc16)#Convert "oldcrc16" to a list, and store the two bytes of the calibration code in the list, in order to facilitate byte-by-byte verification of the calibration code
    crcres = crc16(nocrcdata)#Calculate the CRC checksum of the data part
    crc16byts = crcres.to_bytes(2, byteorder="little", signed=False)#Converts CRC checksums to byte representations in order to be able to easily handle and pass checksum values during data transfer or storage.
    crclist = list(crc16byts)
    if oldcrclist[0] != crclist[0] or oldcrclist[1] != crclist[1]:#By comparing the corresponding elements in oldcrclist and crclist, we determine whether the checksum of the original data and the calculated checksum are equal.
        return False
    return True


#Read command frames saved or entered into registers from host to slave according to 03 or 04 in Modbus-RTU protocol,ensure that only these two basic read operations can be performed
#add means the address of the slave machine, the startregadd means the address start with in order to read, the regnum is the numbers of the registors to read, and thee funcode is defult 3
def sendingmod(add, startregadd, regnum, funcode=3):
    if add < 0 or add > 0xFF or startregadd < 0 or startregadd > 0xFFFF or regnum < 1 or regnum > 0x7D:#Used to check if the parameters passed to the function meet specific requirements
        print("Error: parameter error")
        return
    if funcode != 3 and funcode != 4:
        print("Error: parameter error")
        return
    #Synthetic data frames
    sendbytes = add.to_bytes(1, byteorder="big", signed=False)
    
    sendbytes = sendbytes + funcode.to_bytes(1, byteorder="big", signed=False) + startregadd.to_bytes(2, byteorder="big", signed=False) + \
                regnum.to_bytes(2, byteorder="big", signed=False)
    crcres = crc16(sendbytes)
    crc16bytes = crcres.to_bytes(2, byteorder="little", signed=False)
    sendbytes = sendbytes + crc16bytes
    return sendbytes

'''
--END--
'''


'''
This fuction is used to canculate the evaporation rate according to Penman-Monteith formula used by the Food and Agriculture Organization of the United Nations (FAO)
'''
def evaporation(temperature, humidity, pressure, wind_speed):
    # Calculate the saturated water vapor pressure, where temperature is the temperature in Celsius. The formula for saturated water vapor pressure is used here.
    saturation_vapor_pressure = 0.611 * math.exp((17.27 * temperature) / (temperature + 237.3))
    
    # Calculate the actual water vapor pressure, where humidity is in %.
    actual_vapor_pressure = saturation_vapor_pressure * humidity / 100
    
    # Calculate the slope factor delta
    delta = 4098 * saturation_vapor_pressure / ((temperature + 237.3) ** 2)
    
    # Calculate the atmospheric pressure factor gamma, where pressure is the air pressure in kilopascals (kPa)
    gamma = 0.665e-3 * pressure
    
    return (0.408 * delta * (saturation_vapor_pressure - actual_vapor_pressure) + gamma * (900 / (temperature + 273)) * wind_speed * (saturation_vapor_pressure - actual_vapor_pressure)) / (delta + gamma)

'''
--END--
'''


if __name__ == '__main__':
    try:
        plt.rcParams['font.sans-serif']=['SimHei'] #to show the title in Chinese
        plt.rcParams['axes.unicode_minus'] = False 
        plt.rcParams['toolbar'] = 'None'

        # Create a graphics window
        plt.ion()
        fig = plt.figure(num="水系统检测系统--数据监测",figsize=(30, 10))

        # create 6 subplots
        ax1 = fig.add_subplot(251) #means divide the whole window to 2 parts on y-axis, and 5 means divide the whole windows to 5 parts on x-axis
        ax2 = fig.add_subplot(252)
        ax3 = fig.add_subplot(253)
        ax4 = fig.add_subplot(254)
        ax5 = fig.add_subplot(111)
        ax6 = fig.add_subplot(255)

        # Set the range of the x and y axes
        ax1.set_xlim(0, 10)
        ax1.set_ylim(-20, 50)


        ax2.set_xlim(0, 10)
        ax2.set_ylim(0, 100)


        ax3.set_xlim(0, 10)
        ax3.set_ylim(0, 30)


        ax4.set_xlim(0, 10)
        ax4.set_ylim(50, 150)

        ax6.set_xlim(0, 10)
        ax6.set_ylim(0, 200)

        ax5.set_axis_off()


        # Create 7 empty x and y lists
        x1 = []
        y1 = []
        x2 = []
        y2 = []
        x3 = []
        y3 = []
        x4 = []
        y4 = []
        x5 = []
        y5 = []
        y6 = []
        y7 = []

        # create an empty form(2 dimensional list)
        table_data = [[0, 0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0, 0]]
        table_col_labels = ['temperature(°C)', 'humidity(%RH)', 'wind speed(m/s)', 'pressure(kPa)', 'water level(cm)','water level status', 'evaporation rate (mm/day)']
        table_row_labels = ['value', 'time']
        table_col_widths = [0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3]
        table_row_heights = [0.2, 0.2]

        table = ax5.table(cellText=table_data, colLabels=table_col_labels, rowLabels=table_row_labels, loc='lower center', cellLoc='center')
        # Adjust the font size of table cells
        table.auto_set_font_size(False)
        table.set_fontsize(10)
        while True:
            x = 0
            # Get the current hour and minute ->
            now = datetime.datetime.now()
            hour = now.hour
            minute = now.minute
            second = now.second
            now = str(hour)+':'+str(minute)+':'+str(second)
            #<- Get the current hour and minute
            
            #Get waterlevel ->
            dist = water_level(20,21,GPIO.BCM)
            #<- Get waterlevel
            
            #Get the temperature, humidity and pressure ->
            m = AHT10(1)
            data = m.THP_get()
            pressure = m.THP_cal()
            tem = "{:.3}".format(data[0])
            pre = round(pressure/10,2)
             hum = float(data[1])
            #<- Get the temperature, humidity and pressure
            
            #Get wind speed ->
            slaveadd = 1 #Modbus slave address
            startreg = 0 # The address of the start register
            regnums = 2 # The number of the register
            send_data = sendingmod(slaveadd, startreg, regnums)
            com = serial.Serial("/dev/ttyUSB0", 4800, timeout=0.8) # Serial port number, baud rate and timeout response time
            #<- Get wind speed
            
            
            '''
            print("Measured Distance = {:.1f} cm".format(dist))
            print("Temperature is",tem,"°C")
            print("Humidity is {} %".format(data[1]))
            print("Pressure:",pre,"pa")
            if len(recv_data) > 0:
                windspeedhex = str(recv_data.hex())[8:10] #Split the str from 8 to 10 and 12 to 14 to get the required value.
                windlevelhex = str(recv_data.hex())[12:14]
                windspeed=int(windspeedhex,16)/10 #Convert from hexadecimal to decimal
                windlevel=int(int(windlevelhex,16))
                print("Wind Speed: ",windspeed,"m/s")
                print("Wind Scale: ",windlevel)
            '''
            
            y_val1 = tem
            y_val2 = data[1]
            for x in range 6:
                if checkcrc(recv_data) == True:
                    break
                else
                    com.write(send_data)
                    recv_data = com.read(regnums*2+5)
                    x += 1
                if x == 5:
                    print("Unable to get correct data, exit the program")
                    sys.exit()
                if x >= 3:
                    print("Unable to get correct data, please check the wind speed sensor")
            windspeedhex = str(recv_data.hex())[8:10] 
            windspeed=int(windspeedhex,16)/10
            y_val3 = windspeed
            y_val4 = pre
            y_val5 = round(dist,1)
            y_val6 = round(dist,1)
            y_val7 = round(evaporation(float(tem), int(hum), int(pre), int(windspeed)),2)
            
            # Add the y values to the respective y lists
            y1.append(y_val1)
            y2.append(y_val2)
            y3.append(y_val3)
            y4.append(y_val4)
            y5.append(y_val5)
            if 20 - y_val6 >= 3:
                y6.append('Normal')
                
            else:
                 y6.append('Warning!!!')
                 
            y7.append(y_val7)
            
            # Add x values to their respective x lists
            x1.append(now)
            x2.append(now)
            x3.append(now)
            x4.append(now)
            x5.append(now)

            
            # Clear the original content in the subgraph (prevent overlap)
            ax1.clear()
            ax2.clear()
            ax3.clear()
            ax4.clear()
            ax6.clear()
            
            # Draw a line chart
            ax1.plot(x1[-100:], y1[-100:])
            ax2.plot(x2[-100:], y2[-100:])
            ax3.plot(x3[-100:], y3[-100:])
            ax4.plot(x4[-100:], y4[-100:])
            ax6.plot(x5[-100:], y5[-100:])
            

            # Set the title of each subgraph and the title of the x-y axis
            ax1.set_xlabel('Time')
            ax1.tick_params(axis='x', rotation = -45)
            ax1.set_ylabel('temperature(°C)')
            ax1.set_title('temperature')
            ax2.set_xlabel('Time')
            ax2.set_ylabel('humidity(%)')
            ax2.set_title('humidity')
            ax3.set_xlabel('Time')
            ax3.set_ylabel('speed(m/s)')
            ax3.set_title('wind speed')
            ax4.set_xlabel('Time')
            ax4.set_ylabel('pressure(kPa)')
            ax4.set_title('pressure')
            ax6.set_ylabel('distance(cm)')
            ax6.set_title('water level')
            ax5.set_axis_off()
            # Update the data in the table
            
            table_data[0] = [y1[-1], y2[-1], y3[-1], y4[-1], y5[-1], y6[-1], y7[-1]]
            table_data[1] = [now, now, now, now, now, now, now]
            table._cells[(0, 0)]._text.set_text(str(table_data[0][0]))
            table._cells[(0, 1)]._text.set_text(str(table_data[0][1]))
            table._cells[(0, 2)]._text.set_text(str(table_data[0][2]))
            table._cells[(0, 3)]._text.set_text(str(table_data[0][3]))
            table._cells[(0, 4)]._text.set_text(str(table_data[0][4]))
            table._cells[(1, 0)]._text.set_text(str(table_data[1][0]))
            table._cells[(1, 1)]._text.set_text(str(table_data[1][1]))
            table._cells[(1, 2)]._text.set_text(str(table_data[1][2]))
            table._cells[(1, 3)]._text.set_text(str(table_data[1][3]))
            table._cells[(1, 4)]._text.set_text(str(table_data[1][4]))
            table = ax5.table(cellText=table_data, colLabels=table_col_labels, rowLabels=table_row_labels, loc='lower center', cellLoc='center')

            
            
            # refresh
            plt.draw()
            plt.pause(0.1)
            plt.cla() #delete the axises to avoid the memory be used too much


            com.close()
            time.sleep(1)
  
        # Reset by pressing CTRL + C
    except KeyboardInterrupt:
        print("\nMeasurement stopped by User")
        GPIO.cleanup()

