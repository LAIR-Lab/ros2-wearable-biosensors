# ROS1
#!/usr/bin/env python3

import sys, struct, serial, os  # pip3 install pyserial
# import rclpy
# from rclpy.node import Node
# from rclpy.parameter import Parameter

import rospy 
from std_msgs.msg import Float32, Float32MultiArray

import matplotlib.pyplot as plt
import numpy as np

import neurokit2 as nk
from collections import deque
import matplotlib.pyplot as plt
import matplotlib.animation as animation

################################
################################
#### THIS IS THE ONE TO USE ####
################################
################################

#### COMMANDS ####

# cd ~/prog/shimmer3/LogAndStream/python_scripts/Bluetooth\ commands/
# sudo rfcomm bind hc0 00:06:66:E2:6D:31 
# sudo chown cywong /dev/rfcomm0
# python3 getStatus.py /dev/rfcomm0


## above might not be necessary?
# cd ~/prog/ros2-foxy-wearable-biosensors/src/shimmer3_gsr_unit/shimmer3_gsr_unit
# python3 shimmer_test_gsr_ppg.py




# class ros_shimmer3(Node):
class ros_shimmer3():
    #############################################################
    # Main Loop
    #############################################################
    def __init__(self):
        # super().__init__('shimmer3_gsr_node')
        rospy.init_node('shimmer3_gsr_node', anonymous = True)
        # PPGRate_pub = rospy.Publisher('/physio/PPG_Rate', std_msgs.msg.Float32, queue_size=1)
        # GSR_pub = rospy.Publisher('/physio/GSR', std_msgs.msg.Float32, queue_size=1)
        # PPGRate_msg = std_msgs.msg.Float32()
        # GSR_msg = std_msgs.msg.Float32()


        self.rate = rospy.Rate(30)

        

        # For the generlized structure of ROS2 Node
        # self.declare_parameter('Sensor_Enable', True) # Enable to publish sensor data (true)
        # self.Parm_Sensor_Enable = self.get_parameter('Sensor_Enable').value 
        # self.declare_parameter('Chunk_Enable', True) # Enable to publish chunk data (true)
        # self.Parm_Chunk_Enable = self.get_parameter('Chunk_Enable').value 
        # self.declare_parameter('Chunk_Length', 128) # Define the length of the chunk data
        # self.Parm_Chunk_Length = self.get_parameter('Chunk_Length').value 
        
        self.Parm_Sensor_Enable = True
        self.Parm_Chunk_Enable = True
        self.Parm_Chunk_Length = 128

        # For the Veriner Respriation Belt H10 device.
        # Ensure the Bluetooth radio is available by running the '''$ hciconfig''' command.
        # Scan for the Shimmer by running the '''$ hcitool scan''' command.
        # self.declare_parameter('Device_Name', "00:06:66:E2:6D:31") # Should be changed inro your device name
        # self.Parm_Device_Name = self.get_parameter('Device_Name').value 

        self.Parm_Device_Name = "00:06:66:E2:6D:31"
                
        os.system("sudo chmod a+rw /dev/rfcomm0")
        os.system("sudo rfcomm bind 0 " + self.Parm_Device_Name)

        self.data_index = 0
        self.gsr_data_index = 0
        self.gsr_chunk_data = [] 
        self.ppg_data_index = 0
        self.ppg_chunk_data = []

        self.windowsize = 300 #at 35 Hz, 200 entries is about 5.7 sec of data
        self.ppg_window = deque()
        self.gsr_window = deque()
        self.dt_window = deque()
        self.t_record_window = deque()
        self.HR_hist = deque()
        self.dt_hist = deque()
        self.gsr_hist = deque()

        self.ser = serial.Serial("/dev/rfcomm0", 115200)
        self.ser.flushInput()
        print ("port opening, done.")

        # send the set sensors command
        self.ser.write(struct.pack('BBBB', 0x08 , 0x04, 0x01, 0x00))  #GSR and PPG
        self.wait_for_ack()   
        print ("sensor setting, done.")

        # Enable the internal expansion board power
        self.ser.write(struct.pack('BB', 0x5E, 0x01))
        self.wait_for_ack()
        print ("enable internal expansion board power, done.")

        # send the set sampling rate command
        # sampling_freq = 32768 / clock_wait = X Hz
        sampling_freq = 50
        clock_wait = (2 << 14) / sampling_freq
        self.ser.write(struct.pack('<BH', 0x05, int(clock_wait)))
        self.wait_for_ack()

        # send start streaming command
        self.ser.write(struct.pack('B', 0x07))
        self.wait_for_ack()
        print ("start command sending, done.")


        # read incoming data
        

        #############################################################
        # Publisher Parts
        #############################################################
        # if self.Parm_Sensor_Enable:
        #     self.pub_shimmer3_gsr_data = self.create_publisher(Float32, "biosensors/shimmer3_gsr/gsr", 10)
        #     self.pub_shimmer3_ppg_data = self.create_publisher(Float32, "biosensors/shimmer3_gsr/ppg", 10)

        #     if self.Parm_Chunk_Enable:
        #         self.pub_shimmer3_gsr_chunk_data = self.create_publisher(Float32MultiArray, "biosensors/shimmer3_gsr/gsr_chunk", 10) 
        #         self.pub_shimmer3_ppg_chunk_data = self.create_publisher(Float32MultiArray, "biosensors/shimmer3_gsr/ppg_chunk", 10)

        #print ("Packet Type\tTimestamp\tGSR\tPPG")
        
        # while rclpy.ok() and self.Parm_Sensor_Enable:
        # while self.Parm_Sensor_Enable:
        rospytime = rospy.get_rostime()
        self.t_prev = 0
        self.t = 0
        self.t_init = 12 + 6
        # self.t_runtime = 10
        self.t_start = rospytime.secs + rospytime.nsecs/1000000000

        self.plotflag = False
        self.flag_record = False

        self.flag_logtofile = True

        if self.flag_logtofile == True:
            f = open("test.txt", "w") 
            f.write("t, dt, GSR, HR, HRavg, PPG_raw, PPG_Peaks\n")

        # fig = plt.figure()
        # self.ax1 = fig.add_subplot(1,1,1)

        while not rospy.is_shutdown():
            rospytime = rospy.get_rostime()
            self.t = rospytime.secs + rospytime.nsecs/1000000000 - self.t_start
            self.dt = self.t - self.t_prev
            self.t_prev = self.t

            if self.t < self.t_init:
                (shimmer_raw_GSR, shimmer_raw_PPG) = self.reading_shimmer3_data()
                print("---------------")
                print(self.t, " // flag_record:", self.flag_record)
                print(self.dt, " // GSR:", shimmer_raw_GSR, " // PPG:", shimmer_raw_PPG)

            # elif self.t >= self.t_init and self.t < (self.t_init + self.t_runtime):
                # self.flag_record = True
                # (shimmer_raw_GSR, shimmer_raw_PPG) = self.reading_shimmer3_data()
                # print("---------------")
                # print(self.t, " // flag_record:", self.flag_record)
                # print(self.dt, " // GSR:", shimmer_raw_GSR, " // PPG:", shimmer_raw_PPG)
                
                if len(self.ppg_window) >= self.windowsize:
                    self.ppg_window.popleft()
                    self.gsr_window.popleft()
                    self.dt_window.popleft()
                    self.t_record_window.popleft()
                self.ppg_window.append(shimmer_raw_PPG)
                self.gsr_window.append(shimmer_raw_GSR)
                self.dt_window.append(self.dt)
                self.t_record_window.append(self.t - self.t_init)

                # samplingrate = sampling_freq # len(self.dt_window)/self.t_runtime
                # signals, info = nk.ppg_process(self.ppg_window, sampling_rate=samplingrate)
                # print(signals.PPG_Rate[-1])

            # elif self.t > (self.t_init + self.t_runtime):
            elif self.t >= self.t_init and self.t < self.t_init + 20:
                self.flag_record = True
                (shimmer_raw_GSR, shimmer_raw_PPG) = self.reading_shimmer3_data()
                print("---------------")
                print(self.t, " // flag_record:", self.flag_record)
                print(self.dt, " // GSR:", shimmer_raw_GSR, " // PPG:", shimmer_raw_PPG)
                
                if len(self.ppg_window) >= self.windowsize:
                    self.ppg_window.popleft()
                    self.gsr_window.popleft()
                    self.dt_window.popleft()
                    self.t_record_window.popleft()
                self.ppg_window.append(shimmer_raw_PPG)
                self.gsr_window.append(shimmer_raw_GSR)
                self.dt_window.append(self.dt)
                self.t_record_window.append(self.t - self.t_init)

                samplingrate = len(self.dt_window)/np.sum(self.dt_window)
                # print(self.dt_window)
                # print(np.sum(self.dt_window))
                # print("Sampling:", samplingrate)
                signals, info = nk.ppg_process(self.ppg_window, sampling_rate=samplingrate)
                # print(signals)
                PPG_rate_mean = np.mean(signals.PPG_Rate)
                print("Last:", signals.PPG_Rate[len(signals.PPG_Clean)-1])
                print("Mean:", PPG_rate_mean)
                # print("PPG_Peaks", signals.PPG_Peaks)

                if signals.PPG_Peaks[np.round(len(signals.PPG_Peaks)/2)] == 1:
                    self.HR_hist.append(signals.PPG_Rate[np.round(len(signals.PPG_Peaks)/2)])
                    # self.HR_hist.append(PPG_rate_mean)
                    self.dt_hist.append(self.t - self.t_init)
                    self.gsr_hist.append(shimmer_raw_GSR)

                if self.flag_logtofile == True:
                    # f.write("dt, GSR, HR, PPG_raw, PPG_Peaks\n")
                    # print(range(len(self.dt_hist)))
                    # for i in range(len(self.dt_hist)):
                    f.write(str(self.t - self.t_init)+", "+str(self.dt)+", "+str(shimmer_raw_GSR)+", "+str(signals.PPG_Rate[len(signals.PPG_Clean)-1])+", "+str(PPG_rate_mean)+", "+str(shimmer_raw_PPG)+", "+str(signals.PPG_Peaks[len(signals.PPG_Clean)-1])+"\n")
                    # print(str(i)+":   "+str(self.dt_hist[i])+","+str(self.gsr_hist[i])+","+str(self.HR_hist[i]))
                        # pause(0.001)
                    # f.close()

                # plt.scatter(self.t - self.t_init, PPG_rate_mean)
                # plt.pause(0.05)

                # self.plotx = self.dt_hist
                # self.ploty = self.HR_hist
                # ani = animation.FuncAnimation(fig, self.animate, interval=50)
                # plt.show()

            elif self.t >= self.t_init + 30:
                if self.plotflag == False:
                    # samplingrate = len(self.dt_window)/self.t_runtime
                    # print("samplingrate:", samplingrate)
                    # signals, info = nk.ppg_process(self.ppg_window, sampling_rate=samplingrate)
                    # np.set_printoptions(threshold=np.inf)
                    # # print(signals) # PPG_Raw   PPG_Clean   PPG_Rate  PPG_Peaks
                    plt.figure()
                    plt.subplot(3, 1, 1)
                    plt.plot(signals.PPG_Clean)
                    plt.subplot(3, 1, 2)
                    plt.plot(signals.PPG_Rate)
                    plt.subplot(3, 1, 3)
                    plt.plot(signals.PPG_Peaks)
                    # plt.show()
                    # # signals, info = nk.ppg_analyze(signals, sampling_rate=samplingrate, method='auto')
                    # # nk.ppg_plot(signals, info)

                    plt.figure()
                    plt.subplot(2, 1, 1)
                    plt.plot(self.dt_hist, self.HR_hist)
                    plt.subplot(2, 1, 2)
                    plt.plot(self.dt_hist, self.gsr_hist)

                    # plt.figure() #plotting PPG peaks
                    # plt.plot(signals.PPG_Peaks)
                    plt.show()

                    self.plotflag = True

                    if self.flag_logtofile == True:
                        f.close()

            # self.pub_shimmer3_gsr_data.publish(Float32(data = shimmer_raw_GSR))
            # self.pub_shimmer3_ppg_data.publish(Float32(data = Shimmer_raw_PPG))
            # if self.Parm_Chunk_Enable:
            #     self.gsr_chunk_data.append(shimmer_raw_GSR)
            #     self.ppg_chunk_data.append(Shimmer_raw_PPG)

            #     #print(self.bvp_data_index, stream_id.name, timestamp, sample) #For test
            #     if self.data_index  == self.Parm_Chunk_Length - 1:
            #         # publish chunk
            #         # self.pub_shimmer3_gsr_chunk_data.publish(Float32MultiArray(data = self.gsr_chunk_data))
            #         # self.pub_shimmer3_ppg_chunk_data.publish(Float32MultiArray(data = self.ppg_chunk_data))

            #         # print("---------------")
            #         # print("GSR:", self.gsr_chunk_data)
            #         # print("PPG:", self.ppg_chunk_data)
                            
            #         #Reset index and ref_array
            #         self.gsr_chunk_data = []
            #         self.ppg_chunk_data = []
            #         self.data_index = 0
            #     else:
            #         self.data_index += 1
            # else:
            #     pass

    # def animate(self, i):
    #     self.ax1.clear()
    #     self.ax1.plot(self.plotx, self.ploty)

    def reading_shimmer3_data(self):
        ddata = ""
        numbytes = 0
        framesize = 8 # 1byte packet type + 3byte timestamp + 2 byte GSR + 2 byte PPG(Int A13)

        while numbytes < framesize:
            ddata = self.ser.read(framesize)
            #ddata = ddata + test.decode('utf-8')
            numbytes = len(ddata)
        
        data = ddata[0:framesize]
        ddata = ddata[framesize:]
        numbytes = len(ddata)


        (packettype) = struct.unpack('B', data[0:1])
        #(timestamp0, timestamp1, timestamp2) = struct.unpack('BBB', data[1:4])

        # read packet payload
        (PPG_raw, GSR_raw) = struct.unpack('HH', data[4:framesize])

        # get current GSR range resistor value
        Range = ((GSR_raw >> 14) & 0xff)  # upper two bits
        if(Range == 0):
            Rf = 40.2   # kohm
        elif(Range == 1):
            Rf = 287.0  # kohm
        elif(Range == 2):
            Rf = 1000.0 # kohm
        elif(Range == 3):
            Rf = 3300.0 # kohm

        # convert GSR to kohm value
        gsr_to_volts = (GSR_raw & 0x3fff) * (3.0/4095.0)
        GSR_ohm = Rf/( (gsr_to_volts /0.5) - 1.0)

        # convert PPG to milliVolt value
        PPG_mv = PPG_raw * (3000.0/4095.0)
        #print(GSR_ohm, type(GSR_ohm), PPG_mv, type(PPG_mv))
        return (float(GSR_ohm), float(PPG_mv))

    def wait_for_ack(self):
        ddata = ""
        ack = struct.pack('B', 0xff)
        while ddata != ack:
            ddata = self.ser.read(1)
            #print ("0x%02x" % ord(ddata[0]))
            
        return
        

def main(args=None):
    # rclpy.init(args=args)

    ros_shimmer3_node = ros_shimmer3()

    try:
        pass
        # while rclpy.ok():
        #     pass
        #     rclpy.spin(ros_shimmer3_node)
    
    except KeyboardInterrupt:
        print('repeater stopped cleanly')

    except BaseException:
        print('exception in repeater:', file=sys.stderr)
        raise

    finally:        
        ros_shimmer3_node.destroy_node()
        # rclpy.shutdown() 
        rospy.signal_shutdown()

if __name__ == '__main__':
    main()
