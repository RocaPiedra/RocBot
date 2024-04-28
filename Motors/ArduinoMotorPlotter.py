import time
from serial import Serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import math

class MotorData:

    def __init__(self, motorID):
        self.motorID = motorID;
        self.current_pwr = []
        self.current_rpm = []
        self.filtered_pwr = []
        self.filtered_rpm = []
        self.target_rpm = []
        self.error = []

    def checkMotor(self, str:str):
        # the first value in the block (after the &) must match with the defined motorID
        if str.startswith(self.motorID):
            return True
        else:
            return False

    def update(self, serial_data, num_points_plotted):
        target_rpm = None
        measured_rpm = None
        parts = serial_data.split(";")
            # Iterate through the parts to find the data based on the string format
        for part in parts:
            if part is not None:
                key_value_pair = part.split(":")
                if len(key_value_pair) == 2:
                    key, value = key_value_pair
                    if key == "pwr":
                        try:
                            print(key + ":" + value)
                            self.current_pwr.append(float(value))
                        except:
                            pass
                    if key == "pwr_filt":
                        try:
                            print(key + ":" + value)
                            self.filtered_pwr.append(float(value))
                        except:
                            pass
                    elif key == "rpm":
                        try:
                            print(key + ":" + value)
                            self.current_rpm.append(float(value))
                            measured_rpm = float(value)
                        except:
                            pass
                    elif key == "rpm_filt":
                        try:
                            print(key + ":" + value)
                            self.filtered_rpm.append(float(value))
                        except:
                            pass
                    elif key == "target":
                        try:
                            print(key + ":" + value)
                            self.target_rpm.append(float(value))
                            target_rpm = float(value)
                        except:
                            pass

        if target_rpm is not None and measured_rpm is not None:
            self.error.append(float(abs(target_rpm - measured_rpm)))
        
        self.current_pwr = self.current_pwr[-num_points_plotted:]
        self.filtered_pwr = self.filtered_pwr[-num_points_plotted:]
        self.current_rpm = self.current_rpm[-num_points_plotted:]
        self.filtered_rpm = self.filtered_rpm[-num_points_plotted:]
        self.target_rpm = self.target_rpm[-num_points_plotted:]


class AnimationPlot:
        
    def __init__(self, fig, ax):
        self.fig = fig
        self.ax = ax
        self.numPointsPlotted = 500
        self.maxY = 500
        self.minY = -500
        self.title = "Arduino Data"
        
    def animate(self, i, serial:Serial, motors: list[MotorData]):
        serial.write(b'g')                                 # Transmit the char 'g' to receive the Arduino data point
        arduinoData_string = None
        try:
            arduinoData_string = serial.readline().decode('ascii') # Decode receive Arduino data as a formatted string
            print(arduinoData_string)                                           # 'i' is a incrementing variable based upon frames = x argument
        except:
            pass
        # Initialize variables to store the extracted values
        if arduinoData_string is not None:
                
            # Split the string into blocks of data per motor ";"
            blocks = arduinoData_string.split("&")

            # Iterate through the parts to find the data based on the string format
            for block in blocks:
                if block is not None:
                    for motor in motors:
                        if motor.checkMotor(block):
                            motor.update(block, self.numPointsPlotted)        
        
        self.ax[0].clear() 
        self.ax[1].clear()
        self.ax[2].clear()   
        self.getPlotFormat()
        
        for motor in motors:
            if len(motor.current_rpm) > 0:
                self.ax[0].plot(motor.current_rpm)
            if len(motor.filtered_rpm) > 0:
                self.ax[0].plot(motor.filtered_rpm)
            if len(motor.target_rpm) > 0:
                self.ax[0].plot(motor.target_rpm)
            if len(motor.current_pwr) > 0:
                self.ax[1].plot(motor.current_pwr)
            if len(motor.filtered_pwr) > 0:
                self.ax[1].plot(motor.filtered_pwr)
            if len(motor.error) > 0:
                self.ax[2].plot(motor.error)
        

    def getPlotFormat(self):
        self.ax[0].set_ylim([-300, 300])
        self.ax[0].set_title("RPM tracker")
        self.ax[0].set_ylabel("RPM")
        self.ax[1].set_ylim([-300, 300])
        self.ax[1].set_title("Motor PWR [-255,255]")
        self.ax[1].set_ylabel("Motor PWR")
        # self.ax[1].set_ylim([-300, 300])
        self.ax[2].set_title("RPM Error")
        self.ax[2].set_ylabel("Error [RPM]")
        
def main():
    motorFL = MotorData("FL")
    motorFR = MotorData("FR")
    motores = [motorFL, motorFR]
    fig = plt.figure()
    ax = fig.subplots(3)
    fig.suptitle("Motor Data")
    
    comPort = "/dev/ttyACM0"#"COM9"
    baudRate = 115200
    realTimePlot = AnimationPlot(fig, ax)
    
    # Block that allows us to launch even before the serial is ready
    serial_connected = False
    print(f"Starting serial connection: Port {comPort} - Baud Rate {baudRate}")
    while not serial_connected:
        
        try:        
            serialObj = Serial(comPort, baudRate)                       # Establish Serial object with COM port and BAUD rate to match Arduino Port/rate
            serial_connected = True             
            print(f"Serial connection achieved: Port {comPort} - Baud Rate {baudRate}")
        except:
            print(f"Failed attempted serial connection: Port {comPort} - Baud Rate {baudRate}")
            time.sleep(5)
            # Note that 'fargs' parameter is where we pass in our dataList and Serial object. 
    ani = animation.FuncAnimation(fig, realTimePlot.animate, 
                                  frames=10, 
                                  fargs=(serialObj, motores), 
                                  interval=10) 
    plt.tight_layout()
    plt.show()                                              # Keep Matplotlib plot persistent on screen until it is closed
    serialObj.close()
    
if __name__ == "__main__":
    main()
# Close Serial connection when plot is closed

