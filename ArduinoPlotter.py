import time
from serial import Serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation



class AnimationPlot:
        
    def __init__(self, fig, ax):
        self.fig = fig
        self.ax = ax
        self.numPointsPlotted = 500
        self.maxY = 500
        self.minY = -500
        self.title = "Arduino Data"
        
    def animate(self, i, ser, pwrData, pwrFilteredData, rpmData, rpmFilteredData, targetRpmData):
        ser.write(b'g')                                 # Transmit the char 'g' to receive the Arduino data point
        arduinoData_string = None
        try:
            arduinoData_string = ser.readline().decode('ascii') # Decode receive Arduino data as a formatted string
            print(arduinoData_string)                                           # 'i' is a incrementing variable based upon frames = x argument
        except:
            pass
        # Initialize variables to store the extracted values
        pwr_value = None
        rpm_value = None
        if arduinoData_string is not None:
                
            # Split the string into individual parts based on ";"
            parts = arduinoData_string.split(";")


            # Iterate through the parts to find the data based on the string format
            for part in parts:
                if part is not None:
                    key_value_pair = part.split(":")
                    if len(key_value_pair) == 2:
                        key, value = key_value_pair
                        if key == "pwr":
                            try:
                                print(key + ":" + value)
                                pwrData.append(float(value))
                            except:
                                pass
                        if key == "pwr_filt":
                            try:
                                print(key + ":" + value)
                                pwrFilteredData.append(float(value))
                            except:
                                pass
                        elif key == "rpm":
                            try:
                                print(key + ":" + value)
                                rpmData.append(float(value))
                            except:
                                pass
                        elif key == "rpm_filt":
                            try:
                                print(key + ":" + value)
                                rpmFilteredData.append(float(value))
                            except:
                                pass
                        elif key == "target":
                            try:
                                print(key + ":" + value)
                                targetRpmData.append(float(value))
                            except:
                                pass

        pwrData = pwrData[-self.numPointsPlotted:]
        pwrFilteredData = pwrFilteredData[-self.numPointsPlotted:]
        rpmData = rpmData[-self.numPointsPlotted:]
        rpmFilteredData = rpmFilteredData[-self.numPointsPlotted:]
        targetRpmData = targetRpmData[-self.numPointsPlotted:]  
        
        self.ax[1].clear()
        self.ax[0].clear()    
        self.getPlotFormat()
        
        self.ax[1].plot(pwrData)
        if len(pwrFilteredData) > 0:            
            self.ax[1].plot(pwrFilteredData)
        self.ax[0].plot(rpmData)
        if len(rpmFilteredData) > 0:
            self.ax[0].plot(rpmFilteredData)
        self.ax[0].plot(targetRpmData)
        

    def getPlotFormat(self):
        self.ax[0].set_ylim([-50, 200])
        self.ax[0].set_title("RPM tracker")
        self.ax[0].set_ylabel("RPM")
        self.ax[1].set_ylim([-260, 260])
        self.ax[1].set_title("Motor PWR [-255,255]")
        self.ax[1].set_ylabel("Motor PWR")
        
def main():
    rpmData = []
    rpmFilteredData = []
    pwrData = []
    pwrFilteredData = []
    targetRpm = []
    
    fig = plt.figure()
    ax = fig.subplots(2)
    fig.suptitle("Motor Data")
    
    comPort = "COM5"
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
                                  fargs=(serialObj, pwrData, pwrFilteredData, rpmData, rpmFilteredData, targetRpm), 
                                  interval=10) 
    plt.tight_layout()
    plt.show()                                              # Keep Matplotlib plot persistent on screen until it is closed
    serialObj.close()
    
if __name__ == "__main__":
    main()
# Close Serial connection when plot is closed

