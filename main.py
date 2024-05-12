from machine import Pin, Signal, ADC, Timer, I2C, PWM
from ssd1306 import SSD1306_I2C
from fifo import Fifo
import time
import array

# ADC-converter
adc = ADC(26)

# OLED Setup
i2c = I2C(1, scl=Pin(15), sda=Pin(14))
oled = SSD1306_I2C(128, 64, i2c)

# LEDs
led_onboard = Signal(Pin("LED", Pin.OUT), invert=True)
led21 = PWM(Pin(21))
led21.freq(1000)

# Variables
history = []
bpm, last_beat_time, last_oled_update, timestamp = 0, 0, 0, 0
bpm_detection_enabled = False
freq, max_history = 250, 250
last_time = 0
rot_selection = 0
mode = "mainmenu"
menuitems = ["Heart rate", "HRV analysis"]


# Switch + Rotary Encoder

class Switch:
    def __init__(self, rot_a, rot_b, rot_sw):
        self.a = Pin(rot_a, mode = Pin.IN, pull = Pin.PULL_UP)
        self.b = Pin(rot_b, mode = Pin.IN, pull = Pin.PULL_UP)
        self.switch = Pin(rot_sw, Pin.IN, Pin.PULL_UP)
        self.state = False
        self.switch.irq(handler = self.handler, trigger = Pin.IRQ_RISING, hard = True)
        self.fifo = Fifo(30, typecode = 'i')
        self.a.irq(handler = self.dim, trigger = Pin.IRQ_RISING, hard = True)
        
    # Interrupt handler for encoder
    
    def handler(self, pin): 
        
        global last_time, mode, timestamp
        
        new_time = time.ticks_ms()
        
        if (new_time - last_time) > 200: 

            last_time = new_time
    
            if mode != "measure_hrv": # rotary switch is deactivated HRV analysis
                
                if mode == "mainmenu"  and rot_selection == 0:
                    mode = "pressto"
                    
                elif mode == "mainmenu"  and rot_selection == 1:
                    mode = "measure_hrv"
                    timestamp = time.ticks_ms()

                elif mode == "pressto":
                    mode = "measure_bpm"
                    
                elif mode == "measure_hrv":
                    mode = "measured_hrv"
                    
                elif mode == "measure_bpm":
                    mode = "measured_bpm"
                    
                elif mode == "measured_bpm" or mode == "measured_hrv":
                    mode = "mainmenu"
                    
                print("Interrupt occurred on GPIO pin", pin, pin.value(), self.state, mode)
        
    # Interrupt handler for rotary switch
    
    def dim(self, pin): 
        
        if mode != "measure_bpm" or mode != "measure_hrv": #rotary switch is deactivated during measurements
            
            if self.b():
                self.fifo.put(-1)
                        
            else:
                self.fifo.put(1)
            
rot = Switch(10, 11, 12)

# Main menu

def menu_function():
    
    global menuitems, rot_selection
    
    if rot.fifo.has_data():
            
            rot_selection += rot.fifo.get()
            
            if rot_selection <= 0:
                rot_selection = 0
            
            elif rot_selection >= 1:
                rot_selection = 1
                    
    
    height = 0
    oled.fill(0)
    oled.text(" " + str("MAIN MENU") + " ", 1, height, 1)
    height += 20
    
    for item in menuitems:
        if rot_selection == menuitems.index(item): # lisää nuolen menun aktiiviselle riville
            oled.text("> " + str(item), 1, height, 1)
        else:
            oled.text("  " + str(item), 1, height, 1)
        height += 15
    oled.show()
    
    return

# Start function

def start():
    oled.fill(0)
    oled.text("Press to ", 4, 7, 1)
    oled.text("the measurement", 4, 17, 1)
    oled.show()
    return


# CALCULATIONS

# Mean IBI (inter-beat-interval)
def ibi_formula(intervals):
    ibi_sum = 0 
    for i in intervals:
        ibi_sum += i
    ibi = int(round(ibi_sum/len(intervals), 0))
    return ibi

# Mean Heart rate
def hr_formula(mean_ibi):
    heartrate = int(round(60*1000/mean_ibi, 0))
    return heartrate

# SDNN (the standard deviation of intervals)
def sdnn_formula(intervals, ibi):
    sum = 0
    for i in intervals:
        sum += (i-ibi)**2
    sdnn = (sum/(len(intervals)-1))**(1/2)
    rounded_sdnn = int(round(sdnn, 0))
    return int(rounded_sdnn)

# RMSSD (Root mean square of successive differences)
def rmssd_formula(intervals):
    i, sum = 0, 0
    while i < len(intervals)-1:
        sum += (intervals[i+1]-intervals[i])**2
        i +=1
    rounded_rmssd = int(round((sum/(len(intervals)-1))**(1/2), 0))
    return rounded_rmssd

# SDSD (Standard deviation of successive interval differences)
def sdsd_formula(intervals):
    ibi_array = array.array('l')
    n = 0
    first_value = 0
    second_value = 0
    while n < len(intervals)-1:
        ibi_array.append(int(intervals[n+1])-int(intervals[n]))
        n += 1
    n = 0
    while n < len(ibi_array)-1:
        first_value += float(ibi_array[n]**2)
        second_value += float(ibi_array[n])
        n += 1
    first = first_value/(len(ibi_array)-1)
    second = (second_value/(len(ibi_array)))**2
    rounded_sdsd = int(round((first - second)**(1/2), 0))
    return rounded_sdsd

# SD1 + SD2 (Poincaré plot shape parameters)
def sd1_formula(sdsd):
    rounded_sd1 = int(round(((sdsd**2)/2)**(1/2), 0))
    return rounded_sd1

def sd2_formula(SDNN, SDSD):
    rounded_sd2 = int(round(((2*(SDNN**2))-((SDSD**2)/2))**(1/2), 0))
    return rounded_sd2

# Detecting BPM

def detect_bpm():
    #import time

    global history, last_beat_time, bpm, last_oled_update, bpm_detection_enabled

    # Initialize or reset necessary variables
    history = []
    last_beat_time = 0
    bpm = 0
    last_oled_update = 0
    bpm_detection_enabled = True

    # Main loop to check the heartbeats
    while bpm_detection_enabled:
        
        print(time.ticks_ms(), last_beat_time + 4)
        
        # BPM MEASUREMENT KOKEILU
        if time.ticks_ms() > last_beat_time + 4:
            v = adc.read_u16()
            history.append(v)
        
        # OG BPM MEASUREMENT
        '''
        v = adc.read_u16()
        history.append(v)
        '''
        
        if len(history) > max_history:
            history.pop(0)

        if len(history) > 1:
            minima, maxima = min(history), max(history)
            threshold_on = (minima + 3 * maxima) // 4
            threshold_off = (minima + maxima) // 2

            current_time = time.ticks_ms()
            if v > threshold_on and current_time - last_beat_time > 300:
                led_onboard.on()
                led21.duty_u16(65535)
                if last_beat_time:
                    time_between_beats = current_time - last_beat_time
                    bpm = 60000 / time_between_beats
                    if time.ticks_diff(current_time, last_oled_update) > 5000:  # Every 5 seconds
                        oled.fill(0)
                        oled.text("BPM: {:.2f}".format(bpm), 0, 0)
                        oled.show()
                        last_oled_update = current_time
                last_beat_time = current_time

            if v < threshold_off:
                led_onboard.off()
                led21.duty_u16(0)

        time.sleep(0.01)  # Sleep for a short period to simulate a timer callback frequency of about 100 Hz

        # Check for mode change to exit
        if mode == "measured_bpm":
            bpm_results(bpm)
            break

# detecting HRV analysis

def detect_hrv():
    
    global history, last_beat_time, bpm, timestamp, mode
    
    intervals = []
    last_oled_update = 0
    last_value_update = 0
    
    while True:
            
        if mode == "measure_hrv":
            
            # BPM MEASUREMENT KOKEILU
            if time.ticks_ms() > last_beat_time + 4:
                v = adc.read_u16()
                history.append(v)
            
            # OG BPM MEASUREMENT
            '''
            v = adc.read_u16()
            history.append(v)
            '''
            if len(history) > max_history:
                history.pop(0)

            minima, maxima = min(history), max(history)
            threshold_on = (minima + 3 * maxima) // 4
            threshold_off = (minima + maxima) // 2

            # Check thresholds to turn LED on/off and calculate BPM
            current_time = time.ticks_ms()

            if v > threshold_on and current_time - last_beat_time > 300:
                led_onboard.on()
                led21.duty_u16(65535)
                
                if last_beat_time:
                    time_between_beats = current_time - last_beat_time
                    intervals.append(time_between_beats)
                    bpm = 60000 / time_between_beats
                    mean_ibi = ibi_formula(intervals)
                    countdown = int((timestamp + 30000 - current_time) / 1000)
                    
                    if time.ticks_diff(current_time, last_value_update) > 5000:  # Every 5 seconds
                        show_bpm = bpm
                        show_ibi = mean_ibi
                        last_value_update = current_time
                        
                    # HRV measurement on 
                    if (timestamp + 30000 > current_time):
                    
                        if time.ticks_diff(current_time, last_oled_update) > 1000:  # Every 1 second
                            oled.fill(0)
                            oled.text("Measuring HRV", 0, 0)
                            oled.text("BPM: {:.2f}".format(show_bpm), 0, 10)
                            oled.text("MeanPPI: {:.2f}".format(show_ibi), 0, 20)
                            oled.text(str(countdown) + "s remaining", 0, 50)
                            oled.show()
                            
                    # HRV measurement ready 
                    elif (timestamp + 30000 < current_time):
                        oled.fill(0)
                        oled.text("Measurement", 0, 0)
                        oled.text("ready", 0, 10)
                        oled.text("Calculating", 0, 20)
                        oled.text("results", 0, 30)
                        oled.show()
                        time.sleep(3)
                        mode = "measured_hrv"
                last_beat_time = current_time

            if v < threshold_off:
                led_onboard.off()
                led21.duty_u16(0)
        
        elif mode == "measured_hrv":
            hrv_results(intervals)
            break

# shows BPM results on OLED screen

def bpm_results(beats):
             
    oled.fill(0)
    oled.text("Measured", 4, 7, 1)
    oled.text("result", 4, 17, 1)
    oled.text("BPM: " + str(beats), 4, 27, 1)
    oled.text("Press to", 4, 41, 1)
    oled.text("main menu", 4, 51, 1)
    oled.show()
    
    return

# shows results on OLED screen

def hrv_results(intervals):
    
    mean_ibi = ibi_formula(intervals)
    mean_hr = hr_formula(mean_ibi)
    sdnn = sdnn_formula(intervals, mean_ibi)
    rmssd = rmssd_formula(intervals)
    sdsd = sdsd_formula(intervals)
    sd1 = sd1_formula(sdsd)
    sd2 = sd2_formula(sdnn, sdsd)
        
    oled.fill(0)
    oled.text('HRV analysis', 0, 00, 1)
    oled.text('> MeanPPI:'+ str(int(mean_ibi)) +'ms', 0, 13, 1)
    oled.text('> MeanHR:'+ str(int(mean_hr)) +'bpm', 0, 23, 1)
    oled.text('> SDNN:'+str(int(sdnn)) +'ms', 0, 33, 1)
    oled.text('> RMSSD:'+str(int(rmssd)) +'ms', 0, 43, 1)
    oled.text('> SD1:'+str(int(sd1))+' > SD2:'+str(int(sd2)), 0, 53, 1)
    oled.show()
    return

# main loop


while True:

    try:
        
        _ticks = time.ticks_ms()
            
        if mode == "measure_bpm":
            detect_bpm()
                    
        elif mode == "measure_hrv":
            detect_hrv()
            
        elif mode == "pressto":
            start()
              
        elif mode == "mainmenu":
            menu_function()
            

    except Exception as e:
        print(e)
 

