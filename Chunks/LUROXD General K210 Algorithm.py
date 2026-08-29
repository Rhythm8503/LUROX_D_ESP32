from Maix import MIC_ARRAY as mic
import sensor,lcd,image, time
import math
from machine import UART
import gc, sys
from fpioa_manager import fm

#Set processor from 500MHz to 600MHz

#Anchors for Image
input_size = (224, 224)
labels = ['drill', 'wrench', 'hammer', 'cup', 'screwdriver', 'pliers', 'shovel', 'person', 'table', 'chair', 'can', 'phone', 'scissor', 'wallet', 'trowel', 'saw']
anchors = [0.78, 0.97, 1.31, 2.53, 2.47, 3.81, 5.88, 5.03, 3.31, 1.75]

#Customizable configuration IO
mic.init(i2s_d0=23, i2s_d1=22, i2s_d2=21, i2s_d3=20, i2s_ws=19, i2s_sclk=18, sk9822_dat=24, sk9822_clk=25)

#i2s and i2C Initalization for Microphones and LEDs
# fm.register(23,fm.fpioa.I2S0_IN_D0)
# fm.register(22,fm.fpioa.I2S0_IN_D1)
# fm.register(21,fm.fpioa.I2S0_IN_D2)
# fm.register(20,fm.fpioa.I2S0_IN_D3)
# fm.register(19,fm.fpioa.I2S0_WS)
# fm.register(18,fm.fpioa.I2S0_SCLK)

# sample_rate = 44*1000 #Sample rate of 44KHz
# rx = I2S(I2S.DEVICE_0)
# rx.channel_config(rx.CHANNEL_0, rx.RECEIVER, align_mode = I2S.STANDARD_MODE)
# rx.set_sample_rate(sample_rate)

#UART Initalization
class Comm:
    def __init__(self, uart):
        self.uart = uart

    def send_detect_result(self, objects, labels):
        msg = ""
        for obj in objects:
            pos = obj.rect()
            p = obj.value()
            idx = obj.classid()
            label = labels[idx]
            msg += "{}:{}:{}:{}:{}:{:.2f}:{}, ".format(pos[0], pos[1], pos[2], pos[3], idx, p, label)
        if msg:
            msg = msg[:-2] + "\n"
        self.uart.write(msg.encode())

def init_uart():
    fm.register(15, fm.fpioa.UART1_TX, force=True)
    fm.register(10, fm.fpioa.UART1_RX, force=True)

    uart = UART(UART.UART1, 115200, 8, 0, 0, timeout=1000, read_buf_len=256)
    return uart

#Speech Recognizer (Isolated Word Setup)
#from speech_recognizer import isolated_word

# default: maix dock / maix duino set shift=0
#sr = isolated_word(dmac=2, i2s=I2S.DEVICE_0, size=10, shift=0) # maix bit set shift=1
#print(sr.size()) #Showcase Size
#sr.set_threshold(0, 0, 10000) #Set Threshold

def get_mic_dir(): #Direction algorithm from the 7 Mic Array
    AngleX=0
    AngleY=0
    AngleR=0
    Angle=0
    AngleAddPi=0
    mic_list=[]
    userCommand = false
    imga = mic.get_map()    # Determine location
    b = mic.get_dir(imga)   # Find direction
    for i in range(len(b)):
        if b[i]>=2:
            AngleX+= b[i]*math.sin(i*math.pi/6)
            AngleY+= b[i]*math.cos(i*math.pi/6)
    AngleX=round(AngleX,6)
    AngleY=round(AngleY,6)
    if AngleY<0:AngleAddPi=180
    if AngleX<0 and AngleY > 0:AngleAddPi=360
    if AngleX!=0 or AngleY!=0:
        if AngleY==0:
            Angle=90 if AngleX>0 else 270
        else:
            Angle=AngleAddPi+round(math.degrees(math.atan(AngleX/AngleY)),4)
        AngleR=round(math.sqrt(AngleY*AngleY+AngleX*AngleX),4)
        mic_list.append(AngleX)
        mic_list.append(AngleY)
        mic_list.append(AngleR)
        mic_list.append(Angle)

    if AngleX != 0 or AngleY != 0:
        userCommand = true
    else:
        userCommand= false
    a = mic.set_led(b,(255,0,0)) #Direction of the noise in blue.
    return mic_list, userCommand

#Object Detection Section for LUROX D
def lcd_show_except(e):
    import uio
    err_str = uio.StringIO()
    sys.print_exception(e, err_str)
    err_str = err_str.getvalue()
    img = image.Image(size=input_size)
    img.draw_string(0, 10, err_str, scale=1, color=(0xff,0x00,0x00))
    lcd.display(img)

def main(anchors, labels = None, model_addr="/sd/m.kmodel", sensor_window=input_size, lcd_rotation=0, sensor_hmirror=True, sensor_vflip=False):
    sensor.reset(freq=30000000) #SPI is set at 30MHz
    sensor.set_pixformat(sensor.RGB565)
    sensor.set_framesize(sensor.QVGA)
    sensor.set_saturation(2)
    sensor.set_brightness(2)
    sensor.set_auto_gain(True)
    sensor.set_windowing(sensor_window)
    sensor.set_hmirror(sensor_hmirror)
    sensor.set_vflip(sensor_vflip)
    sensor.run(1)

    uart = init_uart()
    comm = Comm(uart)

    try:
        task = None
        task = kpu.load(model_addr)
        kpu.init_yolo2(task, 0.5, 0.3, 5, anchors) # threshold:[0,1], nms_value: [0, 1]
        while(True):
            img = sensor.snapshot()
            t = time.ticks_ms()
            objects = kpu.run_yolo2(task, img)
            t = time.ticks_ms() - t
            if objects:
                for obj in objects:
                    pos = obj.rect()
                comm.send_detect_result(objects, labels)
    except Exception as e:
        raise e
    finally:
        if not task is None:
            kpu.deinit(task)


if __name__ == "__main__":
    try:
        main(anchors = anchors, labels=labels, model_addr=0x300000, lcd_rotation=0)
    except Exception as e:
        sys.print_exception(e)
        lcd_show_except(e)
    finally:
        gc.collect()

#Follwing path, Wakeword: LUROX/Azami/Ene/Apollo/Ares ->
#Sentence: Request = Can, Order = Need, Executive Order = Executive Order
#Commands -> Grab, Pull, Push.
#Grab = Grab & Hold, Pull = Pull & Bring & Drag, Push = Push & Move
#Desired Objects from Label

OrderType = 0
CommandType = 0
ObjectLabel = 0

def CommandClass():
    if OrderType == 1: #Call to grab through "request"
        if sr.Done == sr.recognize():
            res = sr.result()
            if res == 'Grab':
                CommandType = 1
                ObjectClass()
            if res == 'Pull':
                CommandType = 2
                ObjectClass()
            if res == 'Push':
                CommandType = 3
                ObjectClass()
            if res == 'release':
                CommandType = 4
                ObjectClass()


def ObjectClass():
    if sr.Done == sr.recognize():
        res = sr.result()

def SpeechClass():
    if sr.Done == sr.recognize():
        res = sr.result()
        if res == 'Can':
            OrderType = 1
            CommandClass() #Request Commands

#Main Loop function
while True:
    light = (50,50,50,50,50,50,50,50,50,50,50,50) #Camera On/Awake
    mic.set_led(light,(255,255,255)) #Keep LED on bright white to find objects.
