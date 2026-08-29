# LUROX D K210 Speech & Object Recognition Algorithms

#Initalization K210 Sensors & Protocols
import sensor, image, time
from Maix import GPIO, I2S
from fpioa_manager import fm
import KPU as kpu
import gc, sys
import network, math, rpc, sensor, struct, tf

# user setting
sample_rate   = 16000
record_time   = 4  #s

#Initalizing i2S Microphone on K210
fm.register(20,fm.fpioa.I2S0_IN_D0, force=True)
fm.register(18,fm.fpioa.I2S0_SCLK, force=True)
fm.register(19,fm.fpioa.I2S0_WS, force=True)
rx = I2S(I2S.DEVICE_0)
rx.channel_config(rx.CHANNEL_0, rx.RECEIVER, align_mode=I2S.STANDARD_MODE)
rx.set_sample_rate(sample_rate)
print(rx)

#Load Model
from speech_recognizer import isolated_word
#Calling model
sr = isolated_word(dmac=2, i2s=I2S.DEVICE_0, size=10)
print(sr.size())
print(sr)
## threshold
sr.set_threshold(0, 0, 10000)
## record and get & set
while True:
    time.sleep_ms(100)
    print(sr.state())
    if sr.Done == sr.record(0):
        data = sr.get(0)
        print(data)
        break
    if sr.Speak == sr.state():
        print('speak A')
        img.draw_rectangle((0, 0, 320, 240), fill=True, color=(255, 255, 255))
        img.draw_string(10,80, "Please speak A", color=(255, 0, 0), scale=4, mono_space=0)
        lcd.display(img)
sr.set(1, data)
img.draw_rectangle((0, 0, 320, 240), fill=True, color=(255, 255, 255))
img.draw_string(10, 80, "get !", color=(255, 0, 0), scale=4, mono_space=0)
lcd.display(img)
time.sleep_ms(500)

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time = 2000)

clock = time.clock()

while(True):
    clock.tick()
    img = sensor.snapshot()
    print(clock.fps())

def ObjectRec(labels = None, model_addr="/sd/m.kmodel", lcd_rotation=0, sensor_hmirror=False, sensor_vflip=False):
    gc.collect()

    sensor.reset()
    sensor.set_pixformat(sensor.RGB565)
    sensor.set_framesize(sensor.QVGA)
    sensor.set_windowing((224, 224))
    sensor.set_hmirror(sensor_hmirror)
    sensor.set_vflip(sensor_vflip)
    sensor.run(1)

    lcd.init(type=1)
    lcd.rotation(lcd_rotation)
    lcd.clear(lcd.WHITE)

    if not labels:
        raise Exception("no labels.txt")

    task = kpu.load(model_addr)

    try:
        while(True):
            img = sensor.snapshot()
            t = time.ticks_ms()
            fmap = kpu.forward(task, img)
            t = time.ticks_ms() - t
            plist=fmap[:]
            pmax=max(plist)
            max_index=plist.index(pmax)
            img.draw_string(0,0, "%.2f\n%s" %(pmax, labels[max_index].strip()), scale=2, color=(255, 0, 0))
            img.draw_string(0, 200, "t:%dms" %(t), scale=2, color=(255, 0, 0))
            lcd.display(img)
    except Exception as e:
        sys.print_exception(e)
    finally:
        kpu.deinit(task)


if __name__ == "__main__":
    try:
        with open("labels.txt") as f:
            labels = f.readlines()
        main(labels=labels, model_addr=0x300000, lcd_rotation=0, sensor_hmirror=False, sensor_vflip=False)
        # main(labels=labels, model_addr="/sd/m.kmodel")
    except Exception as e:
        sys.print_exception(e)
    finally:
        gc.collect()

