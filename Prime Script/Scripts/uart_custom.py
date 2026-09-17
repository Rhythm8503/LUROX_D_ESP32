#######################################################################
#
#                  LUROX D Maix-Bit K210 Software
#                  Developed By: Taheemuddin Ahmed
#
#######################################################################

from Maix import I2S, GPIO
from machine import UART
from fpioa_manager import fm

#UART Initalization

################## MAPPING ####################### 

#Request -> 0 - 3 = Request {Can [0], Would[1], Could[2], Please[3]}
#Request -> 4 - 5 = Direct Command {Grab[4], Hold[5]}
#Request -> 6 - 10 = Gestures {Wave[6], Handshake[7], ThumbsUp[8], ThumbsDown[9], High-Five[10]}
#Request -> 11 - 12 = Interrupt/HALT {Stop[11], Cancel[12]}

#Intention -> 0 - 3 = Command (Grab[0], Hold[1], Push[2], Pull[3])
#Intention -> 4 - 11 = Gestures {Wave[4], Handshake[5], Thumbs Up[6], Thumbs Down[7], High-Five[8], Point[9], Peace[10], Ok! [11]}
#Intention -> 12 - 13 =  Interrupt / HALT {Stop[12], Cancel[13]}

#Specification -> 0 - 9 = Colors (Red[0], Orange[1], Yellow[2], Green[3], Blue[4], Purple[5], White[6], Black[7], Brown[8], Grey[9])
#Specification -> 10 - 20 = Objective (Wallet[10], Pliers[11], Wrench[12], Cup[13], Phone[14], Screwdrivers[15], Scissors[16], Drill[17], Hammer[18], Can[19], Bottle[20])
#Specification -> 21 - 22 = Interrupt (Stop[21], Cancel[22])

#Objective -> 0 - 10 = Objective (Wallet[0], Pliers[1], Wrench[2], Cup[3], Phone[4], Screwdrivers[5], Scissors[6], Drill[7], Hammer[8], Can[9], Bottle[10])
#Objective -> 11 - 12 = Interrupt (Stop[11], Cancel[12])

################## MAPPING #######################


class comm:
    def __init__(self, uart):
        self.uart = uart
        self.state = 0
        self.buffer = []
        self.layer1_data = [0, 0, 0, 0]  # request, intent, objective, specification
        self.layer2_data = [0, 0, 0, 0]  # objX, objY, objW, objH

    def UART_Layered_Track(self, Open, track, Close):
        x = int(track['rect'][0] + track['rect'][2] / 2) # Pulling X Center from Box
        y = int(track['rect'][1] + track['rect'][3] / 2) # Pulling Y Center from Box
        w = int(track['rect'][2]) 
        h = int(track['rect'][3])

        x = max(0, min(255, int(x)))
        y = max(0, min(255, int(y)))
        w = max(0, min(255, int(w)))
        h = max(0, min(255, int(h)))

        msg = bytes([Open, x, y, w, h, Close])
        self.uart.write(msg)

    def UART_Layered_Comms(self, Open, R, I, O, S, Close):
        msg = bytes([Open, R, I, O, S, Close])
        self.uart.write(msg)

    def UART_read(self):
        if self.uart.any():
            data = self.uart.read()
            for byte in data:
                if self.state == 0:  # WAITING_LAYER1
                    if byte == 0x01:
                        self.state = 1
                        self.buffer = []
                elif self.state == 1:  # READING_LAYER1
                    if len(self.buffer) < 4:
                        self.buffer.append(byte)
                    else:
                        if byte == 0x03:  # CLOSE
                            self.layer1_data = [
                                self.buffer[0],
                                self.buffer[1],
                                self.buffer[2],
                                self.buffer[3]
                            ]
                            self.state = 0
                            return True
                        else:
                            self.state = 0  # Reset on error
        else:
            return None

    def UART_Test(self):
        msg = "K210 ALIVE"
        if msg:
            msg = msg[:-2] + "\n"
        self.uart.write(msg.encode())

def init_uart():
    fm.register(10, fm.fpioa.UART1_TX, force=True)
    fm.register(15, fm.fpioa.UART1_RX, force=True)

    uart = UART(UART.UART1, 115200, 8, 0, 0, timeout=1000, read_buf_len=256)
    return uart