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
class Comm:
    def __init__(self, uart):
        self.uart = uart

    def UART_Layered_Track(self, Open, track, Close):
        msg = ""
        msg = "{}:{}:{}:{}:{}:{}:{}".format(Open,
                                            track['id'],
                                            track['rect'][0],  # x
                                            track['rect'][1],  # y
                                            track['rect'][2],  # w
                                            track['rect'][3],  # h
                                            Close ) 
        if msg:
            msg = msg[:-2] + "\n"
        self.uart.write(msg.encode())

    def UART_Layered_Comms(self, Open, R, I, O, S, Close):
        msg = ""
        msg += "{}:{}:{}:{}:{}:{}, ".format(Open, R, I, O, S, Close)
        if msg:
            msg = msg[:-2] + "\n"
        self.uart.write(msg.encode())

    def UART_Layered_Direct(self, Open, DGC, Layer, Close):
        msg = ""
        msg += "{}:{}:{}:{}, ".format(Open, DGC, Layer, Close)
        if msg:
            msg = msg[:-2] + "\n"
        self.uart.write(msg.encode())

    def UART_Test(self):
        msg = "UART_TEST"
        if msg:
            msg = msg[:-2] + "\n"
        self.uart.write(msg.encode())

def init_uart():
    fm.register(15, fm.fpioa.UART1_TX, force=True)
    fm.register(10, fm.fpioa.UART1_RX, force=True)

    uart = UART(UART.UART1, 115200, 8, 0, 0, timeout=1000, read_buf_len=256)
    return uart