from machine import UART, Pin
import time

uart = UART(0, baudrate=9600, tx=Pin(0), rx=Pin(1))
print("LoRa TX test starting on UART0 (GP0=TX, GP1=RX)...")

count = 0
while True:
    msg = "DWARKA_SKYGATE_TEST_{}\n".format(count)
    uart.write(msg)
    print("Sent:", msg.strip())
    count += 1
    time.sleep(1)
