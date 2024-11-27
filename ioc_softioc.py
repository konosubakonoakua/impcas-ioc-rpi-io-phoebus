#!/usr/bin/env /home/rpi/.virtualenvs/ioc/bin/python

# import argparse
import gpiozero
import smbus
# import spidev
import serial
from softioc import softioc, builder
# from softioc import asyncio_dispatcher
# import asyncio
import cothread
import os
import socket
import netifaces
import phoebusgen

HOSTNAME = socket.gethostname()

try:
    APPNAME = os.environ["APPNAME"]
    DEBUG = os.environ["DEBUG"]
    PREFIX = os.environ["PREFIX"]
    PHOEBUS_INTERFACE_FILE = os.environ["PHOEBUS_INTERFACE_FILE"]
except:
    APPNAME = "IO"
    DEBUG = False
    PREFIX = f"{HOSTNAME}:{APPNAME}"
    PHOEBUS_INTERFACE_FILE = "rpi_io.bob"


def get_all_ip_addresses():
    ip_addresses = {}
    interfaces = netifaces.interfaces()
    for interface in interfaces:
        addresses = netifaces.ifaddresses(interface)
        if netifaces.AF_INET in addresses:
            ip_addresses[interface] = addresses[netifaces.AF_INET][0]['addr']
    return ip_addresses

IPADDRESSES = get_all_ip_addresses()
# for interface, ip in IPADDRESSES.items():
#     print(f"Interface: {interface}, IP address: {ip}")

builder.SetDeviceName(PREFIX)

gpio_pins = {

    # BCM Pin   Physical Pin    Function 1 (Alt0)
    # -------------------------------------------
    # BCM 0     Pin 27    I2C0_SDA
    # BCM 1     Pin 28    I2C0_SCL
    # BCM 2     Pin 3     I2C1_SDA
    # BCM 3     Pin 5     I2C1_SCL
    # BCM 4     Pin 7     GPCLK0
    # BCM 5     Pin 29    GPCLK1
    # BCM 6     Pin 31    GPCLK2
    # BCM 7     Pin 26    SPI0_CE1_N
    # BCM 8     Pin 24    SPI0_CE0_N
    # BCM 9     Pin 21    SPI0_MISO
    # BCM 10    Pin 19    SPI0_MOSI
    # BCM 11    Pin 23    SPI0_SCLK
    # BCM 12    Pin 32    PWM0
    # BCM 13    Pin 33    PWM1
    # BCM 14    Pin 8     UART0_TXD
    # BCM 15    Pin 10    UART0_RXD
    # BCM 16    Pin 36    SPI1_CE2_N
    # BCM 17    Pin 11    SPI1_CE1_N
    # BCM 18    Pin 12    SPI1_CE0_N
    # BCM 19    Pin 35    SPI1_MISO
    # BCM 20    Pin 38    SPI1_MOSI
    # BCM 21    Pin 40    SPI1_SCLK
    # BCM 22    Pin 15
    # BCM 23    Pin 16
    # BCM 24    Pin 18
    # BCM 25    Pin 22
    # BCM 26    Pin 37
    # BCM 27    Pin 13


    # J8:
    #    3V3  (1) (2)  5V
    #  GPIO2  (3) (4)  5V
    #  GPIO3  (5) (6)  GND
    #  GPIO4  (7) (8)  GPIO14
    #    GND  (9) (10) GPIO15
    # GPIO17 (11) (12) GPIO18
    # GPIO27 (13) (14) GND
    # GPIO22 (15) (16) GPIO23
    #    3V3 (17) (18) GPIO24
    # GPIO10 (19) (20) GND
    #  GPIO9 (21) (22) GPIO25
    # GPIO11 (23) (24) GPIO8
    #    GND (25) (26) GPIO7
    #  GPIO0 (27) (28) GPIO1
    #  GPIO5 (29) (30) GND
    #  GPIO6 (31) (32) GPIO12
    # GPIO13 (33) (34) GND
    # GPIO19 (35) (36) GPIO16
    # GPIO26 (37) (38) GPIO20
    #    GND (39) (40) GPIO21

    2:  gpiozero.LED(2),    # GPIO 2  (pin 3)  (I2C1 SDA)
    3:  gpiozero.LED(3),    # GPIO 3  (pin 5)  (I2C1 SCL)
    4:  gpiozero.LED(4),    # GPIO 4  (pin 7)
    14: gpiozero.LED(14),   # GPIO 14 (pin 8)  (UART TXD)
    15: gpiozero.LED(15),   # GPIO 15 (pin 10) (UART RXD)
    17: gpiozero.LED(17),   # GPIO 17 (pin 11)
    18: gpiozero.LED(18),   # GPIO 18 (pin 12)
    27: gpiozero.LED(27),   # GPIO 27 (pin 13)
    22: gpiozero.LED(22),   # GPIO 22 (pin 15)
    23: gpiozero.LED(23),   # GPIO 23 (pin 16)
    24: gpiozero.LED(24),   # GPIO 24 (pin 18)
    10: gpiozero.LED(10),   # GPIO 10 (pin 19) (SPI0 MOSI)
    9:  gpiozero.LED(9),    # GPIO 9  (pin 21) (SPI0 MISO)
    25: gpiozero.LED(25),   # GPIO 25 (pin 22)
    11: gpiozero.LED(11),   # GPIO 11 (pin 23) (SPI0 SCLK)
    8:  gpiozero.LED(8),    # GPIO 8  (pin 24) (SPI0 CE0)
    7:  gpiozero.LED(7),    # GPIO 7  (pin 26) (SPI0 CE1)
    5:  gpiozero.LED(5),    # GPIO 5  (pin 29)
    6:  gpiozero.LED(6),    # GPIO 6  (pin 31)
    12: gpiozero.LED(12),   # GPIO 12 (pin 32)
    13: gpiozero.LED(13),   # GPIO 13 (pin 33)
    19: gpiozero.LED(19),   # GPIO 19 (pin 35)
    26: gpiozero.LED(26),   # GPIO 26 (pin 37)
    16: gpiozero.LED(16),   # GPIO 16 (pin 36)
    20: gpiozero.LED(20),   # GPIO 20 (pin 38)
    21: gpiozero.LED(21),   # GPIO 21 (pin 40)
}

for pin in [2, 3, 14, 15, 10, 9, 11, 8, 7]:
    gpio_pins.pop(pin)

i2c_bus = smbus.SMBus(1)
i2c_devices = {}

# for address in range(0x03, 0x78):
for address in range(0x03, 0x04):
    try:
        i2c_bus.read_byte(address)
        i2c_devices[address] = {'register': 0x00}
        if DEBUG:
            print(f"i2c:1 addr:0x{address} FOUND!")
    except Exception as e:
        print(f"i2c:1 addr:0x{address} XXXXXXXXXXXXXXXXXXXXXXXXX err:{e}")

# spi = spidev.SpiDev()
# spi.open(0, 0)
# spi.max_speed_hz = 1000000

uart = serial.Serial('/dev/ttyS0', baudrate=115200, timeout=1)

records = {}

for gpio_id in gpio_pins:
    records[f"GPIO:{gpio_id}:VALUE"] = builder.boolIn(f"GPIO:{gpio_id}:VALUE", initial_value=0, ZNAM="0", ONAM="1")
    records[f"GPIO:{gpio_id}:SET"] = builder.boolOut(
        f"GPIO:{gpio_id}:SET",
        initial_value=0,
        always_update=True,
        on_update=lambda v, gpio_id=gpio_id: set_gpio(v, gpio_id)
    )

for i2c_id in i2c_devices:
    records[f"I2C:{i2c_id}:VALUE"] = builder.boolIn(f"I2C:{i2c_id}:VALUE", initial_value=0)
    records[f"I2C:{i2c_id}:SET"] = builder.boolOut(f"I2C:{i2c_id}:SET", initial_value=0)
    records[f"I2C:{i2c_id}:REGISTER"] = builder.stringOut(
        f"I2C:{i2c_id}:REGISTER",
        initial_value="0x00",
        always_update=True,
        on_update=lambda v, i2c_id=i2c_id: set_i2c(v, i2c_id)
    )

# records["SPI:0:VALUE"] = builder.boolIn("SPI:0:VALUE", initial_value=0)
# records["SPI:0:SET"] = builder.boolOut(
#     "SPI:0:SET",
#     initial_value=0,
#     always_update=True,
#     on_update=lambda v: set_spi(v)
# )

records["UART:PORT"] = builder.stringOut(
    "UART:PORT",
    initial_value='/dev/ttyS0',
    on_update=lambda v: set_uart_port(v)
)
records["UART:BAUDRATE"] = builder.stringOut(
    "UART:BAUDRATE",
    initial_value='115200',
    on_update=lambda v: set_uart_baudrate(v)
)
records["UART:TIMEOUT"] = builder.stringOut(
    "UART:TIMEOUT",
    initial_value='1',
    on_update=lambda v: set_uart_timeout(v)
)
records["UART:0:SET"] = builder.stringOut(
    "UART:0:SET",
    initial_value='',
    on_update=lambda v: set_uart(v)
)
records["UART:0:VALUE"] = builder.stringIn("UART:0:VALUE", initial_value='')

def update_gpio():
    while True:
        for gpio_id in gpio_pins:
            value = gpio_pins[gpio_id].value
            record = records[f"GPIO:{gpio_id}:VALUE"]
            record.set(value)
            # if DEBUG:
            #     print(f"gpio{gpio_id}: {value}")
        cothread.Sleep(0.1)

# async def update_gpio():
#     while True:
#         for gpio_id in gpio_pins:
#             records[f"GPIO:{gpio_id}:VALUE"].set(gpio_pins[gpio_id].value)
#         await asyncio.sleep(0.1)

# async def update_i2c():
#     while True:
#         for i2c_id in i2c_devices:
#             try:
#                 register = int(i2c_devices[i2c_id]['register'], 16)
#                 value = i2c_bus.read_byte_data(i2c_id, register)
#                 records[f"I2C:{i2c_id}:VALUE"].set(value)
#             except:
#                 pass
#         await asyncio.sleep(0.1)

# async def update_spi():
#     while True:
#         records["SPI:0:VALUE"].set(spi.xfer2([0x00])[0])
#         await asyncio.sleep(0.1)

# async def update_uart():
#     while True:
#         records["UART:0:VALUE"].set(int(uart.read(1).hex(), 16))
#         await asyncio.sleep(0.1)

def set_gpio(value, gpio_id):
    gpio_pins[gpio_id].value = value
    if DEBUG:
        print(f"DEBUG: gpio{gpio_id}->{value}")

def set_i2c(value, i2c_id):
    try:
        register = int(i2c_devices[i2c_id]['register'], 16)
        i2c_bus.write_byte_data(i2c_id, register, value)
        if DEBUG:
            print(f"DEBUG: i2c{i2c_id}:0x{register:x}->0x{value:x}")
    except:
        pass

def set_i2c_register(register, i2c_id):
    i2c_devices[i2c_id]['register'] = register
    if DEBUG:
        print(f"DEBUG: i2c{i2c_id}:reg->0x{register:x}")

# def set_spi(value):
#     spi.xfer2([value])
#     if DEBUG:
#         print(f"DEBUG: spi->{value}")

def set_uart_port(port):
    global uart
    uart = serial.Serial(port, uart.baudrate, timeout=uart.timeout)
    if DEBUG:
        print(f"DEBUG: uart:port->{port}")

def set_uart_baudrate(baudrate):
    global uart
    uart.baudrate = int(baudrate)
    if DEBUG:
        print(f"DEBUG: uart:baudrate->{baudrate}")
    uart.close()
    uart.open()

def set_uart_timeout(timeout):
    global uart
    uart.timeout = float(timeout)
    if DEBUG:
        print(f"DEBUG: uart:timeout->{timeout}")
    uart.close()
    uart.open()

def set_uart(value):
    uart.write(value.encode('utf-8'))
    if DEBUG:
        print(f"DEBUG: uart->{value}")

def generate_phoebus_interface(title):
    if not os.path.exists(PHOEBUS_INTERFACE_FILE):
        my_screen = phoebusgen.screen.Screen('RPi_IO')
        my_screen.background_color(204, 255, 255)
        my_screen.macro('P', PREFIX)

        widgets = []

        # Add title label
        title = phoebusgen.widget.Label('TitleLabel', title, 0, 0, 0, 0)
        title.font_size(48)
        title.font_style_bold()
        title.auto_size()
        widgets.append(title)

        gpio_x_offset = 0
        gpio_y_offset = 0
        gpio_x_spacing = 180
        gpio_y_spacing = 60
        gpio_row_count = 0
        gpio_row_limit = 4  # Number of GPIOs per row
        gpio_checkbox_size = (120, 30)
        gpio_led_x_offset = gpio_checkbox_size[0]+10
        gpio_led_size = (30, 30)
        gpio_group_size = (740, 360)

        uart_group_size = (360, 380)

        # Add GPIO widgets in a group
        gpio_group = phoebusgen.widget.Group('GPIO_Group', 0, 100, *gpio_group_size)

        for gpio_id in gpio_pins:
            pv_name_value = f"$(P):GPIO:{gpio_id}:VALUE"
            pv_name_set = f"$(P):GPIO:{gpio_id}:SET"

            check_box = phoebusgen.widget.CheckBox(
                f'GPIO_{gpio_id}_CheckBox',
                f'GPIO {gpio_id}',
                pv_name_set,
                gpio_x_offset,
                gpio_y_offset,
                gpio_checkbox_size[0],
                gpio_checkbox_size[1],
            )
            check_box.font_style_bold()
            check_box.font_size(20)
            gpio_group.add_widget(check_box)

            led = phoebusgen.widget.LED(
                f'GPIO_{gpio_id}_LED',
                pv_name_value,
                gpio_led_x_offset,
                gpio_y_offset,
                gpio_led_size[0],
                gpio_led_size[1],
            )
            led.off_color(255, 0, 0)
            gpio_group.add_widget(led)

            gpio_x_offset += gpio_x_spacing
            gpio_led_x_offset += gpio_x_spacing
            gpio_row_count += 1

            if gpio_row_count >= gpio_row_limit:
                gpio_x_offset = 0
                gpio_led_x_offset = gpio_checkbox_size[0]+10
                gpio_y_offset += gpio_y_spacing
                gpio_row_count = 0

        widgets.append(gpio_group)

        # # Add I2C widgets
        # i2c_x_offset = 500
        # i2c_y_offset = 80
        # i2c_spacing = 150
        # for i2c_id in i2c_devices:
        #     pv_name_value = f"$(P):I2C:{i2c_id}:VALUE"
        #     pv_name_set = f"$(P):I2C:{i2c_id}:SET"
        #     pv_name_register = f"$(P):I2C:{i2c_id}:REGISTER"

        #     check_box = phoebusgen.widget.CheckBox(
        #         f'I2C_{i2c_id}_CheckBox',
        #         f'I2C {i2c_id}',
        #         pv_name_set,
        #         i2c_x_offset,
        #         i2c_y_offset,
        #         190,
        #         70
        #     )
        #     check_box.font_style_bold()
        #     check_box.font_size(24)
        #     widgets.append(check_box)

        #     led = phoebusgen.widget.LED(f'I2C_{i2c_id}_LED', pv_name_value, i2c_x_offset + 230, i2c_y_offset - 20, 140, 110)
        #     led.off_color(255, 0, 0)
        #     widgets.append(led)

        #     text_input = phoebusgen.widget.TextEntry(f'I2C_{i2c_id}_Register', pv_name_register, i2c_x_offset, i2c_y_offset + 80, 190, 70)
        #     text_input.font_style_bold()
        #     text_input.font_size(24)
        #     widgets.append(text_input)

        #     i2c_y_offset += i2c_spacing

        # Add UART widgets in a group
        uart_group = phoebusgen.widget.Group('UART_Group', 800, 100, *uart_group_size)
        uart_label_size = (140, 70)
        uart_label_x_offset = 0
        uart_y_offset = 0
        uart_spacing = 80

        # Add labels for UART settings
        uart_port_label = phoebusgen.widget.Label('UART_Port_Label', 'UART Port:', uart_label_x_offset, uart_y_offset, *uart_label_size)
        uart_port_label.font_style_bold()
        uart_port_label.font_size(24)
        uart_group.add_widget(uart_port_label)

        uart_port = phoebusgen.widget.TextEntry('UART_Port', '$(P):UART:PORT', uart_label_x_offset+uart_label_size[0], uart_y_offset, 190, 70)
        uart_port.font_style_bold()
        uart_port.font_size(24)
        uart_group.add_widget(uart_port)

        uart_baudrate_label = phoebusgen.widget.Label('UART_Baudrate_Label', 'Baudrate:', uart_label_x_offset - 100, uart_y_offset + uart_spacing, *uart_label_size)
        uart_baudrate_label.font_style_bold()
        uart_baudrate_label.font_size(24)
        uart_group.add_widget(uart_baudrate_label)

        uart_baudrate = phoebusgen.widget.TextEntry('UART_Baudrate', '$(P):UART:BAUDRATE', uart_label_x_offset+uart_label_size[0], uart_y_offset + uart_spacing, 190, 70)
        uart_baudrate.font_style_bold()
        uart_baudrate.font_size(24)
        uart_group.add_widget(uart_baudrate)

        uart_timeout_label = phoebusgen.widget.Label('UART_Timeout_Label', 'Timeout:', uart_label_x_offset - 100, uart_y_offset + 2 * uart_spacing, *uart_label_size)
        uart_timeout_label.font_style_bold()
        uart_timeout_label.font_size(24)
        uart_group.add_widget(uart_timeout_label)

        uart_timeout = phoebusgen.widget.TextEntry('UART_Timeout', '$(P):UART:TIMEOUT', uart_label_x_offset+uart_label_size[0], uart_y_offset + 2 * uart_spacing, 190, 70)
        uart_timeout.font_style_bold()
        uart_timeout.font_size(24)
        uart_group.add_widget(uart_timeout)

        # uart_value = phoebusgen.widget.LED('UART_LED', '$(P):UART:0:VALUE', uart_x_offset + 230, uart_y_offset + uart_spacing - 20, 140, 110)
        # uart_value.off_color(255, 0, 0)
        # uart_group.add_widget(uart_value)

        uart_set_label = phoebusgen.widget.Label('UART_Set_Label', 'Set Value:', uart_label_x_offset - 100, uart_y_offset + 3 * uart_spacing, *uart_label_size)
        uart_set_label.font_style_bold()
        uart_set_label.font_size(24)
        uart_group.add_widget(uart_set_label)

        uart_set = phoebusgen.widget.TextEntry('UART_Set', '$(P):UART:0:SET', uart_label_x_offset+uart_label_size[0], uart_y_offset + 3 * uart_spacing, 190, 70)
        uart_set.font_style_bold()
        uart_set.font_size(24)
        uart_group.add_widget(uart_set)

        widgets.append(uart_group)

        # Add all widgets to the screen
        my_screen.add_widget(widgets)

        # Write to specified file
        my_screen.write_screen(PHOEBUS_INTERFACE_FILE)
        print(f"Phoebus interface file generated: {PHOEBUS_INTERFACE_FILE}")
    else:
        print(f"Phoebus interface file already exists: {PHOEBUS_INTERFACE_FILE}")

if __name__ == "__main__":
    print("#"*50)
    title = f"prefix={PREFIX}       ip={IPADDRESSES['eth0']}" 
    print(title)
    generate_phoebus_interface(title)
    print("#"*50)

    builder.LoadDatabase()
    softioc.iocInit()

    cothread.Spawn(update_gpio)

    # softioc.iocInit(asyncio_dispatcher.AsyncioDispatcher())
    # asyncio.ensure_future(update_gpio())
    # asyncio.ensure_future(update_i2c())
    # asyncio.ensure_future(update_spi())
    # asyncio.ensure_future(update_uart())

    softioc.interactive_ioc(globals())
