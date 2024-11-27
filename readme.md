# Raspberry Pi IO softioc
![phoebus](https://github.com/user-attachments/assets/88f75c14-31b9-4eb6-bb13-3759f20f7f04)
This project provides an EPICS (Experimental Physics and Industrial Control System) interface for controlling and monitoring GPIO pins, I2C devices, and UART communication on a Raspberry Pi. The project uses the `softioc` library to create EPICS records and the `gpiozero` library to interact with GPIO pins.

## Table of Contents

- [Features](#features)
- [Prerequisites](#prerequisites)
- [Installation](#installation)
- [Usage](#usage)
- [Configuration](#configuration)
- [License](#license)

## Features

- **GPIO Control**: Control and monitor GPIO pins using EPICS records.
- **I2C Communication**: Interact with I2C devices using EPICS records.
- **UART Communication**: Configure and send data over UART using EPICS records.
- **Phoebus Interface**: Generate a Phoebus interface file for easy visualization and control.

## Prerequisites

- Python 3.12 or higher
- `softioc` library
- `gpiozero` library
- `smbus` library
- `serial` library
- `cothread` library
- `netifaces` library
- `phoebusgen` library

## Installation

1. **Clone the repository**:

   ```bash
   git clone https://github.com/konosubakonoakua/impcas-ioc-rpi-io-phoebus.git
   cd impcas-ioc-rpi-io-phoebus
   ```

2. **Create a virtual environment**:

   ```bash
   mkdir -p ~/.virtualenvs
   cd ~/.virutalenvs
   python3 -m venv ioc
   source ~/.virtualenvs/ioc/bin/activate
   ```

3. **Install dependencies**:

   ```bash
   pip install -r requirements.txt
   ```

## Usage

The script will automatically generate a Phoebus interface file (`rpi_io.bob`) if it does not already exist.

```bash
chmod +x ./ioc_softioc.py
APPNAME="IO" DEBUG=True PREFIX="rpi:io" PHOEBUS_INTERFACE_FILE="rpi_io.bob" ./ioc_softioc.py
```


## Configuration

### GPIO Pins

The GPIO pins are configured in the `gpio_pins` dictionary. Each pin is mapped to a `gpiozero.LED` object. You can modify this dictionary to add or remove GPIO pins as needed.

### I2C Devices [WIP]

The I2C devices are detected automatically by scanning the I2C bus. The detected devices are stored in the `i2c_devices` dictionary. You can modify the I2C address range in the `i2c_devices` loop.

### UART Configuration [WIP]

The UART configuration is set using EPICS records. You can modify the initial values for the UART port, baud rate, and timeout in the `records` dictionary.

## License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for more details.
