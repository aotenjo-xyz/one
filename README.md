# Aotenjo One
This is Aotenjo One board firmware repository. It controls a gimbal motor and communicates with the main controller via CAN bus.

[Docs](https://aotenjo.xyz/docs/category/aotenjo-one) | [Shop](https://shop.aotenjo.xyz/products/aotenjo-one)

<img src="/.github/images/aotenjo-one-and-be4108.png" alt="Aotenjo One" width="500"/>

Features
- STM32G431CB (128KB Flash, 32KB RAM, 170MHz)
- Closed loop FOC control
- CAN (up to 1Mbps)
- I2C, UART, SPI
- based on DRV8313
- 2.5A peak current
- 8-35V input voltage
- 14-bit magnetic encoder
- Emergency stop command


## Install

Install this repo
```bash
git clone https://github.com/aotenjo-xyz/one.git 
```

Install dependencies (SimpleCanLib)
```bash
mkdir Libraries
cd Libraries
git clone https://github.com/yuichiroaoki/SimpleCanLib.git
git checkout CANSendByte-Serial1
```


Directory structure
```bash
├── Libraries
│   └── SimpleCanLib
└── one
    ├── include
    ├── lib
    ├── LICENSE
    ├── platformio.ini
    ├── README.md
    ├── src
    └── test
```

### Development

Format
```bash
make format
```


## References

- [Igitigit2/SimpleCanLib: CAN bus library for ESP32 and STM32 G431](https://github.com/Igitigit2/SimpleCanLib)
- [mackelec/meFDCAN: Arduino FDCAN library for stm32G4 microcontrollers](https://github.com/mackelec/meFDCAN)
