# wiimote-mouse-demo

Mouse Demo with Wii Remote Controller

<img width="80%" src="./Product.png" />  

## Requirement

- [ESP32 dev board](https://www.switch-science.com/catalog/3210/)
- ESP-IDF (xtensa-esp32-elf at the 1.22.0-80)
- Digispark
- Arduino IDE (Version: 1.8.5)
- USB Connector (Micro B)

 
##  Pin connections

|USB Connector  |ESP32  |Digispark  |Note  |
|---|---|---|---|
|+5V  |VIN  |5V  ||
|GND  |GND  |GND  ||
|D+  | NC |D+  ||
|D-  | NC |D-  ||
|NC |IO25  |P0(SDA)  |pull-up in ESP32 program|
|NC |IO26  |P2(SCL)  |pull-up in ESP32 program|

## Get started 

### Setup Digispark

1. Install Arduino libraries:
   - [TinyWireS](https://github.com/rambo/TinyWire)

1. Programming to Digispark

### Setup ESP32 dev board

1. Get btstack
`git clone https://github.com/bluekitchen/btstack.git`

1. Create Example Projects
Copy [wiimote_i2c_converter.c](./wiimote_i2c_converter.c) to btstack/examples  
Note: Modify `wiimote_addr_string = "AA-AA-AA-AA-AA-AA" ` in `wiimote_i2c_converter.c` to your Wii Controller's MAC Address

1. Programming to ESP32

```
$ cd port/esp32
$ ./integrate_btstack.py
$ cd example/wiimote_i2c_converter
$ make menuconfig
$ make
$ make flash
```

## Usage 

1. Connect USB Connector to the USB port on your computer

1. To autoconnect, press the 1 and 2 buttons on Wii Remote

1. The LED4 will be on when they have finished connecting
<img width="30%" src="./WiiRemoteLED.png" />  
