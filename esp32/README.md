# ESP-IDF OTA template app

This is a template application with OTA capability, to be used with `Espressif IoT Development Framework`.

See also (component version): https://github.com/yanbe/esp32-ota-server

OTA capability is achieved with rather small binary and memory footprint.

|repository                |binary size  |free memory  |flashing speed              |
|--------------------------|-------------|-------------|----------------------------|
|espressif/esp-idf-template|423,424 bytes|248,604 bytes|10s (via UART in 921,600bps)|
|yanbe/esp-idf-ota-template|474,849 bytes|245,060 bytes|6s (via OTA)                |

With OTA template app, your build-flashing-monitor cycle become much easier and faster.

## Usage

You have to flash via UART first, then confirm IP address (like 192.168.0.3) assigned to ESP32
via log output of `make monitor` with USB UART.

```sh
$ make menuconfig # configure "WiFi Configuration" menu on your network.
$ make erase_flash flash monitor
```

After that, you can use flashing capability with special make target

```sh
$ make ota ESP32_IP=192.168.0.3
```

## What's difference from official OTA example in ESP-IDF?

ESP-IDF's [OTA example project](https://github.com/espressif/esp-idf/tree/master/examples/system/ota) is a good
starting point to understand how ESP32's OTA flashing works with official OTA suppoprt components.
But it is _pull_ architecture. You have to launch HTTP server on your PC every time, and wait ESP32 to update their program.
It is not optimal in terms of speed up build-flashing-monitor cycle on developing phase.

My OTA template app _push_ architecture. So you can flashing natural make command like berow.

```sh
$ make ota ESP32_IP=192.168.0.3
```

Additionally, in esp-idf-ota-template, OTA implementation is placed into `components/ota_server`
so you can concentrate your project without be bothered by OTA-related socket programming code.
