# Thermal Fan Controller

A 4-channel PWM fan controller based on Arduino Nano, designed for active thermal management with reduced noise levels and electrical consumption in mind. Built for the Institut Jožef Stefan (Ljubljana, 2026), as part of a University of Ljubljana, Faculty of Electrical Engineering (UL FE) internship project.

## Features

- Controls up to 4 PWM fans across 2 independent channels
- Support for 1 or 2 DS18B20 temperature sensors
- Linear duty cycle curve between two temperature limits
- RPM monitoring via tacho signal for each fan
- Alarm output on high temperature or low fan RPM
- 16x2 I2C LCD display with real-time status
- Serial monitor output for debugging and sensor setup
- Watchdog timer for automatic recovery from lockups
- Fully configurable via `#define` parameters at the top of the Arduino sketch

## Hardware

- Arduino Nano
- Up to 2x Dallas DS18B20 temperature sensors (1-Wire interface)
- Up to 4x 4-wire DC fans
- 16x2 LCD with I2C backpack
- Custom PCB (included in repository)
- Traco Power TEN 4-2411 DC/DC converter (24V → 5V)
- Reed relay for alarm output

## Repository Structure

This repository contains two versions of the project:

- `prototype/` — initial prototype version
- `v1.0/` — first full version with custom PCB, includes all files listed below

```
/
├── prototype/
├── v1.0/
│   ├── firmware/          # Arduino sketch (.ino)
│   ├── libraries/         # All included libraries (source files)
│   │   ├── lib_I2CLCD/    # Modified I2C LCD library
│   │   ├── DallasTemperature/
│   │   └── OneWire/
│   ├── pcb/               # PCB design files
│   ├── case/              # 3D model files (.step)
│   └── photos/            # Photos of assembled board
└── README.md
```

## Wiring / Pin Assignment

| Function        | Arduino Pin |
|-----------------|-------------|
| Temperature bus | D2          |
| PWM channel 1   | D9          |
| PWM channel 2   | D10         |
| Alarm output    | D3          |
| Fan 1 tacho     | D4          |
| Fan 2 tacho     | D5          |
| Fan 3 tacho     | D6          |
| Fan 4 tacho     | D7          |
| LCD             | I2C (A4/A5) |

## Configuration

All user parameters are at the top of the sketch under `USER SET PARAMETERS`:

| Parameter     | Description                                                |
|---------------|------------------------------------------------------------|
| `SERIAL_FREQ` | Serial baud rate, set to 0 to disable                      |
| `NUM_SENSORS` | Number of connected temperature sensors (1 or 2)           |
| `FAN_MASK`    | Bitmask of occupied fan slots, e.g. `0b0111` for fans 1–3  |
| `PWM_FREQ`    | PWM frequency in Hz (typically 25000 for PC fans)          |
| `UPDATE_FREQ` | Measurement interval in ms (minimum 800ms)                 |
| `UPPER_LIM`   | Temperature at which fans run at 100%                      |
| `LOWER_LIM`   | Temperature at which fans run at minimum duty              |
| `MIN_DUTY`    | Minimum duty cycle (0.0 to 1.0)                            |
| `ALARM_TEMP`  | Temperature threshold for alarm                            |
| `PULSE_COUNT` | Tacho pulses per revolution (typically 2)                  |
| `MIN_RPM`     | Minimum RPM before alarm triggers (0 to disable)           |

### Fan Mask

`FAN_MASK` is a 4-bit mask where each bit represents a fan slot:

```
0b(fan4)(fan3)(fan2)(fan1)
```

Example: `0b0111` means fans connected in slots 1, 2 and 3, while slot 4 is empty.

### Adding a New Temperature Sensor

1. Connect the sensor and open the serial monitor
2. At startup, the sketch prints the address of all detected sensors
3. Copy the address into `sensor1` or `sensor2` in the sketch

A separate `Address_reader` sketch is also included for convenience.

## LCD Display

- Spinning icon in top-left indicates the system is running
- Temperature shown in °C, `No Con.` if sensor can't be found (check if address is correct in sketch)
- Duty cycle shown as percentage or `MAX`
- Fan icons blink when RPM is below `MIN_RPM`
- `!` next to temperature when alarm threshold is exceeded

## Alarm Output

Pin D3 outputs `LOW` when any of the following occur:

- Any monitored fan RPM drops below `MIN_RPM`
- Any sensor reads above `ALARM_TEMP`
- Any sensor is disconnected

## Dependencies

All required libraries are included in the `v1.0/libraries` directory and can be installed manually into the Arduino libraries folder. Original sources:

- [DallasTemperature](https://github.com/milesburton/Arduino-Temperature-Control-Library)
- [OneWire](https://github.com/PaulStoffregen/OneWire)
- lib_I2CLCD — modified version of [Arduino-I2C-RW1063-0B-002](https://github.com/JeffVi/Arduino-I2C-RW1063-0B-002) by JeffVi

## Photos

Photos of the assembled board are in the `v1.0/photos` directory.

## License

No license specified. All rights reserved unless stated otherwise.  
Contact the author for usage permissions.
