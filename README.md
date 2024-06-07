# LVGL ESP32 Button Control Demo

This project demonstrates the use of LVGL (Light and Versatile Graphics Library) on an ESP32-based system. The application includes two buttons that control GPIO pins and reflect their states on the screen. The buttons are labeled "Compressor 1" and "Compressor 2", and change colors and text based on the states of the GPIO pins and connected sensors.

## Features
- Two buttons to control and monitor the state of GPIO pins.
- Button states are visually represented with text and color changes.
- Text labels "Compressor 1" and "Compressor 2" above the respective buttons.

## Hardware Requirements
- ESP32 development board
- Touchscreen display compatible with LVGL
- GPIO connections for sensors and outputs

## Software Requirements
- ESP-IDF (Espressif IoT Development Framework)
- LVGL (Light and Versatile Graphics Library)

## Installation and Usage

### Hardware Setup
1. Connect the touchscreen display to the ESP32.
2. Connect the sensors to GPIO13 and GPIO12.
3. Connect the output control to GPIO2 and GPIO26.

### Software Setup
1. Clone this repository to your local machine:
    ```bash
    git clone https://github.com/yourusername/lvgl-esp32-button-demo.git
    cd lvgl-esp32-button-demo
    ```
2. Configure the ESP-IDF environment:
    ```bash
    . $HOME/esp/esp-idf/export.sh
    ```
3. Build and flash the project:
    ```bash
    idf.py build
    idf.py flash
    ```
4. Monitor the output:
    ```bash
    idf.py monitor
    ```

## Code Explanation
The `create_demo_application` function sets up the LVGL environment and creates two buttons with labels "Compressor 1" and "Compressor 2". The button states are updated based on the GPIO levels.

### Button Event Handlers
- `btn1_event_cb` and `btn2_event_cb` handle the click events for the buttons. They toggle the states of GPIO2 and GPIO26, respectively.

### State Update
- The `update_button_state` function checks the levels of GPIO13 and GPIO12 and updates the button labels and colors accordingly.

## License
This project is licensed under the MIT License. See the LICENSE file for details.

## Contributing
Contributions are welcome! Please open an issue or submit a pull request for any changes or improvements.

## Acknowledgments
- [LVGL](https://lvgl.io/) for the graphics library
- [Espressif](https://www.espressif.com/) for the ESP32 platform

