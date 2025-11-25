# Cooker Whistle Counter (TinyML)

A smart home automation project that counts pressure cooker whistles using audio recognition on an ESP32. This system uses a TinyML model trained with **Edge Impulse** to detect the specific sound of a cooker whistle and track the count.

## Features
*   **Audio Recognition**: Detects cooker whistles in real-time.
*   **Whistle Counting**: Keeps track of the number of whistles.
*   **Edge Computing**: All processing happens locally on the ESP32.
*   **TinyML**: Efficient model deployment using TensorFlow Lite for Microcontrollers (via Edge Impulse).

## Hardware Requirements
*   **ESP32 Development Board**
*   **Microphone Module**: INMP441 (I2S) or PDM microphone.

## Software Requirements
*   **PlatformIO**: Development environment.
*   **Edge Impulse SDK**: Embedded C++ library for inference.

## How it Works
1.  **Training Data**: Audio samples of cooker whistles and kitchen background noise were recorded.
2.  **Model Training**: An audio classification model was trained on Edge Impulse to distinguish "whistle" from "noise".
3.  **Deployment**: The model was exported as a C++ library.
4.  **Application Logic**: The ESP32 listens to audio, runs inference, and increments a counter when a "whistle" event is detected with high confidence.

## Setup & Build
1.  Clone the repository.
2.  Open in **VS Code** with **PlatformIO**.
3.  Connect the microphone to the ESP32 (check `src/main.cpp` or `config.h` for pinout).
4.  Build and Upload:
    ```bash
    pio run --target upload
    ```
5.  Monitor the output:
    ```bash
    pio device monitor
    ```

## License
This project is licensed under the Apache 2.0 License - see the [LICENSE](LICENSE) file for details.
