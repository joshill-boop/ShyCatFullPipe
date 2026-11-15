# ShyCat Full Pipeline

This project uses a webcam and machine learning to control four stepper motors. The sculpture behaves like a shy cat, sort of... reacting to how a person moves.

| File                            | Function                                                                                                                                                        |
| ------------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| **python_ShyCatFullPipe.py**    | Uses the webcam and MediaPipe to track body movements. Calculates position, speed and smoothness. Sends 12 inputs to Wekinator and tracking data to Processing. |
| ****Wekinator**                 | Uses machine learning to interpret movement. Takes 12 inputs from Python and predicts threat and engagement values. Sends these to Processing.                  |
| **3. Processing Sketch (.pde)** | Receives data from Python and Wekinator. Controls fear, trust, mimicking and arm movements. Displays a digital simulation and sends motor positions to Arduino. |
| **4. Arduino Sketch (.ino)**    | Receives motor positions from Processing. Controls four stepper motors using AccelStepper, including speed, acceleration and manual homing.                     |

## Information Flow

Webcam → Python / MediaPipe → Wekinator → Processing → Arduino → Stepper Motors

Python also sends information directly to Processing, bypassing Wekinator.

## Software

- Python

- OpenCV and MediaPipe

- Wekinator

- Processing

- Arduino IDE
