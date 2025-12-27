# Line Following Robot with Adaptive Cruise Control

## Abstract

*This study presents the design and implementation of a line-following robot with PID-based adaptive cruise control, built using the STM32F103RB Nucleo microcontroller. The robot uses an infrared sensor array to detect and follow a predefined path by measuring voltage changes. An ultrasonic sensor is utilized for obstacle detection, enabling the robot to adjust its speed and position dynamically to maintain a safe distance. The PID controller processes data from the sensors to achieve accurate control, providing stable path following and effective obstacle management. Dual DC motors, managed through a motor driver circuit, provide smooth and responsive movement. The embedded software, developed at the register level, maximizes performance and hardware efficiency. Additionally, a user interface displays real-time information about the system's status, supporting reliable operation. This project demonstrates the effective application of PID control in achieving precise line following and obstacle avoidance within defined limits, showcasing efficient and stable robotic performance.*

## Software Components

* Modular codebase written using libraries.
* Functions divided across separate .c and .h files.
* Integrated and executed in the main program file.
* Developed using Keil Microvision 5

## Hardware Requirements

- STM32F103RB Nucleo microcontroller board
- Infrared sensor array (for line detection)
- Ultrasonic sensor (HC-SR04 or similar)
- Dual DC motors
- Motor driver circuit (L298N or similar)
- Power supply (battery pack)
- Chassis and wheels
- Connecting wires and breadboard

## Software Requirements

- Keil µVision 5 IDE
- STM32 HAL libraries
- USB cable for programming and debugging

## Getting Started

### 1. Clone the Repository

```bash
git clone https://github.com/Ibrahimumutdoruk/LineFollowerRobotwithACC.git
cd LineFollowerRobotwithACC
```

### 2. Project Structure

The project follows a modular architecture:

```
LineFollowerRobotwithACC/
├── Core/
│   ├── Inc/
│   │   ├── main.h
│   │   ├── stm32f1xx_hal_conf.h
│   │   └── stm32f1xx_it.h
│   └── Src/
│       ├── main.c
│       ├── stm32f1xx_hal_msp.c
│       ├── stm32f1xx_it.c
│       └── system_stm32f1xx.c
├── Drivers/
│   ├── CMSIS/
│   └── STM32F1xx_HAL_Driver/
├── Libraries/
│   ├── sensor.c / sensor.h
│   ├── motor.c / motor.h
│   ├── pid.c / pid.h
│   └── display.c / display.h
└── README.md
```

### 3. Setting Up Keil µVision 5

1. **Open Keil µVision 5**
2. **Create New Project or Open Existing:**
   - File → Open Project
   - Navigate to the project folder
   - Select the `.uvprojx` file

3. **Configure Target Options:**
   - Right-click on target → Options for Target
   - Device: STM32F103RB
   - Debug: ST-Link Debugger
   - Utilities: Use debug driver to program Flash

### 4. Hardware Connections

#### Pin Configuration:
- **Infrared Sensors:** Connect to ADC pins (PA0, PA1, PA2, PA3, PA4)
- **Ultrasonic Sensor:**
  - Trigger: PB0
  - Echo: PB1
- **Motor Driver:**
  - Motor A: PB6, PB7
  - Motor B: PB8, PB9
  - Enable pins: PA8, PA9
- **Display/LED indicators:** PC13, PC14, PC15

### 5. Code Usage

#### Main Program Flow:

```c
// main.c - Main execution loop
int main(void) {
    // System initialization
    HAL_Init();
    SystemClock_Config();
    
    // Initialize peripherals
    MX_GPIO_Init();
    MX_ADC1_Init();
    MX_TIM2_Init();
    MX_USART2_UART_Init();
    
    // Initialize custom libraries
    Sensor_Init();
    Motor_Init();
    PID_Init();
    Display_Init();
    
    while (1) {
        // Main control loop
        Line_Following_Control();
        Obstacle_Detection();
        PID_Control();
        Motor_Control();
        Display_Update();
    }
}
```

#### Key Functions:

**Sensor Management:**
```c
// sensor.c
uint16_t Read_IR_Sensors(void);
float Read_Ultrasonic_Distance(void);
void Sensor_Calibration(void);
```

**Motor Control:**
```c
// motor.c
void Motor_Set_Speed(uint8_t left_speed, uint8_t right_speed);
void Motor_Forward(void);
void Motor_Turn_Left(void);
void Motor_Turn_Right(void);
void Motor_Stop(void);
```

**PID Controller:**
```c
// pid.c
void PID_Init(float kp, float ki, float kd);
float PID_Calculate(float setpoint, float measured_value);
void PID_Reset(void);
```

### 6. Configuration Parameters

#### PID Tuning:
```c
// In pid.h
#define PID_KP 2.0f    // Proportional gain
#define PID_KI 0.1f    // Integral gain
#define PID_KD 0.5f    // Derivative gain
```

#### Speed Settings:
```c
// In motor.h
#define BASE_SPEED 150     // Base motor speed (0-255)
#define MAX_SPEED 200      // Maximum motor speed
#define MIN_SPEED 100      // Minimum motor speed
#define TURN_SPEED 120     // Speed during turns
```

#### Sensor Thresholds:
```c
// In sensor.h
#define LINE_THRESHOLD 512        // ADC threshold for line detection
#define OBSTACLE_DISTANCE 20.0f   // Obstacle detection distance (cm)
#define SAFE_DISTANCE 15.0f       // Safe following distance (cm)
```

### 7. Building and Flashing

1. **Build the Project:**
   - Press F7 or click Build button
   - Ensure no compilation errors

2. **Flash to Microcontroller:**
   - Connect STM32 Nucleo via USB
   - Press F8 or click Download button
   - Wait for successful programming

3. **Debug (Optional):**
   - Press Ctrl+F5 to start debugging session
   - Set breakpoints to analyze code execution

### 8. Operation Instructions

1. **Initial Setup:**
   - Power on the robot
   - Place on the starting line
   - Wait for sensor calibration (LED indicators)

2. **Line Following Mode:**
   - Robot automatically starts following the line
   - PID controller maintains smooth path tracking
   - Speed adjusts based on line curvature

3. **Adaptive Cruise Control:**
   - When obstacle detected, robot slows down
   - Maintains safe distance from obstacles
   - Resumes normal speed when path is clear

4. **Status Monitoring:**
   - LED indicators show system status
   - UART output provides real-time debugging info

### 9. Troubleshooting

**Common Issues:**

- **Robot not following line:** Check sensor calibration and threshold values
- **Erratic movement:** Tune PID parameters (start with lower gains)
- **Poor obstacle detection:** Verify ultrasonic sensor connections and mounting
- **Motors not responding:** Check motor driver connections and power supply

**Debug Tips:**
- Use UART output to monitor sensor values
- Check LED indicators for system status
- Verify power supply voltage (7-12V recommended)
- Ensure all connections are secure

### 10. Customization

#### Adding New Features:
1. Create new .c and .h files in Libraries folder
2. Include in main.c
3. Add initialization in main() function
4. Integrate into main control loop

#### Modifying Control Algorithm:
- Edit PID parameters in pid.h
- Adjust sensor thresholds in sensor.h
- Modify motor speeds in motor.h
