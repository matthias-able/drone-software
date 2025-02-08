# In /Arduino: 
Quadcopter Flight Controller with PID Stabilization
Developed a flight control system for a quadcopter using an Arduino and MPU-6050 gyroscope/accelerometer. Key features include:

    PID Control: Implemented a Proportional-Integral-Derivative (PID) controller for stabilizing roll, pitch, and yaw movements, ensuring smooth and stable flight.

    Sensor Integration: Integrated the MPU-6050 sensor to measure angular velocity and acceleration, enabling real-time attitude estimation and correction.

    Receiver Input Handling: Processed PWM signals from a remote control receiver to interpret user inputs for manual flight control.

    Motor Control: Calculated and adjusted pulse-width modulation (PWM) signals for four electronic speed controllers (ESCs) to control motor speeds dynamically.

    Auto-Leveling: Added an auto-leveling feature to maintain the quadcopter’s orientation using accelerometer data.

    Battery Monitoring: Implemented voltage monitoring to prevent over-discharging and ensure safe operation.
    
    

# ToDo
- account for (cross)wind
- how to fly a loop
- mission2
