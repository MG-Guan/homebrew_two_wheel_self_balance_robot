# Two-wheel Self Balnce Robot


## How To Work
1. Turn on the power of the robot.
1. Set the mode of the robot to 1 (see how to control the robot brain).

## Control Robot Brain
1. Connect your laptop to the AP-network of the robot brain.
1. Sent HTTP GET request with parameters to set the working mode and other parameters to the brain to control the behavior of the robot.
    ```
    wget -qO- "http://192.168.4.1/set?fps=10&target=0.0&alpha=0.001&max_value=20&kp=0.05&ki=0.01&kd=0.001&mode=0"
    ```
    **mode**: The working mode of the robot, 0: stop 1: work

    **fps**: Frames per second to update sensor data in robot brain.

    **target**: The target roll angle (deg) of the robot to maintain, usually 0.0.

    **alpha**: Alpha value to map PID gain to control signal. (signal = PID_gain*alpha)

    **max_value**: Maximum control signal value. The range of control signal: [-max_value, max_value]

    **kp, ki, kd**: The parameters of PID control. 


