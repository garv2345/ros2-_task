# ROS 2 Package Validator & Simulation

I built this project to automate the testing of ROS 2 packages. The goal was to create a tool that not only checks the code for errors but also visualizes the result on a real robot arm in Gazebo.

## What it does
* **Code Quality:** It scans Python scripts for syntax errors using `flake8` to ensure the code is clean.
* **Safety Checks:** It specifically looks for unsafe joint velocities. If the code tries to move the robot faster than 3.14 rad/s, it blocks the execution.
* **Web Interface:** I created a dashboard using Flask. You can upload a zip file, get an instant pass/fail report, and trigger the robot simulation directly from the browser.

## How to use it
1.  **Launch the Simulation:**
    Run this to start Gazebo with the UR5e robot:
    ```bash
    ros2 launch ur_simulation_gazebo ur_sim_control.launch.py ur_type:=ur5e
    ```

2.  **Start the Web App:**
    Open a new terminal and run the backend:
    ```bash
    python3 web_app/app.py
    ```

3.  **Test the Robot:**
    Go to `http://localhost:5000` in your browser. You can upload packages to test them or click "Run Simulation" to see the arm move.
