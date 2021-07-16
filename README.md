
# "Smart" Self Balancing Robot

![IMG_9093](https://user-images.githubusercontent.com/496532/125873024-a41c8e8c-5947-4981-8f8b-3741dd35afd8.JPG)


Using simple neural networks to replicate sensor fusion and dual-loop PID control of a self-balancing robot. The inner PID loop controls the robot's attitude, and the outer controls its translational position. The robot uses an [MPU-6050 IMU](https://www.adafruit.com/product/3886) (gyroscope + accelerometer) for feedback on its attitude and an encoder on one of its wheels for feedback on its position.

## Repository Contents

**CAD_files/** - files used to 3D print robot's body

**components/** - arduino files for testing different components before integrating together

**PID_controller/** - arduino file for dual-loop PID controller

**NN_controller/** - arduino file for neural network(s) controller

**LQR_control_arduino/** - arduino file for LQR controller (currently not working)

**LQR_model.py** - script to generate LQR gains from state dynamics and simulate performance

**NN_angle_predictor.py** - script to train network that predicts robot attiude from sensor readings

**NN_arctangent.py** - script to approximate arctangent with a neural net (sanity checking)

**NN_data_generator.py** - script to generate training data (randomly generates inputs and calculates PID outputs) for angle prediction and controller networks

**NN_PID_controller.py** - script to train controller network

**NN_util_funcs.py** - utiliy functions for neural network training (random batch selection and calculating loss and accuracy)

**NN_weights_saver.py** - saves neural network weights to .txt files for easy copying into .ino files

**pend_bot_LQR_requirements.txt** - run `pip install -r pend_bot_LQR_requirements.txt` in an anaconda environment to install dependencies for LQR python scripts (or just download the packages listed in the file)

## Neural network structures

### Angle Network

Input: \[last angle estimate, acceleration in y, acceleration in x, rotational velocity about x]

Output: \[ New angle Estimate ]

Layers:

* Input \[4]

* Hidden \[6]

* ReLU

* Output \[1]

### Controller Network

Input: \[last angle estimate, rotational velocity about x, current_encoder_reading, last_encoder_reading]

Output: \[ Desired PWM ]

Layers:

* Input \[4]

* Hidden \[10]

* ReLU

* Output \[1]
