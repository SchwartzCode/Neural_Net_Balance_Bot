
# PID Controlled Pendulum Bot

Self balancing robot controller using PID, and then training a neural network to replace that PID controller 

## Neural network setup

Input: \[acceleration about y, acceleration about x, rotational velocity about x, integral of error (used in PID)]

Output: \[ PWM duty cycle ]

Layers:

* Input \[4]

* Hidden \[4]

* PReLU

* Hidden \[4]

* PReLU

* Hidden \[4]

* PReLU

* Output \[1]
