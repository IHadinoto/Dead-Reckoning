## Advanced Autonomous Systems–Project 0 (training). Version 20 21. 1. 1

## Advanced Autonomous Systems – 2021

## Project 0

This short project is composed by problems which are useful as a training session, for preparing you for projects during
subsequent weeks in AAS.

```
Problem 1
```
Given the following approximate model of a pendulum,

## ( ) ( ( )) ( ) ( )

```
22
```
## sin

## 1

## 110 , 2.2 , 1.

## tAtBtCut

## radrad

## ABC

## sssvolt

##  = −   −  + 

## =  =  = 

## 

## where (t)is the angular position (expressed in radians) of the pendulum, and ut( )is the voltage (expressed in volts)

controlling the pendulum’s electric motor.
a) Obtain a valid state space representation for this system, in continuous time.
b) Obtain an approximate discrete time model (using Euler’s approximation), for a sample time dt=1ms.
c) Implement a program (in plain Matlab language), for simulating the model proposed in (b).
Test your program simulating the following cases:
c.1) The pendulum is released, at time =0, having the following initial conditions: angular velocity =0 and angle =
110 degrees. The voltage of the electric motor is assumed to be constantly 0 volts (no torque being applied by the
motor).
c.2) Similar to (c.1) but having the electric motor controlled with a constant voltage = 3 volts.

```
In both cases, perform the simulation for an interval of time from 0 to t= 7 seconds.
Plot the results (position and angular velocity) in a figure.
```
d) Using the model implemented in item c, implement a simulation in Simulink.

```
Problem 2
```
Given the following simplified 3DoF kinematic model (of a car-like wheeled platform),

# ( ) ( ) ( ( ))

# ( ) ( ) ( ( ))

# ( ) ( ( ))

## ( )

## cos

## sin

## tan

## xtvtt

## ytvtt

## vt

## tt

## L

## 

## 

## 

## =

## =

## =

a) Obtain an approximate discrete-time version of the model, assuming small discrete steps, e.g. of dt=0.01 seconds
(1 0 ms). Consider the case of a vehicle that has L= 2 .5m.

b) Implement a program for simulating the system in (a). Run it under different steering actions (sequences of

## steering angles (k)) and assume constant speed, vkk( )=3.5 m/s,.

```
c.1) See what happen if you keep the steering angle set at a constant value.
c.2) Try to generate a path having an 8 - shape (define a proper sequence of control actions to achieve it).
```

## Advanced Autonomous Systems–Project 0 (training). Version 20 21. 1. 2

```
c.4) Apply a small modification on the model (e.g. a small change in parameter L) and see how the result is
affected, for a long-term simulation (for cases c.1 and c.2). Plot, jointly, both models’ trajectories using different
colors, to appreciate the different responses.
```
Note: The main purpose of this task is to give the students some initial training, before the actual projects are released.
This task is intended to be solved during week 1.


