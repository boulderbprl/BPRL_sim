clc
clear
close all

% Physics model - RTOS drone in SITL
% This sets up the vehicle properties that are used in the simulation
% rtos drone bprl lab

% check out: https://www.motocalc.com/motocalc.htm#topic_21
% Download at https://www.motocalc.com/

% setup the motors
% 
%Motors: Lumenier RX 2206-11 2350 KV
% ESC: Turnigy Multistar 32bit 12A Race Spec ESC 2~4S (OPTO)
% Prop: DAL CYCLONE T5040C Propeller - Black 5 in prop

%%%%%% NEED TO COMPUTE MOIs and prop constants using thrust stand%%%%%%%

% locations (xyz) in m

%The center of the prop is 35mm above the CG,
%The drone has a 370mm wheel base (185mm from the CG to the motor) 

arm_length = 370; %370 mm

% locations (xyz) in m
motor(1).location = [[sind(50),cosd(50)]*arm_length*0.5,35] * 0.001; % front right
motor(2).location = [[sind(130),cosd(130)]*arm_length*0.5,35] * 0.001; % rear right
motor(3).location = [[sind(230),cosd(230)]*arm_length*0.5,35] * 0.001; % rear left
motor(4).location = [[sind(310),cosd(310)]*arm_length*0.5,35] * 0.001; % front left

% PWM output to use
motor(1).channel = 1;
motor(2).channel = 4;
motor(3).channel = 2;
motor(4).channel = 3;

% rotation direction: 1 = cw, -1 = ccw
motor(1).direction = -1;
motor(2).direction = 1;
motor(3).direction = -1;
motor(4).direction = 1;


% Add all to vehicle
copter.motors = motor;

copter.motor_model.coeff_poles = [140.7446 1.0840e+04 4.2208e+05 6.5621e+06];
copter.motor_model.coeff_zeros = [-0.1211 15.4302 -1.0774e+03 3.0115e+04];
copter.motors.model.np = length(copter.motor_model.coeff_poles); % number of poles of motor model
copter.motor_model.nz = length(copter.motor_model.coeff_zeros); % number of zeros of motor model
copter.motor_model.torque_thrust_coeffs = [];

copter.motor_model.pwm_mean = 
copter.motor_model.pwm_std = 

copter.mass = 0.8135; % (kg) 
%inertia = (2/5) * copter.mass * (0.48*0.2)^2; % (sphere)  ###~ dense compact to 1/5 length radius
copter.inertia = [8.1067e-3,-3.8570e-5,1.1986e-5;...
                  -3.8570e-5,6.0941e-3,7.3568e-7;...
                  1.1986e-5,7.3568e-7,1.3050e-2];
copter.cd = [0.5;0.5;0.5];
copter.cd_ref_area = [1;1;1] * pi * (0.61*0.5)^2; % changed from 0.45

save('rtos_drone','copter')

