 %% Example Usage Script for import_ardupilot_data function
clear all;
close all;
clc;

addpath('flight_data');

% Import data 1
filename = 'Flight1_11_21.bin';

dt = 1/40;  % 30 Hz sampling rate

globalTime = [0:dt:600]';
flight_data = import_ardupilot_data(filename, globalTime);

%     .states: [x, y, z, u, v, w, p, q, r, phi, theta, psi]
%     .inputs: [roll_input, pitch_iSnput, yaw_input, thrust_input]
%     .strain: struct of arm data { .arm_1 , .arm_2 , .arm_3 , .arm_4}
%     .acceleration: [ax, ay, az ,p_dot , q_dot] in body frame
%     .time: interpolated time vector

%% data analysis 

 plot_states(flight_data,[100,300])
