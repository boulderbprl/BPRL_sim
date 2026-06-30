 %% Example Usage Script for import_ardupilot_data function
clear all;
close all;
clc;

addpath('flight_data');

% Import data 1
% Uncomment for 2026-06-28_4SR_Flight
%filename = './flight_data/2026-06-28_4SR_Flight';
% startTime = 33; 
% endTime = 125; 

% Uncomment for 2026-06-28_4SR_Flight
filename = './flight_data/2026-06-28_4SR_Flight_NoFilt';
startTime = 40; 
endTime = 112;

dt = 1/40;  % 30 Hz sampling rate
 

globalTime = [startTime:1/40:endTime]';
flight_data = import_ardupilot_data_esc(filename+".BIN", globalTime);

%     .states: [x, y, z, u, v, w, p, q, r, phi, theta, psi]
%     .inputs: [roll_input, pitch_iSnput, yaw_input, thrust_input]
%     .strain: struct of arm data { .arm_1 , .arm_2 , .arm_3 , .arm_4}
%     .acceleration: [ax, ay, az ,p_dot , q_dot] in body frame
%     .time: interpolated time vector

%% data analysis 

plot_states(flight_data,[startTime,endTime],true)

save(filename+".mat")