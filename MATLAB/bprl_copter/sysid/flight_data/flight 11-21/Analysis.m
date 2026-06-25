%% Example Usage Script for import_ardupilot_data function
clear all;
close all;
clc;

addpath('flight_data');

% Import data 1
filename = 'Flight1_11_21.bin';
dt = 1/50;  

globalTime = [0:dt:400]';
flight_data = import_ardupilot_data(filename, globalTime);


time_bounds = [95,106];

withIMU = 0;

%% data analysis 

% %% Plot data
% time_bounds = [55,120];



% Extract p_dot and q_dot from acceleration array
% states: [x, y, z, u, v, w, p, q, r, phi, theta, psi]
% flight_data.acceleration = [ax, ay, az, p_dot, q_dot]

p_imu = flight_data.states(:,7);
q_imu = flight_data.states(:,8);

p_dot_imu = flight_data.acceleration(:,4);  % Roll acceleration from IMU
q_dot_imu = flight_data.acceleration(:,5);  % Pitch acceleration from IMU

strain_front = flight_data.strain.arm_1 / 6000;
strain_right = flight_data.strain.arm_2 / 6000;
strain_left = flight_data.strain.arm_4 / 6000;
strain_back = flight_data.strain.arm_3 / 6000;

time_series = [time_bounds(1)/dt , time_bounds(2)/dt];

X = [strain_front(time_series(1):time_series(2),:),...
     strain_right(time_series(1):time_series(2),:),...
     strain_left(time_series(1):time_series(2),:),...
     strain_back(time_series(1):time_series(2),:)];
if withIMU ==1 ; X = [X,q_imu(time_series(1):time_series(2),:)]; end

% also contribute //X(:,3),X(:,12)
X_p = [X(:,1),X(:,6),X(:,9),X(:,10)]; % bending in front&back twisting in right&left
X_q = [X(:,3),X(:,4),X(:,7),X(:,12)]; % bending in right&left twisting in front&back

Y_p = p_imu(time_series(1):time_series(2),:);
Y_q = q_imu(time_series(1):time_series(2),:);
Y_p_dot = p_dot_imu(time_series(1):time_series(2),:);
Y_q_dot = q_dot_imu(time_series(1):time_series(2),:);

fit_p = fitlm(X_p,Y_p);
fit_q = fitlm(X_q,Y_q);
fit_p_dot = fitlm(X,Y_p_dot);
fit_q_dot = fitlm(X,Y_q_dot);

Strain_arm_data = [strain_front,strain_right,strain_left,strain_back]; 
if withIMU ==1 ; Strain_arm_data = [Strain_arm_data,q_imu]; end

Strain_arm_p_data = [Strain_arm_data(:,1),Strain_arm_data(:,6),Strain_arm_data(:,9),Strain_arm_data(:,10)];
Strain_arm_q_data = [Strain_arm_data(:,3),Strain_arm_data(:,4),Strain_arm_data(:,7),Strain_arm_data(:,12)];

p_fitted = predict( fit_p , Strain_arm_p_data );
q_fitted = predict( fit_q , Strain_arm_q_data );

p_dot_fitted = predict( fit_p_dot , Strain_arm_data );
q_dot_fitted = predict( fit_q_dot , Strain_arm_data );


%% Roll Plots 
% time_bounds = [84,92];
plot_data_raw(flight_data,time_bounds)

% Roll Rate
figure(10); set(gcf, 'Color', 'w');
    
    subplot(3,1,1);
    plot(globalTime, p_fitted, 'b'); hold on;
    plot(globalTime(time_series(1):time_series(2)), fit_p.Fitted, 'g:'); hold on;
    hold on; grid on;
    ylabel('Roll Rate Strain  [rad/s]');
    xlim(time_bounds);
    title('Roll Rate Comparison');

    subplot(3,1,2);
    plot(globalTime, p_imu, 'r'); 
    hold on; grid on;
    ylabel('Roll Rate IMU [rad/s]');
    xlim(time_bounds);

    subplot(3,1,3);
    plot(globalTime, p_fitted, 'b'); hold on;
    plot(globalTime, p_imu, 'r'); hold on;
    plot(globalTime(time_series(1):time_series(2)), fit_p.Fitted, 'g:'); hold on;
    grid on;
    legend('Strain', 'IMU','Fitted section', 'Location', 'best'); %
    ylabel('Roll rate [rad/s]');
    xlim(time_bounds);
    xlabel('Time [s]');

% Roll Acceleration
figure(11); set(gcf, 'Color', 'w');

    subplot(3,1,1);
    plot(globalTime, p_dot_fitted, 'b'); hold on;
    plot(globalTime(time_series(1):time_series(2)), fit_p_dot.Fitted, 'g:'); hold on;
    hold on; grid on;
    ylabel('Roll Accel Strain [rad/s²]');
    xlim(time_bounds);
    title('Roll Acceleration Comparison');

    subplot(3,1,2);
    plot(globalTime, p_dot_imu, 'r'); 
    hold on; grid on;
    ylabel('Roll Accel IMU [rad/s²]');
    xlim(time_bounds);

    subplot(3,1,3);
    plot(globalTime, p_dot_fitted, 'b'); hold on;
    plot(globalTime, p_dot_imu, 'r'); hold on;
    plot(globalTime(time_series(1):time_series(2)), fit_p_dot.Fitted, 'g:'); hold on;
    grid on;
    legend('Strain', 'IMU', 'Fitted section', 'Location', 'best'); %
    ylabel('Roll Accel [rad/s²]');
    xlim(time_bounds);
    xlabel('Time [s]');

%% pitch plots 
% Pitch Rate
figure(12); set(gcf, 'Color', 'w');
    
    subplot(3,1,1);
    plot(globalTime, q_fitted, 'b'); hold on;
    plot(globalTime(time_series(1):time_series(2)), fit_q.Fitted, 'g:'); hold on;
    hold on; grid on;
    ylabel('Pitch Rate Strain  [rad/s]');
    xlim(time_bounds);
    title('Pitch Rate Comparison');

    subplot(3,1,2);
    plot(globalTime, q_imu, 'r'); 
    hold on; grid on;
    ylabel('Pitch Rate IMU [rad/s]');
    xlim(time_bounds);

    subplot(3,1,3);
    plot(globalTime, q_fitted, 'b'); hold on;
    plot(globalTime, q_imu, 'r'); hold on;
    plot(globalTime(time_series(1):time_series(2)), fit_q.Fitted, 'g:'); hold on;
    grid on;
    legend('Strain', 'IMU','Fitted section', 'Location', 'best'); %
    ylabel('Pitch rate [rad/s]');
    xlim(time_bounds);
    xlabel('Time [s]');

% Pitch Acceleration
figure(13); set(gcf, 'Color', 'w');

    subplot(3,1,1);
    plot(globalTime, q_dot_fitted, 'b'); hold on;
    plot(globalTime(time_series(1):time_series(2)), fit_q_dot.Fitted, 'g:'); hold on;
    hold on; grid on;
    ylabel('Pitch Accel Strain [rad/s²]');
    xlim(time_bounds);
    title('Pitch Acceleration Comparison');

    subplot(3,1,2);
    plot(globalTime, q_dot_imu, 'r'); 
    hold on; grid on;
    ylabel('Pitch Accel IMU [rad/s²]');
    xlim(time_bounds);

    subplot(3,1,3);
    plot(globalTime, q_dot_fitted, 'b'); hold on;
    plot(globalTime, q_dot_imu, 'r'); hold on;
    plot(globalTime(time_series(1):time_series(2)), fit_q_dot.Fitted, 'g:'); hold on;
    grid on;
    legend('Strain', 'IMU','Fitted section', 'Location', 'best'); %
    ylabel('Pitch Accel [rad/s²]');
    xlim(time_bounds);
    xlabel('Time [s]');


