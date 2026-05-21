clc
clear
close all

% Physics model - BPRL copter in SITL
% This sets up the vehicle properties that are used in the simulation
% simple 480 size + quad based on strain quad in bprl lab

% check out: https://www.motocalc.com/motocalc.htm#topic_21
% Download at https://www.motocalc.com/

% setup the motors
% Tmotor MN3510 700KV - 12 x4

%%%%%% NEED TO COMPUTE MOIs and prop constants using thrust stand%%%%%%%

% locations (xyz) in m

arm_length = 0.61; %m

motor(1).location = [[sind(0),cosd(0)]*arm_length*0.5,0] ; % FRONT
motor(2).location = [[sind(90),cosd(90)]*arm_length*0.5,0] ; % RIGHT
motor(3).location = [[sind(180),cosd(180)]*arm_length*0.5,0] ; % BACK
motor(4).location = [[sind(270),cosd(270)]*arm_length*0.5,0] ; % LEFT


% PWM output to use
motor(1).channel = 3;
motor(2).channel = 1;
motor(3).channel = 4;
motor(4).channel = 2;

% rotation direction: 1 = cw, -1 = ccw
motor(1).direction = 1;
motor(2).direction = -1;
motor(3).direction = 1;
motor(4).direction = -1;

% motor properties
electrical.kv = 700; % (rpm/volt) JS
electrical.no_load_current = [0.5,10]; % (A) @ (V) JS
electrical.resistance = 0.05; % (ohms) 50mOhms JS

% ESC properties
esc.resistance = 0.01; % (ohms)

% Propeller properties
% P 12 x 4 Tmotor prop
prop.diameter = 304 * 0.001; % (m) JS
prop.pitch = 101.6 * 0.001; % (m) JS
prop.num_blades = 2;
prop.PConst = 1.13; %% need to be changed
prop.TConst = 1; %% need to be changed
prop.mass = 14.5 * 0.001; % (kg) (only used for inertia) JS
prop.inertia = (1/12)*prop.mass*prop.diameter^2;% rotational inertia (kgm^2) (rod about center)

% assign properties to motors
for i = 1:4
    motor(i).electrical = electrical;
    motor(i).esc = esc;
    motor(i).prop = prop;
end

% Setup battery
battery.voltage = 14.8; % (volts) JS 
battery.resistance = 0.0034; % (ohms)
battery.capacity = 4.0; % (ah) JS

% Add all to vehicle
copter.motors = motor;
copter.battery = battery;
copter.mass = 2.198; % (kg) 1.855 without mocap arms JS 
%inertia = (2/5) * copter.mass * (0.48*0.2)^2; % (sphere)  ###~ dense compact to 1/5 length radius
copter.inertia = diag([0.0523,0.0523,0.0592]);
copter.cd = [0.5;0.5;0.5];
copter.cd_ref_area = [1;1;1] * pi * (0.61*0.5)^2; % changed from 0.45

save('bprl_copter','copter')

% Plot motor curves
% http://www.bavaria-direct.co.za/constants/
% http://www.stefanv.com/rcstuff/qf200204.html
% Some calculators estimate heat and increase resistance with temp
% But then we have to estimate the power dissipation
% Max power for plot only
max_power = 260;
battery.voltage = battery.voltage * 0.50;

Kt = 1/(electrical.kv * ((2*pi)/60) ); % Convert Kv to rads/second

% plot the current from 0 to max power
amps = 0:0.1:max_power/battery.voltage;
power_in = amps * battery.voltage;

% voltage drop due to copper and esc
copper_drop = amps * electrical.resistance; 
esc_drop = amps * esc.resistance;

ideal_voltage = battery.voltage - copper_drop - esc_drop;
power_out = ideal_voltage .* (amps - electrical.no_load_current(1));
efficiency = power_out ./ power_in;

torque = Kt * amps;
rpm = ideal_voltage * electrical.kv;

% Plot motor characteristics
figure('name',sprintf('motor characteristics at %0.2f volts',battery.voltage))
subplot(2,2,1)
hold all
title('RPM')
plot(amps,rpm)
xlabel('Current (A)')
ylabel('RPM')
xlim([0,amps(end)])


subplot(2,2,2)
hold all
title('torque')
plot(amps,torque)
xlabel('Current (A)')
ylabel('torque (NM)')
xlim([0,amps(end)])

subplot(2,2,3)
hold all
title('power')
plot(amps,power_in)
plot(amps,power_out)
xlabel('Current (A)')
ylabel('power (W)')
ylim([0,inf])
xlim([0,amps(end)])
legend('Power in','Power out','location','northwest')

subplot(2,2,4)
hold all
title('efficiency')
plot(amps,efficiency)
xlabel('Current (A)')
ylabel('efficiency (%)')
ylim([0,inf])
xlim([0,amps(end)])
