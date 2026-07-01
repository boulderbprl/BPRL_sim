%close all 
close all 
clear all 

addpath(genpath('../../MATLAB'))

% Physics of a multi copter

% load in the parameters for a frame
try
    state = load('rtos_drone','copter');
    load('./sysid/flight_data/2026-06-28_4SR_Flight.mat')
catch
     run('rtos_drone.m')
    fprintf('Could not find rtos_drone.mat file, running copter.m\n')
    return
end

% Setup environmental conditions
state.environment.density = 1.225; % (kg/m^3)
state.gravity_mss = 9.80665; % (m/s^2)

state.copter.max_motor_torque = 0.105;
state.copter.max_motor_arm_moment_roll = 0.795;
state.copter.max_motor_arm_moment_pitch = 0.948;

% Setup the time step size for the Physics model

state.delta_t = dt; 


% define init and time setup functions
init_function = @init;
physics_function = @physics_step;


% setup connection
history = run_sim_finite(state, flight_data, init_function, physics_function);

%%
plot_history(history);

% Simulator model must take and return a structure with the felids: 
% gyro(roll, pitch, yaw) (radians/sec) body frame
% attitude(roll, pitch yaw) (radians)
% accel(north, east, down) (m/s^2) body frame
% velocity(north, east,down) (m/s) earth frame
% position(north, east, down) (m) earth frame 
% the structure can have any other fields required for the physics model

% init values
% flight_data.states = [position (1-3), velocity_body(4-6), angular_rates(7-9), attitude(10-12)];
    
function state = init(state,flight_data)
    for i = 1:numel(state.copter.motors)
        state.copter.motors(i).rpm     = flight_data.esc_inputs(1,state.copter.motors(i).channel)';
       % state.copter.motors(i).pwm     = 0;
        % state.copter.motors(i).current = 0;
        state.copter.motors(i).moment_roll = 0;
        state.copter.motors(i).moment_pitch = 0;
        state.copter.motors(i).moment_yaw = 0;
    end
    state.gyro      = flight_data.states(1,7:9)';
    state.dcm       = flight_data.dcm_init';
    state.attitude  =  flight_data.states(1,10:12)';   % roll, pitch, yaw (rad)
    state.accel     = flight_data.acceleration(1,1:3)';   % body frame (m/s^2)
    state.velocity  = flight_data.velocity_global(1,1:3)';   % NED (m/s)
    state.position  = flight_data.states(1,1:3)';   % NED (m)
    state.bf_velo   = flight_data.states(1,4:6)';   % body frame velocity (m/s)
    state.rot_accel = [flight_data.acceleration(1,4:5)';0];
    % Clear persistent controller so stale integrators don't survive
    % between MATLAB runs (clearvars does NOT clear handle object state)
    state.controller.tau_roll = 0; 
    state.controller.tau_pitch = 0; 
    state.controller.tau_yaw = 0;

end


% compute thrust from PWM
function [thrust,torque] = fit_rpm_to_thrust_torque(rpm)
% poly4 fit from sysid mean_fit       
%mean_fit = 1546;
%std_fit = 255.6;
%pwm_in = (pwm_in - state.copter.motor_model.pwm_mean)/state.copter.motor_model.pwm_std;

rpm_norm = (rpm-2545)/897.3;

x_rpm = [rpm_norm^2;rpm_norm;1];
thrust = [0.4038,1.9926,2.5993]*x_rpm; 

torque = 0.0311*((thrust - 2.893)/1.984) + 0.0463;

end

% % Take a physics time step
% function state = physics_step(pwm_in,state)
% 
% % Calculate the torque and thrust, assume RPM is last step value
% for i = 1:numel(state.copter.motors)
%     motor = state.copter.motors(i);
% 
%     % Calculate the throttle
%     pwm = pwm_in(motor.channel); 
%     pwm_norm = (pwm- 1550) / 212.5; %pwm_in = (pwm_in - mean_fit)/std_fit;
% 
%     rpm = 894.5337*pwm_norm + 2525.5;
% 
%     % Calculate the thrust and torque
%     [thrust_pwm,torque_pwm] = fit_rpm_to_thrust_torque(rpm);
% 
% 
%     thrust = thrust_pwm;
%     torque = torque_pwm;
% 
%     % calculate resulting moments
%     moment_roll = thrust * motor.location(1);
%     moment_pitch = thrust * motor.location(2);
%     moment_yaw = -torque * motor.direction;
% 
%     % sprintf("Motor i:%d Thrust: %0.4f",i,thrust)
% 
%     % Update main structure
%     state.copter.motors(i).torque = torque;
%     state.copter.motors(i).thrust = thrust;
%     state.copter.motors(i).rpm = rpm;
%     state.copter.motors(i).pwm = pwm;
%     state.copter.motors(i).moment_roll = moment_roll;
%     state.copter.motors(i).moment_pitch = moment_pitch;
%     state.copter.motors(i).moment_yaw = moment_yaw;
% end
% 
% 
% drag = sign(state.bf_velo) .* state.copter.cd .* state.copter.cd_ref_area .* 0.5 .* state.environment.density .* state.bf_velo.^2;
% 
% % Calculate the forces about the CG (N,E,D) (body frame)
% force = [0;0;-sum([state.copter.motors.thrust])] - drag;
% 
% % estimate rotational drag
% rotational_drag = 0.2 * sign(state.gyro) .* state.gyro.^2; % estimated to give a reasonable max rotation rate
% 
% % Update attitude, moments to rotational acceleration to rotational velocity to attitude
% moments = [-sum([state.copter.motors.moment_roll]);sum([state.copter.motors.moment_pitch]);sum([state.copter.motors.moment_yaw])] - rotational_drag;
% 
% state = update_dynamics(state,force,moments);
% 
% end

% Take a physics time step
function state = physics_step(rpm_in,state)

% Calculate the torque and thrust, assume RPM is last step value
for i = 1:numel(state.copter.motors)
    motor = state.copter.motors(i);
    
    rpm = rpm_in(motor.channel);
   
    % Calculate the thrust and torque
    [thrust_pwm,torque_pwm] = fit_rpm_to_thrust_torque(rpm);
    
   
    thrust = thrust_pwm;
    torque = torque_pwm;

    % calculate resulting moments
    moment_roll = thrust * motor.location(1);
    moment_pitch = thrust * motor.location(2);
    moment_yaw = -torque * motor.direction;
    
    % sprintf("Motor i:%d Thrust: %0.4f",i,thrust)
    
    % Update main structure
    state.copter.motors(i).torque = torque;
    state.copter.motors(i).thrust = thrust;
    state.copter.motors(i).rpm = rpm;
    %state.copter.motors(i).pwm = pwm;
    state.copter.motors(i).moment_roll = moment_roll;
    state.copter.motors(i).moment_pitch = moment_pitch;
    state.copter.motors(i).moment_yaw = moment_yaw;
end


drag = sign(state.bf_velo) .* state.copter.cd .* state.copter.cd_ref_area .* 0.5 .* state.environment.density .* state.bf_velo.^2;

% Calculate the forces about the CG (N,E,D) (body frame)
force = [0;0;-sum([state.copter.motors.thrust])] - drag;

% estimate rotational drag
rotational_drag = 0.2 * sign(state.gyro) .* state.gyro.^2; % estimated to give a reasonable max rotation rate

% Update attitude, moments to rotational acceleration to rotational velocity to attitude
moments = [-sum([state.copter.motors.moment_roll]);sum([state.copter.motors.moment_pitch]);sum([state.copter.motors.moment_yaw])] - rotational_drag;

state.controller.tau_roll = moments(1)/state.copter.max_motor_arm_moment_roll;
state.controller.tau_pitch = moments(2)/state.copter.max_motor_arm_moment_pitch; 
state.controller.tau_yaw = moments(3)/state.copter.max_motor_torque;

state = update_dynamics(state,force,moments);

end

% integrate the acceleration resulting from the forces and moments to get the
% new state
function state = update_dynamics(state,force,moments)

rot_accel = (moments' / state.copter.inertia)';
state.rot_accel = rot_accel;

state.gyro = state.gyro + rot_accel * state.delta_t;

% Constrain to 2000 deg per second, this is what typical sensors max out at
state.gyro = max(state.gyro,deg2rad(-2000));
state.gyro = min(state.gyro,deg2rad(2000));

% update the dcm and attitude
[state.dcm, state.attitude] = rotate_dcm(state.dcm,state.gyro * state.delta_t);

% body frame accelerations
state.accel = force / state.copter.mass;

% earth frame accelerations (NED)
accel_ef = state.dcm * state.accel;
accel_ef(3) = accel_ef(3) + state.gravity_mss;


% if we're on the ground, then our vertical acceleration is limited
% to zero. This effectively adds the force of the ground on the aircraft
if state.position(3) >= 0 && accel_ef(3) > 0
    accel_ef(3) = 0;
end

% work out acceleration as seen by the accelerometers. It sees the kinematic
% acceleration (ie. real movement), plus gravity
state.accel = state.dcm' * (accel_ef + [0; 0; -state.gravity_mss]);

state.velocity = state.velocity + accel_ef * state.delta_t;
state.position = state.position + state.velocity * state.delta_t;

% make sure we can't go underground (NED so underground is positive)
if state.position(3) >= 0
    state.position(3) = 0;
    state.velocity = [0;0;0];
    state.gyro = [0;0;0];
end

% calculate the body frame velocity for drag calculation
state.bf_velo = state.dcm' * state.velocity;
%sprintf("Gyro: [%0.4f,%0.4f,%0.4f]",state.gyro(1),state.gyro(2),state.gyro(3))
end

function [dcm, euler] = rotate_dcm(dcm, ang)

% rotate
delta = [dcm(1,2) * ang(3) - dcm(1,3) * ang(2),         dcm(1,3) * ang(1) - dcm(1,1) * ang(3),      dcm(1,1) * ang(2) - dcm(1,2) * ang(1);
         dcm(2,2) * ang(3) - dcm(2,3) * ang(2),         dcm(2,3) * ang(1) - dcm(2,1) * ang(3),      dcm(2,1) * ang(2) - dcm(2,2) * ang(1);
         dcm(3,2) * ang(3) - dcm(3,3) * ang(2),         dcm(3,3) * ang(1) - dcm(3,1) * ang(3),      dcm(3,1) * ang(2) - dcm(3,2) * ang(1)];

dcm = dcm + delta;

% normalise
a = dcm(1,:);
b = dcm(2,:);
error = a * b';
t0 = a - (b *(0.5 * error));
t1 = b - (a *(0.5 * error));
t2 = cross(t0,t1);
dcm(1,:) = t0 * (1/norm(t0));
dcm(2,:) = t1 * (1/norm(t1));
dcm(3,:) = t2 * (1/norm(t2));

% calculate euler angles
euler = [atan2(dcm(3,2),dcm(3,3)); -asin(dcm(3,1)); atan2(dcm(2,1),dcm(1,1))]; 

end






% flight_data.states = [position (1-3), velocity_body(4-6), angular_rates(7-9), attitude(10-12)];
    
function history = run_sim_finite(state,flight_data, init_function, physics_function)
    
    % initialize states from flight_data
    state = init_function(state,flight_data);

   
    n_steps = length(flight_data.time);

    % Pre-allocate — vehicle states
    t_log   = zeros(1, n_steps);
    pos_log = zeros(3, n_steps);
    vel_log = zeros(3, n_steps);
    att_log = zeros(3, n_steps);
    gyr_log = zeros(3, n_steps);
    acc_log = zeros(3, n_steps);
    rot_acc_log = zeros(3, n_steps);



    % Pre-allocate — controller torques (physical units, N·m)
    tau_log = zeros(3, n_steps);        % roll, pitch, yaw

    physics_time_s = 0;

    % % Seed controller fields so logging doesn't fail on step 1
    % state.controller.tau_roll  = 0;
    % state.controller.tau_pitch = 0;
    % state.controller.tau_yaw   = 0;

    for k = 1:n_steps
       
       
        % ── Vehicle states ────────────────────────────────────────────
        t_log(k)       = physics_time_s;
        pos_log(:,k)   = state.position;
        vel_log(:,k)   = state.velocity;
        att_log(:,k)   = rad2deg(state.attitude);
        gyr_log(:,k)   = rad2deg(state.gyro);
        acc_log(:,k)   = state.accel;
        rot_acc_log(:,k) = rad2deg(state.rot_accel);
    
        % flight_data.states = [position (1-3), velocity_body(4-6), angular_rates(7-9), attitude(10-12)];
    

        

        % ── Controller torques ────────────────────────────────────────
        tau_log(:,k) = [state.controller.tau_roll; ...
                        state.controller.tau_pitch; ...
                        state.controller.tau_yaw];

        physics_time_s = physics_time_s + state.delta_t;

         rpm_in = flight_data.esc_inputs(k,:)';

      %  state = physics_function(rpm_in, state);
        
        if mod(k,100)==0
           plot(tau_log(:,1:k)')
           hold on 
           plot(flight_data.inputs(:,1:k));
           hold off

        end

    end

    % ── Targets ───────────────────────────────────────────────────
        tgt_pos_log   = flight_data.states(:,1:3)';
        tgt_vel_log   = flight_data.velocity_global(:,1:3)';
        tgt_att_log   = rad2deg(flight_data.states(:,10:12)');
        tgt_rate_log = rad2deg(flight_data.states(:,7:9)');
        tgt_accel_log = rad2deg(flight_data.acceleration');
        tgt_tau_log = flight_data.inputs'; 


    history.t        = t_log;
    history.pos      = pos_log;
    history.vel      = vel_log;
    history.att      = att_log;
    history.gyr      = gyr_log;
    history.acc      = acc_log;
    history.rot_acc  = rot_acc_log;
    history.tgt_pos  = tgt_pos_log;
    history.tgt_vel  = tgt_vel_log;
    history.tgt_att  = tgt_att_log;
    history.tgt_rate = tgt_rate_log;
    history.tgt_accel= tgt_accel_log;
    history.tau      = tau_log;
    history.tgt_tau = tgt_tau_log;
end

function plot_history(h)
    BG  = [0.13 0.13 0.15];
    AXC = [0.20 0.20 0.23];
    TC  = [0.90 0.90 0.90];
    cols     = {[0.35 0.75 0.95], [0.45 0.90 0.55], [0.95 0.60 0.35]};
    cols_tgt = {[0.20 0.48 0.65], [0.25 0.58 0.32], [0.68 0.38 0.18]};

    % Each row: {title, actual data, target data or [], actual labels, target labels}
    panels = { ...
        'Position (m)', ...
            h.pos,      h.tgt_pos, ...
            {'N','E','D'}, {'N tgt','E tgt','D tgt'}; ...
        'Velocity (m/s)', ...
            h.vel,      h.tgt_vel, ...
            {'vN','vE','vD'}, {'vN tgt','vE tgt','vD tgt'}; ...
        'Acceleration body (m/s²)', ...
            h.acc,      [], ...
            {'aN','aE','aD'}, {}; ...
        'Attitude (deg)', ...
            h.att,      h.tgt_att, ...
            {'Roll','Pitch','Yaw'}, {'\phi tgt','\theta tgt','\psi tgt'}; ...
        'Gyro / Rate cmd (deg/s)', ...
            h.gyr,      h.tgt_rate, ...
            {'p','q','r'}, {'p cmd','q cmd','r cmd'}; ...
        'Rot Accel / Accel cmd (deg/s²)', ...
            h.rot_acc,  h.tgt_accel, ...
            {'\alpha_p','\alpha_q','\alpha_r'}, {'\alphap cmd','\alphaq cmd','\alphar cmd'}; ...
        'Controller torques \tau (N·m)', ...
            h.tau,      [], ...
            {'\tau_{roll}','\tau_{pitch}','\tau_{yaw}'}, {}; ...
    };

    n = size(panels,1);
    axes_h = gobjects(n,1);

    % Tile figure positions across the screen instead of stacking subplots
    fig_w = 700; fig_h = 320;
    cols_per_row = 2;
    x0 = 50; y0 = 60; xgap = 30; ygap = 60;

    for p = 1:n
        fig_name = ['Sim Results - ' panels{p,1}];
        fh = findobj('Type','figure','Name',fig_name);
        if isempty(fh)
            row = floor((p-1)/cols_per_row);
            col = mod(p-1, cols_per_row);
            xpos = x0 + col*(fig_w + xgap);
            ypos = y0 + row*(fig_h + ygap);
            fh = figure('Name',fig_name,'NumberTitle','off', ...
                'Position',[xpos ypos fig_w fig_h], ...
                'Color',BG);
        else
            fh = fh(1);
            clf(fh);
            set(fh,'Color',BG);
        end

        ax = axes('Parent',fh,'Position',[0.08 0.15 0.68 0.75]);
        axes_h(p) = ax;
        set(ax,'Color',AXC,'XColor',TC,'YColor',TC,'FontSize',8, ...
            'GridColor',[0.4 0.4 0.4],'GridAlpha',0.35,'TickDir','out');
        grid(ax,'on'); hold(ax,'on'); box(ax,'off');

        actual = panels{p,2};
        target = panels{p,3};
        al     = panels{p,4};
        tl     = panels{p,5};

        % Actual — solid
        for ch = 1:3
            plot(ax, h.t, actual(ch,:), '-', ...
                'Color', cols{ch}, 'LineWidth', 1.4, ...
                'DisplayName', al{ch});
        end

        % Target — dashed, darker shade of same colour
        if ~isempty(target)
            for ch = 1:3
                plot(ax, h.t, target(ch,:), '--', ...
                    'Color', cols_tgt{ch}, 'LineWidth', 1.0, ...
                    'DisplayName', tl{ch});
            end
        end

        title(ax, panels{p,1}, 'Color',TC,'FontSize',9,'FontWeight','bold');
        leg = legend(ax,'show','Location','eastoutside','FontSize',7);
        leg.TextColor = TC;
        leg.Color     = AXC;
        leg.EdgeColor = [0.35 0.35 0.38];

        xlabel(ax,'Time (s)','Color',TC,'FontSize',9);
    end

    linkaxes(axes_h,'x');
end