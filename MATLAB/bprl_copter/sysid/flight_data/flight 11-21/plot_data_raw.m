
function plot_data_raw(flight_data, time_bounds)
    % PLOT_ACCELERATION Plots angular acceleration data (p_dot and q_dot)
    %
    % Inputs:
    %   flight_data - Struct containing acceleration data from import_ardupilot_data
    %   time_bounds - Optional [start_time, end_time] for x-axis limits
    %
    % The function plots p_dot (roll acceleration) and q_dot (pitch acceleration)
    % from both strain and IMU sources for comparison
    
    % Extract data from flight_data struct
    globalTime = flight_data.time;
    
    % Extract p_dot and q_dot from acceleration array
    % flight_data.acceleration = [ax, ay, az, p_dot, q_dot]
    p_dot_imu = flight_data.acceleration(:,4);  % Roll acceleration from IMU
    q_dot_imu = flight_data.acceleration(:,5);  % Pitch acceleration from IMU
    
    % Set default time bounds if not provided
    if nargin < 2
        time_bounds = [min(globalTime), max(globalTime)];
    end
    
    %% Plot angular accelerations
    figure(1); set(gcf, 'Color', 'w');
    
        % Plot p_dot (roll acceleration)
        subplot(2,1,1);
        plot(globalTime, p_dot_imu, 'r'); hold on;
        plot(flight_data.time, flight_data.inputs(:,1) *100, 'k:');
        hold on; grid on;
        ylabel('Roll Accel IMU [rad/s²]');
        legend('Roll Accel','Roll Command *100', 'Location', 'best');
        xlim(time_bounds);
        title('Roll & Pitch Acceleration IMU');
        
        subplot(2,1,2);
        plot(globalTime, q_dot_imu, 'b'); hold on;
        plot(flight_data.time, flight_data.inputs(:,2) *100, 'k:');
        hold on; grid on;
        ylabel('Pitch Accel IMU [rad/s²]');
        legend('Pitch Accel','Pitch Command *100', 'Location', 'best');
        xlim(time_bounds);
        xlabel('Time [s]');

                
    %% Angular rates plot (p, q, r)
    figure(2); set(gcf, 'Color', 'w');
        subplot(2,1,1);
        plot(flight_data.time, flight_data.states(:,7), 'r');
        grid on; ylabel('Roll Rate (p) [rad/s]', 'FontWeight', 'bold');
        xlim(time_bounds); title('Angular Rates');
        title('Roll & Pitch Rate IMU');

        subplot(2,1,2);
        plot(flight_data.time, flight_data.states(:,8), 'g');
        grid on; ylabel('Pitch Rate (q) [rad/s]', 'FontWeight', 'bold');
        xlim(time_bounds);
        xlabel('Time [s]', 'FontWeight', 'bold'); 
   
    %% inouts
    figure(3); set(gcf, 'Color', 'w');
        subplot(2,1,1);
        plot(flight_data.time, flight_data.inputs(:,1), 'r');
        grid on; ylabel('Roll Input', 'FontWeight', 'bold');
        xlim(time_bounds); title('Control Inputs');
        title('Roll & Pitch Control Inputs');
        
        subplot(2,1,2);
        plot(flight_data.time, flight_data.inputs(:,2), 'g');
        grid on; ylabel('Pitch Input', 'FontWeight', 'bold');
        xlim(time_bounds);
        xlabel('Time [s]', 'FontWeight', 'bold');
    
    fprintf('Acceleration plots completed successfully!\n');
end

