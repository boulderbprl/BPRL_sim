
function plot_states(flight_data, time_bounds,plot_esc_data)
    % PLOT_STATES Plots all vehicle states and control inputs
    %
    % Inputs:
    %   flight_data - Struct containing flight data from import_ardupilot_data
    %   time_bounds - Optional [start_time, end_time] for x-axis limits
    %
    % States plotted: [x, y, z, u, v, w, p, q, r, phi, theta, psi]
    % Inputs plotted: [roll_input, pitch_input, yaw_input, thrust_input]
    
    % Set default time bounds if not provided
    if nargin < 2
        time_bounds = [min(flight_data.time), max(flight_data.time)];
    end
    if nargin < 3
        plot_esc_data = false;
    end
    
    %% Position plot (x, y, z)
    figure(1); set(gcf, 'Color', 'w');
        subplot(3,1,1);
        plot(flight_data.time, flight_data.states(:,1), 'r');
        grid on; ylabel('X Position [m]', 'FontWeight', 'bold');
        xlim(time_bounds); title('Vehicle Position');
        
        subplot(3,1,2);
        plot(flight_data.time, flight_data.states(:,2), 'g');
        grid on; ylabel('Y Position [m]', 'FontWeight', 'bold');
        xlim(time_bounds);
        
        subplot(3,1,3);
        plot(flight_data.time, flight_data.states(:,3), 'b');
        grid on; ylabel('Z Position [m]', 'FontWeight', 'bold');
        xlabel('Time [s]', 'FontWeight', 'bold'); xlim(time_bounds);
        
    %% Velocity plot (u, v, w)
    figure(2); set(gcf, 'Color', 'w');
        subplot(3,1,1);
        plot(flight_data.time, flight_data.states(:,4), 'r');
        grid on; ylabel('U Velocity [m/s]', 'FontWeight', 'bold');
        xlim(time_bounds); title('Vehicle Velocity (Body Frame)');
        
        subplot(3,1,2);
        plot(flight_data.time, flight_data.states(:,5), 'g');
        grid on; ylabel('V Velocity [m/s]', 'FontWeight', 'bold');
        xlim(time_bounds);
        
        subplot(3,1,3);
        plot(flight_data.time, flight_data.states(:,6), 'b');
        grid on; ylabel('W Velocity [m/s]', 'FontWeight', 'bold');
        xlabel('Time [s]', 'FontWeight', 'bold'); xlim(time_bounds);
    
    %% Angular rates plot (p, q, r)
    figure(3); set(gcf, 'Color', 'w');
        subplot(3,1,1);
        plot(flight_data.time, flight_data.states(:,7), 'r');
        grid on; ylabel('Roll Rate (p) [rad/s]', 'FontWeight', 'bold');
        xlim(time_bounds); title('Angular Rates');
        
        subplot(3,1,2);
        plot(flight_data.time, flight_data.states(:,8), 'g');
        grid on; ylabel('Pitch Rate (q) [rad/s]', 'FontWeight', 'bold');
        xlim(time_bounds);
        
        subplot(3,1,3);
        plot(flight_data.time, flight_data.states(:,9), 'b');
        grid on; ylabel('Yaw Rate (r) [rad/s]', 'FontWeight', 'bold');
        xlabel('Time [s]', 'FontWeight', 'bold'); xlim(time_bounds);
    
    %% Attitude plot (phi, theta, psi)
    figure(4); set(gcf, 'Color', 'w');
        subplot(3,1,1);
        plot(flight_data.time, unwrap(flight_data.states(:,10)), 'r');
        grid on; ylabel('Roll (phi) [deg]', 'FontWeight', 'bold');
        xlim(time_bounds); title('Vehicle Attitude');
        
        subplot(3,1,2);
        plot(flight_data.time, unwrap(flight_data.states(:,11)), 'g');
        grid on; ylabel('Pitch (theta) [deg]', 'FontWeight', 'bold');
        xlim(time_bounds);
        
        subplot(3,1,3);
        plot(flight_data.time, unwrap(flight_data.states(:,12)), 'b');
        grid on; ylabel('Yaw (psi) [deg]', 'FontWeight', 'bold');
        xlabel('Time [s]', 'FontWeight', 'bold'); xlim(time_bounds);
    
    
    %% Control inputs plot
     i=5;
   if plot_esc_data

    figure(i); set(gcf, 'Color', 'w');
        subplot(2,2,1);
        plot(flight_data.time, flight_data.esc_inputs(:,1), 'r');
        grid on; ylabel('ESC 1 (RPM)', 'FontWeight', 'bold');
        xlim(time_bounds); title('ESCs RPM');
        
        subplot(2,2,2);
        plot(flight_data.time, flight_data.esc_inputs(:,2), 'g');
        grid on; ylabel('ESC 2 (RPM)', 'FontWeight', 'bold');
        xlim(time_bounds);
        
        subplot(2,2,3);
        plot(flight_data.time, flight_data.esc_inputs(:,3), 'b');
        grid on; ylabel('ESC 3 (RPM)', 'FontWeight', 'bold');
        xlabel('Time [s]', 'FontWeight', 'bold'); xlim(time_bounds);
        
        subplot(2,2,4);
        plot(flight_data.time, flight_data.esc_inputs(:,4), 'k');
        grid on; ylabel('ESC 4 (RPM)', 'FontWeight', 'bold');
        xlabel('Time [s]', 'FontWeight', 'bold'); xlim(time_bounds);
    
    fprintf('Plotted ESC inputs successfully!\n');
           i=i+1;
   end

    figure(i); set(gcf, 'Color', 'w');
        subplot(2,2,1);
        plot(flight_data.time, flight_data.inputs(:,1), 'r');
        grid on; ylabel('Roll Input', 'FontWeight', 'bold');
        xlim(time_bounds); title('Control Inputs');
        
        subplot(2,2,2);
        plot(flight_data.time, flight_data.inputs(:,2), 'g');
        grid on; ylabel('Pitch Input', 'FontWeight', 'bold');
        xlim(time_bounds);
        
        subplot(2,2,3);
        plot(flight_data.time, flight_data.inputs(:,3), 'b');
        grid on; ylabel('Yaw Input', 'FontWeight', 'bold');
        xlabel('Time [s]', 'FontWeight', 'bold'); xlim(time_bounds);
        
        subplot(2,2,4);
        plot(flight_data.time, flight_data.inputs(:,4), 'k');
        grid on; ylabel('Thrust Input', 'FontWeight', 'bold');
        xlabel('Time [s]', 'FontWeight', 'bold'); xlim(time_bounds);
    
    fprintf('State and input plots completed successfully!\n');

  
end
