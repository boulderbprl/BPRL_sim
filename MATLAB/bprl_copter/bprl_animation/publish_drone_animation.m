% --- fast_publisher.m ---
% 1. Initialize UDP (Use 'datagram' for speed)
u = udpport("datagram");
target_ip = "127.0.0.1";
target_port = 5101;

% Pre-allocate data array for speed: [x, y, z, roll, pitch, yaw]
data_to_send = zeros(1, 6);

fprintf('Publishing at high speed to port %d...\n', target_port);

% Simulation Loop
tic; 
while toc < 60  % Run for 60 seconds
    t = toc;
    
    % Update your physics/states
    x = 5 * cos(t);
    y = 5 * sin(t);
    z = 2 + sin(t/2);
    roll = 0.2 * sin(t);
    pitch = 0.2 * cos(t);
    yaw = t;

    % Fill array
    data_to_send = [x, y, z, roll, pitch, yaw];

    % 2. Write raw binary (Extremely fast)
    write(u, data_to_send, "single", target_ip, target_port);
    
    % Minimal pause to prevent CPU saturation (e.g., 200Hz)
    pause(0.005); 
end