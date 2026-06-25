%% drone_viz_init.m
% Run once before your sim loop to set up the UDP connection.
% Requires MATLAB R2020b+ (udpport)

udpSock = udpport("byte", "LocalPort", 0);
vizIP   = "127.0.0.1";
vizPort = 25000;

disp("[drone_viz] UDP socket ready. Start drone_viz.py, then run your sim loop.");

%% In your sim loop, call this each timestep:
%
%   drone_viz_send(udpSock, vizIP, vizPort, x, y, z, qw, qx, qy, qz);
%
% where:
%   x, y, z       — position in metres
%   qw, qx, qy, qz — unit quaternion (world frame)


%% drone_viz_send.m  (save as a separate file or paste inline)
function drone_viz_send(sock, ip, port, x, y, z, qw, qx, qy, qz)
    pkt = [x, y, z, qw, qx, qy, qz];            % 1x7 double
    bytes = typecast(pkt, 'uint8');               % 56 bytes, little-endian
    write(sock, bytes, "uint8", ip, port);
end


%% Minimal sim loop example
% (replace physics/controller with your own)

%% Spiral trajectory test

dt     = 0.02;    % 50 Hz
T      = 20;      % total seconds
r      = 2.0;     % spiral radius
climb  = 0.1;     % metres per second ascent

for i = 0:dt:T
    % Position — expanding helix
    theta = i * 0.8;                  % angular rate (rad/s)
    x = r * cos(theta);
    y = r * sin(theta);
    z = 0.5 + climb * i;

    % Yaw to face direction of travel, level flight
    yaw   = theta + pi/2;
    roll  = 0;
    pitch = 0;

    % RPY to quaternion
    qw = cos(roll/2)*cos(pitch/2)*cos(yaw/2) ...
       + sin(roll/2)*sin(pitch/2)*sin(yaw/2);
    qx = sin(roll/2)*cos(pitch/2)*cos(yaw/2) ...
       - cos(roll/2)*sin(pitch/2)*sin(yaw/2);
    qy = cos(roll/2)*sin(pitch/2)*cos(yaw/2) ...
       + sin(roll/2)*cos(pitch/2)*sin(yaw/2);
    qz = cos(roll/2)*cos(pitch/2)*sin(yaw/2) ...
       - sin(roll/2)*sin(pitch/2)*cos(yaw/2);

    % Send to PyBullet
    pkt = [x, y, z, qw, qx, qy, qz];
    write(udpSock, typecast(pkt, 'uint8'), "uint8", vizIP, vizPort);

    pause(dt);
end