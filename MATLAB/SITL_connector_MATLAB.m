% this function handles all the UDP connection to SITL using the TCP/UDP/IP
% Toolbox 2.0.6 by Peter Rydesäter
% https://uk.mathworks.com/matlabcentral/fileexchange/345-tcp-udp-ip-toolbox-2-0-6

function SITL_connector_MATLAB(state,target_function,init_function,physics_function,control_update,max_timestep)

% init physics
state = init_function(state);

% Init the UDP port

state.udpsocket = udpport("byte", "LocalPort", 0);
state.ip  = "127.0.0.1";
state.port = 25000;

disp("[drone_viz] UDP socket ready. Start drone_viz.py, then run your sim loop.");




update_time = tic;
update_count = 0;
physics_time_s = 0;

print_update_count = 1000; 


% Pre-allocate data array for speed: [x, y, z, roll, pitch, yaw]
%data_to_send = zeros(1, 6);


fprintf('SITL initialized\n'); 


while true

    % read in data from AP
    

    target = target_function(physics_time_s);
   
    [pwm_in,target] = control_update(target,state,state.delta_t);
    
    
    update_count = update_count + 1; 


    physics_time_s = physics_time_s + state.delta_t;

    
    % do a physics time step
    state = physics_function(pwm_in,state);
   
    % Report to AP  
    publish_animation(state,target);

    % print a fps and runtime update
    if rem(update_count,print_update_count) == 0
        total_time = toc(update_time);
        update_time = tic;
        time_ratio = (print_update_count*state.delta_t)/total_time;
        fprintf("%0.2f fps, %0.2f%% of realtime\n",print_update_count/total_time,time_ratio*100)
    end
end
end



function publish_animation(state, target)
    roll  = state.attitude(1);
    pitch = state.attitude(2);
    yaw   = state.attitude(3);

    roll_enu  =  pitch;
    pitch_enu = -roll;
    yaw_enu   =  pi/2 - yaw;

    q  = eul2quat([yaw_enu, pitch_enu, roll_enu], 'ZYX');
    qw = q(1); qx = q(2); qy = q(3); qz = q(4);

    x_enu =  state.position(2);
    y_enu =  state.position(1);
    z_enu = -state.position(3);

    tx_enu =  target.position(2);
    ty_enu =  target.position(1);
    tz_enu = -target.position(3);

    pkt = [x_enu, y_enu, z_enu, ...
           qw, qx, qy, qz, ...
           state.velocity(1), state.velocity(2), state.velocity(3), ...
           target.vx, target.vy, target.vz, ...
           rad2deg(roll), rad2deg(pitch), rad2deg(yaw), ...
           rad2deg(target.phi), rad2deg(target.theta), ...
           rad2deg(state.gyro(1)), rad2deg(state.gyro(2)), rad2deg(state.gyro(3)), ...
           rad2deg(target.p), rad2deg(target.q), rad2deg(target.r), ...
           tx_enu, ty_enu, tz_enu, rad2deg(target.yaw)];

    bytes = typecast(pkt, 'uint8');
    write(state.udpsocket, bytes, "uint8", state.ip, state.port);
end