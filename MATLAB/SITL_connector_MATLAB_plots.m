% this function handles all the UDP connection to SITL using the TCP/UDP/IP
% Toolbox 2.0.6 by Peter Rydesäter
% https://uk.mathworks.com/matlabcentral/fileexchange/345-tcp-udp-ip-toolbox-2-0-6
function SITL_connector_MATLAB(state,target_function,init_function,physics_function,control_update,max_timestep)

% init physics
state = init_function(state);

% Init the UDP port
state.udpsocket = udpport("byte", "LocalPort", 0);
state.ip   = "127.0.0.1";
state.port = 25000;
disp("[drone_viz] UDP socket ready. Start drone_viz.py, then run your sim loop.");

update_time        = tic;
update_count       = 0;
physics_time_s     = 0;
print_update_count = 1000;

% ── Live-plot setup ───────────────────────────────────────────────────────
PLOT_EVERY   = 50;           % update plot every N physics steps (~8 Hz @ 400 Hz)
HISTORY_LEN  = 2000;         % rolling window length (samples)

% Ring-buffer pre-allocation
h = init_live_plot(HISTORY_LEN);

fprintf('SITL initialized\n');

while true

    % ── Simulation step ──────────────────────────────────────────────────
    target       = target_function(physics_time_s);
    pwm_in       = control_update(target, state, state.delta_t);
    update_count = update_count + 1;
    physics_time_s = physics_time_s + state.delta_t;
    state        = physics_function(pwm_in, state);

    % ── Publish to visualiser ─────────────────────────────────────────────
    publish_animation(state);

    % ── Live plot update ──────────────────────────────────────────────────
    if rem(update_count, PLOT_EVERY) == 0
        h = update_live_plot(h, physics_time_s, state, target);
    end

    % ── Console FPS report ────────────────────────────────────────────────
    if rem(update_count, print_update_count) == 0
        total_time = toc(update_time);
        update_time = tic;
        time_ratio  = (print_update_count * state.delta_t) / total_time;
        fprintf("%0.2f fps, %0.2f%% of realtime\n", ...
                print_update_count/total_time, time_ratio*100);
    end
end
end


% =========================================================================
%  LIVE PLOT — initialise figure and return handle struct
% =========================================================================
function h = init_live_plot(N)

    fig = figure('Name','SITL Live Monitor','NumberTitle','off', ...
                 'Position',[50 50 1200 800], ...
                 'Color',[0.13 0.13 0.13]);

    % ── 3-D trajectory (top-left, spans two rows) ─────────────────────────
    h.ax3d = subplot(3,3,[1 4]);
    hold(h.ax3d,'on'); grid(h.ax3d,'on'); view(h.ax3d,45,30);
    set(h.ax3d,'Color',[0.15 0.15 0.15],'XColor','w','YColor','w','ZColor','w');
    xlabel(h.ax3d,'East (m)'); ylabel(h.ax3d,'North (m)'); zlabel(h.ax3d,'Up (m)');
    title(h.ax3d,'3-D Trajectory','Color','w');
    h.ln3d_state  = plot3(h.ax3d, NaN,NaN,NaN, 'c-',  'LineWidth',1.2);
    h.ln3d_target = plot3(h.ax3d, NaN,NaN,NaN, 'm--', 'LineWidth',1.0);
    h.pt3d_state  = plot3(h.ax3d, NaN,NaN,NaN, 'co',  'MarkerFaceColor','c','MarkerSize',7);
    h.pt3d_target = plot3(h.ax3d, NaN,NaN,NaN, 'm^',  'MarkerFaceColor','m','MarkerSize',7);
    legend(h.ax3d,{'State','Target'},'TextColor','w','Color',[0.2 0.2 0.2]);

    % ── Position North ────────────────────────────────────────────────────
    h.ax_N = subplot(3,3,2);
    hold(h.ax_N,'on'); grid(h.ax_N,'on');
    set(h.ax_N,'Color',[0.15 0.15 0.15],'XColor','w','YColor','w');
    title(h.ax_N,'North (m)','Color','w'); ylabel(h.ax_N,'m');
    h.ln_N_state  = plot(h.ax_N, NaN,NaN,'c-');
    h.ln_N_target = plot(h.ax_N, NaN,NaN,'m--');

    % ── Position East ─────────────────────────────────────────────────────
    h.ax_E = subplot(3,3,3);
    hold(h.ax_E,'on'); grid(h.ax_E,'on');
    set(h.ax_E,'Color',[0.15 0.15 0.15],'XColor','w','YColor','w');
    title(h.ax_E,'East (m)','Color','w'); ylabel(h.ax_E,'m');
    h.ln_E_state  = plot(h.ax_E, NaN,NaN,'c-');
    h.ln_E_target = plot(h.ax_E, NaN,NaN,'m--');

    % ── Altitude (Up = -Down NED) ─────────────────────────────────────────
    h.ax_alt = subplot(3,3,5);
    hold(h.ax_alt,'on'); grid(h.ax_alt,'on');
    set(h.ax_alt,'Color',[0.15 0.15 0.15],'XColor','w','YColor','w');
    title(h.ax_alt,'Altitude (m)','Color','w'); ylabel(h.ax_alt,'m');
    h.ln_alt_state  = plot(h.ax_alt, NaN,NaN,'c-');
    h.ln_alt_target = plot(h.ax_alt, NaN,NaN,'m--');

    % ── Roll ──────────────────────────────────────────────────────────────
    h.ax_roll = subplot(3,3,6);
    hold(h.ax_roll,'on'); grid(h.ax_roll,'on');
    set(h.ax_roll,'Color',[0.15 0.15 0.15],'XColor','w','YColor','w');
    title(h.ax_roll,'Roll (deg)','Color','w'); ylabel(h.ax_roll,'deg');
    h.ln_roll = plot(h.ax_roll, NaN,NaN,'r-');

    % ── Pitch ─────────────────────────────────────────────────────────────
    h.ax_pitch = subplot(3,3,7);
    hold(h.ax_pitch,'on'); grid(h.ax_pitch,'on');
    set(h.ax_pitch,'Color',[0.15 0.15 0.15],'XColor','w','YColor','w');
    title(h.ax_pitch,'Pitch (deg)','Color','w'); ylabel(h.ax_pitch,'deg'); xlabel(h.ax_pitch,'Time (s)');
    h.ln_pitch = plot(h.ax_pitch, NaN,NaN,'g-');

    % ── Yaw ───────────────────────────────────────────────────────────────
    h.ax_yaw = subplot(3,3,8);
    hold(h.ax_yaw,'on'); grid(h.ax_yaw,'on');
    set(h.ax_yaw,'Color',[0.15 0.15 0.15],'XColor','w','YColor','w');
    title(h.ax_yaw,'Yaw (deg)','Color','w'); ylabel(h.ax_yaw,'deg'); xlabel(h.ax_yaw,'Time (s)');
    h.ln_yaw       = plot(h.ax_yaw, NaN,NaN,'y-');
    h.ln_yaw_target= plot(h.ax_yaw, NaN,NaN,'m--');

    % ── Position error magnitude ───────────────────────────────────────────
    h.ax_err = subplot(3,3,9);
    hold(h.ax_err,'on'); grid(h.ax_err,'on');
    set(h.ax_err,'Color',[0.15 0.15 0.15],'XColor','w','YColor','w');
    title(h.ax_err,'Position Error (m)','Color','w'); ylabel(h.ax_err,'m'); xlabel(h.ax_err,'Time (s)');
    h.ln_err = plot(h.ax_err, NaN,NaN,'w-');

    % ── Ring buffers ──────────────────────────────────────────────────────
    nan_col = NaN(N,1);
    h.t          = nan_col;
    h.pos_state  = NaN(N,3);   % N E D
    h.pos_target = NaN(N,3);
    h.att        = NaN(N,3);   % roll pitch yaw (deg)
    h.yaw_target = nan_col;
    h.err        = nan_col;

    h.idx = 0;   % write pointer (wraps around N)
    h.N   = N;
    h.fig = fig;
end


% =========================================================================
%  LIVE PLOT — push one sample and refresh graphics
% =========================================================================
function h = update_live_plot(h, t, state, target)

    % bail out silently if the user closed the window
    if ~isvalid(h.fig); return; end

    % ── Push into ring buffer ─────────────────────────────────────────────
    h.idx = mod(h.idx, h.N) + 1;
    i = h.idx;

    h.t(i)           = t;
    h.pos_state(i,:) = state.position';          % NED
    h.pos_target(i,:)= target.position';
    h.att(i,:)       = rad2deg(state.attitude');  % roll pitch yaw
    h.yaw_target(i)  = rad2deg(target.yaw);
    h.err(i)         = norm(state.position - target.position);

    % ── Build ordered index for time-series plots (oldest → newest) ───────
    if h.t(mod(h.idx, h.N)+1) > 0     % buffer has wrapped
        idx_ord = [h.idx+1:h.N , 1:h.idx];
    else
        idx_ord = 1:h.idx;             % not yet full
    end

    tt  = h.t(idx_ord);

    % ── Convert NED to plot-friendly axes ─────────────────────────────────
    % 3-D: X=East, Y=North, Z=Up  (flip D sign)
    N_s = h.pos_state(idx_ord,1);
    E_s = h.pos_state(idx_ord,2);
    U_s = -h.pos_state(idx_ord,3);

    N_t = h.pos_target(idx_ord,1);
    E_t = h.pos_target(idx_ord,2);
    U_t = -h.pos_target(idx_ord,3);

    att = h.att(idx_ord,:);
    yt  = h.yaw_target(idx_ord);
    err = h.err(idx_ord);

    % ── 3-D trajectory ────────────────────────────────────────────────────
    set(h.ln3d_state,  'XData',E_s,'YData',N_s,'ZData',U_s);
    set(h.ln3d_target, 'XData',E_t,'YData',N_t,'ZData',U_t);
    set(h.pt3d_state,  'XData',E_s(end),'YData',N_s(end),'ZData',U_s(end));
    set(h.pt3d_target, 'XData',E_t(end),'YData',N_t(end),'ZData',U_t(end));

    % ── Time-series ───────────────────────────────────────────────────────
    set(h.ln_N_state,   'XData',tt,'YData',N_s);
    set(h.ln_N_target,  'XData',tt,'YData',N_t);

    set(h.ln_E_state,   'XData',tt,'YData',E_s);
    set(h.ln_E_target,  'XData',tt,'YData',E_t);

    set(h.ln_alt_state,  'XData',tt,'YData',U_s);
    set(h.ln_alt_target, 'XData',tt,'YData',U_t);

    set(h.ln_roll,  'XData',tt,'YData',att(:,1));
    set(h.ln_pitch, 'XData',tt,'YData',att(:,2));
    set(h.ln_yaw,        'XData',tt,'YData',att(:,3));
    set(h.ln_yaw_target, 'XData',tt,'YData',yt);

    set(h.ln_err, 'XData',tt,'YData',err);

    % ── Flush to screen ───────────────────────────────────────────────────
    drawnow limitrate;
end


% =========================================================================
%  UDP publish (unchanged)
% =========================================================================
function publish_animation(state)
    % state.attitude = [roll, pitch, yaw] in NED, ZYX convention
    roll  = state.attitude(1);
    pitch = state.attitude(2);
    yaw   = state.attitude(3);

    % NED -> ENU axis remap:
    %   ENU roll  =  NED pitch
    %   ENU pitch = -NED roll  (Y flips)
    %   ENU yaw   = 90deg - NED yaw  (bearing from East vs North)
    roll_enu  =  pitch;
    pitch_enu = -roll;
    yaw_enu   =  pi/2 - yaw;

    % eul2quat returns [qw qx qy qz], PyBullet wants [qx qy qz qw]
    q = eul2quat([yaw_enu, pitch_enu, roll_enu], 'ZYX');  % [qw qx qy qz]
    qw = q(1); qx = q(2); qy = q(3); qz = q(4);

    % Position: NED [N, E, D] -> ENU [E, N, U]
    x_enu =  state.position(2);   % East
    y_enu =  state.position(1);   % North
    z_enu = -state.position(3);   % Up

    pkt   = [x_enu, y_enu, z_enu, qw, qx, qy, qz];
    bytes = typecast(pkt, 'uint8');
    write(state.udpsocket, bytes, "uint8", state.ip, state.port);
end