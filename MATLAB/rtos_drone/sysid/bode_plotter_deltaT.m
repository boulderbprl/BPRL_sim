%% Initial Read-In
clear all; close all; clc;

%T = readtable("data/thrust_stand/2026_05_08-Lumenier/2026-05-08_1250-1300_60s_0,5-0,5-10.csv");
%T = readtable("data/thrust_stand/2026_05_08-Lumenier/2026-05-08_1450-1500_60s_0,5-0,5-10.csv");
T = readtable("data/thrust_stand/2026_05_07-Lumenier/2026-05-07_1250-1300_60s_0,5-0,5-10.csv");

t = T.Time_s_;
pwm = T.ESCSignal__s_;
thr = T.Thrust_N_;

%% Setting starting time
ti = 11.1;
[~,im] = min(abs(t-ti));

t = t(im:end)-t(im);
pwm = pwm(im:end);
thr = thr(im:end);

%% Interpolate/downsample data
fs = 90;
% fs = 40;
ti = linspace(t(1),t(end),fs*(t(end)-t(1)));
pwm = interp1(t,pwm,ti);
thr = interp1(t,thr,ti);
t = ti;

%% Setup
% pwm = smooth(t,pwm,0.0005,'loess');
% thr = smooth(t,thr,0.0005,'loess');

t_win = 60; % How long each frequency window lasts
win_off = 7; % How much of each window we chop from the front and back

sz = (t_win-2*win_off)*fs-1;

fspan = 0.5:0.5:10; % Hz
wspan = fspan*2*pi; % rad

%% Splitting Time Data
t_sep = [];
pwm_sep = [];
thr_sep = [];

% sz = [];

figure(1);clf;
plot(t(1:end-1),diff(thr),'b--');
hold on;
xlabel("Time (s)");
ylabel("Thrust (N)");

figure(2);clf;
plot(t(1:end-1),diff(pwm),'b--');
hold on;
xlabel("Time (s)");
ylabel("Commanded PWM");

for idx = 1:length(fspan)
    [~,i1] = min(abs(t-(t_win*(idx-1)+win_off)));
    [~,i2] = min(abs(t-(t_win*(idx)-win_off)));

    tw = t(i1:i2-1);
    % pwmw = pwm(i1:i2)-mean(pwm(i1:i2));
    % thrw = thr(i1:i2)-mean(thr(i1:i2));
    pwmw = diff(pwm(i1:i2));
    thrw = diff(thr(i1:i2));

    t_sep(:,idx) = tw(1:sz);
    pwm_sep(:,idx) = pwmw(1:sz);
    thr_sep(:,idx) = thrw(1:sz);

    % sz(end+1) = length(tw);

    figure(1);
    plot(tw,thrw);

    figure(2);
    plot(tw,pwmw);

end

%% DFT

Q_vec = [];

for idx = 1:length(wspan)
    fs = 1/mean(diff(t_sep(:,idx)));
    % N = sz(idx);
    N = sz;
    w = wspan(idx);

    tss = t_sep(:,idx);
    pwmss = pwm_sep(:,idx);
    thrss = thr_sep(:,idx);

    bas = exp(-1j*w*tss);

    Y = (2/N)*sum(thrss.*bas);
    F = (2/N)*sum(pwmss.*bas);

    Q = Y/F;
    Q_vec(end+1) = Q;
end

figure(3);clf;
subplot(211);
semilogy(fspan,abs(Q_vec),'o');
ylabel("Magnitude (N/PWM)")

subplot(212);
plot(fspan, unwrap(angle(Q_vec))*180/pi,'o');
xlabel('Frequency (Hz)');
ylabel('Phase (deg)');

%% Fitting
frd_meas = idfrd(Q_vec,wspan,0);
np = 4;
nz = 3;
sys = tfest(frd_meas,np,nz);
z = zero(sys);
p = pole(sys);
% dc = dcgain(sys);

[mm, pm, woutm] = bode(frd_meas, wspan);
[mf, pf, woutf] = bode(sys, wspan);

foutm = woutm/(2*pi);
foutf = woutf/(2*pi);

figure(4);clf;
subplot(211);
semilogy(foutm,squeeze(mm),'bo',foutf,squeeze(mf),'r-');
% loglog(foutm,squeeze(mm),'bo',foutf,squeeze(mf),'r-');
ylabel("Magnitude (N/PWM)")

phas = squeeze(pf);
pmult = 0;
if phas(1) > 660
    pmult = 2;
elseif phas(1) > 300
    pmult = 1;
end

subplot(212);
plot(foutm,squeeze(pm),'o',foutf,phas-360*pmult,'r-');
% semilogx(foutm,squeeze(pm),'o',foutf,phas-360*pmult,'r-');
legend("Measured","Fitted")
xlabel('Frequency (Hz)');
ylabel('Phase (deg)');

%% Plotting for different frequencies

delta_t = tw(2) - tw(1); %~ 10 Hz
sysd = c2d(sys,delta_t);
disp(sysd);

idx = 1; 
pwmw = pwm_sep(:,idx);
tw = t_sep(:,idx);
thrw = thr_sep(:,idx);

[ys,ts] = lsim(sysd,pwmw,tw);

figure(5);clf;
idx_start=1;
interval=idx_start:idx_start+ceil(500/fspan(idx)); % 
%yyaxis left
plot(tw(interval),thrw(interval),ts(interval),ys(interval))
%plot(tw,thrw,ts,ys+mean(thrw));
legend("Measured","Fitted");
xlabel("Time (s)");
ylabel("Thrust (N)")

hold on

disp(strcat("Mean PWM: ", num2str(mean(pwmw))));
disp(strcat("PWM Std: ", num2str(std(pwmw))));
disp(strcat("Mean Thrust: ", num2str(mean(thrw))));

new_mf = squeeze(mf);
band = interp1(new_mf,foutf,new_mf(1)*10^(-3/10));
disp(strcat("Bandwidth: ", num2str(band), " Hz"));
disp(strcat("Zero: ", num2str(z)));
disp(strcat("Pole: ", num2str(p)));


% -------------------------------------------------------
% Fourth-order ARX motor model: thrust prediction
% Transfer function: H(z) = (b1*z^-1 + b2*z^-2 + b3*z^-3 + b4*z^-4)
%                          / (1 + a1*z^-1 + a2*z^-2 + a3*z^-3 + a4*z^-4)
% -------------------------------------------------------

% --- Paste your identified coefficients here ---
a = [-2.426204560796965,2.375420713521035,-1.113216986080528,0.209329333306887];%sysd.Denominator(2:end);   % denominator (feedback) coefficients
b = [-0.000136204311417,0.000696618702280,-0.001310374847237,0.000957984035365];%sysd.Numerator(2:end);   % numerator   (feedforward) coefficients

% --- Load your data ---
% t         : time vector          [N x 1]
% cmd       : ESC signal (µs)      [N x 1]
% thrust_meas : measured thrust (N) [N x 1]
% (adjust variable names to match your workspace)

N = length(pwmw);
thrust_pred = zeros(N, 1);
cmd = zeros(N,1) ;



% --- Simulate difference equation ---
% f[k] = -a1*f[k-1] - a2*f[k-2] - a3*f[k-3] - a4*f[k-4]
%       +  b1*u[k-1] + b2*u[k-2] + b3*u[k-3] + b4*u[k-4]
k_cmd = zeros(4,1);
k_thrust_pred = zeros(4,1);

for k = 1:N
    cmd(k) = pwmw(k);
    k_cmd = [cmd(k);k_cmd(1:end-1)];

   thrust_pred(k+1) = b*k_cmd-a*k_thrust_pred;
   k_thrust_pred = [thrust_pred(k+1);k_thrust_pred(1:end-1)];

end

% --- Fig 3.16-style plot ---

% % Top subplot: input command
% subplot(2,1,1);
% 
% grid on;

% Bottom subplot: measured vs predicted thrust
plot(tw(interval), thrust_pred(interval), '-o', 'LineWidth', 1.5, 'DisplayName', 'Model Predicted','LineWidth',1);
grid on;
axis tight
hold off

% yyaxis right 
% plot(tw(interval), cmd(interval), 'LineWidth', 1.2,'DisplayName', 'PWM Input','LineWidth',1);
%  ylabel('ESC signal (\mus)');
