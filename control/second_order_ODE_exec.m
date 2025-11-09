% simulate_coupled_tank.m
clc
clear variables
close all

% Load only the tank constants
second_order_ODE_param

global u_in

%% — Control & Performance settings (back in main file) —

% PID gains & set‑point
k_p   = 40;
k_d   = 3;
k_i   = 1.2;
ref_  = 4;      % desired h2 [cm]

% Performance thresholds
settling_threshold     = 0.05;   % ±5%
rise_time_lower_bound  = 0;    % 0% of ref
rise_time_upper_bound  = 0.9;    % 90% of ref

% Simulation timing
t_final    = 100;    % seconds
t_interval = 0.01;  % seconds

%% — Pre‑allocate storage —
cnt_f     = round(t_final / t_interval);
T2        = zeros(1, cnt_f+1);
init_val_M= zeros(cnt_f+1, 2);

% Initial state [h1, h2]
init_val = [0  0];

% PID integrator & previous error
e_prev = 0;
e_int  = 0;

cnt = 1;
for i = 0 : t_interval : t_final

    h1 = init_val(1);
    h2 = init_val(2);

    % PID on h2
    e    = ref_ - h2;
    de   = (e - e_prev) / t_interval;
    e_int= e_int + e * t_interval;
    u_in = k_p*e + k_d*de + k_i*e_int;

    % Integrate nonlinear ODE one step
    [t, Y] = ode45(@second_order_ODE, [i, i+t_interval], init_val);
    init_val = Y(end,:);

    % Store for plotting/metrics
    T2(cnt)            = t(1);
    init_val_M(cnt,:)  = init_val;
    e_prev = e;
    cnt = cnt + 1;
end

% Extract h2 history
h2M = init_val_M(:,2);

%% — Compute performance metrics — 
final_val = ref_;
idx10 = find(h2M >= rise_time_lower_bound*final_val, 1, 'first');
idx90 = find(h2M >= rise_time_upper_bound*final_val, 1, 'first');
if ~isempty(idx10) && ~isempty(idx90)
    rise_time = T2(idx90) - T2(idx10);
else
    rise_time = NaN;
end

[peak_val, idx_peak] = max(h2M);
peak_time         = T2(idx_peak);
percent_overshoot = (peak_val - final_val)/final_val * 100;

lowb = final_val*(1 - settling_threshold);
upb  = final_val*(1 + settling_threshold);
settling_time = NaN;
for k = 1:length(h2M)
    if all(h2M(k:end) >= lowb & h2M(k:end) <= upb)
        settling_time = T2(k);
        break
    end
end

%% — Plot response & annotate —
lw_  = 2; fs_1 = 16; fs_3 = 18; ax_lw = 1.5;

figure('Position',[40 60 450 450]);
plot(T2, h2M, 'k','LineWidth',lw_), grid on, axis square
set(gca,'FontSize',fs_1,'LineWidth',ax_lw)
xlabel('$$t$$, s','Interpreter','latex','FontSize',fs_3)
ylabel('$$h_2$$, cm','Interpreter','latex','FontSize',fs_3)
title('Closed‑loop Response','FontSize',fs_3)

ann = sprintf([ ...
    'Rise Time: %.3f s\nPeak Time: %.3f s\n' ...
    '%% Overshoot: %.2f%%\nSettling (±%.0f%%): %.3f s'], ...
    rise_time, peak_time, percent_overshoot, ...
    settling_threshold*100, settling_time);

annotation('textbox',[0.7 0.83 0.25 0.15], ...
    'String',ann,'FitBoxToText','on','BackgroundColor','white', ...
    'EdgeColor','black','FontSize',fs_1);
