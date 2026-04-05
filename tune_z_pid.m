% tune_z_pid.m
% =============
% Z-Axis PID tuning from step-response data.
%
% Workflow:
%   1. Run z_step_logger.py to generate z_step_response.csv
%   2. Run this script in MATLAB  
%   3. Read the recommended Kp/Ki/Kd at the bottom and put them in RobotConfig.h
%
% Requirements: System Identification Toolbox

clear; clc; close all;

% ── CONFIG ────────────────────────────────────────────────────────────────────
CSV_FILE  = 'z_step_response.csv';
LOOP_HZ   = 20;          % Must match LOOP_HZ in z_step_logger.py
STEP_AMP  = 60.0;        % degrees — must match STEP_DEG in z_step_logger.py
TARGET_PM = 5;           % Desired phase margin (degrees) for pidtune
TARGET_BW = 1.5;         % Desired closed-loop bandwidth (rad/s) — tune this
% ─────────────────────────────────────────────────────────────────────────────

Ts = 1 / LOOP_HZ;

%% ── 1. Load Data ─────────────────────────────────────────────────────────────
T = readtable(CSV_FILE);
fprintf('Loaded %d samples from %s\n', height(T), CSV_FILE);

% Extract the positive step phase for system identification
pos_mask = strcmp(T.phase, 'step_pos');
t_raw    = T.time_ms(pos_mask) / 1000.0;     % seconds
y_raw    = T.actual_deg(pos_mask);            % degrees (actual)
u_raw    = T.target_deg(pos_mask);            % degrees (target)

% Normalize time to start at 0
t = t_raw - t_raw(1);

% Remove the offset — we're fitting the step-response behaviour
y_offset = y_raw(1);                          % pre-step position
y = y_raw - y_offset;                         % starts near 0
u = STEP_AMP * ones(size(t));                 % ideal unit step scaled to STEP_AMP

%% ── 2. Plot Raw Step Response ────────────────────────────────────────────────
figure(1);
subplot(2,1,1);
plot(t, y_raw, 'b-', 'LineWidth', 1.5); hold on;
yline(STEP_AMP, 'r--', 'Target');
xlabel('Time (s)'); ylabel('Z Position (deg)');
title('Z-Axis Step Response — Raw Data');
legend('Actual', 'Target'); grid on;

subplot(2,1,2);
plot(t, u_raw, 'k-', t, y_raw, 'b-', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('deg');
legend('Target', 'Actual'); title('Overlay'); grid on;

%% ── 3. System Identification (tfest — 2nd order) ────────────────────────────
fprintf('\n--- System Identification ---\n');

% Resample if needed
data = iddata(y, u, Ts);

% Try 1st and 2nd order transfer function fits
tf1 = tfest(data, 1, 0);    % 1 pole, 0 zeros
tf2 = tfest(data, 2, 0);    % 2 poles (captures oscillation/resonance)

fprintf('1st order fit:  fit = %.1f%%\n', goodnessOfFit(predict(tf1,data).OutputData, y, 'NRMSE')*100);
fprintf('2nd order fit:  fit = %.1f%%\n', goodnessOfFit(predict(tf2,data).OutputData, y, 'NRMSE')*100);

% Pick the model with better fit
fit1 = goodnessOfFit(predict(tf1,data).OutputData, y, 'NRMSE');
fit2 = goodnessOfFit(predict(tf2,data).OutputData, y, 'NRMSE');
if fit2 > fit1
    plant = tf2;
    fprintf('Using 2nd-order model\n');
else
    plant = tf1;
    fprintf('Using 1st-order model\n');
end

%% ── 4. PID Tuning ────────────────────────────────────────────────────────────
fprintf('\n--- PID Tuning ---\n');
opts = pidtuneOptions('PhaseMargin', TARGET_PM + 45, ...
                      'DesignFocus', 'reference-tracking');
C = pidtune(plant, 'PID', TARGET_BW, opts);

Kp = C.Kp;
Ki = C.Ki;
Kd = C.Kd;

fprintf('Recommended PID gains:\n');
fprintf('  Kp = %.4f\n', Kp);
fprintf('  Ki = %.4f\n', Ki);
fprintf('  Kd = %.4f\n', Kd);

%% ── 5. Step Response with PID ───────────────────────────────────────────────
cl_sys = feedback(C * plant, 1);
[y_cl, t_cl] = step(cl_sys * STEP_AMP, linspace(0, max(t), 500));

figure(2);
plot(t, y, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Measured'); hold on;
plot(t_cl, y_cl, 'r--', 'LineWidth', 1.5, 'DisplayName', 'Simulated (tuned PID)');
yline(STEP_AMP, 'k:', 'Target');
xlabel('Time (s)'); ylabel('Z Position (deg)');
title(sprintf('Z-Axis — Measured vs Simulated PID\nKp=%.3f  Ki=%.4f  Kd=%.4f', Kp, Ki, Kd));
legend; grid on;

%% ── 6. Bode Plot ─────────────────────────────────────────────────────────────
figure(3);
margin(C * plant);
title('Open-Loop Bode — Check Phase & Gain Margin');

%% ── 7. Summary ───────────────────────────────────────────────────────────────
fprintf('\n==================================================\n');
fprintf('  Copy these into RobotConfig.h:\n');
fprintf('==================================================\n');
fprintf('  constexpr float Kp_enc[4] = {1.0f, 2.0f, 2.0f, %.4ff};\n', Kp);
fprintf('  constexpr float Ki_enc[4] = {0.05f, 0.02f, 0.02f, %.4ff};\n', Ki);
fprintf('  constexpr float Kd_enc[4] = {0.05f, 0.01f, 0.01f, %.4ff};\n', Kd);
fprintf('==================================================\n');

%% ── Helper ───────────────────────────────────────────────────────────────────
function s = goodnessOfFit(y_pred, y_meas, ~)
    % Simple NRMSE-style fit (0=bad, 1=perfect)
    err = y_meas - y_pred;
    s   = 1 - norm(err) / norm(y_meas - mean(y_meas));
end
