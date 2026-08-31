%% UAVs_Kalman - Simulasi UAV CBF dengan Estimasi State Rintangan Berbasis Kalman Filter
% Script ini mensimulasikan gerak UAV 3D yang menghindari rintangan bergerak dinamis
% di mana posisi, kecepatan, dan percepatan rintangan diestimasi menggunakan 9-state Kalman Filter.

clear;
close all;
clc;

%% Global Parameters
global A B Kp Kd mu r f Ox Oy Oz Ak Hk Pk Jk Qk xk step_idx

%% Parameter Trajektori (Feedforward)
% Titik asal
Ox = 20;
Oy = 3;
Oz = 20;

% Kecepatan sudut (rad/s)
f = pi/4;

%% Parameter Kontroler
% Gain Proporsional dan Derivatif
Kp = 100;
Kd = 30;

% Gain Matrix
K = [Kp,  0,  0, Kd,  0,  0;
      0, Kp,  0,  0, Kd,  0;
      0,  0, Kp,  0,  0, Kd];

% Parameter CBF
alpha = 5;
mu = 0.05;
d = 2;
d1 = 5;
r = 1;

% Saturasi kontroler
sat = 40;

%% Matriks Dinamika Sistem
A = [zeros(3, 3), eye(3);
     zeros(3, 6)];

B = [zeros(3, 3);
     eye(3)];

%% Setup Kalman Filter (9-State: Posisi, Kecepatan, Percepatan 3D)
dt = 0.05;  % Interval sampling
Qk = diag([0.01, 0.01, 0.01, 0.1, 0.1, 0.1, 0.5, 0.5, 0.5]);  % Covariance Proses
Jk = diag([0.05, 0.05, 0.05]);                               % Covariance Pengukuran

% Matriks Transisi Ak
Ak = [eye(3), dt*eye(3), 0.5*(dt^2)*eye(3);
      zeros(3), eye(3),  dt*eye(3);
      zeros(3), zeros(3), eye(3)];

% Matriks Observasi Hk (hanya posisi yang terukur)
Hk = [eye(3), zeros(3, 6)];

% Kondisi Awal Filter Kalman
x0k = [0; 10; 20; 0; 0; 0; 0; 0; 0];
Pk = eye(9);

% Waktu simulasi
tspan = 0:0.05:30;
xk = zeros(9, length(tspan) + 1);
xk(:, 1) = x0k;
step_idx = 1;

%% Simulasi Closed-Loop
% Kondisi awal robot
x0 = [20; 3; 20; 0; 0; 0];

% Jalankan simulasi numerik
[t, x] = ode45(@simulation, tspan, x0);

% Konversi format data simulasi untuk plotting
[~, Xd, Yd, Zd, Ux, Uy, Uz, Ob1, h] = ...
    cellfun(@(t_val, x_val) simulation(t_val, x_val.'), num2cell(t), num2cell(x, 2), 'UniformOutput', false);

H = cell2mat(h);

% Ekstraksi posisi rintangan nyata
Pobs1 = cell2mat(Ob1);
Pobs1x = Pobs1(1, :);
Pobs1y = Pobs1(2, :);
Pobs1z = Pobs1(3, :);

%% Plotting Hasil Simulasi

% Posisi UAV
X = x(:, 1);
Y = x(:, 2);
Z = x(:, 3);

% Kecepatan UAV
X_dot = x(:, 4);
Y_dot = x(:, 5);
Z_dot = x(:, 6);

% Trajektori Referensi
xd = cell2mat(Xd);
yd = cell2mat(Yd);
zd = cell2mat(Zd);

% 1. Plot Control Effort
figure(1);
ux = cell2mat(Ux);
uy = cell2mat(Uy);
uz = cell2mat(Uz);
plot(t, ux, 'b', t, uy, 'r', t, uz, 'm', 'LineWidth', 2);
hold on;
yline(sat, 'k--', 'Label', 'Saturation', 'LineWidth', 1.2);
yline(-sat, 'k--', 'Label', '-Saturation', 'LineWidth', 1.2);
hold off;
title('Control Effort');
xlabel('t [s]');
ylabel('u(t) [m/s^2]');
legend('x acceleration', 'y acceleration', 'z acceleration', 'Location', 'best');
grid on;
box on;
ylim([min([ux; uy; uz]) - 2, max([ux; uy; uz]) + 2]);

% 2. Plot Kecepatan UAV
figure(2);
plot(t, X_dot, 'b', t, Y_dot, 'r', t, Z_dot, 'm', 'LineWidth', 2);
title('Robot Velocities');
xlabel('t [s]');
ylabel('v(t) [m/s]');
legend('x velocity', 'y velocity', 'z velocity', 'Location', 'best');
grid on;
box on;
ylim([min([X_dot; Y_dot; Z_dot]) - 2, max([X_dot; Y_dot; Z_dot]) + 2]);

% 3. Plot Kecepatan Rintangan: Real vs Estimasi
figure(3);
T = [0, tspan];
numT = length(T);

subplot(2, 1, 1);
plot(T, 10*f*cos(f*T), 'r', T, -10*f*sin(f*T), 'g', T, zeros(1, numT), 'b', 'LineWidth', 2);
title('Real Velocity of the Obstacle');
xlabel('t [s]');
ylabel('v(t) [m/s]');
legend('v_x real', 'v_y real', 'v_z real', 'Location', 'best');
grid on;
box on;

subplot(2, 1, 2);
plot(T, xk(4, 1:numT), 'r', T, xk(5, 1:numT), 'g', T, xk(6, 1:numT), 'b', 'LineWidth', 2);
title('Estimated Velocity of the Obstacle (Kalman Filter)');
xlabel('t [s]');
ylabel('v(t) [m/s]');
legend('v_x est', 'v_y est', 'v_z est', 'Location', 'best');
grid on;
box on;

% 4. Plot Percepatan Rintangan: Real vs Estimasi
figure(4);

subplot(2, 1, 1);
plot(T, -10*(f^2)*sin(f*T), 'r', T, -10*(f^2)*cos(f*T), 'g', T, zeros(1, numT), 'b', 'LineWidth', 2);
title('Real Acceleration of the Obstacle');
xlabel('t [s]');
ylabel('a(t) [m/s^2]');
legend('a_x real', 'a_y real', 'a_z real', 'Location', 'best');
grid on;
box on;

subplot(2, 1, 2);
plot(T, xk(7, 1:numT), 'r', T, xk(8, 1:numT), 'g', T, xk(9, 1:numT), 'b', 'LineWidth', 2);
title('Estimated Acceleration of the Obstacle (Kalman Filter)');
xlabel('t [s]');
ylabel('a(t) [m/s^2]');
legend('a_x est', 'a_y est', 'a_z est', 'Location', 'best');
grid on;
box on;

%% Fungsi Simulasi Sistem Dinamik dan Kontroler CBF
function [dx, Xd, Yd, Zd, Ux, Uy, Uz, Ob1, h] = simulation(t, x)
    global A B Kp Kd mu r f Ox Oy Oz Ak Hk Pk Jk Qk xk step_idx

    deltaFunc1 = 2;

    % Trajektori Referensi (Feedforward)
    yref = [Ox - f*t; Oy; Oz];
    yref_dot = [-f; 0; 0];
    yref_dd = [0; 0; 0];

    % 1. Kalman Filter Step untuk Estimasi Rintangan
    x_pred = Ak * xk(:, step_idx);
    P_pred = Ak * Pk * Ak' + Qk;

    % Simulasi Pengukuran dengan noise
    zk = [10*sin(f*t); 10*cos(f*t); 20] + randn(3, 1) * 0.05;

    % Update Kalman Filter
    yk = zk - Hk * x_pred;
    Sk = Hk * P_pred * Hk' + Jk;
    Kk = (P_pred * Hk') / Sk;

    xk(:, step_idx + 1) = x_pred + Kk * yk;
    Pk = (eye(9) - Kk * Hk) * P_pred;

    % Nilai estimasi
    xkk = xk(:, step_idx);
    Ob1Dot_pred = xkk(4:6);
    Ob1DotDot_pred = xkk(7:9);

    % Nilai nyata rintangan
    Ob1 = [10*sin(f*t); 10*cos(f*t); 20];

    % Posisi dan kecepatan UAV
    Pi = x(1:3);
    Pi_dot = x(4:6);

    % Kontrol Nominal
    uNominal = yref_dd + Kd*(yref_dot - Pi_dot) + Kp*(yref - Pi);

    % Vektor relatif terhadap rintangan
    V = Pi - Ob1;
    V_dot = Pi_dot - Ob1Dot_pred;

    % Operator Proyeksi
    po = (V * V') / (V' * V);
    poPerp = eye(3) - po;

    % Komponen tegak lurus
    u_perp = zeros(3, 1);
    if rank([V, Pi_dot, x(4:6)], 0.1) == 1
        u_perp = [-V(2); V(1); 0];
    end

    % Evaluasi Control Barrier Function (CBF)
    h1 = V' * (mu*uNominal + 2*V_dot - mu*Ob1DotDot_pred);
    h2 = (V'*V + mu*(V'*V_dot));
    gamma = 12;

    % Switching Logika CBF
    if h1 > 0 || h2 > deltaFunc1 + (r*gamma)
        u = uNominal;
    else
        u = ((-2/mu) * (po * V_dot)) + (poPerp * uNominal) + Ob1DotDot_pred + u_perp;
    end

    % Saturasi kontroler
    sat = 40;
    u = min(max(u, -sat), sat);

    % Output Kontrol
    Ux = u(1);
    Uy = u(2);
    Uz = u(3);

    % Output Referensi
    Xd = yref(1);
    Yd = yref(2);
    Zd = yref(3);

    % Indeks Evaluasi CBF
    h = V'*V + mu*(V'*V_dot) - (deltaFunc1 + r);

    % Persamaan diferensial state
    dx = A*x + B*u;

    % Perbarui step index untuk sampling discrete Kalman Filter
    if t > (step_idx - 1) * 0.05 && step_idx < size(xk, 2) - 1
        step_idx = step_idx + 1;
    end
end
