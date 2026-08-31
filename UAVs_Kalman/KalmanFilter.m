%% KalmanFilter - Simulasi Estimasi Posisi dan Kecepatan Rintangan
% Script ini mensimulasikan Kalman Filter linier (6 state: posisi dan kecepatan 3D)
% untuk memperkirakan trajektori rintangan dari pengukuran posisi dengan noise.

clear;
close all;
clc;

%% Parameter Filter Kalman
dt = 0.1;                                       % Sampling time (detik)
Q = diag([0.1, 0.1, 0.1, 0.01, 0.01, 0.01]);   % Matriks kovariansi proses
R = diag([0.5, 0.5, 0.5]);                     % Matriks kovariansi pengukuran

% Matriks transisi state A
A = [eye(3), dt*eye(3);
     zeros(3), eye(3)];

% Matriks observasi H
H = [eye(3), zeros(3)];

%% Inisialisasi State dan Kovariansi
x = [0; 3; 20; 0; 0; 0];  % Kondisi awal [x; y; z; vx; vy; vz]
P = eye(6);               % Kovariansi awal

%% Simulasi dan Estimasi
num_steps = 20;
positions = zeros(num_steps, 3);
velocities = zeros(num_steps, 3);
measurements = zeros(num_steps, 3);

for k = 1:num_steps
    % 1. Tahap Prediksi
    x_pred = A * x;
    P_pred = A * P * A' + Q;

    % 2. Simulasi Pengukuran dengan Gaussian Noise
    z = [2*k; 3; 20] + randn(3, 1) * 0.05;
    measurements(k, :) = z';

    % 3. Tahap Koreksi (Update)
    y = z - H * x_pred;               % Inovasi / measurement residual
    S = H * P_pred * H' + R;          % Kovariansi inovasi
    K = (P_pred * H') / S;            % Kalman Gain

    x = x_pred + K * y;               % Update state estimate
    P = (eye(6) - K * H) * P_pred;    % Update error covariance

    % Simpan hasil estimasi
    positions(k, :) = x(1:3)';
    velocities(k, :) = x(4:6)';
end

%% Plot Hasil Estimasi Posisi 3D
figure(1);
plot3(positions(:, 1), positions(:, 2), positions(:, 3), 'b-', 'LineWidth', 2);
hold on;
scatter3(measurements(:, 1), measurements(:, 2), measurements(:, 3), 40, 'r', 'filled');
xlabel('Posisi X [m]');
ylabel('Posisi Y [m]');
zlabel('Posisi Z [m]');
title('Trajektori Rintangan: Estimasi Kalman vs Pengukuran');
legend('Posisi Terestimasi', 'Pengukuran (dengan Noise)', 'Location', 'best');
grid on;
box on;
view(3);
axis equal;

%% Plot Hasil Estimasi Kecepatan
figure(2);

subplot(3, 1, 1);
plot(1:num_steps, velocities(:, 1), 'r-', 'LineWidth', 2);
title('Kecepatan Estimasi Sumbu X');
xlabel('Langkah Waktu (k)');
ylabel('v_x [m/s]');
grid on;
box on;

subplot(3, 1, 2);
plot(1:num_steps, velocities(:, 2), 'g-', 'LineWidth', 2);
title('Kecepatan Estimasi Sumbu Y');
xlabel('Langkah Waktu (k)');
ylabel('v_y [m/s]');
grid on;
box on;

subplot(3, 1, 3);
plot(1:num_steps, velocities(:, 3), 'b-', 'LineWidth', 2);
title('Kecepatan Estimasi Sumbu Z');
xlabel('Langkah Waktu (k)');
ylabel('v_z [m/s]');
grid on;
box on;
