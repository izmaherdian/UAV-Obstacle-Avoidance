%% UAVs_NoFeasible - Simulasi Penghindaran Rintangan Non-Feasible UAV Menggunakan CBF
% Script ini mensimulasikan gerak UAV 3D mengikuti trajektori piecewise 3D
% sembari menghadapi rintangan bergerak menggunakan Control Barrier Functions (CBF).

clear;
close all;
clc;

%% Global Parameters
global A B Kp Kd mu r f Ox Oy Oz

%% Parameter Trajektori (Feedforward)
% Titik asal
Ox = 0;
Oy = 0;
Oz = 0;

% Kecepatan sudut / faktor kecepatan
f = 1;

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
sat = 20;

%% Matriks Dinamika Sistem
A = [zeros(3, 3), eye(3);
     zeros(3, 6)];

B = [zeros(3, 3);
     eye(3)];

%% Simulasi Sistem Closed-Loop
% Kondisi awal [x, y, z, vx, vy, vz]
x0 = [0; 0; 0; 0; 0; 0];

% Vektor waktu simulasi
tspan = 0:0.05:30;

% Jalankan simulasi numerik
[t, x] = ode45(@simulation, tspan, x0);

% Konversi format data simulasi untuk plotting
[~, Xd, Yd, Zd, Ux, Uy, Uz, Ob1, h] = ...
    cellfun(@(t_val, x_val) simulation(t_val, x_val.'), num2cell(t), num2cell(x, 2), 'UniformOutput', false);

H = cell2mat(h);

% Ekstraksi posisi rintangan
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

% 1. Plot Trajektori Robot & Referensi
figure(1);
plot3(X, Y, Z, 'b-', 'LineWidth', 2);
hold on;
plot3(xd, yd, zd, 'c--', 'LineWidth', 1.5);
title('Robot Trajectory in 3D');
xlabel('x [m]');
ylabel('y [m]');
zlabel('z [m]');
legend('Robot Trajectory', 'Reference Trajectory', 'Location', 'best');
grid on;
box on;
axis equal;

% 2. Plot Control Effort
figure(2);
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

% 3. Plot Kecepatan UAV
figure(3);
plot(t, X_dot, 'b', t, Y_dot, 'r', t, Z_dot, 'm', 'LineWidth', 2);
title('Robot Velocities');
xlabel('t [s]');
ylabel('v(t) [m/s]');
legend('x velocity', 'y velocity', 'z velocity', 'Location', 'best');
grid on;
box on;

%% Fungsi Simulasi Sistem Dinamik dan Kontroler CBF
function [dx, Xd, Yd, Zd, Ux, Uy, Uz, Ob1, h] = simulation(t, x)
    global A B Kp Kd mu r f Ox Oy Oz

    deltaFunc1 = 2;

    % Trajektori Referensi Piecewise (Feedforward)
    if t <= 10
        yref = [Ox + f*t; Oy; Oz];
        yref_dot = [f; 0; 0];
        yref_dd = [0; 0; 0];
    elseif t <= 20
        yref = [Ox + 10*f; Oy + f*(t - 10); Oz];
        yref_dot = [0; f; 0];
        yref_dd = [0; 0; 0];
    else
        yref = [Ox + 10*f; Oy + 10*f; Oz + f*(t - 20)];
        yref_dot = [0; 0; f];
        yref_dd = [0; 0; 0];
    end

    % Rintangan Bergerak
    Ob1 = [f*t - 10; 10; 0];
    Ob1Dot = [f; 0; 0];
    Ob1DotDot = [0; 0; 0];

    % Posisi dan kecepatan UAV saat ini
    Pi = x(1:3);
    Pi_dot = x(4:6);

    % Kontrol Nominal
    uNominal = yref_dd + Kd*(yref_dot - Pi_dot) + Kp*(yref - Pi);

    % Vektor relatif terhadap rintangan
    V = Pi - Ob1;
    V_dot = Pi_dot - Ob1Dot;

    % Operator Proyeksi
    po = (V * V') / (V' * V);
    poPerp = eye(3) - po;

    % Komponen tegak lurus jika arah segaris
    u_perp = zeros(3, 1);
    if rank([V, Pi_dot, Ob1Dot], 0.1) == 1
        u_perp = [-V(2); V(1); 0];
    end

    % Evaluasi Control Barrier Function (CBF)
    h1 = V' * (mu*uNominal + 2*V_dot - mu*Ob1DotDot);
    h2 = (V'*V + mu*(V'*V_dot));
    gamma = 12;

    % Switching Logika CBF
    if h1 > 0 || h2 > deltaFunc1 + (r*gamma)
        u = uNominal;
    else
        u = ((-2/mu) * (po * V_dot)) + (poPerp * uNominal) + Ob1DotDot + u_perp;
    end

    % Saturasi kontroler
    sat = 10;
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
end
