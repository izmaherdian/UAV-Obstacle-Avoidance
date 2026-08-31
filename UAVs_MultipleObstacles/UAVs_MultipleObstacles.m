%% UAVs_MultipleObstacles - Simulasi Penghindaran Rintangan Berganda UAV Menggunakan CBF
% Script ini mensimulasikan gerak UAV 3D yang mengikuti trajektori referensi
% sembari menghindari 7 rintangan (bergerak dan statis) menggunakan Control Barrier Functions (CBF).

clear;
close all;
clc;

%% Global Parameters
global A B Kp Kd mu r f Ox Oy Oz a b c

%% Parameter Trajektori (Feedforward)
% Titik asal
Ox = 10;
Oy = 3;
Oz = 60;

% Kecepatan sudut (rad/s)
f = pi/4;

% Dimensi trajektori
a = 30;
b = 20;
c = 16;

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
d = 1;
d1 = 3;
r = 5;

% Saturasi kontroler
sat = 100;

%% Matriks Dinamika Sistem
A = [zeros(3, 3), eye(3);
     zeros(3, 6)];

B = [zeros(3, 3);
     eye(3)];

%% Simulasi Sistem Closed-Loop
% Kondisi awal [x, y, z, vx, vy, vz]
x0 = [6; 3; 40; 0; 0; 0];

% Vektor waktu simulasi
tspan = 0:0.05:30;

% Jalankan simulasi numerik
[t, x] = ode45(@simulation, tspan, x0);

% Konversi format data simulasi untuk plotting
[~, Xd, Yd, Zd, Ux, Uy, Uz, Ob1, Ob2, Ob3, Ob4, Ob5, Ob6, Ob7, h] = ...
    cellfun(@(t_val, x_val) simulation(t_val, x_val.'), num2cell(t), num2cell(x, 2), 'UniformOutput', false);

H = cell2mat(h);

% Ekstraksi posisi rintangan
Pobs1 = cell2mat(Ob1); Pobs1x = Pobs1(1, :); Pobs1y = Pobs1(2, :); Pobs1z = Pobs1(3, :);
Pobs2 = cell2mat(Ob2); Pobs2x = Pobs2(1, :); Pobs2y = Pobs2(2, :); Pobs2z = Pobs2(3, :);
Pobs3 = cell2mat(Ob3); Pobs3x = Pobs3(1, :); Pobs3y = Pobs3(2, :); Pobs3z = Pobs3(3, :);
Pobs4 = cell2mat(Ob4); Pobs4x = Pobs4(1, :); Pobs4y = Pobs4(2, :); Pobs4z = Pobs4(3, :);
Pobs5 = cell2mat(Ob5); Pobs5x = Pobs5(1, :); Pobs5y = Pobs5(2, :); Pobs5z = Pobs5(3, :);
Pobs6 = cell2mat(Ob6); Pobs6x = Pobs6(1, :); Pobs6y = Pobs6(2, :); Pobs6z = Pobs6(3, :);
Pobs7 = cell2mat(Ob7); Pobs7x = Pobs7(1, :); Pobs7y = Pobs7(2, :); Pobs7z = Pobs7(3, :);

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

%% Fungsi Simulasi Sistem Dinamik dan Kontroler CBF
function [dx, Xd, Yd, Zd, Ux, Uy, Uz, Ob1, Ob2, Ob3, Ob4, Ob5, Ob6, Ob7, h] = simulation(t, x)
    global A B Kp Kd mu r f Ox Oy Oz a b c

    % Kecepatan sudut rintangan
    foCirc = 4;
    fo = 1;
    deltaFunc1 = 1;

    % Trajektori Referensi (Feedforward)
    yref = [Ox + a*sin(f*t);
            Oy + b*sin(f*t)*cos(f*t);
            Oz + c*sin(0.5*f*t)];

    yref_dot = [a*f*cos(f*t);
                b*f*cos(2*f*t);
                c*0.5*f*cos(0.5*f*t)];

    yref_dd = [-a*f^2*sin(f*t);
               -b*f^2*sin(2*f*t);
               -c*0.25*f^2*sin(0.5*f*t)];

    % Rintangan 1 (Bergerak)
    Ob1 = [-20 + 30*sin(fo*t); 6; 70];
    Ob1Dot = [30*fo*cos(fo*t); 0; 0];
    Ob1DotDot = [-30*fo^2*sin(fo*t); 0; 0];

    % Rintangan 2 (Bergerak)
    Ob2 = [30 - 30*sin(fo*t); 6; 53];
    Ob2Dot = [-30*fo*cos(fo*t); 0; 0];
    Ob2DotDot = [30*fo^2*sin(fo*t); 0; 0];

    % Rintangan 3 (Bergerak melingkar)
    Ob3 = [5*sin(foCirc*t); 6*cos(foCirc*t); 60];
    Ob3Dot = [5*foCirc*cos(foCirc*t); -6*foCirc*sin(foCirc*t); 0];
    Ob3DotDot = [-5*foCirc^2*sin(foCirc*t); -6*foCirc^2*cos(foCirc*t); 0];

    % Rintangan 4 (Statis)
    Ob4 = [48; 3; 70];
    Ob4Dot = [0; 0; 0];
    Ob4DotDot = [0; 0; 0];

    % Rintangan 5 (Statis)
    Ob5 = [34; 3; 70];
    Ob5Dot = [0; 0; 0];
    Ob5DotDot = [0; 0; 0];

    % Rintangan 6 (Statis)
    Ob6 = [19; 6; 62];
    Ob6Dot = [0; 0; 0];
    Ob6DotDot = [0; 0; 0];

    % Rintangan 7 (Statis)
    Ob7 = [19; 10; 62];
    Ob7Dot = [0; 0; 0];
    Ob7DotDot = [0; 0; 0];

    Obs = [Ob1, Ob2, Ob3, Ob4, Ob5, Ob6, Ob7];
    ObsDot = [Ob1Dot, Ob2Dot, Ob3Dot, Ob4Dot, Ob5Dot, Ob6Dot, Ob7Dot];
    ObsDotDot = [Ob1DotDot, Ob2DotDot, Ob3DotDot, Ob4DotDot, Ob5DotDot, Ob6DotDot, Ob7DotDot];

    % Posisi dan kecepatan UAV saat ini
    Pi = x(1:3);
    Pi_dot = x(4:6);

    % Kontrol Nominal (Tracking PD + Feedforward Acceleration)
    uNominal = yref_dd + Kd*(yref_dot - Pi_dot) + Kp*(yref - Pi);

    % Hitung kuadrat jarak terhadap semua rintangan
    dist = sum((Obs - Pi).^2, 1);

    % Cari rintangan terdekat pertama
    [~, index] = min(dist);
    Pobs = Obs(:, index);
    PobsDot = ObsDot(:, index);
    PobsDotDot = ObsDotDot(:, index);
    dist(index) = inf;

    % Cari rintangan terdekat kedua
    [~, ind] = min(dist);
    Pobs2 = Obs(:, ind);

    % Vektor relatif terhadap rintangan terdekat
    V = Pi - Pobs;

    % Jarak antar dua rintangan terdekat
    l = norm(Pobs2 - Pobs);

    % Jika UAV tidak cukup ruang di antara kedua rintangan, anggap sebagai rintangan gabungan
    if norm(V) < l + deltaFunc1 + 2*r && l <= 2*(r + deltaFunc1)
        Pobs = (Pobs2 + Pobs) / 2;
        deltaFunc1 = 2 * (l/2 + deltaFunc1 + r);
        V = Pi - Pobs;
    end

    % Perbedaan kecepatan relatif
    V_dot = Pi_dot - PobsDot;

    % Operator Proyeksi
    po = (V * V') / (V' * V);
    poPerp = eye(3) - po;

    % Komponen tegak lurus jika arah segaris
    u_perp = zeros(3, 1);
    if rank([V, Pi_dot, PobsDot], 0.1) == 1
        u_perp = [-V(2); V(1); 0];
    end

    % Evaluasi Control Barrier Function (CBF)
    h1 = V' * (mu*uNominal + 2*V_dot - mu*PobsDotDot);
    h2 = (V'*V + mu*(V'*V_dot));
    gamma = 12;

    % Switching Logika CBF
    if h1 > 0 || h2 > deltaFunc1 + (r*gamma)
        u = uNominal;
    else
        u = ((-2/mu) * (po * V_dot)) + (poPerp * uNominal) + PobsDot + u_perp;
    end

    % Saturasi Kontroler
    sat = 100;
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