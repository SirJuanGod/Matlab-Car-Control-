%% init_parametros.m — Parametros del sistema: compuerta, ventilador, carro, controladores.

%% --- 1. Constantes Generales ---
g       = 9.81;          % [m/s^2]
rho_air = 1.225;         % [kg/m^3]

%% --- 2. Compuerta Fisica (Planta: Pendulo Invertido) ---
gate_t      = 0.0015;    % [m]
gate_W      = 0.15;      % [m]
gate_L      = 0.20;      % [m]
gate_A      = gate_W * gate_L; % [m^2]
m_g         = 0.015;     % [kg]
l_c         = 0.10;      % [m]
J_g         = 0.0002;    % [kg*m^2]
b_c         = 0.0010;    % [N*m*s/rad]
tau_f       = 0.0001;    % [N*m]
theta_min   = 0.0;       % [rad]
theta_max   = pi;        % [rad]
tau_stop    = 0.005;     % [s]

%% --- 3. Sensor de Angulo (Potenciometro 0 - 3.3 V, ADC 12 bits del ESP32) ---
V_pot_min   = 0.0;       % [V]
V_pot_mid   = 1.65;      % [V]
V_pot_max   = 3.3;       % [V]
K_pot_deg   = V_pot_max / 180.0;   % [V/deg]
K_pot_rad   = V_pot_max / pi;      % [V/rad]

adc_bits    = 12;                  % [-]
adc_counts  = 2^adc_bits - 1;
adc_Vref    = V_pot_max;           % [V]
adc_res_deg = 180.0 / adc_counts;  % [deg/LSB]

%% --- 4. Ventilador (Actuador Aerodinamico: Sanyo Denki San Ace 40) ---
fan_Vnom    = 24.0;      % [V]
fan_tau     = 0.15;      % [s]
pwm_min     = 10.0;      % [%]
pwm_max     = 90.0;      % [%]

% d0 = distancia de REFERENCIA aerodinamica (donde k_m fue definido); no cambia.
d0          = 0.10;      % [m]
k_m         = 0.00030;   % [N*m/%] calibrar experimentalmente

%% --- 5. Carro Movil (Plataforma Independiente con IMU) ---
m_car        = 0.421;    % [kg]
car_b        = 3.5;      % [N*s/m]
car_F_max    = 3.0;      % [N]
d_ini_cm     = 10.0;     % [cm]
d_min        = 0.05;     % [m]
d_max        = 0.25;     % [m]
car_deadband = 0.005;    % [m] banda muerta del lazo de posicion (+/-5 mm)

% PID de posicion del carro (u_car en % de empuje maximo; positivo = avanzar):
car_Kp = 130;            % [%/m]
car_Ki = 10;             % [%/(m*s)]
car_Kd = 5;              % [%*s/m]

% Sensores IMU (acelerometro longitudinal; positivo = avanzar hacia la compuerta):
imu_bias      = 0.02;    % [m/s^2]
imu_noise_pow = 1e-5;    % [(m/s^2)^2*s]
imu_Ts        = 0.01;    % [s]
obs_qv        = 0.5;     % covarianza de proceso para velocidad
obs_qb        = 1.0;     % covarianza de proceso para sesgo

%% --- 6. Condiciones Iniciales, Referencia y Parametros del Controlador SIMO ---
theta_ref_deg = 35.0;    % [deg]
theta0_deg    = 0.0;     % [deg]
t_sim         = 8.0;     % [s]
tau_ref       = 0.15;    % [s]
ref_rate_deg  = 200.0;    % [deg/s]
usar_imu      = 1;       % 1: el controlador usa d estimada (IMU+Kalman); 0: d real (ideal)
fan_fac       = 1.0;     % [-] 1 = sin degradacion; 0.4 = pierde 60 %. Solo afecta a la planta
fan_t0        = 3.0;     % [s] inicio de la degradacion del ventilador
fan_dur       = 10.0;    % [s] duracion de la rampa de degradacion
gov_deg       = 180.0;     % [deg] gobernador: theta_ref <= theta_medida + gov_deg (180 = desactivado)

% Supervisor de distancia (acerca el carro cuando el ventilador no alcanza):
sup_v_hi = 70;           % [%]
sup_v_lo = 20;           % [%]
sup_adv  = 5;            % [cm/s]
sup_ret  = 1;            % [cm/s]
xi_lim   = 2.0;          % [rad*s] la integral se congela si el PWM satura empujando en la misma direccion
xi_band_deg = 5.0;      % [deg]  la integral del error solo acumula con |error| < xi_band_deg (evita sobrepico en la subida)

% LQR+I SIMO: x = [d_theta; omega; d_v_fan; d_d(cm); x_i], u = [d_u_PWM; d_d_ref(cm)]
tau_car   = 0.8;         % [s]
v_car_max = 0.75;        % [m/s]
q_th      = 10000;          % peso de d_theta   [1/rad^2]
q_om      = 60;           % peso de omega
q_d       = 0.5;         % peso de d_d       [1/cm^2]
q_xi      = 10000;         % peso de la integral del error
r_pwm     = 0.02;        % peso de d_u_PWM   [1/%^2]
r_d       = 1.0;        % peso de d_d_ref   [1/cm^2] (r_d alto => carro casi inactivo)
tau_w     = 0.04;        % [s] filtro de la derivada que estima omega a partir del ADC

% Perturbacion externa de torque sobre la compuerta (pulso rectangular):
pert_tau_mNm = 0.0;      % [mN*m] amplitud (0 = sin perturbacion)
pert_t_on    = 4.0;      % [s]
pert_dur     = 1.0;      % [s]

%% --- 7. Sobrescritura desde Interfaz de Usuario (struct 'ovr') ---
if exist('ovr','var') && isstruct(ovr)
    fn_ovr = fieldnames(ovr);
    for i_ovr = 1:numel(fn_ovr)
        eval([fn_ovr{i_ovr} ' = ovr.' fn_ovr{i_ovr} ';']);
    end
end
clear ovr fn_ovr i_ovr

% Magnitudes derivadas (DEPENDEN de lo sobrescrito, por eso van despues):
theta_ref = deg2rad(theta_ref_deg);
theta0    = deg2rad(theta0_deg);
d_ini     = d_ini_cm / 100;      % [m] distancia inicial real del carro
d_eq      = d_ini;               % [m] distancia nominal de linealizacion del SIMO

% Observador de Kalman de la IMU: estados [v_adv; sesgo], entradas [F; a_meas]
R_obs  = imu_noise_pow / imu_Ts;
A_obs  = [-car_b/m_car, 0; 0, 0];
C_obs  = [-car_b/m_car, 1];
L_obs  = lqe(A_obs, eye(2), C_obs, diag([obs_qv, obs_qb]), R_obs);
D_obs  = 1/m_car;                          % la fuerza F entra tambien en la salida a = (F-b*v)/m
obs_A  = A_obs - L_obs*C_obs;
obs_B  = [[1/m_car; 0] - L_obs*D_obs, L_obs];

%% --- 8. Linealizacion Formal de la Planta (Polos y Ceros en Lazo Abierto) ---
th_lin = min(max(theta_ref, deg2rad(10)), deg2rad(80));
u_eq_ref = (m_g * g * l_c * sin(th_lin)) / k_m; % [%] PWM de sostenimiento (gravedad ~ sin(theta): compuerta colgante)

% Coeficientes de la funcion de transferencia G_p(s) = b2 / (s^2 + a22_pos*s - a21):
a21 = -(m_g * g * l_c * cos(th_lin)) / J_g; % rigidez gravitacional RESTAURADORA (negativa): planta estable
a22 = -b_c / J_g;                          % amortiguamiento viscoso
b2  = k_m / J_g;                           % ganancia de entrada

% Polos de la planta en lazo abierto: s^2 + (b_c/J_g)*s - a21 = 0
polos_ol = roots([1, -a22, -a21]);
p_rhp = polos_ol(real(polos_ol) > 0); % semiplano derecho (vacio: la compuerta colgante es estable)
p_lhp = polos_ol(real(polos_ol) < 0); % polos estables (par oscilatorio poco amortiguado)

% Matrices de estado de la planta pura [delta_theta; delta_omega]:
A_p = [0, 1; a21, a22];
B_p = [0; b2];
C_p = [1, 0];
D_p = 0;

%% --- 9. Diseno Analitico del PID por Lugar Geometrico de las Raices (LGR) ---
zeta_des = 0.85;
wn_des   = 3.2;
p_int    = 1.0;

% El diseno INCLUYE el polo del ventilador (tau_fan): lazo de 4.o orden
p4_des = (-a22*fan_tau + 1)/fan_tau - 2*zeta_des*wn_des - p_int;
P_des  = fan_tau * conv(conv([1, 2*zeta_des*wn_des, wn_des^2], [1, p_int]), [1, p4_des]);  % [s^4 ... s^0]

pid_Kd = (P_des(3) - (-a22 - a21*fan_tau)) / b2;
pid_Kp = (P_des(4) + a21) / b2;
pid_Ki = P_des(5) / b2;
pid_N  = 30.0; % [rad/s] polo de filtrado derivativo

ceros_pid = roots([pid_Kd, pid_Kp, pid_Ki]);

%% --- 10. Diseno del Controlador 2: LQR con Accion Integral (LQR+I) y Gain Scheduling ---
A_aug = [A_p, [0; 0]; 1, 0, 0];
B_aug = [B_p; 0];

Q_lqr = diag([60.0, 3.0, 120.0]);
R_lqr = 0.05;

[K_lqr_nom, P_ricc, autovalores_lqr] = lqr(A_aug, B_aug, Q_lqr, R_lqr);

theta_grid_deg = 10:5:80;
nG = numel(theta_grid_deg);
K_tab = zeros(nG, 3);
ueq_tab = zeros(nG, 1);

for iG = 1:nG
    th_g = deg2rad(theta_grid_deg(iG));
    a21_g = -(m_g * g * l_c * cos(th_g)) / J_g;
    A_g = [0, 1, 0; a21_g, a22, 0; 1, 0, 0];
    K_tab(iG, :) = lqr(A_g, B_aug, Q_lqr, R_lqr);
    ueq_tab(iG)  = (m_g * g * l_c * sin(th_g)) / k_m;
end

K_lqr = interp1(theta_grid_deg, K_tab, min(max(theta_ref_deg, 10), 80), 'linear');
u_eq  = interp1(theta_grid_deg, ueq_tab, min(max(theta_ref_deg, 10), 80), 'linear');

%% --- 11. Controlador 3: Compensador Robusto por Loop Shaping ---
% C_hinf(s) = K_h * (s + z_lead) * (s + z_lag) / [ s * (s + p_lag) ]
z_lead = 4.214;  % [rad/s]
z_lag  = 0.196;  % [rad/s]
p_lag  = 30.0;   % [rad/s]
K_hinf = 293.9;

num_hinf = K_hinf * conv([1, z_lead], [1, z_lag]);
den_hinf = [1, p_lag, 0];
G_hinf = tf(num_hinf, den_hinf);

%% --- 12. LQR+I SIMO: tablas de ganancias programadas (para el .slx) ---
% fila (j-1)*nGs+i <-> (theta_i, d_j); reshape(K(2x5),1,10)
thg_simo = 10:5:80;                         % [deg] rejilla de scheduling en theta
dg_simo  = linspace(d_min, d_max, 9)*100;   % [cm]  rejilla de scheduling en d (5:2.5:25 cm)
nGs      = numel(thg_simo);   nDs = numel(dg_simo);
Q_simo   = diag([q_th, q_om, 0, q_d, q_xi]);
R_simo   = diag([r_pwm, r_d]);
kd_eq    = k_m * (d0/d_eq)^2;               % [N*m/%] ganancia aerodinamica en d_eq (= d_ini)
B_simo   = [0 0; 0 0; 1/fan_tau 0; 0 1/tau_car; 0 0];
Kg_simo  = zeros(nGs*nDs, 10);
for jS = 1:nDs
    dj   = dg_simo(jS)/100;                                   % [m]
    kd_j = k_m * (d0/dj)^2;                                   % [N*m/%]
    for iS = 1:nGs
        th_s   = deg2rad(thg_simo(iS));
        veq_s  = m_g*g*l_c*sin(th_s) / kd_j;                  % [%] (puede superar pwm_max: equilibrio inalcanzable)
        a21_s  = -m_g*g*l_c*cos(th_s) / J_g;                  % [1/s^2] restauradora (negativa)
        gd_s   = -2*k_m*veq_s*d0^2 / (J_g*dj^3) / 100;        % [rad/s^2 por cm]
        A_s = [0,     1,        0,            0,            0;
               a21_s, -b_c/J_g, kd_j/J_g,     gd_s,         0;
               0,     0,        -1/fan_tau,   0,            0;
               0,     0,        0,            -1/tau_car,   0;
               1,     0,        0,            0,            0];
        Kg_simo((jS-1)*nGs+iS,:) = reshape(lqr(A_s, B_simo, Q_simo, R_simo), 1, []);
    end
end
ctrl_prm = [d0, k_m, m_g*g*l_c, d_min, d_max, pwm_min, pwm_max];                 % parametros del bloque LQR_SIMO
sup_prm  = [sup_v_hi, sup_v_lo, sup_adv/100, sup_ret/100, d_ini, pwm_min];  % bloque Supervisor_d
u_eq0    = min(max(m_g*g*l_c*sin(theta0)/kd_eq, 0), pwm_max); % [%] estado inicial del ventilador (0 = apagado en reposo)

fprintf('=================================================================\n');
fprintf('  PARAMETROS DEL SISTEMA (init_parametros.m)\n');
fprintf('=================================================================\n');
fprintf('  * Planta G_p(s):             %.2f / (s^2 + %.2f s + %.2f)  (estable, gravedad restauradora)\n', b2, -a22, -a21);
fprintf('  * Polos de la planta:        %s rad/s\n', mat2str(round(polos_ol.', 3)));
fprintf('  * Sensor potenciometro:      0V=0 deg | %.2fV=90 deg | %.1fV=180 deg (K_pot = %.5f V/deg)\n', V_pot_mid, V_pot_max, K_pot_deg);
fprintf('  * ADC:                       %d bits (%d cuentas), %.4f deg/LSB\n', adc_bits, adc_counts, adc_res_deg);
fprintf('  * PWM de equilibrio nominal: %.2f %%\n', u_eq);
fprintf('  * PID (LGR formal):          Kp=%.2f, Ki=%.2f, Kd=%.2f (Ceros en s=%.2f, s=%.2f)\n', ...
    pid_Kp, pid_Ki, pid_Kd, ceros_pid(1), ceros_pid(2));
fprintf('  * LQR+I (Riccati):           K=[%.2f, %.2f, %.2f]\n', K_lqr(1), K_lqr(2), K_lqr(3));
fprintf('  * Loop shaping:              K_h=%.2f, z1=%.2f, z2=%.2f, p=%.2f\n', K_hinf, z_lead, z_lag, p_lag);
fprintf('  * SIMO: tau_car=%.2f s, r_pwm=%.3g, r_d=%.3g, d_ini=%.1f cm, usar_imu=%d | supervisor: acerca carro si PWM>%g %%\n', tau_car, r_pwm, r_d, d_ini_cm, usar_imu, sup_v_hi);
fprintf('=================================================================\n');
