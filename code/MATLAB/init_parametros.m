% Nota: no se hace 'clear' aqui para preservar variables del llamador
% (ani, txt_esc, ovr, etc.). Todas las variables de este script se
% asignan explicitamente, por lo que no quedan valores residuales.
clc;
%% Constantes Generales
g       = 9.81;          % [m/s^2] aceleracion de la gravedad
rho_air = 1.225;         % [kg/m^3] densidad del aire a nivel del mar

%% Compuerta Fisica 
gate_t      = 0.0015;    % [m] espesor del material 
gate_W      = 0.15;      % [m] ancho de la compuerta 
gate_L      = 0.20;      % [m] largo de la compuerta 
gate_A      = gate_W * gate_L; % [m^2] area expuesta total 
m_g         = 0.015;     % [kg] masa total de la compuerta 
l_c         = 0.10;      % [m] distancia del pivote al centro de masa 
J_g         = 0.0002;    % [kg*m^2] momento de inercia respecto al eje de giro
b_c         = 0.0010;    % [N*m*s/rad] amortiguamiento viscoso del pivote
tau_f       = 0.0001;    % [N*m] friccion de Coulomb/estatica en el pivote
theta_min   = 0.0;       % [rad] limite mecanico inferior (reposo)
theta_max   = pi/2;        % [rad] limite mecanico superior (90 deg)
tau_stop    = 0.005;     % [s] constante de tiempo del tope mecanico (choque inelastico)

%% Sensor de Angulo (Potenciometro 0 - 3.3 V, ADC 12 bits del ESP32)
V_pot_min   = 0.0;       % [V] voltaje a 0 deg
V_pot_mid   = 0.825;     % [V] voltaje a 45 deg
V_pot_max   = 1.65;      % [V] voltaje a 90 deg
K_pot_deg   = V_pot_max / 90.0;   % [V/deg] 0.018333 V/deg (18.33 mV/deg)
K_pot_rad   = V_pot_max / (pi/2);      % [V/rad] ~1.0504 V/rad

adc_bits    = 12;                  % [-] resolucion del ADC del ESP32 (0 - 4095)
adc_counts  = 2^adc_bits - 1;      % 4095 cuentas
adc_Vref    = 3.3;                 % [V] rango nativo 0 - 3.3 V
adc_res_V   = adc_Vref/adc_counts;    % [V/LSB] voltaje por cuenta (aprox 0.0008 V)
adc_res_deg = adc_res_V / K_pot_deg;  % [deg/LSB] ~0.0440 deg/LSB

%% Ventilador (Actuador Aerodinamico: Sanyo Denki San Ace 40)
fan_Vnom    = 24.0;      % [V] voltaje nominal de operacion (L298N a 24 VDC)
fan_tau     = 0.15;      % [s] constante de tiempo del ventilador (polo en -6.67 rad/s)
pwm_min     = 10.0;      % [%] ciclo de trabajo minimo del ventilador (limite de saturacion)
pwm_max     = 90.0;      % [%] ciclo de trabajo maximo del ventilador (limite de saturacion)

% d0 = distancia de REFERENCIA aerodinamica; no cambia.
d0          = 0.10;      % [m] distancia de referencia carro-compuerta (10 cm)
k_m         = 0.00030;   % [N*m/%] torque aerodinamico a d=d0 por cada 1% de PWM 

%% Carro Movil
m_car        = 0.421;    % [kg] masa del carro completo
car_b        = 3.5;      % [N*s/m] amortiguamiento lineal
car_F_max    = 3.0;      % [N] empuje maximo de traccion (2 motorreductores TGP01D-A130)
d_ini_cm     = 10.0;     % [cm] distancia inicial de arranque (10 cm)
d_min        = 0.05;     % [m] distancia minima permitida (5 cm)
d_max        = 0.25;     % [m] distancia maxima permitida (25 cm)
car_deadband = 0.005;    % [m] banda muerta del lazo de posicion (+/-5 mm)

% PID de posicion del carro (u_car en % de empuje maximo; positivo = avanzar):
%   u_car = Kp*e_a + Ki*int(e_a) - Kd*v_adv ,  e_a = d_est - d_ref
car_Kp = 130;            % [%/m]     (equivale a ~3.9 N/m -> tau del lazo ~0.7 s)
car_Ki = 10;             % [%/(m*s)]
car_Kd = 5;              % [%*s/m]   (derivada sobre la velocidad estimada)

ceros_pid = roots([car_Kd, car_Kp, car_Ki]);

% Sensores IMU (acelerometro longitudinal; positivo = avanzar hacia la compuerta):
imu_bias      = 0.02;    % [m/s^2] sesgo constante del acelerometro
imu_noise_pow = 1e-5;    % [(m/s^2)^2*s] densidad espectral de potencia del ruido
imu_Ts        = 0.01;    % [s] periodo de muestreo de la IMU (100 Hz)
obs_qv        = 0.5;     % covarianza de proceso para velocidad
obs_qb        = 1.0;     % covarianza de proceso para sesgo

%% Condiciones Iniciales, Referencia y Parametros del Controlador SIMO
theta_ref_deg = 35.0;    % [deg] angulo de consigna deseado
theta0_deg    = 0.0;     % [deg] angulo inicial 
t_sim         = 10.0;     % [s] tiempo de simulacion
tau_ref       = 0.15;    % [s] filtro de primer orden de la referencia
ref_rate_deg  = 200.0;    % [deg/s] velocidad maxima de la referencia
usar_imu      = 1;       % 1: el controlador usa d estimada (IMU+Kalman); 0: d real (ideal)
fan_fac       = 1.0;     % [-] fuerza del ventilador REAL al final de la degradacion / la supuesta (k_m).
                         %     1 = sin degradacion; 0.4 = pierde 60 % (p. ej. caida de bateria). Solo afecta a la planta
fan_t0        = 3.0;     % [s] inicio de la degradacion del ventilador
fan_dur       = 12.0;    % [s] duracion de la rampa de degradacion
gov_deg       = 90.0;     % [deg] gobernador de referencia: theta_ref <= theta_medida + gov_deg
                         %       (levanta la compuerta suave desde el suelo; 90 = desactivado)

% Supervisor de distancia:
sup_v_hi = 90;           % [%]    si el PWM demandado supera este umbral, el carro AVANZA
sup_v_lo = 10;           % [%]    si queda por debajo, el carro vuelve hacia d_ini
sup_adv  = 5;            % [cm/s] velocidad maxima del punto de operacion al acercarse
sup_ret  = 2;            % [cm/s] velocidad de regreso hacia d_ini
xi_lim   = 2.0;          % [rad*s] limite de la integral del error (autoridad ~ |K15|*xi_lim en % de PWM);
                         %         la integral se congela si el PWM satura empujando en la misma direccion
xi_band_deg = 5.0;      % [deg]  la integral del error solo acumula con |error| < xi_band_deg (evita sobrepico en la subida)

% LQR+I SIMO: x = [d_theta; omega; d_v_fan; d_d(cm); x_i], u = [d_u_PWM; d_d_ref(cm)]
tau_car   = 0.8;         % [s]   lazo interno del carro (canal lento, ~5x fan_tau)
v_car_max = 0.75;        % [m/s] velocidad maxima del carro (motorreductores)
q_th      = 10000;          % peso de d_theta   [1/rad^2]
q_om      = 60;           % peso de omega
q_d       = 0.5;         % peso de d_d       [1/cm^2]
q_xi      = 10000;         % peso de la integral del error
r_pwm     = 0.02;        % peso de d_u_PWM   [1/%^2]
r_d       = 1.0;        % peso de d_d_ref   [1/cm^2] (r_d alto => carro casi inactivo)
tau_w     = 0.04;        % [s] filtro de la derivada que estima omega a partir del ADC

% Perturbacion externa de torque sobre la compuerta (pulso rectangular):
pert_tau_mNm = 0.5;      % [mN*m] amplitud (0 = sin perturbacion)
pert_t_on    = 4.0;      % [s] instante de inicio
pert_dur     = 2.0;      % [s] duracion

%% Geometria visual del montaje (para animacion cinematica)
% Estas constantes describen las dimensiones fisicas reales del montaje [cm]
H_pivote    = 24;      % [cm] altura del pivote sobre el suelo
car_len_cm  = 16;      % [cm] longitud del chasis del carro
wheel_r_cm  = 3.25;    % [cm] radio de las ruedas
y_chasis_lo = 3.5;     % [cm] altura inferior del chasis (sobre el suelo)
y_chasis_hi = 10;      % [cm] altura superior del chasis
fan_w_cm    = 4.0;     % [cm] ancho del ventilador (40 mm)
fan_t_cm    = 2.8;     % [cm] profundidad del ventilador (28 mm)
x_post_cm   = -26;     % [cm] posicion x del poste de la L invertida

%% Sobrescritura desde Interfaz de Usuario
if exist('ovr','var') && isstruct(ovr)
    fn_ovr = fieldnames(ovr);
    for i_ovr = 1:numel(fn_ovr)
        eval([fn_ovr{i_ovr} ' = ovr.' fn_ovr{i_ovr} ';']);
    end
end
clear ovr fn_ovr i_ovr

% Magnitudes derivadas:
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

%% Linealizacion Formal de la Planta (Polos y Ceros en Lazo Abierto)
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

%% LQR+I SIMO: tablas de ganancias programadas (para el .slx) ---

thg_simo = 10:5:80;                         % [deg] rejilla de scheduling en theta
dg_simo  = linspace(d_min, d_max, 9)*100;   % [cm]  rejilla de scheduling en d (5:2.5:25 cm)
nGs      = numel(thg_simo);   nDs = numel(dg_simo);
Q_simo   = diag([q_th, q_om, 0, q_d, q_xi]);
R_simo   = diag([r_pwm, r_d]);
kd_eq    = k_m * (d0/d_eq)^2;               % [N*m/%] ganancia aerodinamica en d_eq (= d_ini)
B_simo   = [0 0; 0 0; 1/fan_tau 0; 0 1/tau_car; 0 0];
Kg_simo  = zeros(nGs*nDs, 10);              % fila (j-1)*nGs+i <-> (theta_i, d_j); reshape(K(2x5),1,10)
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
fprintf('  * Planta G_p(s):             %.2f / (s^2 + %.2f s + %.2f)\n', b2, -a22, -a21);
fprintf('  * Polos de la planta:        %s rad/s\n', mat2str(round(polos_ol.', 3)));
fprintf('  * Sensor potenciometro:      0V=0 deg | %.2fV=45 deg | %.1fV=90 deg (K_pot = %.5f V/deg)\n', V_pot_mid, V_pot_max, K_pot_deg);
fprintf('  * ADC:                       %d bits (%d cuentas), %.4f deg/LSB\n', adc_bits, adc_counts, adc_res_deg);
fprintf('  * PWM de equilibrio nominal: %.2f %%\n', u_eq_ref);
fprintf('  * PID rapido (carro):        Kp=%.2f, Ki=%.2f, Kd=%.2f (Ceros en s=%.2f, s=%.2f)\n', ...
    car_Kp, car_Ki, car_Kd, ceros_pid(1), ceros_pid(2)); 
fprintf('  * LQR+I SIMO (Riccati):      Generadas %d matrices K por Gain Scheduling\n', size(Kg_simo, 1)); 
fprintf('  * Supervisor de distancia:   Acerca el carro si el ventilador satura > %g %%\n', sup_v_hi);
fprintf('=================================================================\n');