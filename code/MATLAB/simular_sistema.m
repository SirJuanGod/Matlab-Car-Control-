%% simular_sistema.m — Comparativa: PID (LGR) vs LQR+I vs Loop Shaping.

clear; clc; close all;

%% --- 1. Inicializar parametros y sintonia analitica ---
run('init_parametros.m');

%% --- 2. Perfil de Simulacion y Perturbacion ---
dt = 0.002; % 500 Hz
tspan = 0:dt:t_sim;
Nt = numel(tspan);

% Trayectoria de referencia con filtro suave (tau_ref = 0.45 s) para evitar flip:
th_ref_prof = zeros(Nt, 1);
th_curr_ref = theta0;
tau_ref = 0.45;
for k = 1:Nt
    th_curr_ref = th_curr_ref + dt * ((theta_ref - th_curr_ref) / tau_ref);
    th_ref_prof(k) = th_curr_ref;
end

% Perfil de distancia real del carro d(t):
d_real = zeros(Nt, 1);
for k = 1:Nt
    tk = tspan(k);
    if tk < 3.5
        d_real(k) = d0; % 10 cm
    else
        d_real(k) = d0 + 0.04 * (1 - exp(-(tk - 3.5)/0.4)); % retroceso a 14 cm
    end
end

v_car_real = [0; diff(d_real)/dt];
a_car_real = [0; diff(v_car_real)/dt];
rng(42);
a_meas = a_car_real + imu_bias + sqrt(imu_noise_pow/imu_Ts)*randn(Nt, 1);

x_obs = [0; 0];
d_est = zeros(Nt, 1);
d_raw = zeros(Nt, 1);
x_pos_est = 0;
x_pos_raw = 0;
v_raw = 0;

for k = 1:Nt
    y_meas = a_meas(k);
    y_pred = C_obs * x_obs;
    e_y = y_meas - y_pred;
    x_obs = x_obs + dt * (A_obs * x_obs + L_obs * e_y);
    x_pos_est = x_pos_est + dt * x_obs(1);
    d_est(k) = d0 + x_pos_est;
    
    v_raw = v_raw + dt * a_meas(k);
    x_pos_raw = x_pos_raw + dt * v_raw;
    d_raw(k) = d0 + x_pos_raw;
end

%% --- 3. Preparacion de los Controladores Analiticos ---
modos = {'PID (Diseno por LGR)', 'LQR+I (Ecuacion de Riccati)', 'Control Robusto H_\infty (Loop Shaping)'};
colores = {'#D9531F', '#0072BD', '#2CA02C'}; % Naranja, Azul, Verde

kp_p = pid_Kp;
ki_p = pid_Ki;
kd_p = pid_Kd;  % derivada sobre omega; diseno de init_parametros.m (incluye el lag del ventilador)

G_hinf_d = c2d(G_hinf, dt, 'tustin');
[b_hinf, a_hinf] = tfdata(G_hinf_d, 'v');

resultados = struct();

for m = 1:3
    theta_hist  = zeros(Nt, 1);
    omega_hist  = zeros(Nt, 1);
    pwm_hist    = zeros(Nt, 1);
    vfan_hist   = zeros(Nt, 1);
    tau_hist    = zeros(Nt, 1);
    vpot_hist   = zeros(Nt, 1);
    adc_hist    = zeros(Nt, 1);
    
    theta = theta0;
    omega = 0.0;
    vfan  = 0.0;
    integ_err = 0.0;
    
    e_buf = zeros(numel(b_hinf), 1);
    u_buf = zeros(numel(a_hinf), 1);
    
    for k = 1:Nt
        tk = tspan(k);
        dk = d_real(k);
        th_ref_k = th_ref_prof(k);
        
        % Sensor de potenciometro lineal: 0V <=> 0 deg, 1.65V <=> 90 deg, 3.3V <=> 180 deg
        theta_deg = rad2deg(theta);
        v_pot = K_pot_deg * theta_deg;
        v_pot = min(max(v_pot, V_pot_min), V_pot_max);
        
        % Cuantizacion ADC (12 bits sobre 3.3 V, conexion directa al ESP32):
        adc_val = round((v_pot / adc_Vref) * adc_counts);
        adc_val = min(max(adc_val, 0), adc_counts);
        theta_meas_deg = (adc_val / adc_counts) * 180.0;
        theta_meas = deg2rad(theta_meas_deg);
        
        err = th_ref_k - theta_meas;
        
        % Feedforward gravitacional base a d0:
        u_ff = (m_g * g * l_c * sin(th_ref_k)) / k_m;      % gravedad ~ sin(theta): compuerta colgante
        
        if m == 1
            integ_err = integ_err + dt * err;
            integ_err = min(max(integ_err, -0.6), 0.6);
            
            u_pid = kp_p * err + ki_p * integ_err - kd_p * omega;
            u_raw = u_ff + u_pid;
            u_cmd = min(max(u_raw, pwm_min), pwm_max);
            
        elseif m == 2
            integ_err = integ_err + dt * (theta_meas - th_ref_k);
            integ_err = min(max(integ_err, -0.6), 0.6);
            
            u_lqr = -K_lqr(1)*(theta_meas - th_ref_k) - K_lqr(2)*omega - K_lqr(3)*integ_err;
            u_raw = u_ff + u_lqr;
            u_cmd = min(max(u_raw, pwm_min), pwm_max);
            
        else
            e_buf = [err; e_buf(1:end-1)];
            u_filt = (b_hinf * e_buf - a_hinf(2:end) * u_buf(1:end-1)) / a_hinf(1);
            u_buf = [u_filt; u_buf(1:end-1)];
            
            u_raw = u_ff + u_filt;
            u_cmd = min(max(u_raw, pwm_min), pwm_max);
        end
        
        % Dinamica del ventilador:
        vfan = vfan + dt * ((u_cmd - vfan) / fan_tau);
        vfan = min(max(vfan, 0.0), 100.0);
        
        % Torque aerodinamico con ley cuadratica de distancia:
        dist_factor = (d0 / dk)^2;
        tau_aero = k_m * vfan * dist_factor;
        
        % Dinamica de la compuerta pasiva:
        tau_grav = m_g * g * l_c * sin(theta);
        tau_fricc = b_c * omega + tau_f * tanh(50.0 * omega);
        alpha = (tau_aero - tau_grav - tau_fricc) / J_g;
        
        omega = omega + dt * alpha;
        theta = theta + dt * omega;
        
        if theta <= 0.0
            theta = 0.0;
            if omega < 0, omega = 0.0; end
        elseif theta >= pi
            theta = pi;
            if omega > 0, omega = 0.0; end
        end
        
        theta_hist(k) = rad2deg(theta);
        omega_hist(k) = omega;
        pwm_hist(k)   = u_cmd;
        vfan_hist(k)  = vfan;
        tau_hist(k)   = tau_aero * 1000; % [mN*m]
        vpot_hist(k)  = v_pot;
        adc_hist(k)   = adc_val;
    end
    
    resultados.(sprintf('modo_%d', m)).theta = theta_hist;
    resultados.(sprintf('modo_%d', m)).pwm   = pwm_hist;
    resultados.(sprintf('modo_%d', m)).vpot  = vpot_hist;
    resultados.(sprintf('modo_%d', m)).tau   = tau_hist;
end

%% --- 4. Generacion de Graficas Comparativas ---
fig = figure('Name','Comparativa Formal: PID (LGR) vs LQR+I vs H-infinita','Position',[50 50 1240 830],'Color','w');

% Subplot 1: Angulo de la compuerta
subplot(3, 2, 1); hold on; grid on; box on;
plot(tspan, rad2deg(th_ref_prof), '--k', 'LineWidth', 1.5, 'DisplayName', 'Referencia Suave');
yline(theta_ref_deg, ':k', '35^\circ Final', 'HandleVisibility', 'off');
xline(3.5, ':r', 'Retroceso Carro (10 \rightarrow 14 cm)', 'LabelVerticalAlignment', 'bottom');
for m = 1:3
    plot(tspan, resultados.(sprintf('modo_%d', m)).theta, 'Color', colores{m}, 'LineWidth', 1.8, ...
        'DisplayName', modos{m});
end
ylabel('\theta [^\circ] (Inclinacion)', 'FontWeight', 'bold');
xlabel('Tiempo [s]'); title('Respuesta Angular de la Compuerta \theta(t)');
legend('Location', 'southeast'); ylim([0, 45]);

% Subplot 2: Senal PWM enviada al Ventilador
subplot(3, 2, 2); hold on; grid on; box on;
xline(3.5, ':r', 'Retroceso Carro', 'HandleVisibility', 'off');
for m = 1:3
    plot(tspan, resultados.(sprintf('modo_%d', m)).pwm, 'Color', colores{m}, 'LineWidth', 1.6, ...
        'DisplayName', modos{m});
end
ylabel('PWM [%]', 'FontWeight', 'bold');
xlabel('Tiempo [s]'); title('Senal PWM Enviada al Ventilador u_{PWM}(t)');
legend('Location', 'northwest'); ylim([0, 105]);

% Subplot 3: Voltaje del Potenciometro (0V - 1.65V - 3.3V)
subplot(3, 2, 3); hold on; grid on; box on;
v_ref_expected = K_pot_deg * theta_ref_deg;
yline(v_ref_expected, '--k', sprintf('V_{ref} = %.2f V', v_ref_expected), 'LineWidth', 1.5);
yline(V_pot_mid, ':m', '90^\circ = 1.65 V', 'HandleVisibility', 'off');
for m = 1:3
    plot(tspan, resultados.(sprintf('modo_%d', m)).vpot, 'Color', colores{m}, 'LineWidth', 1.6, ...
        'DisplayName', modos{m});
end
ylabel('V_{pot} [V] (Sensor)', 'FontWeight', 'bold');
xlabel('Tiempo [s]'); title('Voltaje del Potenciometro (0V=0^\circ, 1.65V=90^\circ, 3.3V=180^\circ)');
legend('Location', 'southeast'); ylim([0, 1.5]);

% Subplot 4: Distancia Carro-Compuerta (Real vs Estimada IMU)
subplot(3, 2, 4); hold on; grid on; box on;
plot(tspan, d_real * 100, 'k', 'LineWidth', 2.0, 'DisplayName', 'Distancia Real d(t)');
plot(tspan, d_est * 100, '--b', 'LineWidth', 1.5, 'DisplayName', 'Estimada IMU + Kalman');
plot(tspan, d_raw * 100, ':r', 'LineWidth', 1.2, 'DisplayName', 'Doble Integracion Cruda (Deriva)');
xline(3.5, ':r', 'Movimiento Carro', 'HandleVisibility', 'off');
ylabel('Distancia d [cm]', 'FontWeight', 'bold');
xlabel('Tiempo [s]'); title('Plataforma Movil Independiente: Distancia Inicial = 10 cm');
legend('Location', 'northwest'); ylim([8, 18]);

% Subplot 5: Torque Aerodinamico del Chorro
subplot(3, 2, 5); hold on; grid on; box on;
xline(3.5, ':r', 'Retroceso Carro', 'HandleVisibility', 'off');
for m = 1:3
    plot(tspan, resultados.(sprintf('modo_%d', m)).tau, 'Color', colores{m}, 'LineWidth', 1.6, ...
        'DisplayName', modos{m});
end
ylabel('\tau_{aero} [mN\cdot m]', 'FontWeight', 'bold');
xlabel('Tiempo [s]'); title('Torque Aerodinamico Incidente \tau_{aero}(t)');
legend('Location', 'southeast');

% Subplot 6: Evaluacion de Metricas Comparativas
subplot(3, 2, 6);
cla; axis off;
text(0.02, 0.95, 'TABLA COMPARATIVA DE METRICAS FORMALES', 'FontSize', 12, 'FontWeight', 'bold');
text(0.02, 0.85, 'Escenario: Arranque a \theta_{ref}=35^\circ y Perturbacion en t=3.5s (d: 10 \rightarrow 14 cm)', 'FontSize', 9, 'FontAngle', 'italic');

headers = {'Estrategia', 'Sobrepico [^\circ]', 'Error SS [^\circ]', 'Caida Perturb. [^\circ]', 'PWM Final [%]'};
text(0.02, 0.70, sprintf('%-24s %-15s %-15s %-18s %-12s', headers{:}), 'FontName', 'Courier', 'FontWeight', 'bold');
text(0.02, 0.65, repmat('-', 1, 85), 'FontName', 'Courier');

for m = 1:3
    th_m = resultados.(sprintf('modo_%d', m)).theta;
    pwm_m = resultados.(sprintf('modo_%d', m)).pwm;
    
    mp = max(0, max(th_m(1:round(3.5/dt))) - theta_ref_deg);
    ess = abs(th_m(end) - theta_ref_deg);
    drop = theta_ref_deg - min(th_m(round(3.5/dt):end));
    p_end = pwm_m(end);
    
    line_str = sprintf('%-24s %-15.2f %-15.3f %-18.2f %-12.1f', modos{m}, mp, ess, drop, p_end);
    text(0.02, 0.65 - m*0.12, line_str, 'FontName', 'Courier', 'Color', colores{m}, 'FontWeight', 'bold');
end

text(0.02, 0.16, '* PID (LGR): Asignacion analitica de polos (\zeta=0.85, \omega_n=3.2 rad/s).', 'FontSize', 8.5);
text(0.02, 0.09, '* LQR+I (Riccati): Solucion formal de Riccati con estabilidad KYP garantizada.', 'FontSize', 8.5);
text(0.02, 0.02, '* H_\infty (Loop Shaping): Margen de fase robusto y maxima rapidez ante alejamiento del carro.', 'FontSize', 8.5);

out_dir = fullfile(fileparts(mfilename('fullpath')), 'outputs');
if ~exist(out_dir, 'dir'), mkdir(out_dir); end
saveas(fig, fullfile(out_dir, 'simulacion_comparativa.png'));
fprintf('\nGrafica formal guardada exitosamente en: outputs/simulacion_comparativa.png\n');
