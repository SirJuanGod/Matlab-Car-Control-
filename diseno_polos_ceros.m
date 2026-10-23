%% diseno_polos_ceros.m — Analisis formal de controladores por LGR: PID, LQR+I y loop shaping.

clear; clc; close all;

%% --- 1. Planta (desde init_parametros.m) ---
run('init_parametros.m');

G_p   = tf(b2, [1, -a22, -a21]);     % compuerta pura: b2 / (s^2 + b_c/J s - a21)
G_fan = tf(1, [fan_tau, 1]);         % actuador: 1/(tau_fan s + 1)
G_pl  = G_p * G_fan;                 % planta vista por el controlador
polos_planta = polos_ol;             % polos de la compuerta en lazo abierto

fprintf('=================================================================\n');
fprintf('  ANALISIS FORMAL DE POLOS Y CEROS DE LA PLANTA\n');
fprintf('=================================================================\n');
fprintf('  * Planta mecanica G_p(s) = %.2f / (s^2 + %.2f s - %.2f)\n', b2, -a22, a21);
fprintf('  * Polo inestable en semiplano derecho (RHP): p1 = %+7.3f rad/s\n', p_rhp);
fprintf('  * Polo estable en semiplano izquierdo (LHP):  p2 = %+7.3f rad/s\n', p_lhp);
fprintf('  * Polo del actuador (ventilador):             p_fan = -%.2f rad/s\n', 1/fan_tau);
fprintf('=================================================================\n\n');

%% --- 2. PID por asignacion de polos (ganancias de init_parametros.m) ---
C_pid_tf = tf([pid_Kd, pid_Kp, pid_Ki], [1, 0]);
polos_pid_cl = pole(feedback(C_pid_tf * G_pl, 1));
polos_pid_des = roots(P_des);        % polos deseados (4.o orden: incluye el ventilador)

fprintf('=================================================================\n');
fprintf('  PID POR ASIGNACION DE POLOS (INCLUYE EL VENTILADOR)\n');
fprintf('=================================================================\n');
fprintf('  * Kp = %.3f, Ki = %.3f, Kd = %.3f\n', pid_Kp, pid_Ki, pid_Kd);
fprintf('  * Ceros del PID: z1 = %.3f, z2 = %.3f rad/s\n', ceros_pid(1), ceros_pid(2));
fprintf('  * Polos deseados en LC: %s\n', mat2str(round(sort(real(polos_pid_des)),3).'));
fprintf('  * Polos obtenidos en LC (max Re = %+.3f): %s\n', max(real(polos_pid_cl)), mat2str(round(polos_pid_cl.',3)));
fprintf('=================================================================\n\n');

%% --- 3. LQR+I (Riccati) ---
% Nominal (sin actuador) y con el ventilador: x = [d_theta, d_omega, v_fan, x_i]
K = K_lqr_nom;
A_cl_fan = [0, 1, 0, 0; ...
            a21, a22, b2, 0; ...
            -K(1)/fan_tau, -K(2)/fan_tau, -1/fan_tau, -K(3)/fan_tau; ...
            1, 0, 0, 0];
e_lqr = eig(A_cl_fan);

fprintf('=================================================================\n');
fprintf('  LQR CON ACCION INTEGRAL (LQR+I)\n');
fprintf('=================================================================\n');
fprintf('  * K = [%.3f, %.3f, %.3f]\n', K(1), K(2), K(3));
fprintf('  * Autovalores nominales (sin ventilador): %s\n', mat2str(round(autovalores_lqr.',3)));
fprintf('  * Autovalores CON ventilador (max Re = %+.3f): %s\n', max(real(e_lqr)), mat2str(round(e_lqr.',3)));
fprintf('=================================================================\n\n');

%% --- 4. Compensador por loop shaping (NO es sintesis H-inf) ---
% C(s) = K_h (s + z_lead)(s + z_lag) / [ s (s + p_lag) ]
C_hinf = G_hinf;
L_hinf = C_hinf * G_pl;
info_mrg = allmargin(L_hinf);
polos_hinf_cl = pole(feedback(L_hinf, 1));
hinf_estable = max(real(polos_hinf_cl)) < 0;

fprintf('=================================================================\n');
fprintf('  COMPENSADOR POR LOOP SHAPING\n');
fprintf('=================================================================\n');
fprintf('  * C(s) = %.2f (s + %.2f)(s + %.2f) / [ s (s + %.2f) ]\n', K_hinf, z_lead, z_lag, p_lag);
fprintf('  * Margen de fase: %s deg | margen de ganancia: %s\n', mat2str(round(info_mrg.PhaseMargin,1)), mat2str(round(info_mrg.GainMargin,2)));
fprintf('  * Polos en LC (max Re = %+.3f): %s\n', max(real(polos_hinf_cl)), mat2str(round(polos_hinf_cl.',3)));
if hinf_estable
    fprintf('  -> Lazo cerrado ESTABLE.\n');
else
    fprintf('  -> ATENCION: lazo cerrado INESTABLE con esta sintonia; hay que re-sintonizarlo.\n');
end
fprintf('=================================================================\n\n');

%% --- 5. Graficas ---
fig = figure('Name','Analisis Formal de Polos, Ceros y Lugar Geometrico','Position',[50 50 1200 850],'Color','w');

% Panel 1: Polos de la planta en lazo abierto
subplot(2, 2, 1); hold on; grid on; box on;
fill([-15, 0, 0, -15], [-10, -10, 10, 10], [0.85 0.95 0.85], 'FaceAlpha', 0.3, 'EdgeColor', 'none');
fill([0, 10, 10, 0], [-10, -10, 10, 10], [1.0 0.85 0.85], 'FaceAlpha', 0.3, 'EdgeColor', 'none');
plot(real(polos_planta), imag(polos_planta), 'rx', 'MarkerSize', 12, 'LineWidth', 2.5);
plot(-1/fan_tau, 0, 'mx', 'MarkerSize', 12, 'LineWidth', 2.5);
xline(0, '-k', 'LineWidth', 1.2); yline(0, '-k', 'LineWidth', 1.2);
text(real(p_rhp)+0.3, imag(p_rhp)+0.8, sprintf('Polo inestable\np_1 = +%.2f', p_rhp), 'Color', 'r', 'FontWeight', 'bold');
text(real(p_lhp)-4.5, imag(p_lhp)+0.8, sprintf('Polo estable\np_2 = %.2f', p_lhp), 'Color', 'b', 'FontWeight', 'bold');
text(-1/fan_tau-3.5, 1.8, sprintf('Ventilador\n%.2f', -1/fan_tau), 'Color', 'm', 'FontWeight', 'bold');
text(-12, -8, 'SEMIPLANO ESTABLE (LHP)', 'Color', [0 0.5 0], 'FontWeight', 'bold');
text(1, -8, 'INESTABLE (RHP)', 'Color', [0.7 0 0], 'FontWeight', 'bold');
xlim([-15, 8]); ylim([-10, 10]);
xlabel('Eje Real \sigma [s^{-1}]'); ylabel('Eje Imaginario j\omega [rad/s]');
title('Polos de la Planta en Lazo Abierto');

% Panel 2: LGR con el PID (planta + ventilador)
subplot(2, 2, 2);
rlocus(C_pid_tf * G_pl);
grid on; hold on;
plot(real(ceros_pid), imag(ceros_pid), 'bo', 'MarkerSize', 10, 'LineWidth', 2.0);
plot(real(polos_pid_cl), imag(polos_pid_cl), 'gp', 'MarkerSize', 12, 'MarkerFaceColor', 'g');
title('LGR del PID (planta + ventilador); pentagramas = polos en LC');
xlim([-20, 6]); ylim([-15, 15]);

% Panel 3: Bode de lazo abierto del compensador por loop shaping
subplot(2, 2, 3);
bode(L_hinf);
grid on;
title('Bode de Lazo Abierto (loop shaping + ventilador)');

% Panel 4: Polos en lazo cerrado de los 3 controladores (con ventilador)
subplot(2, 2, 4); hold on; grid on; box on;
fill([-25, 0, 0, -25], [-20, -20, 20, 20], [0.85 0.95 0.85], 'FaceAlpha', 0.25, 'EdgeColor', 'none');
xline(0, '-k', 'LineWidth', 1.2); yline(0, '-k', 'LineWidth', 1.2);
plot(real(polos_pid_cl), imag(polos_pid_cl), 's', 'Color', '#D9531F', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', 'PID (asignacion de polos)');
plot(real(e_lqr), imag(e_lqr), 'd', 'Color', '#0072BD', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', 'LQR+I (Riccati)');
plot(real(polos_hinf_cl), imag(polos_hinf_cl), '^', 'Color', '#2CA02C', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', 'Loop shaping');
xlim([-25, 5]); ylim([-15, 15]);
xlabel('Eje Real \sigma [s^{-1}]'); ylabel('Eje Imaginario j\omega [rad/s]');
title('Polos en Lazo Cerrado (con el ventilador)');
legend('Location', 'northwest');

out_dir = fullfile(fileparts(mfilename('fullpath')), 'outputs');
if ~exist(out_dir, 'dir'), mkdir(out_dir); end
saveas(fig, fullfile(out_dir, 'analisis_polos_ceros_rlocus.png'));
fprintf('Grafica diagnostica actualizada en: outputs/analisis_polos_ceros_rlocus.png\n');
