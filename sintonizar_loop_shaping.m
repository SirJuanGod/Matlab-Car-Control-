%% sintonizar_loop_shaping.m — Sintonia estructurada (systune) del compensador de la compuerta.
clear; clc; close all;
run('init_parametros.m');

GM_obj = 3;      % [dB]  margen de ganancia objetivo
PM_obj = 25;     % [deg] margen de fase objetivo
n_inicios = 10;  % inicios aleatorios de la optimizacion (evita minimos locales)

s = tf('s');
G = tf(b2, [1, -a22, -a21]) * tf(1, [fan_tau, 1]);

%% --- Estructura y sintonia ---
C = ltiblock.pid('C', 'pid');
C.Kp.Value = 38; C.Ki.Value = 5; C.Kd.Value = 6; C.Tf.Value = 1/pid_N;
C.Kp.Minimum = 0; C.Ki.Minimum = 0; C.Kd.Minimum = 0;
C.Tf.Minimum = 1/pid_N; C.Tf.Maximum = 1/10;          % N en [10, pid_N] rad/s

AP = AnalysisPoint('u');                               % punto de apertura: entrada de la planta
T0 = feedback(G * AP * C, 1);
Rmarg = TuningGoal.Margins('u', GM_obj, PM_obj);
Rpol  = TuningGoal.Poles(1.0, 0.4, 60);                % Re < -1, zeta > 0.4, |p| < 60 rad/s
rng(2);
[Tt, f_soft] = systune(T0, Rmarg, Rpol, systuneOptions('RandomStart', n_inicios, 'Display', 'off'));
Ct = tf(getBlockValue(Tt, 'C'));
[num, den] = tfdata(Ct, 'v');
K_h = num(1)/den(1);                                    % ganancia de alta frecuencia
zc  = roots(num);  pc = roots(den);
[~, ip] = max(abs(pc));  p_new = -pc(ip);               % polo de filtro (el otro es el integrador)

%% --- Comparacion: PID / compensador anterior / compensador sintonizado ---
C_pid  = tf([pid_Kd, pid_Kp, pid_Ki], [1, 0]) * tf(1, [1/pid_N, 1]);
C_prev = tf(38 * conv([1, 2.5], [1, 0.8]), [1, 18, 0]);
cand = {'PID (polos asignados)', C_pid; 'Loop shaping anterior', C_prev; 'Loop shaping systune', Ct};
fprintf('\n%-24s %-14s %-8s %-7s %-22s %s\n', 'Compensador', 'GM (lo,hi)', 'PM[deg]', 'Ms', 'Distancia tolerada', 'Gan. alta frec.');
for i = 1:size(cand, 1)
    L = cand{i,2} * G;  m = allmargin(L);
    est = max(real(pole(feedback(L, 1)))) < 0;
    [num_i, den_i] = tfdata(cand{i,2}, 'v');
    if est
        gm = m.GainMargin;
        g_lo = max([gm(gm < 1); 0]);  g_hi = min([gm(gm > 1); Inf]);
        d_txt = sprintf('%.1f a %.1f cm', 10/sqrt(g_hi), 10/sqrt(max(g_lo, eps)));
        fprintf('%-24s [%4.2f %5.2f]   %-8.1f %-7.2f %-22s %.1f\n', cand{i,1}, g_lo, g_hi, min(m.PhaseMargin), norm(feedback(1, L), inf), d_txt, num_i(1)/den_i(1));
    else
        fprintf('%-24s %-14s %-8s %-7s %-22s %.1f\n', cand{i,1}, 'INESTABLE', '-', '-', '-', num_i(1)/den_i(1));
    end
end

fprintf('\nEstabilidad del lazo cerrado segun la distancia del carro (ganancia x (10/d)^2):\n');
for d = [5 7 10 12 14 18 25]
    f = (10/d)^2;
    fprintf('  d = %2d cm (x%.2f):  PID %+7.3f | anterior %+7.3f | systune %+7.3f   (max Re de polos)\n', d, f, ...
        max(real(pole(feedback(f*C_pid*G,1)))), max(real(pole(feedback(f*C_prev*G,1)))), max(real(pole(feedback(f*Ct*G,1)))));
end

fprintf('\nsystune: f_soft = %.3f (<= 1 cumple GM=%g dB / PM=%g deg)\n', f_soft, GM_obj, PM_obj);
fprintf('Valores para init_parametros.m (seccion 11):\n');
if all(abs(imag(zc)) < 1e-9)
    zc = sort(real(zc), 'descend');
    fprintf('  z_lead = %.3f;  z_lag = %.3f;  p_lag = %.1f;  K_hinf = %.1f;\n', -zc(end), -zc(1), p_new, K_h);
else
    fprintf('  ATENCION: ceros complejos (%s); usar num/den directamente.\n', mat2str(zc.', 3));
end

figure('Name', 'Loop shaping: comparacion', 'Color', 'w', 'Position', [60 60 1100 520]);
subplot(1, 2, 1); hold on; grid on; box on;
for i = 1:size(cand, 1), bodemag(cand{i,2} * G); end
legend(cand{:,1}, 'Location', 'southwest'); title('|L(j\omega)| con el ventilador');
subplot(1, 2, 2); hold on; grid on; box on;
t = 0:0.002:6;
for i = 1:size(cand, 1)
    y = step(feedback(cand{i,2} * G, 1), t);
    plot(t, min(max(y, -3), 3), 'LineWidth', 1.5);
end
legend(cand{:,1}, 'Location', 'best'); xlabel('t [s]'); title('Escalon unitario de referencia (recortado a \pm3)');
out_dir = fullfile(fileparts(mfilename('fullpath')), 'outputs');
if ~exist(out_dir, 'dir'), mkdir(out_dir); end
saveas(gcf, fullfile(out_dir, 'comparacion_loop_shaping.png'));
