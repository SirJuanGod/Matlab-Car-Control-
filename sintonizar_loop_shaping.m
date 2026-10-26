%% sintonizar_loop_shaping.m
% =========================================================================
% SINTONIA DE LOS CONTROLADORES DE SOLO VENTILADOR (compuerta colgante, planta estable)
%   1) PID rapido:         C(s) = Kp + Ki/s + Kd*s/(s/N+1)   (N = pid_N)
%   2) Compensador robusto (loop shaping): C(s) = K_h (s+z_lead)(s+z_lag) / [ s (s+p_lag) ]
% Planta vista por el controlador: G(s) = G_p(s) * 1/(tau_fan*s+1)  (con ventilador), en theta_ref.
% Metodo: optimizacion directa (fminsearch) sobre el modelo lineal con Control System Toolbox
% (feedback, allmargin, step). systune sin una meta de desempeno converge a un controlador
% casi puramente integral (sobrepico 25 %), por eso no se usa aqui.
% PID:       minimiza ITAE de la respuesta al escalon con sobrepico <= 2 % y PM >= 50 deg.
% Robusto:   minimiza Ms + 2*ITAE con PM >= 55 deg y tolerancia a que la ganancia efectiva del
%            ventilador cambie al mover el carro (factor (10/d)^2: d de 7 a 15 cm => x2 a x0.44).
% NO modifica init_parametros.m: imprime los valores a copiar en sus secciones 9 y 11.
% =========================================================================
clear; clc; close all;
run('init_parametros.m');
s = tf('s');
G = tf(b2, [1, -a22, -a21]) * tf(1, [fan_tau, 1]);
N  = pid_N;  tg = (0:0.002:3)';
opt = optimset('Display','off','TolX',1e-5,'TolFun',1e-8,'MaxIter',6000,'MaxFunEvals',12000);
rng(5);

%% --- 1. PID rapido ---
fpid = @(q) costo_pid(exp(q), G, N, tg);
best = []; fb = Inf;
for k = 1:25
    q0 = log([10, 20, 1] .* exp(0.8*randn(1,3)));
    [q, f] = fminsearch(fpid, q0, opt);
    if f < fb, fb = f; best = exp(q); end
end
pid_new = best;   % [Kp Ki Kd]
C_pid = pid_new(1) + pid_new(2)/s + pid_new(3)*s/(s/N+1);

%% --- 2. Compensador robusto (loop shaping) ---
fls = @(q) costo_ls(exp(q), G, p_lag, tg);
best = []; fb = Inf;
for k = 1:25
    q0 = log([100, 5, 1] .* exp(0.8*randn(1,3)));
    [q, f] = fminsearch(fls, q0, opt);
    if f < fb, fb = f; best = exp(q); end
end
ls_new = best;    % [K_h z_lead z_lag]
C_ls = ls_new(1) * (s + ls_new(2)) * (s + ls_new(3)) / (s * (s + p_lag));

%% --- Comparacion ---
C_old = tf(293.9*conv([1 4.214],[1 0.196]), [1 30 0]);   % sintonia anterior (planta inestable)
cand = {'PID rapido (nuevo)', C_pid; 'Loop shaping robusto (nuevo)', C_ls; 'Loop shaping anterior', C_old};
fprintf('\n%-30s %-12s %-8s %-6s %-9s %-26s\n', 'Compensador', 'GM (lo,hi)', 'PM[deg]', 'Ms', 'Estable', 'Escalon: subida / sobrepico / ts2%');
for i = 1:size(cand,1)
    L = cand{i,2} * G;  T = feedback(L, 1);  est = max(real(pole(T))) < 0;  m = allmargin(L);
    gm = m.GainMargin(:);  g_lo = max([gm(gm < 1); 0]);  g_hi = min([gm(gm > 1); Inf]);
    if est
        y = step(T, tg);  yf = y(end);  i10 = find(y >= 0.1*yf, 1);  i90 = find(y >= 0.9*yf, 1);  ib = find(abs(y-yf) > 0.02*abs(yf), 1, 'last');
        if isempty(ib), ib = 1; end
        fprintf('%-30s [%4.2f %5.2f]  %-8.1f %-6.2f %-9s %.2f s / %.1f %% / %.2f s\n', cand{i,1}, g_lo, g_hi, min(m.PhaseMargin), norm(feedback(1,L), inf), 'si', tg(i90)-tg(i10), 100*(max(y)-yf)/yf, tg(min(ib+1,numel(tg))));
    else
        fprintf('%-30s %-12s %-8s %-6s %-9s\n', cand{i,1}, '-', '-', '-', 'NO');
    end
end
fprintf('\nEstabilidad segun la distancia del carro (ganancia x (10/d)^2), max Re de los polos:\n');
for d = [5 7 10 12 15 20]
    f = (10/d)^2;
    fprintf('  d = %2d cm (x%.2f):  PID %+7.2f | robusto %+7.2f | anterior %+7.2f\n', d, f, max(real(pole(feedback(f*C_pid*G,1)))), max(real(pole(feedback(f*C_ls*G,1)))), max(real(pole(feedback(f*C_old*G,1)))));
end
fprintf('\nValores para init_parametros.m:\n');
fprintf('  seccion 9 (PID):       pid_Kp = %.3f;  pid_Ki = %.3f;  pid_Kd = %.3f;\n', pid_new);
fprintf('  seccion 11 (robusto):  z_lead = %.3f;  z_lag = %.3f;  p_lag = %.1f;  K_hinf = %.1f;\n', max(ls_new(2:3)), min(ls_new(2:3)), p_lag, ls_new(1));

figure('Name','Compensadores de solo ventilador','Color','w','Position',[60 60 1100 480]);
subplot(1,2,1); hold on; grid on; box on;
for i = 1:size(cand,1), bodemag(cand{i,2}*G); end
legend(cand{:,1}, 'Location', 'southwest'); title('|L(j\omega)| con el ventilador');
subplot(1,2,2); hold on; grid on; box on;
for i = 1:size(cand,1)
    T = feedback(cand{i,2}*G, 1);
    if max(real(pole(T))) < 0, plot(tg, step(T, tg), 'LineWidth', 1.5); else, plot(NaN, NaN); end
end
legend(cand{:,1}, 'Location', 'best'); xlabel('t [s]'); title('Escalon de referencia (lazo cerrado lineal)'); ylim([0 1.6]);
saveas(gcf, 'comparacion_loop_shaping.png');

%% ============================ Funciones locales ============================
function c = costo_pid(p, G, N, tg)
    s = tf('s');  C = p(1) + p(2)/s + p(3)*s/(s/N+1);  L = C*G;  T = feedback(L, 1);
    pol = pole(T);
    if max(real(pol)) >= -0.5, c = 1e6 + max(real(pol)); return; end
    y = step(T, tg);  e = 1 - y;  itae = trapz(tg, tg .* abs(e));
    ovs = max(0, max(y) - 1);  m = allmargin(L);  pm = min([m.PhaseMargin(:); 90]);
    c = itae + 200*max(0, ovs-0.02) + 0.05*max(0, 50-pm);
end

function c = costo_ls(p, G, p_lag, tg)
    s = tf('s');  C = p(1)*(s+p(2))*(s+p(3))/(s*(s+p_lag));  L = C*G;  T = feedback(L, 1);
    pol = pole(T);
    if max(real(pol)) >= -0.5, c = 1e6 + max(real(pol)); return; end
    y = step(T, tg);  itae = trapz(tg, tg .* abs(1 - y));
    m = allmargin(L);  pm = min([m.PhaseMargin(:); 90]);  gm = m.GainMargin(:);
    g_lo = max([gm(gm < 1); 0]);  g_hi = min([gm(gm > 1); 100]);
    Ms = norm(feedback(1, L), inf);
    c = Ms + 2*itae + 0.1*max(0, 55-pm) + 5*max(0, 2.5-g_hi) + 5*max(0, g_lo-0.45);
end
