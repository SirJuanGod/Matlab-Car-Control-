%% generar_figuras.m — Figuras de resultados para la presentacion.
% Corre el modelo no lineal (modelo_miso_compuerta.slx) en varios escenarios,
% la comparativa de simular_sistema.m y una prueba de banco EMULADA
% (simulacion con parametros desajustados + ruido de sensor/turbulencia).
% Uso: matlab -batch "run('outputs/presentacion/matlab/generar_figuras.m')"

set(0, 'DefaultFigureVisible', 'off');
raiz   = fileparts(fileparts(fileparts(fileparts(mfilename('fullpath')))));
dirM   = fullfile(raiz, 'code', 'MATLAB');
dirImg = fullfile(raiz, 'outputs', 'presentacion', 'img');
if ~exist(dirImg, 'dir'), mkdir(dirImg); end
cd(dirM); addpath(dirM);
global PRES_INIT; PRES_INIT = fullfile(dirM, 'init_parametros.m');

C.blue = [0 113 227]/255;  C.red = [255 59 48]/255;  C.green = [52 199 89]/255;
C.orange = [255 149 0]/255; C.purple = [175 82 222]/255; C.gray = [0.45 0.45 0.48];
C.ink = [0.11 0.11 0.12];
mdl = 'modelo_miso_compuerta';  load_system(mdl);
M = struct();

%% 1. Escalon nominal 0 -> 35 deg, carro a 10 cm
fprintf('[1] nominal\n');
o = correr(mdl, struct('pert_tau_mNm', 0));
[t, th] = sig(o.theta_out); th = rad2deg(th);
M.nominal = metricas(t, th, 35, o);
f = fig([1400 800]); tl = tiledlayout(f, 2, 2, 'TileSpacing', 'loose', 'Padding', 'compact');
ax = nexttile(tl, [1 2]); hold(ax, 'on');
yline(ax, 35, '--', 'Color', C.gray, 'LineWidth', 2);
plot(ax, t, th, 'Color', C.blue, 'LineWidth', 4);
estilo(ax, '\theta  [°]', 'Ángulo de la compuerta'); xlim(ax, [0 4]);
ax = nexttile(tl); [tp, p] = sig(o.pwm_out); plot(ax, tp, p, 'Color', C.orange, 'LineWidth', 3.5);
estilo(ax, 'u_{PWM}  [%]', 'Ventilador'); xlim(ax, [0 4]); xlabel(ax, 't  [s]');
ax = nexttile(tl); [td, d] = sig(o.d_out); plot(ax, td, d*100, 'Color', C.green, 'LineWidth', 3.5);
estilo(ax, 'd  [cm]', 'Distancia del carro'); xlim(ax, [0 4]); xlabel(ax, 't  [s]'); ylim(ax, [8 12]);
guardar(f, fullfile(dirImg, 'f1_nominal.png'));
nom = struct('t', t, 'th', th, 'tp', tp, 'p', p, 'td', td, 'd', d);

%% 2. Carro lejos (18 cm) y consigna baja (8 deg)
fprintf('[2] carro lejos / consigna baja\n');
oL = correr(mdl, struct('pert_tau_mNm', 0, 'd_ini_cm', 18));
oB = correr(mdl, struct('pert_tau_mNm', 0, 'theta_ref_deg', 8));
[tL, thL] = sig(oL.theta_out); thL = rad2deg(thL);
[tB, thB] = sig(oB.theta_out); thB = rad2deg(thB);
M.lejos = metricas(tL, thL, 35, oL);  M.baja = metricas(tB, thB, 8, oB);
f = fig([1400 900]); tl = tiledlayout(f, 2, 2, 'TileSpacing', 'loose', 'Padding', 'compact');
ax = nexttile(tl); hold(ax, 'on'); yline(ax, 35, '--', 'Color', C.gray, 'LineWidth', 2);
plot(ax, tL, thL, 'Color', C.blue, 'LineWidth', 4); estilo(ax, '\theta  [°]', 'Carro arranca a 18 cm');
ax = nexttile(tl); hold(ax, 'on'); yline(ax, 8, '--', 'Color', C.gray, 'LineWidth', 2);
plot(ax, tB, thB, 'Color', C.purple, 'LineWidth', 4); estilo(ax, '\theta  [°]', 'Consigna baja: 8°');
ax = nexttile(tl); [t1, d1] = sig(oL.d_out); [t2, p1] = sig(oL.pwm_out);
yyaxis(ax, 'left'); plot(ax, t1, d1*100, '-', 'Color', C.green, 'LineWidth', 3.5); ylabel(ax, 'd  [cm]');
yyaxis(ax, 'right'); plot(ax, t2, p1, '-', 'Color', C.orange, 'LineWidth', 2.5); ylabel(ax, 'u_{PWM}  [%]');
estilo(ax, '', 'El carro se acerca'); xlabel(ax, 't  [s]'); colorejes(ax, C.green, C.orange);
ax = nexttile(tl); [t1, d1] = sig(oB.d_out); [t2, p1] = sig(oB.pwm_out);
yyaxis(ax, 'left'); plot(ax, t1, d1*100, '-', 'Color', C.green, 'LineWidth', 3.5); ylabel(ax, 'd  [cm]');
yyaxis(ax, 'right'); plot(ax, t2, p1, '-', 'Color', C.orange, 'LineWidth', 2.5); ylabel(ax, 'u_{PWM}  [%]');
estilo(ax, '', 'El carro se aleja'); xlabel(ax, 't  [s]'); colorejes(ax, C.green, C.orange);
guardar(f, fullfile(dirImg, 'f2_carro_actua.png'));

%% 3. Perturbaciones: degradacion del ventilador y pulsos de torque
fprintf('[3] perturbaciones\n');
oD = correr(mdl, struct('pert_tau_mNm', 0, 'fan_fac', 0.5, 'fan_t0', 3, 'fan_dur', 12, 't_sim', 20, 'usar_degradacion_fan', 1));
oN = correr(mdl, struct('pert_tau_mNm', -5, 'pert_t_on', 4, 'pert_dur', 3, 't_sim', 12));
oP = correr(mdl, struct('pert_tau_mNm',  5, 'pert_t_on', 4, 'pert_dur', 3, 't_sim', 12));
f = fig([1700 720]); tl = tiledlayout(f, 1, 3, 'TileSpacing', 'loose', 'Padding', 'compact');
casos = {oD, 'Ventilador pierde 50 %', [3 15]; oN, 'Torque −5 mN·m', [4 7]; oP, 'Torque +5 mN·m', [4 7]};
nombres = {'degradacion', 'pulso_neg', 'pulso_pos'};
for k = 1:3
    ax = nexttile(tl); hold(ax, 'on');
    xregion(ax, casos{k,3}(1), casos{k,3}(2), 'FaceColor', C.red, 'FaceAlpha', 0.10);
    yline(ax, 35, '--', 'Color', C.gray, 'LineWidth', 2);
    [tk, thk] = sig(casos{k,1}.theta_out); thk = rad2deg(thk);
    plot(ax, tk, thk, 'Color', C.blue, 'LineWidth', 3.5);
    estilo(ax, '\theta  [°]', casos{k,2}); xlabel(ax, 't  [s]'); ylim(ax, [0 55]);
    M.(nombres{k}) = metricas(tk, thk, 35, casos{k,1});
end
guardar(f, fullfile(dirImg, 'f3_perturbaciones.png'));

%% 4. Comparativa de controladores (simular_sistema.m: el carro retrocede sin avisar)
fprintf('[4] comparativa\n');
try
    [R, ts4, dr, ref4] = correr_comparativa();   % en una funcion: simular_sistema hace 'clear'
    f = fig([1350 800]); tl = tiledlayout(f, 1, 2, 'TileSpacing', 'loose', 'Padding', 'compact');
    ax = nexttile(tl); hold(ax, 'on');
    xregion(ax, 3.5, ts4(end), 'FaceColor', C.red, 'FaceAlpha', 0.07, 'HandleVisibility', 'off');
    plot(ax, ts4, rad2deg(ref4), '--', 'Color', C.gray, 'LineWidth', 2, 'DisplayName', 'Referencia');
    cols = {C.orange, C.blue, C.green};
    labs = {'PID rápido', 'LQR+I', 'Loop shaping'};
    for m = [1 3 2]
        plot(ax, ts4, R.(sprintf('modo_%d', m)).theta, 'Color', cols{m}, 'LineWidth', 3.5, 'DisplayName', labs{m});
    end
    estilo(ax, '\theta  [°]', 'Escalón + carro retrocede en t = 3.5 s'); xlabel(ax, 't  [s]');
    legend(ax, 'Location', 'southeast', 'Box', 'off', 'FontSize', 25);
    ax = nexttile(tl); plot(ax, ts4, dr*100, 'Color', C.ink, 'LineWidth', 4);
    estilo(ax, 'd  [cm]', 'Distancia real (no informada)'); xlabel(ax, 't  [s]'); ylim(ax, [9 15]);
    guardar(f, fullfile(dirImg, 'f4_comparativa.png'));
catch e
    fprintf('  comparativa fallo: %s\n', e.message);
end
set(0, 'DefaultFigureVisible', 'off');

%% 5. Mapa de polos (punto nominal 35 deg, 10 cm; valores de lqr_simo / README §5.4)
fprintf('[5] polos\n');
pa = [0, -2.5+7.35i, -2.5-7.35i, -6.67, -1.25];
pc = [-15.47+18.48i, -15.47-18.48i, -12.3, -1.8, -1];
f = fig([1000 780]); ax = axes(f); hold(ax, 'on');
xline(ax, 0, 'Color', C.gray, 'LineWidth', 1.5, 'HandleVisibility', 'off'); yline(ax, 0, 'Color', C.gray, 'LineWidth', 1.5, 'HandleVisibility', 'off');
for z = [0.32 0.64]
    r = linspace(0, 30, 50); a = acos(z);
    plot(ax, -r*cos(a), r*sin(a), ':', 'Color', C.gray, 'LineWidth', 1.5, 'HandleVisibility', 'off');
    plot(ax, -r*cos(a), -r*sin(a), ':', 'Color', C.gray, 'LineWidth', 1.5, 'HandleVisibility', 'off');
end
scatter(ax, real(pa), imag(pa), 420, C.red, 'x', 'LineWidth', 5, 'DisplayName', 'Lazo abierto');
scatter(ax, real(pc), imag(pc), 320, C.blue, 'filled', 'DisplayName', 'Lazo cerrado LQR+I');
text(ax, -2.2, 9.6, '\zeta = 0.32', 'FontSize', 28, 'Color', C.red, 'FontWeight', 'bold');
text(ax, -14.5, 21.5, '\zeta = 0.64', 'FontSize', 28, 'Color', C.blue, 'FontWeight', 'bold');
estilo(ax, 'Im  [rad/s]', ''); xlabel(ax, 'Re  [rad/s]'); xlim(ax, [-20 2]); ylim(ax, [-24 24]);
legend(ax, 'Location', 'southwest', 'Box', 'off', 'FontSize', 26);
guardar(f, fullfile(dirImg, 'f5_polos.png'));

%% 6. Prueba de banco EMULADA: parametros desajustados + ruido de sensor y turbulencia
fprintf('[6] banco emulado\n');
oE = correr(mdl, struct('pert_tau_mNm', 0, 'k_m', 0.8*3e-4, 'tau_f', 3e-4, 'b_c', 1.3e-3, ...
                        'imu_noise_pow', 1e-4, 'imu_bias', 0.05, 't_sim', 10));
rng(7); Ts = 0.02;                                   % telemetria a 50 Hz (UART)
tb = (0:Ts:10)';
[t0, x0] = sig(oE.theta_out); thb = interp1(t0, rad2deg(x0), tb, 'linear', 'extrap');
turb = filter(0.06, [1 -0.94], randn(size(tb))) * 3.2;   % turbulencia del chorro (coloreada)
thb = thb + turb .* (thb > 3) + 0.22*randn(size(tb));    % solo hay chorro cuando la compuerta se separa
thb = round(thb / 0.044) * 0.044;                    % cuantizacion del ADC
[t0, x0] = sig(oE.pwm_out); pb = interp1(t0, x0, tb, 'linear', 'extrap');
pb = min(90, max(10, pb + 1.4*randn(size(tb)) + filter(0.05, [1 -0.95], randn(size(tb)))*6));
[t0, x0] = sig(oE.d_out); db = interp1(t0, x0*100, tb, 'linear', 'extrap') + 0.12*randn(size(tb)) + 0.15;
M.banco = metricas(tb, movmean(thb, 5), 35, []);
M.banco.ruido_rms = std(thb(tb > 3) - movmean(thb(tb > 3), 25));
f = fig([1400 860]); tl = tiledlayout(f, 2, 2, 'TileSpacing', 'loose', 'Padding', 'compact');
ax = nexttile(tl, [1 2]); hold(ax, 'on');
yline(ax, 35, '--', 'Color', C.gray, 'LineWidth', 2, 'HandleVisibility', 'off');
plot(ax, tb, thb, 'Color', [C.red 0.85], 'LineWidth', 2.2, 'DisplayName', 'Banco (emulado)');
plot(ax, nom.t, nom.th, 'Color', C.blue, 'LineWidth', 3.5, 'DisplayName', 'Simulación');
estilo(ax, '\theta  [°]', 'Ángulo: banco vs simulación'); xlim(ax, [0 10]);
legend(ax, 'Location', 'southeast', 'Box', 'off', 'FontSize', 25);
ax = nexttile(tl); hold(ax, 'on');
plot(ax, tb, pb, 'Color', [C.red 0.85], 'LineWidth', 2); plot(ax, nom.tp, nom.p, 'Color', C.orange, 'LineWidth', 3);
estilo(ax, 'u_{PWM}  [%]', 'Ventilador'); xlabel(ax, 't  [s]'); xlim(ax, [0 10]);
ax = nexttile(tl); hold(ax, 'on');
plot(ax, tb, db, 'Color', [C.red 0.85], 'LineWidth', 2); plot(ax, nom.td, nom.d*100, 'Color', C.green, 'LineWidth', 3);
estilo(ax, 'd  [cm]', 'Distancia (IMU + Kalman)'); xlabel(ax, 't  [s]'); xlim(ax, [0 10]); ylim(ax, [8 12]);
guardar(f, fullfile(dirImg, 'f6_banco.png'));

%% Metricas
fid = fopen(fullfile(dirImg, 'metricas.json'), 'w'); fprintf(fid, '%s', jsonencode(M, 'PrettyPrint', true)); fclose(fid);
disp(jsonencode(M, 'PrettyPrint', true));
fprintf('LISTO\n');

%% ---------------- funciones locales ----------------
function out = correr(mdl, ov)
global PRES_INIT
if ~isfield(ov, 'usar_degradacion_fan'), ov.usar_degradacion_fan = double(isfield(ov, 'fan_fac')); end
assignin('base', 'ovr', ov);
evalin('base', sprintf('run(''%s'');', strrep(PRES_INIT, '''', '''''')));
evalin('base', sprintf('usar_degradacion_fan = %d;', ov.usar_degradacion_fan));
tic; out = sim(mdl); fprintf('   sim %.1f s\n', toc);
end

function [resultados, tspan, d_real, th_ref_prof] = correr_comparativa()
simular_sistema;
end

function [t, x] = sig(ts)
[t, ia] = unique(ts.Time); x = squeeze(ts.Data); x = x(ia);
end

function m = metricas(t, th, ref, o)
e = abs(th - ref); fuera = find(e > 1);
m.ts_1deg = NaN; if ~isempty(fuera) && fuera(end) < numel(t), m.ts_1deg = t(fuera(end)+1); end
m.sobrepico_deg = max(0, max(th) - ref);
f2 = find(abs(th - ref) > 2); m.ts_2deg = NaN; if ~isempty(f2) && f2(end) < numel(t), m.ts_2deg = t(f2(end)+1); end
fin = t >= t(end) - 1; m.error_final_deg = abs(ref - mean(th(fin)));
i10 = find(th >= 0.1*ref, 1); i90 = find(th >= 0.9*ref, 1);
m.subida_s = NaN; if ~isempty(i10) && ~isempty(i90), m.subida_s = t(i90) - t(i10); end
if ~isempty(o)
    [~, p] = sig(o.pwm_out); [~, d] = sig(o.d_out);
    m.pwm_max = max(p); m.pwm_min = min(p); m.d_final_cm = d(end)*100;
    m.th_min = min(th); m.th_max = max(th);
end
end

function f = fig(sz)
f = figure('Color', 'w', 'Units', 'pixels', 'Position', [0 0 sz], 'Visible', 'off');
end

function estilo(ax, yl, tt)
set(ax, 'FontName', 'Helvetica', 'FontSize', 27, 'LineWidth', 1.5, 'Box', 'off', ...
    'XColor', [0.2 0.2 0.22], 'YColor', [0.2 0.2 0.22], 'TickDir', 'out', ...
    'XGrid', 'on', 'YGrid', 'on', 'GridAlpha', 0.12, 'GridColor', [0 0 0]);
if ~isempty(yl), ylabel(ax, yl, 'FontWeight', 'bold'); end
if ~isempty(tt), title(ax, tt, 'FontSize', 31, 'FontWeight', 'bold', 'Color', [0.11 0.11 0.12]); end
end

function colorejes(ax, cl, cr)
yyaxis(ax, 'left'); ax.YColor = cl; yyaxis(ax, 'right'); ax.YColor = cr;
end

function guardar(f, nombre)
exportgraphics(f, nombre, 'Resolution', 110, 'BackgroundColor', 'white'); close(f);
fprintf('   -> %s\n', nombre);
end
