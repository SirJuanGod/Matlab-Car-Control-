%% lqr_simo.m — LQR+I SIMO: [u_PWM, d_ref] controlando theta.
% NOTA: realimentacion de estado completo (estados ideales). Falta el
%       observador de omega, v_fan y d (IMU + Kalman) para el hardware real.

if ~exist('ovr_simo','var'), clear; end   % con ovr_simo (barridos) no se borra el workspace
clc; close all;
run('init_parametros.m');

%% --- 1. Parametros del diseno SIMO (ajustables) ---
tau_dist_mNm = 1.0;  % [mN*m] amplitud del pulso de torque externo (con 2 mN*m el SIMO empeora vs solo fan)
graficar  = true;    % false: solo imprime metricas (barridos)

% Sobrescritura opcional desde el workspace: ovr_simo = struct('r_d',1,'tau_car',0.5);
if exist('ovr_simo','var') && isstruct(ovr_simo)
    fn_o = fieldnames(ovr_simo);
    for i_o = 1:numel(fn_o), eval([fn_o{i_o} ' = ovr_simo.' fn_o{i_o} ';']); end
end
Q = diag([q_th, q_om, 0, q_d, q_xi]);   % [theta(rad), omega, v_fan(%), d(cm), x_i]
R = diag([r_pwm, r_d]);

sc.dt = 1e-3;  sc.T = 10;
sc.th0_deg = 0;  sc.th1_deg = 35;  sc.t_step = 0.5;  sc.tau_ref = tau_ref;
sc.t_dist = [6 7];  sc.tau_dist = tau_dist_mNm*1e-3;     % pulso de torque externo [N*m]

p = struct('g',g,'m_g',m_g,'l_c',l_c,'J_g',J_g,'b_c',b_c,'tau_f',tau_f, ...
           'k_m',k_m,'fan_tau',fan_tau,'tau_car',tau_car,'d0',d0, ...
           'd_min',d_min,'d_max',d_max,'v_car_max',v_car_max,'pwm_min',pwm_min,'pwm_max',pwm_max,'xi_lim',xi_lim);

%% --- 2. Verificacion del modelo linealizado en theta = 35 deg ---
[A35, B35] = linealizar(deg2rad(35), p);
fprintf('\n=================================================================\n');
fprintf('  LQR+I SIMO  (u_PWM, d_ref)  -  planta MISO [u_PWM, d] -> theta\n');
fprintf('=================================================================\n');
fprintf('  rank(ctrb(A,B)) = %d de %d estados\n', rank(ctrb(A35,B35)), size(A35,1));
fprintf('  Polos de lazo abierto:'); fprintf(' %+.2f', sort(real(eig(A35)))'); fprintf('\n');
K35 = lqr(A35, B35, Q, R);
fprintf('  Polos de lazo cerrado (35 deg):'); fprintf(' %+.2f%+.2fj', [real(eig(A35-B35*K35)) imag(eig(A35-B35*K35))].'); fprintf('\n');
fprintf('  K (35 deg), fila PWM: '); fprintf('%8.2f', K35(1,:)); fprintf('\n');
fprintf('  K (35 deg), fila d  : '); fprintf('%8.2f', K35(2,:)); fprintf('\n');

%% --- 3. Tabla de ganancias programadas ---
thg = deg2rad(10:5:80);  nG = numel(thg);
Kg  = zeros(nG,10);  Kg4 = zeros(nG,4);
idx = [1 2 3 5];                               % subsistema solo ventilador
for i = 1:nG
    [A,B] = linealizar(thg(i), p);
    Kg(i,:)  = reshape(lqr(A, B, Q, R), 1, []);
    Kg4(i,:) = lqr(A(idx,idx), B(idx,1), Q(idx,idx), r_pwm);
end

%% --- 4. Simulacion no lineal: SIMO vs solo ventilador ---
o_simo = simular('simo', p, Kg, Kg4, thg, sc);
o_fan  = simular('fan',  p, Kg, Kg4, thg, sc);
m_simo = metricas(o_simo, sc, p);
m_fan  = metricas(o_fan,  sc, p);

fprintf('\n  Escenario: theta %g -> %g deg en t=%g s; pulso de %.1f mN*m entre t=%g y %g s\n', ...
        sc.th0_deg, sc.th1_deg, sc.t_step, sc.tau_dist*1e3, sc.t_dist(1), sc.t_dist(2));
fprintf('  %-22s %10s %10s\n', 'Metrica', 'SIMO', 'Solo fan');
fprintf('  %-22s %10.2f %10.2f\n', 'Sobrepico [deg]',        m_simo.sobrepico, m_fan.sobrepico);
fprintf('  %-22s %10.2f %10.2f\n', 'Asentamiento 1deg [s]',  m_simo.ts,        m_fan.ts);
fprintf('  %-22s %10.3f %10.3f\n', 'Error final [deg]',      m_simo.err_ss,    m_fan.err_ss);
fprintf('  %-22s %10.2f %10.2f\n', 'Desv. max. pert. [deg]', m_simo.desv_pert, m_fan.desv_pert);
fprintf('  %-22s %10.2f %10.2f\n', 'Recuperacion pert. [s]', m_simo.t_rec,     m_fan.t_rec);
fprintf('  %-22s %10.1f %10.1f\n', 'PWM maximo [%]',         m_simo.pwm_max,   m_fan.pwm_max);
fprintf('  %-22s %10.1f %10.1f\n', 'PWM minimo [%]',         m_simo.pwm_min,   m_fan.pwm_min);
fprintf('  %-22s %10.2f %10.2f\n', 'Excursion carro [cm]',   m_simo.d_dev,     m_fan.d_dev);
fprintf('=================================================================\n');

%% --- 5. Graficas ---
if ~graficar, return; end
t = o_simo.t;  cS = '#0072BD';  cF = '#D9531F';
fig = figure('Name','LQR+I SIMO vs solo ventilador','Position',[50 50 1240 760],'Color','w');

subplot(2,3,1); hold on; grid on; box on;
plot(t, o_simo.H(:,2), '--k', 'LineWidth', 1.3, 'DisplayName', 'Referencia filtrada');
plot(t, o_fan.H(:,1),  'Color', cF, 'LineWidth', 1.6, 'DisplayName', 'Solo ventilador');
plot(t, o_simo.H(:,1), 'Color', cS, 'LineWidth', 1.6, 'DisplayName', 'SIMO (PWM + carro)');
xregion(sc.t_dist(1), sc.t_dist(2), 'FaceColor', [0.9 0.9 0.9], 'HandleVisibility', 'off');
ylabel('\theta [deg]'); xlabel('t [s]'); title('Angulo de la compuerta'); legend('Location','southeast');

subplot(2,3,2); hold on; grid on; box on;
plot(t, o_fan.H(:,3),  'Color', cF, 'LineWidth', 1.4, 'DisplayName', 'Solo ventilador');
plot(t, o_simo.H(:,3), 'Color', cS, 'LineWidth', 1.4, 'DisplayName', 'SIMO');
xregion(sc.t_dist(1), sc.t_dist(2), 'FaceColor', [0.9 0.9 0.9], 'HandleVisibility', 'off');
ylabel('u_{PWM} [%]'); xlabel('t [s]'); title('Salida 1: PWM del ventilador'); legend('Location','best'); ylim([0 105]);

subplot(2,3,3); hold on; grid on; box on;
plot(t, o_simo.H(:,5), ':', 'Color', cS, 'LineWidth', 1.2, 'DisplayName', 'd_{ref} (salida 2)');
plot(t, o_simo.H(:,4), 'Color', cS, 'LineWidth', 1.6, 'DisplayName', 'd real del carro');
yline(p.d0*100, ':k', 'd_0', 'HandleVisibility', 'off');
xregion(sc.t_dist(1), sc.t_dist(2), 'FaceColor', [0.9 0.9 0.9], 'HandleVisibility', 'off');
ylabel('d [cm]'); xlabel('t [s]'); title('Salida 2: distancia del carro'); legend('Location','best');

subplot(2,3,4); hold on; grid on; box on;
plot(t, o_fan.H(:,6),  'Color', cF, 'LineWidth', 1.4, 'DisplayName', 'Solo ventilador');
plot(t, o_simo.H(:,6), 'Color', cS, 'LineWidth', 1.4, 'DisplayName', 'SIMO');
xregion(sc.t_dist(1), sc.t_dist(2), 'FaceColor', [0.9 0.9 0.9], 'HandleVisibility', 'off');
ylabel('\tau_{aero} [mN m]'); xlabel('t [s]'); title('Torque aerodinamico'); legend('Location','best');

subplot(2,3,5); hold on; grid on; box on;
plot(t, o_fan.H(:,1)  - o_fan.H(:,2),  'Color', cF, 'LineWidth', 1.4, 'DisplayName', 'Solo ventilador');
plot(t, o_simo.H(:,1) - o_simo.H(:,2), 'Color', cS, 'LineWidth', 1.4, 'DisplayName', 'SIMO');
yline(0, 'k', 'HandleVisibility', 'off');
xregion(sc.t_dist(1), sc.t_dist(2), 'FaceColor', [0.9 0.9 0.9], 'HandleVisibility', 'off');
ylabel('error [deg]'); xlabel('t [s]'); title('Error \theta - \theta_{ref}'); legend('Location','best');

subplot(2,3,6); axis off;
text(0.00, 0.95, 'Metricas', 'FontWeight', 'bold', 'FontSize', 11);
txt = sprintf(['%-22s %8s %9s\n' ...
               '%-22s %8.2f %9.2f\n%-22s %8.2f %9.2f\n%-22s %8.2f %9.2f\n' ...
               '%-22s %8.1f %9.1f\n%-22s %8.2f %9.2f'], ...
    '', 'SIMO', 'Solo fan', ...
    'Sobrepico [deg]', m_simo.sobrepico, m_fan.sobrepico, ...
    'Asent. 1deg [s]', m_simo.ts, m_fan.ts, ...
    'Desv. pert. [deg]', m_simo.desv_pert, m_fan.desv_pert, ...
    'PWM max [%]', m_simo.pwm_max, m_fan.pwm_max, ...
    'Excursion carro [cm]', m_simo.d_dev, m_fan.d_dev);
text(0.00, 0.80, txt, 'FontName', 'Courier', 'FontSize', 9, 'VerticalAlignment', 'top');
text(0.00, 0.25, sprintf('tau_{car}=%.2f s | q_d=%.2f | r_{PWM}=%.2f | r_d=%.2f', tau_car, q_d, r_pwm, r_d), 'FontSize', 8.5);

out_dir = fullfile(fileparts(mfilename('fullpath')), 'outputs');
if ~exist(out_dir, 'dir'), mkdir(out_dir); end
saveas(fig, fullfile(out_dir, 'simulacion_lqr_simo.png'));
fprintf('Grafica guardada en: outputs/simulacion_lqr_simo.png\n');

%% =========================== Funciones locales ===========================
function [A,B,veq] = linealizar(th, p)
    veq = p.m_g*p.g*p.l_c*sin(th)/p.k_m;                 % [%] PWM de equilibrio
    a21 = -p.m_g*p.g*p.l_c*cos(th)/p.J_g;                % [1/s^2] rigidez gravitacional
    kv  = p.k_m/p.J_g;                                    % [rad/s^2 por %]
    gd  = -2*p.k_m*veq/(p.J_g*p.d0)/100;                  % [rad/s^2 por cm] (<0: alejar reduce torque)
    A = [0, 1, 0, 0, 0;
         a21, -p.b_c/p.J_g, kv, gd, 0;
         0, 0, -1/p.fan_tau, 0, 0;
         0, 0, 0, -1/p.tau_car, 0;
         1, 0, 0, 0, 0];
    B = [0, 0; 0, 0; 1/p.fan_tau, 0; 0, 1/p.tau_car; 0, 0];
end

function out = simular(modo, p, Kg, Kg4, thg, sc)
    dt = sc.dt;  N = round(sc.T/dt) + 1;  t = (0:N-1)'*dt;
    th = deg2rad(sc.th0_deg);  om = 0;
    vf = min(max(p.m_g*p.g*p.l_c*sin(th)/p.k_m, 0), p.pwm_max);  d = p.d0;  xi = 0;  thr = th;
    H = zeros(N,6);
    for k = 1:N
        tk = t(k);
        thr_cmd = deg2rad(sc.th0_deg);
        if tk >= sc.t_step, thr_cmd = deg2rad(sc.th1_deg); end
        thr   = thr + dt*(thr_cmd - thr)/sc.tau_ref;      % referencia filtrada
        thr_s = min(max(thr, thg(1)), thg(end));
        veq   = p.m_g*p.g*p.l_c*sin(thr)/p.k_m;
        if strcmp(modo, 'simo')
            K = reshape(interp1(thg, Kg, thr_s), 2, 5);
            x = [th-thr; om; vf-veq; (d-p.d0)*100; xi];
            u = -K*x;
            u_pwm = veq + u(1);
            d_ref = p.d0 + u(2)/100;
        else
            K4 = interp1(thg, Kg4, thr_s);
            x  = [th-thr; om; vf-veq; xi];
            u_pwm = veq - K4*x;
            d_ref = p.d0;
        end
        u_cmd = min(max(u_pwm, p.pwm_min), p.pwm_max);
        d_ref = min(max(d_ref, p.d_min), p.d_max);

        tau_aero = p.k_m*vf*(p.d0/d)^2;
        tau_d = 0;
        if tk >= sc.t_dist(1) && tk < sc.t_dist(2), tau_d = sc.tau_dist; end
        alpha = (tau_aero - p.m_g*p.g*p.l_c*sin(th) - p.b_c*om - p.tau_f*tanh(50*om) + tau_d)/p.J_g;
        H(k,:) = [rad2deg(th), rad2deg(thr), u_cmd, d*100, d_ref*100, tau_aero*1000];

        om = om + dt*alpha;  th = th + dt*om;
        if th <= 0, th = 0; om = max(om, 0); elseif th >= pi, th = pi; om = min(om, 0); end
        vf = min(max(vf + dt*(u_cmd - vf)/p.fan_tau, 0), 100);
        dd = min(max((d_ref - d)/p.tau_car, -p.v_car_max), p.v_car_max);
        d  = min(max(d + dt*dd, p.d_min), p.d_max);
        e_i = th - thr;
        if (u_pwm >= p.pwm_max && e_i < 0) || (u_pwm <= p.pwm_min && e_i > 0)
            e_i = 0;
        end
        xi = min(max(xi + dt*e_i, -p.xi_lim), p.xi_lim);
    end
    out.t = t;  out.H = H;
end

function m = metricas(out, sc, p)
    t = out.t;  th = out.H(:,1);  pwm = out.H(:,3);  dd = out.H(:,4);
    r1 = sc.th1_deg;
    i1 = find(t >= sc.t_step & t < sc.t_dist(1));
    m.sobrepico = max(0, max(th(i1)) - r1);
    bad = find(abs(th(i1) - r1) > 1, 1, 'last');
    if isempty(bad), m.ts = 0; elseif bad == numel(i1), m.ts = NaN; else, m.ts = t(i1(bad+1)) - sc.t_step; end
    m.err_ss = abs(mean(th(i1(end-99:end))) - r1);
    i2 = find(t >= sc.t_dist(1));
    m.desv_pert = max(abs(th(i2) - r1));
    i3 = find(t >= sc.t_dist(2));
    bad3 = find(abs(th(i3) - r1) > 1, 1, 'last');
    if isempty(bad3), m.t_rec = 0; elseif bad3 == numel(i3), m.t_rec = NaN; else, m.t_rec = t(i3(bad3+1)) - sc.t_dist(2); end
    m.pwm_max = max(pwm);  m.pwm_min = min(pwm);
    m.d_dev = max(abs(dd - p.d0*100));
end
