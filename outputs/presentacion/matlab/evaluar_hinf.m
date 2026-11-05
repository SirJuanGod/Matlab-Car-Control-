%% evaluar_hinf.m — Sintesis H-infinito de sensibilidad mixta (S/KS) sin Robust Control Toolbox.
% Resuelve las dos ecuaciones de Riccati de Glover-Doyle (DGKF) con icare y busca gamma por
% biseccion. Evalua el diseno solo-ventilador (SISO) y el SIMO (ventilador + carro) en el
% punto nominal (35 deg, 10 cm), con el mismo escalon que la tabla del README (§4.3).

s = tf('s');
Gp   = 1.5/(s^2 + 5*s + 60.27);          % v_fan -> theta [rad/%]
Gd   = -8.44/(s^2 + 5*s + 60.27);        % d -> theta [rad/cm]
Fan  = 1/(0.15*s + 1);                   % u_PWM -> v_fan
Car  = 1/(0.8*s + 1);                    % d_ref -> d (carro con su PID)
r2d  = 180/pi;
G1 = ss(r2d*Gp*Fan);                     % u_PWM -> theta [deg]
G2 = ss([r2d*Gp*Fan, r2d*Gd*Car]);       % [u_PWM, d_ref] -> theta [deg]

% Pesos: W1 ~ integrador (error), W2 penaliza el esfuerzo de cada actuador
Ms = 1.6; wb = 4; ep = 1e-3;
W1 = ss((s/Ms + wb)/(s + wb*ep));
R = struct();
if ~exist('w2pwm','var'), w2pwm = 0.05; end
if ~exist('w2car','var'), w2car = 0.05; end
R.siso = disenar(G1, W1, w2pwm);
R.simo = disenar(G2, W1, diag([w2pwm w2car]));

% Evaluacion en lazo cerrado: escalon de 35 deg, nominal y con el carro a 14 cm sin aviso
t = (0:0.002:6)';
for nm = {'siso', 'simo'}
    k = R.(nm{1}); K = k.K; G = k.G;
    nu = size(G, 2);
    L  = G*K;                                            % lazo abierto (1x1)
    T  = feedback(L, 1); S = feedback(1, L);
    y  = step(35*T, t);
    m  = stepinfo(y, t, 35, 'SettlingTimeThreshold', 0.02, 'RiseTimeLimits', [0.1 0.9]);
    [~, Pm] = margin(L);
    U  = step(35*feedback(K, G), t);                     % esfuerzo r -> u
    % carro 4 cm mas lejos: el canal del ventilador pierde (10/14)^2
    Gl = G; Gl(:,1) = G(:,1)*(10/14)^2;
    yl = step(35*feedback(Gl*K, 1), t);
    R.(nm{1}).subida = m.RiseTime; R.(nm{1}).sobrepico = m.Overshoot; R.(nm{1}).ts2 = m.SettlingTime;
    R.(nm{1}).pm = Pm; R.(nm{1}).Ms = norm(S, inf); R.(nm{1}).orden = order(K);
    R.(nm{1}).ess = abs(35 - y(end)); R.(nm{1}).pwm_pico = max(abs(U(:,1)));
    R.(nm{1}).dc_lejos = yl(end); R.(nm{1}).estable_lejos = isstable(feedback(Gl*K, 1));
    if nu == 2, R.(nm{1}).dref_pico = max(abs(U(:,2))); end
    fprintf('\n[%s] gamma = %.3f | orden K = %d | subida %.2f s | sobrepico %.1f %% | ts(2%%) %.2f s | PM %.1f deg | Ms %.2f | e_ss %.3f deg | pico PWM %.0f %%\n', ...
        upper(nm{1}), k.gamma, order(K), m.RiseTime, m.Overshoot, m.SettlingTime, Pm, R.(nm{1}).Ms, R.(nm{1}).ess, R.(nm{1}).pwm_pico);
    fprintf('       carro a 14 cm: estable = %d, theta final = %.2f deg\n', R.(nm{1}).estable_lejos, yl(end));
    if nu == 2, fprintf('       pico d_ref = %.2f cm\n', R.(nm{1}).dref_pico); end
end
out = rmfield(R, {}); for nm = {'siso','simo'}, out.(nm{1}) = rmfield(R.(nm{1}), {'K','G'}); end
fid = fopen(fullfile(fileparts(mfilename('fullpath')), '..', 'img', 'hinf.json'), 'w');
fprintf(fid, '%s', jsonencode(out, 'PrettyPrint', true)); fclose(fid);

%% ---------------- funciones locales ----------------
function r = disenar(G, W1, W2)
% Planta aumentada P: w = referencia, z = [W1*e; W2*u], y = e = w - G*u
[Ag, Bg, Cg, ~] = ssdata(G); [Aw, Bw, Cw, Dw] = ssdata(W1);
ng = size(Ag,1); nw = size(Aw,1); nu = size(Bg,2);
A  = [Ag, zeros(ng,nw); -Bw*Cg, Aw];
B1 = [zeros(ng,1); Bw];
B2 = [Bg; zeros(nw,nu)];
C1 = [-Dw*Cg, Cw; zeros(nu, ng+nw)];
D12 = [zeros(1,nu); W2];
C2 = [-Cg, zeros(1,nw)];  D21 = 1;
lo = 1e-3; hi = 200; mejor = [];
for it = 1:60                                         % biseccion en gamma
    g = sqrt(lo*hi); c = central(A,B1,B2,C1,C2,D12,D21,g);
    if ~isempty(c), hi = g; mejor = g; else, lo = g; end
    if hi/lo < 1.002, break; end
end
g = 1.05*mejor;                                       % 5 % sobre el optimo: controlador bien condicionado
r.K = central(A,B1,B2,C1,C2,D12,D21,g); r.gamma = g; r.gamma_opt = mejor; r.G = G;
P = ss(A, [B1 B2], [C1; C2], [zeros(size(C1,1),1) D12; D21 zeros(1,nu)]);
Tzw = lft(P, r.K);
r.norma_Tzw = norm(Tzw, inf);
fprintf('  verificacion: ||Tzw||inf = %.3f < gamma = %.3f, estable = %d\n', r.norma_Tzw, g, isstable(Tzw));
end

function K = central(A,B1,B2,C1,C2,D12,D21,g)
% Controlador central H-inf (D11 = 0, terminos cruzados generales). Vacio si gamma no es factible.
K = [];
try
    R1 = D12'*D12; R2 = D21*D21';
    [X,~,~,ix] = icare(A, B2, C1'*C1, R1, C1'*D12, [], g^-2*(B1*B1'));
    [Y,~,~,iy] = icare(A', C2', B1*B1', R2, B1*D21', [], g^-2*(C1'*C1));
    if isempty(X) || isempty(Y) || ix.Report > 1 || iy.Report > 1, return; end
    if min(eig((X+X')/2)) < -1e-8 || min(eig((Y+Y')/2)) < -1e-8, return; end
    if max(abs(eig(X*Y))) >= g^2, return; end
    F = -R1\(B2'*X + D12'*C1);
    L = -(Y*C2' + B1*D21')/R2;
    Z = inv(eye(size(A)) - g^-2*Y*X);
    Ak = A + g^-2*(B1*B1')*X + B2*F + Z*L*(C2 + g^-2*D21*B1'*X);
    K = ss(Ak, -Z*L, F, 0);
    if ~isstable(lft(ss(A,[B1 B2],[C1;C2],[zeros(size(C1,1),1) D12; D21 zeros(1,size(B2,2))]), K)), K = []; end
catch
    K = [];
end
end
