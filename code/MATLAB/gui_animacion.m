function gui_animacion()
% =========================================================================
% GUI_ANIMACION.M - Interfaz grafica para la animacion cinematica del sistema
% =========================================================================
% Carga los valores por defecto directamente de init_parametros.m y
% permite modificarlos antes de lanzar la animacion o simulacion.
%
% Corrige:
%   - BUG 5: 'ovr' se preserva antes del clear de init_parametros.m
%   - Geometria leida de init_parametros.m (no hardcoded)
%   - Parametros faltantes: k_m, pwm_min, pwm_max, gov_deg, tau_ref,
%     usar_imu, fan_t0, fan_dur, ref_rate_deg
%   - onLanzar aplica override incluso en escenarios predefinidos
%
% Uso:
%   >> gui_animacion
% =========================================================================

folder = fileparts(mfilename('fullpath'));
addpath(folder);

% ---- Cargar valores por defecto desde init_parametros.m ----
def = cargarDefaults();

%% ---- Figura principal ----
fig = uifigure( ...
    'Name',     'Animacion del Sistema - Panel de Control', ...
    'Position', [60 80 1420 860], ...
    'Color',    [0.96 0.96 0.98]);

gl = uigridlayout(fig, [1 2]);
gl.ColumnWidth   = {390, '1x'};
gl.Padding       = [8 8 8 8];
gl.ColumnSpacing = 10;

%% ======================================================================
%  PANEL IZQUIERDO: controles
%% ======================================================================
pCtrl = uipanel(gl, ...
    'Title',           'Parametros de Animacion', ...
    'FontWeight',      'bold', ...
    'FontSize',        12, ...
    'BackgroundColor', [0.97 0.97 1.0]);

pgC = uigridlayout(pCtrl, [52 2]);
pgC.RowHeight    = repmat({24}, 1, 52);
pgC.ColumnWidth  = {'1.5x', '1x'};
pgC.Padding      = [8 6 8 6];
pgC.RowSpacing   = 2;
pgC.Scrollable   = 'on';

% ===== Seccion: Escenario =====
makeSectionLabel(pgC, '--- ESCENARIO BASE ---', [0.2 0.35 0.7]);
uilabel(pgC, 'Text', 'Escenario');
ddEsc = uidropdown(pgC, ...
    'Items',    {'carro_lejos', 'nominal', 'angulo_bajo', 'perturbacion', 'personalizado'}, ...
    'Value',    'carro_lejos', ...
    'ValueChangedFcn', @onEscenarioChange);

% ===== Seccion: Angulo =====
makeSectionLabel(pgC, '--- ANGULO DE LA COMPUERTA ---', [0.1 0.55 0.25]);

uilabel(pgC, 'Text', 'Angulo referencia [deg]');
sldThRef = uislider(pgC, 'Limits', [1 90], 'Value', 35, ...
    'ValueChangedFcn', @(s,~) syncSlider(s, efThRef));

uilabel(pgC, 'Text', '');
efThRef = makeEdit(pgC, 35, [1 90], '%.1f', @(~,~) syncEdit(efThRef, sldThRef));

uilabel(pgC, 'Text', 'Angulo inicial [deg]');
sldTh0 = uislider(pgC, 'Limits', [0 89], 'Value', def.theta0_deg, ...
    'ValueChangedFcn', @(s,~) syncSlider(s, efTh0));

uilabel(pgC, 'Text', '');
efTh0 = makeEdit(pgC, def.theta0_deg, [0 89], '%.1f', @(~,~) syncEdit(efTh0, sldTh0));

uilabel(pgC, 'Text', 'Gobernador ref. [deg]');
efGov = makeEdit(pgC, def.gov_deg, [5 180], '%.1f', []);

uilabel(pgC, 'Text', 'Filtro ref. tau [s]');
efTauRef = makeEdit(pgC, def.tau_ref, [0.01 5], '%.3f', []);

uilabel(pgC, 'Text', 'Rampa ref. max. [deg/s]');
efRefRate = makeEdit(pgC, def.ref_rate_deg, [1 1000], '%.1f', []);

% ===== Seccion: Carro =====
makeSectionLabel(pgC, '--- VARIACIONES DEL CARRO ---', [0.75 0.2 0.1]);

uilabel(pgC, 'Text', 'Dist. inicial carro [cm]');
sldDini = uislider(pgC, 'Limits', [5 25], 'Value', 18, ...
    'ValueChangedFcn', @(s,~) syncSlider(s, efDini));

uilabel(pgC, 'Text', '');
efDini = makeEdit(pgC, 18, [5 25], '%.1f', @(~,~) syncEdit(efDini, sldDini));

uilabel(pgC, 'Text', 'Supervisor: acercar si PWM > [%]');
efSupVhi = makeEdit(pgC, def.sup_v_hi, [0 100], '%.0f', []);

uilabel(pgC, 'Text', 'Veloc. acercamiento max. [cm/s]');
efSupAdv = makeEdit(pgC, def.sup_adv, [0.1 30], '%.1f', []);

uilabel(pgC, 'Text', 'Veloc. retorno a d_ini [cm/s]');
efSupRet = makeEdit(pgC, def.sup_ret, [0.1 20], '%.1f', []);

% ===== Seccion: Ventilador / Actuador =====
makeSectionLabel(pgC, '--- VENTILADOR / ACTUADOR ---', [0.15 0.45 0.65]);

uilabel(pgC, 'Text', 'k_m [N m / %]  (ganancia aero.)');
efKm = makeEdit(pgC, def.k_m, [1e-5 0.01], '%.6f', []);

uilabel(pgC, 'Text', 'PWM minimo [%]');
efPwmMin = makeEdit(pgC, def.pwm_min, [0 50], '%.1f', []);

uilabel(pgC, 'Text', 'PWM maximo [%]');
efPwmMax = makeEdit(pgC, def.pwm_max, [50 100], '%.1f', []);

uilabel(pgC, 'Text', 'Usar IMU + observador (1/0)');
efUsarImu = makeEdit(pgC, def.usar_imu, [0 1], '%.0f', []);

uilabel(pgC, 'Text', 'Factor fuerza ventilador final');
efFanFac = makeEdit(pgC, def.fan_fac, [0.1 2.0], '%.3f', []);

uilabel(pgC, 'Text', 'Inicio degradacion [s]');
efFanT0 = makeEdit(pgC, def.fan_t0, [0 60], '%.1f', []);

uilabel(pgC, 'Text', 'Duracion degradacion [s]');
efFanDur = makeEdit(pgC, def.fan_dur, [0.5 60], '%.1f', []);

% ===== Seccion: Compuerta fisica =====
makeSectionLabel(pgC, '--- COMPUERTA FISICA ---', [0.3 0.3 0.3]);

uilabel(pgC, 'Text', 'Amort. pivote b_c [N m s/rad]');
efBc = makeEdit(pgC, def.b_c, [1e-5 0.1], '%.6f', []);

uilabel(pgC, 'Text', 'Friccion Coulomb tau_f [N m]');
efTauF = makeEdit(pgC, def.tau_f, [0 0.01], '%.6f', []);

% ===== Seccion: Perturbaciones =====
makeSectionLabel(pgC, '--- PERTURBACIONES ---', [0.6 0.1 0.6]);

uilabel(pgC, 'Text', 'Pulso de torque ext. [mN m]');
efPert = makeEdit(pgC, 0, [-50 50], '%.1f', []);

uilabel(pgC, 'Text', 'Inicio perturbacion [s]');
efPertT = makeEdit(pgC, def.pert_t_on, [0 60], '%.1f', []);

uilabel(pgC, 'Text', 'Duracion perturbacion [s]');
efPertDur = makeEdit(pgC, def.pert_dur, [0.1 30], '%.1f', []);

% ===== Seccion: Tiempo y animacion =====
makeSectionLabel(pgC, '--- SIMULACION Y ANIMACION ---', [0.1 0.1 0.1]);

uilabel(pgC, 'Text', 'Tiempo de simulacion [s]');
efTsim = makeEdit(pgC, def.t_sim, [2 120], '%.0f', []);

uilabel(pgC, 'Text', 'FPS de animacion');
efFps = makeEdit(pgC, 24, [5 60], '%.0f', []);

uilabel(pgC, 'Text', 'Velocidad (1 = tiempo real)');
efVel = makeEdit(pgC, 1, [0.1 5], '%.2f', []);

uilabel(pgC, 'Text', 'Guardar resultado');
ddSal = uidropdown(pgC, ...
    'Items', {'ninguna', 'mp4', 'gif', 'ambos'}, ...
    'Value', 'ninguna');

% ===== Botones =====
makeSectionLabel(pgC, '--- ACCIONES ---', [0.1 0.1 0.1]);

btnAnimar = uibutton(pgC, 'Text', '▶  LANZAR ANIMACION', ...
    'BackgroundColor', [0.15 0.55 0.25], ...
    'FontColor',       [1 1 1], ...
    'FontWeight',      'bold', ...
    'FontSize',        13, ...
    'ButtonPushedFcn', @onLanzar);
btnAnimar.Layout.Column = [1 2];

btnSimOnly = uibutton(pgC, 'Text', '📊  Solo Simular (ver graficas aqui)', ...
    'BackgroundColor', [0.15 0.40 0.75], ...
    'FontColor',       [1 1 1], ...
    'FontWeight',      'bold', ...
    'ButtonPushedFcn', @onSoloSimular);
btnSimOnly.Layout.Column = [1 2];

btnReset = uibutton(pgC, 'Text', '↺  Restablecer desde init_parametros', ...
    'BackgroundColor', [0.50 0.50 0.50], ...
    'FontColor',       [1 1 1], ...
    'ButtonPushedFcn', @onReset);
btnReset.Layout.Column = [1 2];

lblStatus = uilabel(pgC, ...
    'Text',      'Listo — valores cargados de init_parametros.m', ...
    'WordWrap',  'on', ...
    'FontWeight','bold', ...
    'FontColor', [0.2 0.55 0.25]);
lblStatus.Layout.Column = [1 2];

%% ======================================================================
%  PANEL DERECHO
%% ======================================================================
pRight = uipanel(gl, ...
    'Title',           'Vista previa y resultados', ...
    'FontWeight',      'bold', ...
    'FontSize',        12, ...
    'BackgroundColor', [0.97 0.97 0.97]);

pgR = uigridlayout(pRight, [5 1]);
pgR.RowHeight = {170, 34, '1x', '1x', '1x'};
pgR.Padding   = [10 10 10 10];

txaPrev = uitextarea(pgR, ...
    'Editable', 'off', ...
    'FontName', 'Consolas', ...
    'FontSize', 10, ...
    'Value',    {'Ajuste los parametros y pulse "LANZAR ANIMACION".'});

lblEsc = uilabel(pgR, ...
    'Text',       '', ...
    'FontSize',   11, ...
    'FontWeight', 'bold', ...
    'FontColor',  [0.15 0.35 0.65], ...
    'WordWrap',   'on');

agPrev = uigridlayout(pgR, [1 3]);
axPrev = gobjects(1, 3);
titP   = {'\theta (deg)', 'PWM (%)', 'd (cm)'};
ylP    = {'\theta [deg]', 'PWM [%]', 'd [cm]'};
for kk = 1:3
    axPrev(kk) = uiaxes(agPrev);
    title(axPrev(kk), titP{kk});
    ylabel(axPrev(kk), ylP{kk});
    xlabel(axPrev(kk), 't [s]');
    grid(axPrev(kk), 'on');
    hold(axPrev(kk), 'on');
end

agBar = uigridlayout(pgR, [1 2]);
axBar = gobjects(1, 2);
axBar(1) = uiaxes(agBar);
title(axBar(1), 'Barrido: \theta_{ref} vs \theta_{final}');
ylabel(axBar(1), '\theta final [deg]'); xlabel(axBar(1), '\theta_{ref} [deg]');
grid(axBar(1), 'on'); hold(axBar(1), 'on');

axBar(2) = uiaxes(agBar);
title(axBar(2), 'Barrido: d_{ini} vs \theta_{final}');
ylabel(axBar(2), '\theta final [deg]'); xlabel(axBar(2), 'd_{ini} [cm]');
grid(axBar(2), 'on'); hold(axBar(2), 'on');

agBtnBar = uigridlayout(pgR, [1 3]);
agBtnBar.ColumnWidth = {'1x','1x','1x'};
uibutton(agBtnBar, 'Text', '📈 Barrer angulos (10-80 deg)', ...
    'BackgroundColor', [0.2 0.55 0.75], 'FontColor', [1 1 1], ...
    'ButtonPushedFcn', @onBarrerAngulos);
uibutton(agBtnBar, 'Text', '📈 Barrer distancias (5-25 cm)', ...
    'BackgroundColor', [0.7 0.35 0.1], 'FontColor', [1 1 1], ...
    'ButtonPushedFcn', @onBarrerDistancias);
uibutton(agBtnBar, 'Text', '🗑 Limpiar graficas', ...
    'BackgroundColor', [0.55 0.55 0.55], 'FontColor', [1 1 1], ...
    'ButtonPushedFcn', @onLimpiarPrev);

% Vista previa inicial
actualizarPrev();

%% ======================================================================
%  HELPERS DE CONSTRUCCION DE UI
%% ======================================================================
    function lbl = makeSectionLabel(parent, txt, col)
        lbl = uilabel(parent, 'Text', txt, 'FontWeight', 'bold', 'FontColor', col);
        lbl.Layout.Column = [1 2];
    end

    function ef = makeEdit(parent, val, lims, fmt, cb)
        args = {'numeric', 'Value', val, 'Limits', lims, ...
                'ValueDisplayFormat', fmt};
        if ~isempty(cb)
            args = [args, {'ValueChangedFcn', cb}];
        end
        ef = uieditfield(parent, args{:});
    end

    function syncSlider(slider, editfield)
        editfield.Value = round(slider.Value, 2);
        ddEsc.Value = 'personalizado';
        actualizarPrev();
    end

    function syncEdit(editfield, slider)
        slider.Value = editfield.Value;
        ddEsc.Value = 'personalizado';
        actualizarPrev();
    end

%% ======================================================================
%  CALLBACKS
%% ======================================================================

    function onEscenarioChange(~, ~)
        switch ddEsc.Value
            case 'carro_lejos'
                setVal(efThRef, sldThRef, 35);
                setVal(efTh0,   sldTh0,   0);
                setVal(efDini,  sldDini,  18);
                efPert.Value  = 0;
                efTsim.Value  = 10;
            case 'nominal'
                setVal(efThRef, sldThRef, 35);
                setVal(efTh0,   sldTh0,   0);
                setVal(efDini,  sldDini,  10);
                efPert.Value  = 0;
                efTsim.Value  = 8;
            case 'angulo_bajo'
                setVal(efThRef, sldThRef, 8);
                setVal(efTh0,   sldTh0,  0);
                setVal(efDini,  sldDini, 10);
                efPert.Value  = 0;
                efTsim.Value  = 12;
            case 'perturbacion'
                setVal(efThRef, sldThRef, 35);
                setVal(efTh0,   sldTh0,   0);
                setVal(efDini,  sldDini,  10);
                efPert.Value    = -5;
                efPertT.Value   = 5;
                efPertDur.Value = 3;
                efTsim.Value    = 12;
            case 'personalizado'
                % mantener valores actuales
        end
        actualizarPrev();
    end

    function setVal(ef, slider, v)
        ef.Value     = v;
        slider.Value = v;
    end

    function actualizarPrev()
        esc = ddEsc.Value;
        lines_prev = {
            sprintf('Escenario      : %s',  esc), ...
            sprintf('theta_ref      : %.1f deg',  efThRef.Value), ...
            sprintf('theta0         : %.1f deg',  efTh0.Value), ...
            sprintf('d_ini_cm       : %.1f cm',   efDini.Value), ...
            sprintf('Perturbacion   : %.1f mN m  (t0=%.1f s, dur=%.1f s)', efPert.Value, efPertT.Value, efPertDur.Value), ...
            sprintf('k_m            : %.6f N m/%%', efKm.Value), ...
            sprintf('b_c / tau_f    : %.5f / %.5f', efBc.Value, efTauF.Value), ...
            sprintf('PWM min/max    : %.0f / %.0f %%', efPwmMin.Value, efPwmMax.Value), ...
            sprintf('fan_fac        : %.2f  (t0=%.1f s, dur=%.1f s)', efFanFac.Value, efFanT0.Value, efFanDur.Value), ...
            sprintf('usar_imu       : %.0f  |  gov_deg=%.1f  |  tau_ref=%.3f', efUsarImu.Value, efGov.Value, efTauRef.Value), ...
            sprintf('t_sim          : %.0f s  |  FPS=%.0f  |  vel=%.2f x', efTsim.Value, efFps.Value, efVel.Value), ...
            sprintf('Guardar como   : %s', ddSal.Value), ...
            '', ...
            'Pulsa LANZAR ANIMACION para iniciar.'
        };
        txaPrev.Value = lines_prev;
        lblEsc.Text = sprintf('Escenario: "%s"  |  theta_ref=%.1f deg  |  d_ini=%.1f cm  |  k_m=%.6f', ...
            esc, efThRef.Value, efDini.Value, efKm.Value);
    end

    function ov = buildOvr()
        ov = struct();
        % -- Angulo --
        ov.theta_ref_deg = efThRef.Value;
        ov.theta0_deg    = efTh0.Value;
        ov.gov_deg       = efGov.Value;
        ov.tau_ref       = efTauRef.Value;
        ov.ref_rate_deg  = efRefRate.Value;
        % -- Carro --
        ov.d_ini_cm      = efDini.Value;
        ov.sup_v_hi      = efSupVhi.Value;
        ov.sup_adv       = efSupAdv.Value;
        ov.sup_ret       = efSupRet.Value;
        % -- Ventilador --
        ov.k_m           = efKm.Value;
        ov.pwm_min       = efPwmMin.Value;
        ov.pwm_max       = efPwmMax.Value;
        ov.usar_imu      = efUsarImu.Value;
        ov.fan_fac       = efFanFac.Value;
        ov.fan_t0        = efFanT0.Value;
        ov.fan_dur       = efFanDur.Value;
        % -- Compuerta --
        ov.b_c           = efBc.Value;
        ov.tau_f         = efTauF.Value;
        % -- Perturbacion --
        ov.pert_tau_mNm  = efPert.Value;
        ov.pert_t_on     = efPertT.Value;
        ov.pert_dur      = efPertDur.Value;
        % -- Tiempo --
        ov.t_sim         = efTsim.Value;
    end

    % ---- LANZAR ANIMACION ----
    function onLanzar(~, ~)
        lblStatus.Text      = 'Preparando simulacion...';
        lblStatus.FontColor = [0.7 0.5 0.0];
        drawnow;
        try
            ov  = buildOvr();
            esc = ddEsc.Value;

            if strcmp(esc, 'personalizado')
                correrAnimacionPersonalizada(ov, ddSal.Value, efFps.Value, efVel.Value);
            else
                % Escenarios predefinidos: dejar que animar_sistema use
                % su ovr base del escenario, pero sobreescribir con los
                % valores que el usuario haya ajustado en la GUI.
                assignin('base', 'ovr', ov);
                assignin('base', 'ani', struct( ...
                    'escenario',    esc, ...
                    'salida',       ddSal.Value, ...
                    'fps',          efFps.Value, ...
                    'velocidad',    efVel.Value, ...
                    'instantaneas', []));
                evalin('base', sprintf('cd(''%s''); run(''animar_sistema.m'');', ...
                    strrep(folder, '''', '''''')));
            end
            lblStatus.Text      = 'Animacion completada.';
            lblStatus.FontColor = [0.15 0.55 0.25];
        catch ME
            lblStatus.Text      = ['Error: ' ME.message];
            lblStatus.FontColor = [0.8 0.1 0.1];
            uialert(fig, ME.message, 'Error al animar');
        end
    end

    % ---- SOLO SIMULAR ----
    function onSoloSimular(~, ~)
        lblStatus.Text      = 'Simulando...';
        lblStatus.FontColor = [0.7 0.5 0.0];
        drawnow;
        try
            ov  = buildOvr();
            out = correrSim(ov);

            cla(axPrev(1)); cla(axPrev(2)); cla(axPrev(3));
            plot(axPrev(1), out.theta_out.Time,  rad2deg(out.theta_out.Data), '-', 'Color', [0.1 0.55 0.2], 'LineWidth', 1.6);
            yline(axPrev(1), ov.theta_ref_deg, '--k');
            plot(axPrev(2), out.pwm_out.Time,    out.pwm_out.Data,          '-', 'Color', [0.15 0.4 0.8],  'LineWidth', 1.6);
            yline(axPrev(2), ov.sup_v_hi, '--r');
            plot(axPrev(3), out.d_out.Time,      out.d_out.Data*100,        '-', 'Color', [0.1 0.1 0.1],   'LineWidth', 1.6);
            plot(axPrev(3), out.d_est_out.Time,  out.d_est_out.Data*100,    '--','Color', [0.9 0.5 0.1],   'LineWidth', 1.0);

            th_fin  = rad2deg(out.theta_out.Data(end));
            d_fin   = out.d_out.Data(end) * 100;
            pwm_mx  = max(out.pwm_out.Data);
            lblStatus.Text      = sprintf('theta final=%.1f deg | d final=%.1f cm | PWM max=%.1f%%', th_fin, d_fin, pwm_mx);
            lblStatus.FontColor = [0.15 0.55 0.25];
        catch ME
            lblStatus.Text      = ['Error: ' ME.message];
            lblStatus.FontColor = [0.8 0.1 0.1];
            uialert(fig, ME.message, 'Error en simulacion');
        end
    end

    % ---- Barrido de angulos ----
    function onBarrerAngulos(~, ~)
        lblStatus.Text      = 'Barriendo angulos...';
        lblStatus.FontColor = [0.7 0.5 0.0]; drawnow;
        try
            ov_base  = buildOvr();
            angulos  = 10:10:80;
            th_fin   = zeros(size(angulos));
            cla(axBar(1));
            for ii = 1:numel(angulos)
                ov_b = ov_base;
                ov_b.theta_ref_deg = angulos(ii);
                out = correrSim(ov_b);
                th_fin(ii) = rad2deg(out.theta_out.Data(end));
                lblStatus.Text = sprintf('Barrido angulos: %d/%d (%d deg)...', ii, numel(angulos), angulos(ii));
                drawnow;
            end
            plot(axBar(1), angulos, th_fin,   'o-', 'Color', [0.1 0.45 0.8], 'LineWidth', 1.8, 'MarkerFaceColor', [0.1 0.45 0.8]);
            plot(axBar(1), angulos, angulos, '--k', 'LineWidth', 1);
            legend(axBar(1), {'\theta final', 'ideal'}, 'Location', 'northwest');
            lblStatus.Text      = sprintf('Barrido angulos completo (%d puntos).', numel(angulos));
            lblStatus.FontColor = [0.15 0.55 0.25];
        catch ME
            lblStatus.Text      = ['Error: ' ME.message];
            lblStatus.FontColor = [0.8 0.1 0.1];
            uialert(fig, ME.message, 'Error en barrido de angulos');
        end
    end

    % ---- Barrido de distancias ----
    function onBarrerDistancias(~, ~)
        lblStatus.Text      = 'Barriendo distancias...';
        lblStatus.FontColor = [0.7 0.5 0.0]; drawnow;
        try
            ov_base  = buildOvr();
            dists    = 5:4:25;
            th_fin   = zeros(size(dists));
            cla(axBar(2));
            for ii = 1:numel(dists)
                ov_b = ov_base;
                ov_b.d_ini_cm = dists(ii);
                out = correrSim(ov_b);
                th_fin(ii) = rad2deg(out.theta_out.Data(end));
                lblStatus.Text = sprintf('Barrido distancias: %d/%d (%.0f cm)...', ii, numel(dists), dists(ii));
                drawnow;
            end
            plot(axBar(2), dists, th_fin, 's-', 'Color', [0.75 0.25 0.1], 'LineWidth', 1.8, 'MarkerFaceColor', [0.75 0.25 0.1]);
            yline(axBar(2), ov_base.theta_ref_deg, '--k');
            lblStatus.Text      = sprintf('Barrido distancias completo (%d puntos).', numel(dists));
            lblStatus.FontColor = [0.15 0.55 0.25];
        catch ME
            lblStatus.Text      = ['Error: ' ME.message];
            lblStatus.FontColor = [0.8 0.1 0.1];
            uialert(fig, ME.message, 'Error en barrido de distancias');
        end
    end

    function onLimpiarPrev(~, ~)
        for kk = 1:3, cla(axPrev(kk)); end
        for kk = 1:2, cla(axBar(kk));  end
        lblStatus.Text      = 'Graficas limpias.';
        lblStatus.FontColor = [0.2 0.55 0.25];
    end

    function onReset(~, ~)
        d2 = cargarDefaults();
        setVal(efThRef, sldThRef, 35);
        setVal(efTh0,   sldTh0,   d2.theta0_deg);
        setVal(efDini,  sldDini,  18);
        efGov.Value     = d2.gov_deg;
        efTauRef.Value  = d2.tau_ref;
        efRefRate.Value = d2.ref_rate_deg;
        efSupVhi.Value  = d2.sup_v_hi;
        efSupAdv.Value  = d2.sup_adv;
        efSupRet.Value  = d2.sup_ret;
        efKm.Value      = d2.k_m;
        efPwmMin.Value  = d2.pwm_min;
        efPwmMax.Value  = d2.pwm_max;
        efUsarImu.Value = d2.usar_imu;
        efFanFac.Value  = d2.fan_fac;
        efFanT0.Value   = d2.fan_t0;
        efFanDur.Value  = d2.fan_dur;
        efBc.Value      = d2.b_c;
        efTauF.Value    = d2.tau_f;
        efPert.Value    = 0;
        efPertT.Value   = d2.pert_t_on;
        efPertDur.Value = d2.pert_dur;
        efTsim.Value    = d2.t_sim;
        efFps.Value     = 24;
        efVel.Value     = 1;
        ddSal.Value     = 'ninguna';
        ddEsc.Value     = 'carro_lejos';
        actualizarPrev();
        lblStatus.Text      = 'Valores restablecidos desde init_parametros.m';
        lblStatus.FontColor = [0.2 0.55 0.25];
    end

%% ======================================================================
%  FUNCIONES AUXILIARES
%% ======================================================================

    % Carga los valores por defecto ejecutando init_parametros.m
    % en un entorno limpio sin cerrar figuras del usuario.
    function d = cargarDefaults()
        ws_bak = evalin('base', 'whos');
        % Ejecutar en una funcion anonima para aislar el workspace
        % (no podemos usar feval con scripts, lo haremos en base con backup)
        nombres_bak = {ws_bak.name};
        % Guardar variables existentes del base workspace que no sean ovr
        datos_bak = struct();
        for nb = 1:numel(nombres_bak)
            if ~strcmp(nombres_bak{nb}, 'ovr')
                try
                    datos_bak.(nombres_bak{nb}) = evalin('base', nombres_bak{nb});
                catch
                end
            end
        end

        % Ejecutar init sin ovr para obtener defaults
        evalin('base', sprintf('cd(''%s''); run(''init_parametros.m'');', ...
            strrep(folder, '''', '''''')));

        campos = {'theta_ref_deg','theta0_deg','gov_deg','tau_ref','ref_rate_deg', ...
                  'd_ini_cm','sup_v_hi','sup_adv','sup_ret', ...
                  'k_m','pwm_min','pwm_max','usar_imu','fan_fac','fan_t0','fan_dur', ...
                  'b_c','tau_f','pert_t_on','pert_dur','t_sim'};
        d = struct();
        for ic = 1:numel(campos)
            try
                d.(campos{ic}) = evalin('base', campos{ic});
            catch
                % Si el campo no existe poner NaN (no deberia pasar)
                d.(campos{ic}) = NaN;
            end
        end

        % Restaurar base workspace
        evalin('base','clear');
        fn_bak = fieldnames(datos_bak);
        for nb = 1:numel(fn_bak)
            try
                assignin('base', fn_bak{nb}, datos_bak.(fn_bak{nb}));
            catch
            end
        end
    end

    % Ejecuta init_parametros.m con ovr correctamente (sobrevive al clear)
    function out = correrSim(ov)
        % Asignamos ovr, ejecutamos init, el mecanismo de preservacion
        % en init_parametros.m (lineas 1-9) guarda y restaura ovr.
        assignin('base', 'ovr', ov);
        evalin('base', sprintf('cd(''%s''); run(''init_parametros.m'');', ...
            strrep(folder, '''', '''''')));
        mdl = 'modelo_miso_compuerta';
        load_system(mdl);
        out = sim(mdl);
    end

    % ---- Animacion personalizada (modo 'personalizado') ----
    function correrAnimacionPersonalizada(ov, salida, fps_v, velocidad)
        % Simular
        out_ani = correrSim(ov);

        % Leer geometria desde el workspace (ya cargado por correrSim)
        gate_L_v   = evalin('base', 'gate_L');
        l_c_v      = evalin('base', 'l_c');
        H          = evalin('base', 'H_pivote');
        car_len    = evalin('base', 'car_len_cm');
        wheel_r    = evalin('base', 'wheel_r_cm');
        y_top      = evalin('base', 'y_chasis_hi');
        fan_w      = evalin('base', 'fan_w_cm');
        fan_t      = evalin('base', 'fan_t_cm');
        x_post     = evalin('base', 'x_post_cm');
        sup_v_hi_v = ov.sup_v_hi;

        L  = gate_L_v * 100;
        lc = l_c_v    * 100;
        y_fan    = y_top + 0.6 + fan_w/2;
        y_arm    = H + 2;
        ang_c    = linspace(0, 2*pi, 40);
        xlim_a   = [-32 64];
        ylim_a   = [-8 31];

        % Senales interpoladas
        t_log = out_ani.theta_out.Time;
        S.th   = rad2deg(out_ani.theta_out.Data);
        S.thr  = rad2deg(out_ani.thref_out.Data);
        S.pwm  = out_ani.pwm_out.Data;
        S.vfan = out_ani.vfan_out.Data;
        S.d    = out_ani.d_out.Data    * 100;
        S.dest = out_ani.d_est_out.Data * 100;
        S.dref = out_ani.dref_out.Data  * 100;
        S.dnom = out_ani.dnom_out.Data  * 100;
        S.a    = out_ani.a_meas_out.Data;

        dt_f = velocidad / fps_v;
        tF   = 0 : dt_f : t_log(end);
        nF   = numel(tF);
        Sf   = structfun(@(y) interp1(t_log, y, tF, 'linear', 'extrap'), S, 'UniformOutput', false);
        vdn  = gradient(Sf.dnom, tF);

        txt_esc = sprintf('Personalizado: theta_{ref}=%.1f deg | d_{ini}=%.1f cm | k_m=%.5f', ...
            ov.theta_ref_deg, ov.d_ini_cm, ov.k_m);

        % ---- Figura de animacion ----
        figA = figure('Name', 'Cinematica del sistema (GUI)', ...
            'Color', 'w', 'Position', [80 80 1280 720], 'Resize', 'off');

        axM = axes(figA, 'Position', [0.04 0.45 0.92 0.51]);
        hold(axM, 'on'); box(axM, 'on');
        daspect(axM, [1 1 1]);
        axM.XLim = xlim_a; axM.YLim = ylim_a;
        axM.XLimMode = 'manual'; axM.YLimMode = 'manual';
        axM.XTick = -30:10:60; axM.YTick = [];
        xlabel(axM, 'x [cm]  (0 = posicion de reposo de la compuerta)');
        title(axM, ['Cinematica - ' txt_esc], 'Interpreter', 'none');
        fill(axM, [xlim_a(1) xlim_a(2) xlim_a(2) xlim_a(1)], ...
            [ylim_a(1) ylim_a(1) 0 0], [0.90 0.88 0.82], 'EdgeColor', 'none');
        plot(axM, xlim_a, [0 0], 'k-', 'LineWidth', 2);
        % Soporte en L invertida
        patch(axM, 'XData', x_post+[-6 6 6 -6], 'YData', [0 0 1 1], ...
            'FaceColor', [0.3 0.3 0.34], 'EdgeColor', 'k');
        plot(axM, [x_post x_post 0 0], [1 y_arm y_arm H], '-', ...
            'Color', [0.35 0.36 0.42], 'LineWidth', 7);
        text(axM, x_post-2.2, 13, 'soporte en L invertida', ...
            'FontSize', 8, 'Color', [0.3 0.3 0.35], 'Rotation', 90, 'HorizontalAlignment', 'center');
        plot(axM, [0 0], [H H-L], ':', 'Color', [0.55 0.55 0.55]);
        text(axM, 0.8, H-L-1.2, 'reposo (\theta = 0)', 'FontSize', 8, 'Color', [0.4 0.4 0.4]);
        hRef  = plot(axM, [0 0], [H H-L], '--', 'Color', [0.35 0.35 0.35], 'LineWidth', 1.5);
        hGate = plot(axM, [0 0], [H H-L], '-', 'LineWidth', 10, 'Color', [0.2 0.6 0.3]);
        hMass = plot(axM, 0, H-lc, 'o', 'MarkerSize', 8, 'MarkerFaceColor', 'k', 'MarkerEdgeColor', 'w');
        plot(axM, 0, H, 'o', 'MarkerSize', 13, 'MarkerFaceColor', [0.95 0.85 0.25], ...
            'MarkerEdgeColor', 'k', 'LineWidth', 1.2);
        text(axM, -2.5, H+4.2, 'pivote + potenciometro', 'FontSize', 8, ...
            'Color', [0.3 0.3 0.3], 'HorizontalAlignment', 'right');
        hArc  = plot(axM, NaN, NaN, 'k-', 'LineWidth', 1);
        hArcT = text(axM, 0, 0, '', 'FontSize', 10, 'FontWeight', 'bold', 'BackgroundColor', [1 1 1 0.7]);
        % Carro
        hBody = patch(axM, 'XData', [0 1 1 0], 'YData', [3.5 3.5 y_top y_top], ...
            'FaceColor', [0.25 0.45 0.80], 'EdgeColor', 'k', 'LineWidth', 1.2);
        hMnt  = patch(axM, 'XData', [0 1 1 0], 'YData', [y_top y_top y_top+0.6 y_top+0.6], ...
            'FaceColor', [0.3 0.3 0.3], 'EdgeColor', 'k');
        hImu  = patch(axM, 'XData', [0 1 1 0], 'YData', [y_top y_top y_top+1.6 y_top+1.6], ...
            'FaceColor', [0.95 0.85 0.25], 'EdgeColor', 'k');
        hImuT = text(axM, 0, y_top+0.8, 'IMU', 'FontSize', 7, 'HorizontalAlignment', 'center');
        hFanB = patch(axM, 'XData', [0 1 1 0], ...
            'YData', [y_top+0.6 y_top+0.6 y_top+0.6+fan_w y_top+0.6+fan_w], ...
            'FaceColor', [0.55 0.55 0.58], 'EdgeColor', 'k', 'LineWidth', 1.2);
        hBlad = plot(axM, [0 0], [0 0], '-', 'Color', 'k', 'LineWidth', 3);
        hW  = gobjects(2, 1);
        hSp = gobjects(2, 2);
        for k = 1:2
            hW(k)    = patch(axM, 'XData', zeros(1,40), 'YData', zeros(1,40), ...
                'FaceColor', [0.15 0.15 0.15], 'EdgeColor', 'k');
            hSp(k,1) = plot(axM, [0 0], [0 0], '-', 'Color', [0.85 0.85 0.85], 'LineWidth', 1.5);
            hSp(k,2) = plot(axM, [0 0], [0 0], '-', 'Color', [0.85 0.85 0.85], 'LineWidth', 1.5);
        end
        hAcc = quiver(axM, 0, y_top+4.6, 0, 0, 0, 'Color', [0.8 0.4 0], 'LineWidth', 2, 'MaxHeadSize', 2);
        % Particulas de aire
        nS = 5; nP_air = 7;
        yS = y_fan + (-2:1:2)*0.7;
        s0 = repmat(linspace(0,1,nP_air+1), nS, 1);
        s0 = s0(:, 1:nP_air) + 0.07*(1:nS)';
        hAir   = plot(axM, NaN, NaN, '.', 'Color', [0.2 0.55 0.95], 'MarkerSize', 12);
        ph_air = 0;
        % Indicadores de distancia
        plot(axM, xlim_a, [-3 -3], '-', 'Color', [0.85 0.85 0.85]);
        plot(axM, [0 0], [-5 0], ':', 'Color', [0.5 0.5 0.5]);
        text(axM, 0, -6.3, 'd = 0', 'HorizontalAlignment', 'center', 'FontSize', 8, 'Color', [0.4 0.4 0.4]);
        hDim  = plot(axM, [0 0], [-3 -3], 'k-', 'LineWidth', 2);
        hDimT = text(axM, 0, -1.8, '', 'HorizontalAlignment', 'center', 'FontSize', 10, 'FontWeight', 'bold');
        hDref = plot(axM, [0 0], [-5 -1.5], '-', 'Color', [0.1 0.6 0.1], 'LineWidth', 2);
        hDest = plot(axM, 0, -4.2, 'v', 'MarkerSize', 7, 'MarkerFaceColor', [0.9 0.5 0.1], 'MarkerEdgeColor', 'k');
        hDnom = plot(axM, 0, -4.2, 'd', 'MarkerSize', 7, 'MarkerFaceColor', [0.6 0.2 0.8], 'MarkerEdgeColor', 'k');
        text(axM, 62, -5.2, {'| verde: d_{ref}','v naranja: d estimada (IMU)','\diamondsuit violeta: d_{nom} (supervisor)'}, ...
            'FontSize', 7, 'HorizontalAlignment', 'right', 'VerticalAlignment', 'top');
        hInfo = text(axM, 13, 30.5, '', 'FontName', 'Consolas', 'FontSize', 9, ...
            'VerticalAlignment', 'top', 'BackgroundColor', [1 1 1 0.85], 'Margin', 4, 'Interpreter', 'none');

        % Inset ventilador frontal
        axF = axes(figA, 'Position', [0.83 0.75 0.12 0.19]);
        hold(axF, 'on'); axis(axF, 'equal', 'off'); axF.XLim = [-1.4 1.4]; axF.YLim = [-1.4 1.4];
        rectangle(axF, 'Position', [-1.3 -1.3 2.6 2.6], 'Curvature', 0.15, 'FaceColor', [0.8 0.8 0.82]);
        rectangle(axF, 'Position', [-1.1 -1.1 2.2 2.2], 'Curvature', [1 1], 'FaceColor', [0.95 0.95 0.97]);
        nB = 7; hB = gobjects(nB, 1);
        for k = 1:nB
            hB(k) = patch(axF, 'XData', [0 0 0], 'YData', [0 0 0], ...
                'FaceColor', [0.3 0.3 0.35], 'EdgeColor', 'none');
        end
        rectangle(axF, 'Position', [-0.22 -0.22 0.44 0.44], 'Curvature', [1 1], 'FaceColor', [0.2 0.2 0.2]);
        title(axF, 'Ventilador (frontal)', 'FontSize', 9);
        hFT = text(axF, 0, -1.45, '', 'HorizontalAlignment', 'center', 'FontSize', 9, 'VerticalAlignment', 'top');
        phi = 0;

        % Graficas inferiores
        ax1 = axes(figA, 'Position', [0.05 0.08 0.26 0.25]);
        hold(ax1,'on'); grid(ax1,'on'); box(ax1,'on');
        plot(ax1, tF, Sf.thr, '--', 'Color', [0.4 0.4 0.4], 'LineWidth', 1.2);
        plot(ax1, tF, Sf.th,  '-',  'Color', [0.1 0.5 0.2], 'LineWidth', 1.8);
        ylabel(ax1, '\theta [deg]'); xlabel(ax1, 't [s]');
        title(ax1, 'Angulo (--- ref)'); xlim(ax1, [0 tF(end)]);

        ax2 = axes(figA, 'Position', [0.38 0.08 0.26 0.25]);
        hold(ax2,'on'); grid(ax2,'on'); box(ax2,'on');
        plot(ax2, tF, Sf.pwm, '-', 'Color', [0.1 0.4 0.8], 'LineWidth', 1.8);
        yline(ax2, sup_v_hi_v, '--r', sprintf('%g %% (sup)', sup_v_hi_v), 'LabelHorizontalAlignment', 'left');
        yline(ax2, 100, ':k');
        ylabel(ax2, 'PWM [%]'); xlabel(ax2, 't [s]');
        title(ax2, 'PWM del ventilador'); xlim(ax2, [0 tF(end)]); ylim(ax2, [0 105]);

        ax3 = axes(figA, 'Position', [0.71 0.08 0.26 0.25]);
        hold(ax3,'on'); grid(ax3,'on'); box(ax3,'on');
        hL_a(1) = plot(ax3, tF, Sf.d,    '-',  'Color', [0 0 0],        'LineWidth', 1.8);
        hL_a(2) = plot(ax3, tF, Sf.dest, '-',  'Color', [0.9 0.5 0.1],  'LineWidth', 1);
        hL_a(3) = plot(ax3, tF, Sf.dref, '-',  'Color', [0.1 0.6 0.1],  'LineWidth', 1);
        hL_a(4) = plot(ax3, tF, Sf.dnom, '-',  'Color', [0.6 0.2 0.8],  'LineWidth', 1.5);
        ylabel(ax3, 'd [cm]'); xlabel(ax3, 't [s]'); title(ax3, 'Dist. carro-compuerta');
        xlim(ax3, [0 tF(end)]); ylim(ax3, [max(0, min(Sf.d)-2) min(30, max(Sf.d)+4)]);
        legend(ax3, hL_a, {'d real','d IMU','d_{ref}','d_{nom}'}, 'Location', 'best', 'FontSize', 7, 'AutoUpdate', 'off');

        cur = gobjects(3, 1); mk = gobjects(3, 1);
        axs3 = [ax1 ax2 ax3];
        yl   = {ylim(ax1), ylim(ax2), ylim(ax3)};
        for k = 1:3
            cur(k) = plot(axs3(k), [0 0], yl{k}, '-', 'Color', [0.8 0.1 0.1], 'LineWidth', 1);
        end
        mk(1) = plot(ax1, 0, Sf.th(1),  'o', 'MarkerFaceColor', [0.1 0.5 0.2], 'MarkerEdgeColor', 'k');
        mk(2) = plot(ax2, 0, Sf.pwm(1), 'o', 'MarkerFaceColor', [0.1 0.4 0.8], 'MarkerEdgeColor', 'k');
        mk(3) = plot(ax3, 0, Sf.d(1),   'o', 'MarkerFaceColor', 'k',           'MarkerEdgeColor', 'w');

        % Video
        use_mp4 = any(strcmp(salida, {'mp4','ambos'}));
        use_gif = any(strcmp(salida, {'gif','ambos'}));
        base_nom = sprintf('animacion_gui_th%03d_d%02d', round(ov.theta_ref_deg), round(ov.d_ini_cm));
        if use_mp4
            vw = VideoWriter([base_nom '.mp4'], 'MPEG-4');
            vw.FrameRate = fps_v; vw.Quality = 90; open(vw);
        end
        gif_cada = max(1, round(fps_v/12)); gif_ok = false;

        % ---- Bucle de animacion ----
        for k = 1:nF
            if ~isvalid(figA), break; end
            t    = tF(k);
            th   = Sf.th(k);   thr  = Sf.thr(k);
            pwm  = Sf.pwm(k);  vf   = Sf.vfan(k);
            d    = Sf.d(k);    dest = Sf.dest(k);
            dref = Sf.dref(k); dnom = Sf.dnom(k);

            % Compuerta
            sx = -sind(th); sy = -cosd(th);
            set(hGate, 'XData', [0 L*sx],          'YData', [H H+L*sy]);
            set(hMass, 'XData', lc*sx,              'YData', H+lc*sy);
            set(hRef,  'XData', [0 -L*sind(thr)],  'YData', [H H-L*cosd(thr)]);
            aa = linspace(-pi/2, -pi/2-deg2rad(th), 30);
            set(hArc,  'XData', 7*cos(aa),          'YData', H+7*sin(aa));
            am = -pi/2 - deg2rad(max(th,10))/2;
            set(hArcT, 'Position', [9*cos(am) H+9*sin(am)], ...
                'String', sprintf('%.1f%s', th, char(176)));

            if th < 2
                col_g = [0.45 0.45 0.55]; est_g = 'en reposo (colgando)';
            elseif th > 150
                col_g = [0.75 0.25 0.25]; est_g = 'SOBREPASADA (>150 deg)';
            elseif abs(th-thr) < 2
                col_g = [0.2 0.6 0.3];   est_g = 'sosteniendo la consigna';
            else
                col_g = [0.95 0.65 0.1]; est_g = 'fuera de consigna';
            end
            set(hGate, 'Color', col_g);

            % Carro
            xf = d;
            set(hBody, 'XData', [xf xf+car_len xf+car_len xf]);
            set(hMnt,  'XData', [xf xf+6       xf+6       xf]);
            set(hFanB, 'XData', [xf xf+fan_t   xf+fan_t   xf]);
            set(hImu,  'XData', [xf+9 xf+13 xf+13 xf+9]);
            set(hImuT, 'Position', [xf+11 y_top+0.8]);

            wheel_ang = (ov.d_ini_cm - d) / wheel_r;
            for kw = 1:2
                xw = xf + (4.5 + 7*(kw-1));
                set(hW(kw), 'XData', xw + wheel_r*cos(ang_c), ...
                    'YData', wheel_r + wheel_r*sin(ang_c));
                for ks = 1:2
                    a_sp = wheel_ang + (ks-1)*pi/2;
                    set(hSp(kw,ks), ...
                        'XData', xw + wheel_r*[-cos(a_sp) cos(a_sp)], ...
                        'YData', wheel_r + wheel_r*[-sin(a_sp) sin(a_sp)]);
                end
            end

            % Ventilador
            phi = phi + 38*(vf/100);
            hb  = 1.9*abs(cosd(phi));
            set(hBlad, 'XData', [xf+fan_t/2 xf+fan_t/2], 'YData', [y_fan-hb y_fan+hb]);
            for kb = 1:nB
                a0 = phi + (kb-1)*360/nB;
                set(hB(kb), 'XData', [0 cosd(a0)*1.0 cosd(a0+22)*0.95], ...
                    'YData', [0 sind(a0)*1.0 sind(a0+22)*0.95]);
            end
            set(hFT, 'String', sprintf('PWM %.0f %%   v_{fan} %.0f %%', pwm, vf));

            % Particulas de aire
            ph_air = mod(ph_air + 0.03 + 0.05*(vf/100), 1);
            xo = xf - 0.3; ax_ = []; ay_ = [];
            if vf > 3
                for ky = 1:nS
                    dy = H - yS(ky);
                    if th < 89 && dy > 0 && dy/cosd(th) <= L - 0.5
                        x_end = -dy*tand(th);
                    else
                        x_end = -14;
                    end
                    sp  = mod(s0(ky,:) + ph_air, 1);
                    ax_ = [ax_ xo - sp*(xo - x_end) NaN]; %#ok<AGROW>
                    ay_ = [ay_ yS(ky)*ones(1,nP_air) NaN]; %#ok<AGROW>
                end
            end
            set(hAir, 'XData', ax_, 'YData', ay_, 'MarkerSize', 6 + 8*(vf/100));

            % IMU / aceleracion
            set(hAcc, 'XData', xf+11, 'UData', -max(min(Sf.a(k),4),-4)*3);

            % Distancias
            set(hDim,  'XData', [0 xf]);
            set(hDimT, 'Position', [xf/2 -1.8], 'String', sprintf('d = %.1f cm', d));
            set(hDref, 'XData', [dref dref]);
            set(hDest, 'XData', dest);
            set(hDnom, 'XData', dnom);

            % Estado del carro
            if     vdn(k) < -0.3, est_c = 'ACERCANDO (supervisor)';
            elseif vdn(k) >  0.3, est_c = 'ALEJANDO / REGRESANDO';
            else,                  est_c = 'manteniendo'; end

            set(hInfo, 'String', sprintf( ...
                't = %5.2f s\ntheta = %5.1f deg (ref %4.1f)   %s\nPWM = %5.1f %%   v_fan = %4.1f %%\nd = %5.2f cm (d_ref %5.2f | IMU %5.2f | d_nom %5.2f)\nCarro: %s', ...
                t, th, thr, est_g, pwm, vf, d, dref, dest, dnom, est_c));

            set(cur, 'XData', [t t]);
            set(mk(1), 'XData', t, 'YData', th);
            set(mk(2), 'XData', t, 'YData', pwm);
            set(mk(3), 'XData', t, 'YData', d);
            drawnow;

            if use_mp4 || use_gif
                fr = getframe(figA);
                if ~exist('fsz','var'), fsz = size(fr.cdata); end
                if ~isequal(size(fr.cdata), fsz)
                    cv = uint8(255*ones(fsz, 'uint8'));
                    hh = min(fsz(1),size(fr.cdata,1));
                    ww = min(fsz(2),size(fr.cdata,2));
                    cv(1:hh,1:ww,:) = fr.cdata(1:hh,1:ww,:);
                    fr.cdata = cv;
                end
                if use_mp4, writeVideo(vw, fr); end
                if use_gif && mod(k-1,gif_cada) == 0
                    [A, map] = rgb2ind(fr.cdata, 128);
                    if ~gif_ok
                        imwrite(A, map, [base_nom '.gif'], 'gif', 'LoopCount', inf, 'DelayTime', gif_cada/fps_v);
                        gif_ok = true;
                    else
                        imwrite(A, map, [base_nom '.gif'], 'gif', 'WriteMode', 'append', 'DelayTime', gif_cada/fps_v);
                    end
                end
            end
        end
        if use_mp4 && exist('vw','var'), close(vw); end
        fprintf('Animacion completada: theta_ref=%.1f deg | d_ini=%.1f cm | k_m=%.6f\n', ...
            ov.theta_ref_deg, ov.d_ini_cm, ov.k_m);
    end % correrAnimacionPersonalizada

end % gui_animacion
