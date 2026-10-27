function gui_simulacion()
mdl    = 'modelo_miso_compuerta';
folder = fileparts(mfilename('fullpath'));
addpath(folder);

% Bandera para evitar que Simulink colapse si se envían muchos comandos rápidos
is_simulating = false; 

% {variable en init_parametros.m, etiqueta, valor por defecto}
P = {'theta_ref_deg','Angulo ref. [deg]',35; ...
     'theta0_deg','Angulo inicial [deg]',0; ...
     'd_ini_cm','Distancia inicial [cm]',10; ...
     'ref_rate_deg','Rampa de ref. [deg/s]',200; ...
     'gov_deg','Gobernador de ref. [deg]',90; ...
     't_sim','Tiempo de simulacion [s]',10; ...
     'k_m','k_m [N m/%]',0.0003; ...
     'tau_f','Friccion Coulomb [N m]',1e-4; ...
     'b_c','Amortig. pivote [N m s/rad]',1e-3; ...
     'imu_bias','Sesgo IMU [m/s^2]',0.02; ...
     'imu_noise_pow','Ruido IMU (potencia)',1e-5; ...
     'pert_tau_mNm','Pulso de torque ext. [mN m]',0; ...
     'fan_fac','Degradacion: factor de fuerza final',0.4; ...
     'fan_t0','Degradacion: inicio [s]',3.0; ...
     'fan_dur','Degradacion: duracion [s]',12.0; ...
     'pwm_min','PWM minimo [%]',10; ...
     'pwm_max','PWM maximo [%]',90; ...
     'sup_v_hi','Supervisor: acercar carro si PWM > [%]',70; ...
     'tau_car','tau lazo carro [s]',0.8; ...
     'q_th','Peso Q theta',10000; ...
     'q_d','Peso Q d [1/cm^2]',0.5; ...
     'q_xi','Peso Q integral',10000; ...
     'r_pwm','Peso R PWM',0.02; ...
     'r_d','Peso R d_ref [1/cm^2]',1};

nP = size(P,1);

fig = uifigure('Name','Simulacion MISO: carro + ventilador + compuerta','Position',[40 40 1300 760]);
gl  = uigridlayout(fig,[1 2]);
gl.ColumnWidth = {340,'1x'};

% ---- Panel de parametros ----
pl = uipanel(gl,'Title','Parametros y variaciones');
pg = uigridlayout(pl,[nP+13 2]); % Aumentado para el nuevo checkbox Auto-Simular
pg.RowHeight   = repmat({24},1,nP+13);
pg.ColumnWidth = {'1.6x','1x'};
pg.Scrollable  = 'on';

ef = struct();
for i = 1:nP
    uilabel(pg,'Text',P{i,2});
    % Cada vez que se cambia un valor numérico, se llama a autoSimular()
    ef.(P{i,1}) = uieditfield(pg,'numeric','Value',P{i,3}, ...
        'ValueDisplayFormat','%.6g', ...
        'ValueChangedFcn', @(~,~) autoSimular());
end

% Checkboxes (También disparan simulación al hacerles clic)
chkIMU = uicheckbox(pg,'Text','Lazo con IMU + observador','Value',true, 'ValueChangedFcn', @(~,~) autoSimular());
chkIMU.Layout.Column = [1 2];

chkDeg = uicheckbox(pg,'Text','Simular caida de bateria (degradacion)','Value',false, 'ValueChangedFcn', @(~,~) autoSimular());
chkDeg.Layout.Column = [1 2];

chkHold = uicheckbox(pg,'Text','Mantener curvas anteriores','Value',false);
chkHold.Layout.Column = [1 2];

chkAuto = uicheckbox(pg,'Text','Auto-Simular al cambiar parametros','Value',true, 'FontWeight', 'bold', 'FontColor', [0.85 0.33 0.1]);
chkAuto.Layout.Column = [1 2];

uilabel(pg,'Text','Barrer parametro');
ddP = uidropdown(pg,'Items',P(:,1)','Value','gov_deg');

uilabel(pg,'Text','Desde');
efA = uieditfield(pg,'numeric','Value',10);
uilabel(pg,'Text','Hasta');
efB = uieditfield(pg,'numeric','Value',90);
uilabel(pg,'Text','N puntos');
efN = uieditfield(pg,'numeric','Value',4,'Limits',[2 12],'RoundFractionalValues','on');

btnSim = uibutton(pg,'Text','Simular Manualmente','ButtonPushedFcn',@(~,~) onSimular());
btnSim.Layout.Column = [1 2];

btnSwp = uibutton(pg,'Text','Barrer parametro','ButtonPushedFcn',@(~,~) onBarrido());
btnSwp.Layout.Column = [1 2];

btnClr = uibutton(pg,'Text','Limpiar graficas','ButtonPushedFcn',@(~,~) limpiar());
btnClr.Layout.Column = [1 2];

lblSt = uilabel(pg,'Text','Listo.','WordWrap','on');
lblSt.Layout.Column = [1 2];

lblPWM = uilabel(pg,'Text','PWM enviado: -- %','FontWeight','bold','FontColor',[0 0.45 0.74],'WordWrap','on');
lblPWM.Layout.Column = [1 2];

% ---- Graficas ----
ag = uigridlayout(gl,[3 2]);
titulos = {'Angulo de la compuerta \theta','Senal PWM enviada al actuador [%]','Distancia carro-compuerta d (IMU vs Real)','Torque aerodinamico \tau_a', ...
           'Error del estimador de distancia (est. - real)','Aceleracion medida por la IMU'};
ylabs   = {'\theta [deg]','PWM [%]','d [cm]','\tau_a [mN m]','error [cm]','a_x [m/s^2]'};

axs = gobjects(1,6);
for k = 1:6
    axs(k) = uiaxes(ag);
    title(axs(k),titulos{k}); ylabel(axs(k),ylabs{k}); xlabel(axs(k),'t [s]');
    grid(axs(k),'on'); hold(axs(k),'on');
end

nCurvas = 0;

    function ov = leerCampos()
        ov = struct();
        for ii = 1:nP, ov.(P{ii,1}) = ef.(P{ii,1}).Value; end
        ov.usar_imu = double(chkIMU.Value);
        ov.usar_degradacion_fan = double(chkDeg.Value);
    end

    function out = correr(ov)
        assignin('base','ovr',ov);
        evalin('base',sprintf('run(''%s'');',strrep(fullfile(folder,'init_parametros.m'),'''','''''')));
        load_system(mdl);
        out = sim(mdl);
    end

    function y = remuestrear(ts,tq)
        [t,ia] = unique(ts.Time);
        d = ts.Data(ia);
        y = interp1(t,d,tq,'linear','extrap');
    end

    function limpiar()
        for kk = 1:6, cla(axs(kk)); end
        nCurvas = 0;
        lblSt.Text = 'Graficas limpias.';
        lblPWM.Text = 'PWM enviado: -- %';
    end

    function dibujar(out,etq,col)
        tq = out.d_out.Time; dreal = out.d_out.Data;
        plot(axs(1),out.theta_out.Time,rad2deg(out.theta_out.Data),'Color',col,'LineWidth',1.4,'DisplayName',etq);
        plot(axs(2),out.pwm_out.Time,out.pwm_out.Data,'Color',col,'LineWidth',1.4,'DisplayName',etq);
        plot(axs(3),tq,dreal*100,'Color',col,'LineWidth',1.4,'DisplayName',[etq ' real']);
        plot(axs(3),out.d_est_out.Time,out.d_est_out.Data*100,'--','Color',col,'LineWidth',1,'DisplayName',[etq ' observador']);
        plot(axs(3),out.dref_out.Time,out.dref_out.Data*100,':','Color',col,'LineWidth',1,'DisplayName',[etq ' d_{ref} (SIMO)']);
        plot(axs(4),out.tau_out.Time,out.tau_out.Data*1000,'Color',col,'LineWidth',1.4,'DisplayName',etq);
        plot(axs(5),tq,(remuestrear(out.d_est_out,tq)-dreal)*100,'Color',col,'LineWidth',1.4,'DisplayName',[etq ' observador']);
        plot(axs(5),tq,(remuestrear(out.d_raw_out,tq)-dreal)*100,':','Color',col,'LineWidth',1.2,'DisplayName',[etq ' doble integracion']);
        plot(axs(6),out.a_meas_out.Time,out.a_meas_out.Data,'Color',col,'LineWidth',0.8,'DisplayName',etq);
    end

    function resumen = resumir(out,ov)
        th = rad2deg(out.theta_out.Data);
        t = out.theta_out.Time;
        ess = ov.theta_ref_deg - mean(th(t >= t(end) - 1)); % promedio del ultimo segundo
        resumen = sprintf('theta final = %.2f deg (ref %.1f) | error estacionario = %.3f deg | min/max = %.1f/%.1f deg | d final = %.2f cm | PWM %.0f/%.0f %%', ...
            th(end),ov.theta_ref_deg,ess,min(th),max(th),out.d_out.Data(end)*100,min(out.pwm_out.Data),max(out.pwm_out.Data));
    end

    function marcarRef(ov)
        yline(axs(1),ov.theta_ref_deg,'--k','HandleVisibility','off');
    end

    function leyendas()
        legend(axs(1),'show','Location','best');
        legend(axs(2),'show','Location','best');
        legend(axs(3),'show','Location','best');
        legend(axs(5),'show','Location','best');
    end

    % --- NUEVA FUNCION INTERMEDIA PARA AUTO-SIMULAR ---
    function autoSimular()
        if chkAuto.Value
            onSimular();
        end
    end

    function onSimular()
        % Seguro anti-choque: Si ya esta simulando, ignorar la peticion
        if is_simulating
            return;
        end
        is_simulating = true; % Activar candado
        
        try
            lblSt.Text = 'Simulando...'; drawnow;
            ov = leerCampos();
            out = correr(ov);
            if ~chkHold.Value, limpiar(); end
            nCurvas = nCurvas + 1;
            cols = lines(8);
            dibujar(out,sprintf('corrida %d',nCurvas),cols(mod(nCurvas-1,8)+1,:));
            marcarRef(ov); leyendas();
            lblSt.Text = resumir(out,ov);
            lblPWM.Text = sprintf('PWM enviado: final = %.1f %% | min = %.1f %% | max = %.1f %%', out.pwm_out.Data(end), min(out.pwm_out.Data), max(out.pwm_out.Data));
        catch ME
            lblSt.Text = ['Error: ' ME.message];
            uialert(fig,ME.message,'Error de simulacion');
        end
        
        is_simulating = false; % Liberar candado
    end

    function onBarrido()
        if is_simulating, return; end
        is_simulating = true;
        
        try
            ov = leerCampos();
            prm = ddP.Value;
            vals = linspace(efA.Value,efB.Value,efN.Value);
            limpiar();
            cols = lines(numel(vals));
            for jj = 1:numel(vals)
                lblSt.Text = sprintf('Barrido %s: %d/%d ...',prm,jj,numel(vals)); drawnow;
                ov.(prm) = vals(jj);
                out = correr(ov);
                dibujar(out,sprintf('%s=%.4g',prm,vals(jj)),cols(jj,:));
                th = rad2deg(out.theta_out.Data);
                fprintf('%s=%.4g -> theta final %.2f deg (min/max %.1f/%.1f)\n',prm,vals(jj),th(end),min(th),max(th));
            end
            marcarRef(ov); leyendas();
            lblSt.Text = sprintf('Barrido de %s completo (%d corridas).',prm,numel(vals));
        catch ME
            lblSt.Text = ['Error: ' ME.message];
            uialert(fig,ME.message,'Error en el barrido');
        end
        
        is_simulating = false;
    end
end