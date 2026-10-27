function fig = gui_esp32()
%GUI_ESP32 Interfaz en tiempo real con el hardware (Estacion Base ESP32 por UART).
% Envia la consigna $CMD y grafica la telemetria $TEL (protocolo: code/C/README.md, 2.1).
% Muestra en vivo theta, la senal PWM enviada al ventilador (grafica + indicador numerico
% con min/max y saturacion), la distancia estimada por la IMU y el esfuerzo del carro.
% Al desconectar o cerrar la ventana apaga motores y ventilador y deja los datos en
% 'datos_esp32' del workspace base. Cierra 'idf.py monitor' antes: el COM no se comparte.
p = cargar_parametros(fileparts(mfilename('fullpath')));

T_VENTANA = 30;    % [s] historial visible en las graficas
T_SIN_DATOS = 0.5; % [s] sin tramas -> base desconectada
T_ASIENTO = 2.0;   % [s] espera tras cambiar la consigna antes de medir el error estacionario
T_PROMEDIO = 1.0;  % [s] ventana del promedio del error estacionario

sp = [];
M = nan(3000, 9);  % [t theta d v ax u_fan u_car e_d status]
n = 0;
t0 = tic; t_ultima = -inf; t_consigna = 0;

fig = uifigure('Name','ESP32: compuerta + ventilador + carro (hardware)','Position',[60 60 1250 720], ...
    'CloseRequestFcn', @(~,~) cerrar());
gl = uigridlayout(fig,[1 2]);
gl.ColumnWidth = {300,'1x'};

% ---- Panel de conexion y consignas ----
pl = uipanel(gl,'Title','Conexion y consignas');
pg = uigridlayout(pl,[15 2]);
pg.RowHeight = repmat({26},1,15);
pg.ColumnWidth = {'1.4x','1x'};

uilabel(pg,'Text','Puerto');
ddCOM = uidropdown(pg,'Items',puertos());
btnRef = uibutton(pg,'Text','Actualizar puertos','ButtonPushedFcn',@(~,~) set(ddCOM,'Items',puertos()));
btnRef.Layout.Column = [1 2];
btnCon = uibutton(pg,'Text','Conectar','FontWeight','bold','ButtonPushedFcn',@(~,~) onConectar());
btnCon.Layout.Column = [1 2];

uilabel(pg,'Text','Angulo ref. [deg]');
efTh = uieditfield(pg,'numeric','Value',p.theta_ref_deg,'Limits',[0 90],'ValueChangedFcn',@(~,~) onConsigna());
uilabel(pg,'Text','Distancia ref. [cm]');
efD = uieditfield(pg,'numeric','Value',p.d_ini_cm,'Limits',[p.d_min p.d_max]*100,'ValueChangedFcn',@(~,~) onConsigna());
chkMot = uicheckbox(pg,'Text','Motores del carro','Value',false,'ValueChangedFcn',@(~,~) onConsigna());
chkMot.Layout.Column = [1 2];
chkFan = uicheckbox(pg,'Text','Ventilador','Value',false,'ValueChangedFcn',@(~,~) onConsigna());
chkFan.Layout.Column = [1 2];
btnStop = uibutton(pg,'Text','PARADA','FontWeight','bold','FontColor','w','BackgroundColor',[0.8 0.1 0.1], ...
    'ButtonPushedFcn',@(~,~) onParada());
btnStop.Layout.Column = [1 2];

lblCon = uilabel(pg,'Text','Desconectado.','WordWrap','on');
lblCon.Layout.Column = [1 2];
lblPWM = uilabel(pg,'Text','PWM enviado: -- %','FontWeight','bold','FontColor',[0 0.45 0.74],'FontSize',14);
lblPWM.Layout.Column = [1 2];
lblPWMx = uilabel(pg,'Text','min -- % | max -- %','FontColor',[0 0.45 0.74]);
lblPWMx.Layout.Column = [1 2];
lblTh = uilabel(pg,'Text','theta = -- deg');
lblTh.Layout.Column = [1 2];
lblD = uilabel(pg,'Text','d (IMU) = -- cm | v = -- m/s');
lblD.Layout.Column = [1 2];
lblCar = uilabel(pg,'Text','Carro: --','WordWrap','on');
lblCar.Layout.Column = [1 2];
lblEE = uilabel(pg,'Text','Error estacionario: --','FontWeight','bold','WordWrap','on');
lblEE.Layout.Column = [1 2];
pg.RowHeight{15} = 40; % el texto ocupa dos lineas

% ---- Graficas ----
ag = uigridlayout(gl,[2 2]);
titulos = {'Angulo de la compuerta \theta','Senal PWM enviada al ventilador [%]', ...
           'Distancia carro-compuerta d (IMU)','Esfuerzo de los motores del carro [%]'};
ylabs = {'\theta [deg]','PWM [%]','d [cm]','u_{car} [%]'};
axs = gobjects(1,4);
for k = 1:4
    axs(k) = uiaxes(ag);
    title(axs(k),titulos{k}); ylabel(axs(k),ylabs{k}); xlabel(axs(k),'t [s]');
    grid(axs(k),'on'); hold(axs(k),'on');
end
nMax = T_VENTANA * 50; % la base envia a 50 Hz
hTh    = animatedline(axs(1),'Color',[0 0.45 0.74],'LineWidth',1.4,'MaximumNumPoints',nMax);
hThRef = animatedline(axs(1),'Color','r','LineStyle','--','MaximumNumPoints',nMax);
hPWM   = animatedline(axs(2),'Color',[0 0.45 0.74],'LineWidth',1.4,'MaximumNumPoints',nMax);
hD     = animatedline(axs(3),'Color','k','LineWidth',1.4,'MaximumNumPoints',nMax);
hDRef  = animatedline(axs(3),'Color','r','LineStyle','--','MaximumNumPoints',nMax);
hCar   = animatedline(axs(4),'Color',[0.85 0.33 0.1],'LineWidth',1.4,'MaximumNumPoints',nMax);
yline(axs(2),p.pwm_min,'--r','min'); yline(axs(2),p.pwm_max,'--r','max');
ylim(axs(1),[0 90]); ylim(axs(2),[0 100]); ylim(axs(3),[p.d_min p.d_max]*100); ylim(axs(4),[-100 100]);

% Reenvio periodico de la consigna (abrir el COM puede reiniciar la base) y vigilancia del enlace
tmr = timer('ExecutionMode','fixedSpacing','Period',1.0,'TimerFcn',@(~,~) vigilar());

% Permiten probar la GUI sin hardware: inyectar tramas $TEL y leer el $CMD que se enviaria
fig.UserData.procesar = @procesar;
fig.UserData.lineaCmd = @lineaCmd;

    function onConectar()
        if ~isempty(sp), desconectar(); return; end
        try
            sp = serialport(ddCOM.Value, 115200, 'Timeout', 1);
        catch err
            uialert(fig, err.message, 'No se pudo abrir el puerto'); sp = []; return;
        end
        configureTerminator(sp, "LF");
        flush(sp);
        configureCallback(sp, "terminator", @(src,~) procesar(readline(src)));
        btnCon.Text = 'Desconectar'; ddCOM.Enable = 'off';
        lblCon.Text = sprintf('Conectado a %s. Esperando telemetria...', ddCOM.Value);
        enviarCmd();
        start(tmr);
    end

    function desconectar()
        stop(tmr);
        onParada();
        delete(sp); sp = [];
        btnCon.Text = 'Conectar'; ddCOM.Enable = 'on';
        lblCon.Text = 'Desconectado. Datos en ''datos_esp32''.';
        guardar();
    end

    function onParada()
        chkMot.Value = false; chkFan.Value = false;
        onConsigna();
    end

    function onConsigna()
        t_consigna = toc(t0); % el error estacionario se vuelve a medir desde aqui
        enviarCmd();
    end

    function enviarCmd()
        if isempty(sp), return; end
        try
            writeline(sp, lineaCmd());
        catch
            lblCon.Text = 'No se pudo enviar: puerto desconectado.'; lblCon.FontColor = [0.8 0.1 0.1];
        end
    end

    function s = lineaCmd()
        % Modo 0: el control corre en el carro; matlab_pwm solo se usa en modo HIL (1)
        s = sprintf("$CMD,%.2f,%.2f,0.00,0,%d,%d", efTh.Value, efD.Value, chkMot.Value, chkFan.Value);
    end

    function vigilar()
        enviarCmd();
        if toc(t0) - t_ultima > T_SIN_DATOS
            lblCon.Text = 'Base sin datos: revise cable, puerto o reinicio del ESP32.';
            lblCon.FontColor = [0.8 0.1 0.1];
        end
    end

    function procesar(linea)
        v = sscanf(linea, '$TEL,%f,%f,%f,%f,%f,%f,%f,%f');
        if numel(v) ~= 8, return; end % texto de arranque o linea incompleta
        t = toc(t0); t_ultima = t;
        n = n + 1;
        if n > size(M,1), M = [M; nan(size(M))]; end
        M(n,:) = [t, v.'];

        addpoints(hTh, t, v(1));  addpoints(hThRef, t, efTh.Value);
        addpoints(hPWM, t, v(5));
        addpoints(hD, t, v(2));   addpoints(hDRef, t, efD.Value);
        addpoints(hCar, t, v(6));

        pwm = M(1:n,6);
        sat = '';
        if v(5) <= p.pwm_min || v(5) >= p.pwm_max, sat = '  (SATURADO)'; end
        lblPWM.Text = sprintf('PWM enviado: %.1f %%%s', v(5), sat);
        lblPWMx.Text = sprintf('min %.1f %% | max %.1f %%', min(pwm), max(pwm));
        lblTh.Text = sprintf('theta = %.2f deg (ref %.1f, error %.2f)', v(1), efTh.Value, efTh.Value - v(1));
        lblD.Text = sprintf('d (IMU) = %.2f cm | v = %.3f m/s', v(2), v(3));
        % La base envia ceros cuando no recibe nada del carro por ESP-NOW
        if all(v(2:7) == 0)
            lblCar.Text = 'Carro: sin enlace ESP-NOW.'; lblCar.FontColor = [0.8 0.1 0.1];
        else
            lblCar.Text = sprintf('Carro: enlazado (modo %d).', v(8)); lblCar.FontColor = [0 0.5 0];
        end
        % Error estacionario: promedio de la ultima ventana, solo cuando la consigna lleva T_ASIENTO quieta
        if t - t_consigna >= T_ASIENTO
            w = M(1:n,1) >= t - T_PROMEDIO;
            lblEE.Text = sprintf('Error estacionario (prom. %.0f s): theta %.3f deg | d %.2f cm', ...
                T_PROMEDIO, efTh.Value - mean(M(w,2)), mean(M(w,8)));
        else
            lblEE.Text = sprintf('Error estacionario: esperando %.1f s a que se asiente.', T_ASIENTO - (t - t_consigna));
        end
        lblCon.Text = sprintf('Recibiendo: %d tramas.', n); lblCon.FontColor = [0 0 0];
        drawnow limitrate;
    end

    function guardar()
        nombres = {'t','theta_deg','d_cm','v_ms','ax_ms2','u_fan','u_car','e_d_cm','status'};
        datos = struct();
        for c = 1:numel(nombres), datos.(nombres{c}) = M(1:n,c); end
        assignin('base', 'datos_esp32', datos);
    end

    function cerrar()
        if ~isempty(sp), desconectar(); else, guardar(); end
        delete(tmr);
        delete(fig);
    end
end

function p = cargar_parametros(folder) %#ok<INUSD> usado dentro de evalc
% Funcion local sin anidadas: run() puede crear variables aqui (no en gui_esp32)
evalc('run(fullfile(folder,''init_parametros.m''))');
p = struct('theta_ref_deg',theta_ref_deg,'d_ini_cm',d_ini_cm,'d_min',d_min,'d_max',d_max, ...
           'pwm_min',pwm_min,'pwm_max',pwm_max);
end

function p = puertos()
p = cellstr(serialportlist("available"));
if isempty(p), p = {'(sin puertos)'}; end
end
