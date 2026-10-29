function datos = uart_esp32(puerto, duracion_s, theta_ref_deg, d_ref_cm)
%% uart_esp32.m
% Lee la telemetria $TEL de la Estacion Base (ESP32) por UART y le envia la
% consigna $CMD (modo autonomo, motores y ventilador habilitados).
% Protocolo: code/C/README.md, seccion 2.1.
%
% Uso:
%   datos = uart_esp32("COM5");            % 20 s, theta_ref = 35 deg, d_ref = 10 cm
%   datos = uart_esp32("COM5", 30, 40, 12);
%
% Al terminar (o con Ctrl+C) se envia un $CMD con motores y ventilador apagados.
% Cierra 'idf.py monitor' antes: el puerto COM no se puede compartir.

if nargin < 2, duracion_s    = 20;   end
if nargin < 3, theta_ref_deg = 35.0; end
if nargin < 4, d_ref_cm      = 10.0; end

sp = serialport(puerto, 115200);
configureTerminator(sp, "LF");
sp.Timeout = 1;
flush(sp);
warning('off', 'serialport:serialport:ReadlineWarning'); % timeouts se manejan abajo
% Parada segura al salir por cualquier motivo (fin, error o Ctrl+C)
limpieza = onCleanup(@() parar(sp, theta_ref_deg, d_ref_cm));

N = ceil(duracion_s * 50) + 100;   % la base envia a 50 Hz
M = nan(N, 9);                     % [t theta d v ax u_fan u_car e_d status]
n = 0;

figure('Name', 'Telemetria ESP32');
subplot(2,1,1); h_th = animatedline('Color', 'b'); grid on;
ylabel('\theta [deg]'); yline(theta_ref_deg, 'r--');
subplot(2,1,2); h_d = animatedline('Color', 'k'); grid on;
ylabel('d [cm]'); xlabel('t [s]'); yline(d_ref_cm, 'r--');

t0 = tic;
t_cmd = -inf;
while toc(t0) < duracion_s && n < N
    % Reenvio periodico: abrir el puerto puede reiniciar la base y perder la consigna
    if toc(t0) - t_cmd > 1.0
        writeline(sp, sprintf("$CMD,%.2f,%.2f,0.00,0,1,1", theta_ref_deg, d_ref_cm));
        t_cmd = toc(t0);
    end

    linea = readline(sp);
    if isempty(linea), continue; end          % timeout sin datos
    v = sscanf(linea, '$TEL,%f,%f,%f,%f,%f,%f,%f,%f');
    if numel(v) ~= 8, continue; end           % texto de arranque u otra linea

    n = n + 1;
    M(n,:) = [toc(t0), v.'];
    addpoints(h_th, M(n,1), M(n,2));
    addpoints(h_d,  M(n,1), M(n,3));
    drawnow limitrate;
end

M = M(1:n,:);
nombres = {'t','theta_deg','d_cm','v_ms','ax_ms2','u_fan','u_car','e_d_cm','status'};
for k = 1:numel(nombres)
    datos.(nombres{k}) = M(:,k);
end
fprintf('%d tramas recibidas en %.1f s.\n', n, toc(t0));
end

function parar(sp, theta_ref_deg, d_ref_cm)
try
    writeline(sp, sprintf("$CMD,%.2f,%.2f,0.00,0,0,0", theta_ref_deg, d_ref_cm));
catch
    % puerto ya cerrado o desconectado: nada mas que hacer
end
delete(sp);
end
