% COMPARACION: LGR EN LAZO ABIERTO VS LGR + POLOS LQR EN LAZO CERRADO

clear; clc; close all;

%% Planta
run('init_parametros.m'); 

G_s   = tf(b2, [1, -a22, -a21]);     % compuerta pura
G_fan = tf(1, [fan_tau, 1]);         % actuador
G_pl  = G_s * G_fan;                 % planta completa en lazo abierto

%% LQR+I (Riccati)
K = K_lqr_nom;
A_cl_fan = [0, 1, 0, 0; ...
    a21, a22, b2, 0; ...
    -K(1)/fan_tau, -K(2)/fan_tau, -1/fan_tau, -K(3)/fan_tau; ...
    1, 0, 0, 0];
e_lqr = eig(A_cl_fan);               % Polos en lazo cerrado logrados por el LQR

%% Graficas
fig = figure('Name','LGR Lazo Abierto vs LGR + Polos Lazo Cerrado','Position',[50 100 1200 600],'Color','w');

%Sistema en Lazo Abierto

subplot(1, 2, 1);
rlocus(G_pl);
grid on; hold on;

xlim([-25, 5]); ylim([-20, 20]);
xlabel('Eje Real \sigma [s^{-1}]', 'FontSize', 10, 'FontWeight', 'bold'); 
ylabel('Eje Imaginario j\omega [rad/s]', 'FontSize', 10, 'FontWeight', 'bold');
title('Sistema en Lazo Abierto', 'FontSize', 12, 'FontWeight','bold');

%Sistema en Lazo Cerrado

subplot(1, 2, 2);
rlocus(G_pl);
hold on; grid on; box on;

% Superponer los polos del LQR en lazo cerrado
plot(real(e_lqr), imag(e_lqr), 'd', 'Color', '#d90b12', 'MarkerFaceColor', '#870303', 'MarkerSize', 3, 'LineWidth', 1.0, 'DisplayName', 'Polos LQR+I');

xlim([-25, 5]); ylim([-20, 20]);
xlabel('Eje Real \sigma [s^{-1}]', 'FontSize', 10, 'FontWeight', 'bold'); 
ylabel('Eje Imaginario j\omega [rad/s]', 'FontSize', 10, 'FontWeight', 'bold');
title('Sistema en Lazo Cerrado', 'FontSize', 12, 'FontWeight','bold');
legend('Location', 'best', 'FontSize', 8);

outputDir = fullfile(fileparts(mfilename('fullpath')), 'Outputs');
if ~exist(outputDir, 'dir')
    mkdir(outputDir);
end

% Ajustar el diseño para evitar recortes al guardar la figura
set(fig, 'PaperPositionMode', 'auto');
drawnow;

ruta_salida = fullfile(outputDir, 'Sistema_de_Polos.png');
saveas(fig, ruta_salida);
fprintf('\nGrafica generada con exito en: %s\n', ruta_salida);