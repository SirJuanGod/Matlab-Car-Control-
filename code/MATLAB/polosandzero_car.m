%% 1. Parámetros del Carro y Conversión
m_car     = 0.421;               % [kg]
car_b     = 3.5;                 % [N*s/m]
car_F_max = 3.0;                 % [N]
K_act     = car_F_max / 100;     % [N/%] Factor de conversión a Newtons (0.03)

car_Kd    = 5;                   % Ganancia derivativa fija [%*s/m]
car_Ki    = 10;                  % Ganancia integral fija [%/(m*s)]

Kd_N      = car_Kd * K_act;      % 0.15 N*s/m
Ki_N      = car_Ki * K_act;      % 0.30 N/m*s

%% 2. Función de Transferencia Equivalente para aislar Kp
% La ecuación característica es: m*s^3 + (b + Kd_N)*s^2 + (Kp*K_act)*s + Ki_N = 0
% Dividiendo entre los términos conocidos: 1 + Kp * [ K_act*s / (m*s^3 + (b + Kd_N)*s^2 + Ki_N) ] = 0
s = tf('s');
G_eq = (K_act * s) / (m_car*s^3 + (car_b + Kd_N)*s^2 + Ki_N);

%% 3. Graficar el Lugar Geométrico de las Raíces
figure('Color', 'w');
rlocus(G_eq);
grid on; hold on;

% Rejilla de coeficiente de amortiguamiento (zeta = 0.707 para ~5% de sobreimpulso)
sgrid(0.707, []); 

% 4. Marcar la posición de tus polos con el Kp actual (130)
Kp_actual = 130;
polos_actuales = roots([m_car, (car_b + Kd_N), (Kp_actual * K_act), Ki_N]);
plot(real(polos_actuales), imag(polos_actuales), 'r^', 'MarkerSize', 9, ...
    'LineWidth', 2, 'DisplayName', sprintf('Puntos con K_p = %d', Kp_actual));

title('Lugar Geométrico de las Raíces (Variación de K_p)');
xlabel('Eje Real \sigma (rad/s)');
ylabel('Eje Imaginario j\omega (rad/s)');
legend('Location', 'northeast');