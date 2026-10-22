# Estrategias de Control: Compuerta (PID, LQR, H-infinita) y Carro Independiente

## Enlaces del Grafo
* **Subsistema Padre:** [[arquitectura_sistema]]
* **Actuador Manipulado Compuerta (L298N 24V):** [[especificaciones_ventilador]]
* **Planta Pasiva y Sensor (ESP32 ADC Directo 0-3.3V):** [[especificaciones_compuerta]]
* **Plataforma Móvil (IMU Odometría, Inicio 10cm):** [[especificaciones_carro]]

## Etiquetas
#control #simo #pid #lqr #h-infinita #control-carro #esp32 #adc-12bits #0-3.3v #l298n #24vdc #imu-odometria #inicio-10cm #gui-pwm #control-robusto

---

## 1. Modelo Matemático del Lazo Angular de la Compuerta (canal PWM → θ)

El lazo de inclinación angular de la compuerta se ejecuta en el **ESP32**, muestreando la señal del potenciómetro conectada directamente a su **ADC de 12 bits** en el rango nativo de **$0.0\text{ V} - 3.3\text{ V}$** (resolución $\Delta \theta \approx 0.044^\circ/\text{LSB}$) y modulando el ventilador San Ace 40 mediante el **driver L298N a 24 VDC**.

### 1.1 Función de Conversión Directa del Potenciómetro
Sin circuitos de atenuación intermedios:
$$V_{pot}(\theta) = \left(\frac{3.3\text{ V}}{180^\circ}\right) \theta \approx 0.01833 \cdot \theta \quad [\text{V}]$$
$$\theta[^\circ] = \left(\frac{\text{Lectura\_ADC}}{4095}\right) \times 180.0^\circ$$

### 1.2 Ecuación Dinámica No Lineal
$$\begin{aligned}
\tau_{fan} \dot{v}_a(t) + v_a(t) &= K_v \cdot u_{PWM}(t) \\
J_g \ddot{\theta}(t) + b_c \dot{\theta}(t) + m_g g \ell_c \cos(\theta(t)) + \tau_{fric}\text{sign}(\dot{\theta}) &= \tau_{aero}(v_a, d, \theta)
\end{aligned}$$

Donde:
* $u_{PWM} \in [0\%, 100\%]$ (señal PWM enviada al L298N y visualizada en la GUI).
* $\tau_{aero} \approx k_m(d) \cdot u_{PWM}(t)$, con $k_m(d) \approx k_0 \left(\frac{d_0}{d}\right)^2$.
* $d(t) = 10\text{ cm} - \Delta x(t)$ es la distancia instantánea estimada mediante la **IMU del carro**.

### 1.3 Punto de Operación y Equilibrio ($\theta_{eq}$)
$$u_{eq} = \frac{m_g g \ell_c \cos(\theta_{eq})}{k_m(d)}$$

### 1.4 Modelo Linealizado (Pequeña Señal)
$$G_p(s) = \frac{\delta \theta(s)}{\delta u(s)} = \frac{\frac{k_m(d)}{J_g}}{s^2 + \frac{b_c}{J_g}s - \frac{m_g g \ell_c \sin(\theta_{eq})}{J_g}}$$

Incluyendo la dinámica del actuador y el sensor ($K_{pot} = \frac{3.3}{180}\text{ V/deg}$):
$$G_{total}(s) = \frac{V_{pot}(s)}{\delta u(s)} = \left(\frac{1}{\tau_{fan}s + 1}\right) G_p(s) \cdot K_{pot}$$

---

## 2. Estrategia 1: Control PID de la Compuerta con Feedforward y Anti-Windup

Algoritmo implementado en el ESP32:

$$e(t) = \theta_{ref}(t) - \theta_{ADC}(t)$$
$$u_{PID}(t) = K_p e(t) + K_i \int_0^t e(\tau) d\tau + K_d \frac{d e_f(t)}{dt}$$

### 2.1 Componentes de Implementación
1. **Compensación Feedforward Gravitacional ($u_{ff}$):**
   $$u(t) = \text{sat}_{[0, 100]}\left( u_{PID}(t) + \frac{m_g g \ell_c \cos(\theta_{ref})}{k_m(d_0)} \right)$$
   con $d_0 = 10\text{ cm}$.
2. **Filtrado Derivativo en ESP32:**
   El ADC nativo de 12 bits entrega lecturas estables. Se utiliza un derivativo filtrado $D(s) = \frac{K_d s}{s/N + 1}$ ($N \approx 30$).
3. **Anti-Windup (Clamping):**
   Evita la saturación del integrador cuando el PWM alcanza $0\%$ o $100\%$.
4. **Monitoreo en GUI:**
   La señal $u(t)$ resultante se envía continuamente por telemetría para su graficación en la GUI.
5. **Sintonía (`init_parametros.m`):** asignación de polos de 4.º orden que **incluye el polo del ventilador** ($\zeta=0.85$, $\omega_n=3.2$ rad/s, $p_{int}=1$, $p_4=-5.23$): $K_p=37.35$, $K_i=5.35$, $K_d=5.82$. Diseñarlo sin el ventilador da $K_d=0.96$ y un lazo inestable.

---

## 3. Estrategia 2: Control Óptimo LQR con Acción Integral ($LQR+I$)

### 3.1 Espacio de Estados Aumentado
Vector de estados: $x(t) = [\delta \theta, \; \delta \dot{\theta}, \; x_i]^T$, donde $\dot{x}_i = \theta(t) - \theta_{ref}$:

$$A_a = \begin{bmatrix} 0 & 1 & 0 \\ \frac{m_g g \ell_c \sin(\theta_{eq})}{J_g} & -\frac{b_c}{J_g} & 0 \\ 1 & 0 & 0 \end{bmatrix}, \quad B_a = \begin{bmatrix} 0 \\ \frac{k_m(d)}{J_g} \\ 0 \end{bmatrix}, \quad C_a = \begin{bmatrix} 1 & 0 & 0 \end{bmatrix}$$

### 3.2 Optimización Cuadrática
$$J = \int_0^\infty \left( x^T Q x + R \cdot \delta u^2 \right) dt$$
* $Q = \text{diag}([q_\theta, \; q_\omega, \; q_{int}])$, $R > 0$.
* Ganancia: $K_{LQR} = \text{lqr}(A_a, B_a, Q, R)$.
* Ley de control:
  $$u(t) = \text{sat}_{[0, 100]}\left( u_{eq} - K_{LQR} x(t) \right)$$
* Se implementa **Gain Scheduling** interpolando las ganancias según $\theta_{eq}$.

---

### 3.3 Extensión a Dos Salidas del Controlador (SIMO: $u_{PWM}$ y $d_{ref}$)

El controlador recibe solo $\theta_{meas}$ y entrega dos salidas, $u = [\delta u_{PWM}, \; \delta d_{ref}]^T$. Estados: $x = [\delta\theta, \; \delta\omega, \; \delta v_{fan}, \; \delta d, \; x_i]^T$, con el lazo interno del carro aproximado como primer orden ($\tau_{car}$).

Linealizando $\tau_{aero} = k_m \, v_{fan} \, (d_0/d)^2$ alrededor de $(v_{eq}, d_{eq})$:
$$k_v = k_m \left(\frac{d_0}{d_{eq}}\right)^2, \qquad g_d = -\frac{2 \, k_m \, v_{eq} \, d_0^2}{J_g \, d_{eq}^3} \; (<0: \text{alejar el carro reduce el torque})$$

$$A = \begin{bmatrix} 0 & 1 & 0 & 0 & 0 \\ \frac{m_g g \ell_c \sin\theta_{eq}}{J_g} & -\frac{b_c}{J_g} & \frac{k_v}{J_g} & g_d & 0 \\ 0 & 0 & -\frac{1}{\tau_{fan}} & 0 & 0 \\ 0 & 0 & 0 & -\frac{1}{\tau_{car}} & 0 \\ 1 & 0 & 0 & 0 & 0 \end{bmatrix}, \quad B = \begin{bmatrix} 0 & 0 \\ 0 & 0 \\ \frac{1}{\tau_{fan}} & 0 \\ 0 & \frac{1}{\tau_{car}} \\ 0 & 0 \end{bmatrix}$$

* Ganancia: $K = \text{lqr}(A, B, Q, R) \in \mathbb{R}^{2 \times 5}$, con $R = \text{diag}(r_{PWM}, r_d)$.
* El reparto del esfuerzo entre ventilador y carro lo fijan $R$ y $\tau_{car}$ (propuesta: carro bastante más lento que el ventilador); a verificar y sintonizar en simulación.
* Solo $\theta$ se mide directamente con el potenciómetro: $\omega$ y $v_{fan}$ requieren observador, y $d$ viene de la IMU con Kalman.
* **Implementado en `lqr_simo.m`** (ganancias programadas sobre $\theta_{ref}$, equilibrio con el carro en $d_0$, realimentación de estado completo ideal). Parámetros en `init_parametros.m`: $\tau_{car}=0.8\text{ s}$, $Q=\text{diag}(60, 3, 0, 0.5, 120)$, $R=\text{diag}(0.05, 0.05)$. Genera `simulacion_lqr_simo.png`.
* **Implementado en `modelo_miso_compuerta.slx`** (planta MISO en tres subsistemas: cinemática del carro + IMU, aerodinámica del ventilador, compuerta como péndulo invertido con tope inelástico; más `Sensor_Potenciometro_ADC`, `Lazo_Carro_IMU` y `Controlador_SIMO`). Estimación sin medir $\omega$ ni $v_{fan}$: $\hat\omega$ por derivada filtrada del ADC ($\tau_w=0.02$ s), $\hat v_{fan}$ por el modelo del ventilador (entrada conocida) y $\hat d$ por Kalman de la IMU con la fuerza de tracción como entrada. La ganancia $K$ se interpola sobre $\theta_{ref}$ y el equilibrio se linealiza en la distancia inicial $d_{ini}$.
* **Resultado preliminar** (simulación no lineal, escalón $30^\circ \to 40^\circ$ y pulso de 1 mN·m, SIMO vs solo ventilador): asentamiento a $\pm 1^\circ$ en 1.0 s vs 3.1 s; desviación por perturbación $6.25^\circ$ vs $6.91^\circ$; excursión real del carro 0.84 cm. Con pulso de 2 mN·m el SIMO empeora ($31^\circ$ vs $16^\circ$) y con $r_d$ alto ($\geq 3$) el carro casi no actúa.
* **El carro se mueve cuando hace falta (`Supervisor_d`):** el punto de operación de distancia $d_{nom}$ deja de ser fijo. Si el PWM demandado supera el 80 %, $d_{nom}$ avanza (hasta 5 cm/s, proporcional al exceso); si baja del 30 %, regresa a 1 cm/s hacia $d_{ini}$. Las ganancias $K$ se programan sobre $(\theta_{ref}, d_{nom})$ (rejilla $15\times 9$, $d\in[5,25]\text{ cm}$). Se añadieron un gobernador de referencia ($\theta_{ref}\le\theta_{med}+5^\circ$, para levantar la compuerta desde el suelo sin voltearla) y anti-windup condicional de la integral (límite $\pm 2$ rad·s). Resultados en el `.slx` ($35^\circ\to30^\circ$): con el carro arrancando a 18 cm (exige 130 % de PWM) **sin supervisor la compuerta queda en el suelo y con él se recupera a $30.3^\circ$**; una pérdida de hasta 50 % de fuerza del ventilador se compensa con la integral ($30.3^\circ$) y un pulso de $-8\text{ mN·m}$ se absorbe ($30.6^\circ$). **Límite:** con pérdidas $\ge 60\%$ (exigen $\ge 100\%$ de PWM a 10 cm) el carro se acerca a 6–7 cm pero el lazo oscila lentamente ($\theta$ entre $0^\circ$ y $38^\circ$); requiere calibrar $k_m$ o estimarlo en línea. Un gobernador más permisivo ($\ge 10^\circ$) recupera más rápido, pero un pulso de $-8\text{ mN·m}$ puede voltear la compuerta.
* **Reservas:** el modelo de primer orden con $\tau_{car}$ de `lqr_simo.m` es optimista a la escala de la banda muerta de $\pm 5\text{ mm}$. En el `.slx` (escenario $35^\circ \to 30^\circ$, $d=10\text{ cm}$) con $d$ ideal el error final es $0.014^\circ$ y asienta en 1.0 s; con IMU + Kalman + banda muerta queda un ciclo límite lento de $\approx \pm 0.5^\circ$ (asentamiento 1.4 s). Sigue pendiente validar con el carro y la IMU reales.

---

## 4. Estrategia 3: Control Robusto $H_\infty$ ($H$-infinita)

### 4.1 Incertidumbre Paramétrica por Movimiento del Carro
El carro arranca a $d_0 = 10\text{ cm}$, pero al desplazarse de forma independiente varía la distancia $d \in [5\text{ cm}, 25\text{ cm}]$, generando una variación en la ganancia de empuje de hasta $\pm 75\%$.

### 4.2 Sensibilidad Mixta ($S/KS/T$)
Se modela la familia de plantas con incertidumbre multiplicativa:
$$G(s) = G_0(s) [1 + W_m(s) \Delta(s)], \quad \|\Delta\|_\infty \le 1$$

El controlador $K_{H_\infty}(s)$ se obtiene minimizando:
$$\left\| \begin{bmatrix} W_p(s) S(s) \\ W_u(s) K(s) S(s) \\ W_m(s) T(s) \end{bmatrix} \right\|_\infty < 1$$

* $W_p(s)$: Desempeño y error nulo en régimen permanente.
* $W_u(s)$: Limita la energía del PWM para no saturar el L298N.
* $W_m(s)$: Robustez garantizada ante los cambios de distancia calculados por la IMU.
* **Estado:** la síntesis de sensibilidad mixta de orden completo (`mixsyn`/`hinfsyn`) requiere Robust Control Toolbox, que no está instalada. En su lugar, el compensador $C(s)=293.9\,(s+4.21)(s+0.20)/[s(s+30)]$ se sintoniza con `systune` (Control System Toolbox; script `sintonizar_loop_shaping.m`): estructura fija con integrador, planta **con** el ventilador, margen de disco objetivo $3\text{ dB}/25^\circ$, polos con $\mathrm{Re}<-1$ y $N\le 30$. Resultado: $GM\in[0.68,\,2.87]$, $PM=21^\circ$, $M_s=2.9$ (el objetivo de margen no se cumple del todo; es el óptimo de esta estructura). El compensador anterior ($K=38$, ceros en $-2.5$ y $-0.8$) era inestable.
* **Límite del control de ganancia fija:** como la ganancia efectiva del ventilador varía como $(d_0/d)^2$, ningún compensador de esta estructura estabiliza todo el rango $d\in[5,25]\text{ cm}$ ni el $\pm 75\%$ de 4.1. Este tolera $d\approx 5.9$ a $12.1\text{ cm}$ (el PID de 2.1, 5.7 a 11.2 cm). Cubrir más rango exige que el control conozca $d$, que es lo que hace el SIMO.

---

## 5. Control del Carro y Odometría por IMU (Arranque a 10 cm)

El carro cuenta con un lazo interno que regula su posición; su referencia $d_{ref}$ es la segunda salida del controlador SIMO (ver [[arquitectura_sistema]], sección 1.1):

### 5.1 Estimación de Distancia por IMU
Partiendo de $d_0 = 10\text{ cm}$:
1. La IMU mide la aceleración longitudinal $a_x(t)$.
2. Un observador de Kalman estima la velocidad longitudinal $\hat{v}(t)$ compensando el sesgo del sensor $b_{imu}$.
3. La distancia instantánea hacia la compuerta es:
   $$\hat{d}(t) = 10\text{ cm} - \int_0^t \hat{v}(\tau) d\tau = 10\text{ cm} - \Delta \hat{x}(t)$$

### 5.2 Control PID de Distancia del Carro
Convención: $\Delta x>0$ es avanzar (reduce $d$) y $e_a = \hat{d} - d_{ref}$, de modo que $e_a>0$ pide avanzar. Con banda muerta de $\pm 5\text{ mm}$ sobre $e_a$ y derivada sobre la velocidad estimada $\hat v$ (la entrega el Kalman, sin derivar ruido):
$$u_{car}(t) = \text{sat}_{[-100, 100]}\left( K_{p,d}\, e_a(t) + K_{i,d} \int_0^t e_a(\tau) d\tau - K_{d,d}\, \hat{v}(t) \right), \qquad F_{trac} = \frac{u_{car}}{100}\, F_{max}$$
* Si $\hat{d} > d_{ref}$: el carro avanza hacia la compuerta ($\Delta x > 0$).
* Si $\hat{d} < d_{ref}$: el carro retrocede alejándose ($\Delta x < 0$).
* Valores en `init_parametros.m`: $K_{p,d}=130\ \%/\text{m}$ (≈3.9 N/m, $\tau$ del lazo ≈0.7 s), $K_{i,d}=10$, $K_{d,d}=5$; integral limitada a ±30 %.

---

## 6. Monitoreo en la Interfaz Gráfica (GUI)
La GUI en MATLAB (`gui_simulacion.m`):
1. **Gráfica de la señal PWM:** Muestra la evolución temporal del PWM ($0-100\%$) enviado al actuador en el subplot superior derecho.
2. **Indicador de Estado PWM:** Muestra en texto el valor final y los picos mínimos/máximos de PWM aplicados.
3. **Validación IMU:** Grafica la distancia real frente a la estimada por la IMU a partir de la condición inicial de $10\text{ cm}$.
