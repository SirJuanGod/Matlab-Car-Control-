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
J_g \ddot{\theta}(t) + b_c \dot{\theta}(t) + m_g g \ell_c \sin(\theta(t)) + \tau_{fric}\text{sign}(\dot{\theta}) &= \tau_{aero}(v_a, d, \theta)
\end{aligned}$$

Donde:
* $u_{PWM} \in [10\%, 90\%]$ (señal PWM enviada al L298N y visualizada en la GUI).
* $\tau_{aero} \approx k_m(d) \cdot u_{PWM}(t)$, con $k_m(d) \approx k_0 \left(\frac{d_0}{d}\right)^2$.
* $d(t) = 10\text{ cm} - \Delta x(t)$ es la distancia instantánea estimada mediante la **IMU del carro**.
* $\theta = 0$ es la compuerta **colgando en reposo** del soporte en L invertida (el aire la empuja alejándola del carro); ver [[especificaciones_compuerta]].

### 1.3 Punto de Operación y Equilibrio ($\theta_{eq}$)
$$u_{eq} = \frac{m_g g \ell_c \sin(\theta_{eq})}{k_m(d)}$$

### 1.4 Modelo Linealizado (Pequeña Señal)
$$G_p(s) = \frac{\delta \theta(s)}{\delta u(s)} = \frac{\frac{k_m(d)}{J_g}}{s^2 + \frac{b_c}{J_g}s + \frac{m_g g \ell_c \cos(\theta_{eq})}{J_g}}$$

Incluyendo la dinámica del actuador y el sensor ($K_{pot} = \frac{3.3}{180}\text{ V/deg}$):
$$G_{total}(s) = \frac{V_{pot}(s)}{\delta u(s)} = \left(\frac{1}{\tau_{fan}s + 1}\right) G_p(s) \cdot K_{pot}$$

---

## 2. Estrategia 1: Control PID de la Compuerta con Feedforward y Anti-Windup

Algoritmo implementado en el ESP32:

$$e(t) = \theta_{ref}(t) - \theta_{ADC}(t)$$
$$u_{PID}(t) = K_p e(t) + K_i \int_0^t e(\tau) d\tau + K_d \frac{d e_f(t)}{dt}$$

### 2.1 Componentes de Implementación
1. **Compensación Feedforward Gravitacional ($u_{ff}$):**
   $$u(t) = \text{sat}_{[10, 90]}\left( u_{PID}(t) + \frac{m_g g \ell_c \sin(\theta_{ref})}{k_m(d_0)} \right)$$
   con $d_0 = 10\text{ cm}$.
2. **Filtrado Derivativo en ESP32:**
   El ADC nativo de 12 bits entrega lecturas estables. Se utiliza un derivativo filtrado $D(s) = \frac{K_d s}{s/N + 1}$ ($N \approx 30$).
3. **Anti-Windup (Clamping):**
   Evita la saturación del integrador cuando el PWM alcanza $10\%$ o $90\%$.
4. **Monitoreo en GUI:**
   La señal $u(t)$ resultante se envía continuamente por telemetría para su graficación en la GUI.
5. **Sintonía (`init_parametros.m`, `sintonizar_loop_shaping.m`):** optimización directa sobre el lazo lineal **con el ventilador** (ITAE del escalón, sobrepico $\le 2\%$, $PM \ge 50^\circ$, $N = 30$): $K_p=10.70$, $K_i=128.8$, $K_d=1.65$. Respuesta lineal: subida 0.38 s, sobrepico 2 %, $PM = 66^\circ$, $M_s = 1.5$; estable con el carro entre 5 y 20 cm. La asignación de polos anterior ya no sirve: con la compuerta colgante (planta estable, polos $-2.5 \pm 7.35j$) los polos objetivo lentos daban ganancias negativas.

---

## 3. Estrategia 2: Control Óptimo LQR con Acción Integral ($LQR+I$)

### 3.1 Espacio de Estados Aumentado
Vector de estados: $x(t) = [\delta \theta, \; \delta \dot{\theta}, \; x_i]^T$, donde $\dot{x}_i = \theta(t) - \theta_{ref}$:

$$A_a = \begin{bmatrix} 0 & 1 & 0 \\ -\frac{m_g g \ell_c \cos(\theta_{eq})}{J_g} & -\frac{b_c}{J_g} & 0 \\ 1 & 0 & 0 \end{bmatrix}, \quad B_a = \begin{bmatrix} 0 \\ \frac{k_m(d)}{J_g} \\ 0 \end{bmatrix}, \quad C_a = \begin{bmatrix} 1 & 0 & 0 \end{bmatrix}$$

### 3.2 Optimización Cuadrática
$$J = \int_0^\infty \left( x^T Q x + R \cdot \delta u^2 \right) dt$$
* $Q = \text{diag}([q_\theta, \; q_\omega, \; q_{int}])$, $R > 0$.
* Ganancia: $K_{LQR} = \text{lqr}(A_a, B_a, Q, R)$.
* Ley de control:
  $$u(t) = \text{sat}_{[10, 90]}\left( u_{eq} - K_{LQR} x(t) \right)$$
* Se implementa **Gain Scheduling** interpolando las ganancias según $\theta_{eq}$.

---

### 3.3 Extensión a Dos Salidas del Controlador (SIMO: $u_{PWM}$ y $d_{ref}$)

El controlador recibe solo $\theta_{meas}$ y entrega dos salidas, $u = [\delta u_{PWM}, \; \delta d_{ref}]^T$. Estados: $x = [\delta\theta, \; \delta\omega, \; \delta v_{fan}, \; \delta d, \; x_i]^T$, con el lazo interno del carro aproximado como primer orden ($\tau_{car}$).

Linealizando $\tau_{aero} = k_m \, v_{fan} \, (d_0/d)^2$ alrededor de $(v_{eq}, d_{eq})$:
$$k_v = k_m \left(\frac{d_0}{d_{eq}}\right)^2, \qquad g_d = -\frac{2 \, k_m \, v_{eq} \, d_0^2}{J_g \, d_{eq}^3} \; (<0: \text{alejar el carro reduce el torque})$$

$$A = \begin{bmatrix} 0 & 1 & 0 & 0 & 0 \\ -\frac{m_g g \ell_c \cos\theta_{eq}}{J_g} & -\frac{b_c}{J_g} & \frac{k_v}{J_g} & g_d & 0 \\ 0 & 0 & -\frac{1}{\tau_{fan}} & 0 & 0 \\ 0 & 0 & 0 & -\frac{1}{\tau_{car}} & 0 \\ 1 & 0 & 0 & 0 & 0 \end{bmatrix}, \quad B = \begin{bmatrix} 0 & 0 \\ 0 & 0 \\ \frac{1}{\tau_{fan}} & 0 \\ 0 & \frac{1}{\tau_{car}} \\ 0 & 0 \end{bmatrix}$$

* Ganancia: $K = \text{lqr}(A, B, Q, R) \in \mathbb{R}^{2 \times 5}$, con $R = \text{diag}(r_{PWM}, r_d)$.
* El reparto del esfuerzo entre ventilador y carro lo fijan $R$ y $\tau_{car}$ (propuesta: carro bastante más lento que el ventilador); a verificar y sintonizar en simulación.
* Solo $\theta$ se mide directamente con el potenciómetro: $\omega$ y $v_{fan}$ requieren observador, y $d$ viene de la IMU con Kalman.
* **Implementado en `lqr_simo.m`** (ganancias programadas sobre $\theta_{ref}$, realimentación de estado completo ideal). Parámetros en `init_parametros.m`: $\tau_{car}=0.8\text{ s}$, $Q=\text{diag}(10^4, 60, 0, 0.5, 10^4)$, $R=\text{diag}(0.02, 1)$ ($r_d$ alto: el carro actúa poco mientras el ventilador alcanza). Genera `simulacion_lqr_simo.png`.
* **Implementado en `modelo_miso_compuerta.slx`** (planta MISO en tres subsistemas: cinemática del carro + IMU, aerodinámica del ventilador, compuerta colgante (péndulo, gravedad ∝ sin θ) con tope inelástico; más `Sensor_Potenciometro_ADC`, `Lazo_Carro_IMU` y `Controlador_SIMO`). Estimación sin medir $\omega$ ni $v_{fan}$: $\hat\omega$ por derivada filtrada del ADC ($\tau_w=0.02$ s), $\hat v_{fan}$ por el modelo del ventilador (entrada conocida) y $\hat d$ por Kalman de la IMU con la fuerza de tracción como entrada. La ganancia $K$ se interpola sobre $\theta_{ref}$ y el equilibrio se linealiza en la distancia inicial $d_{ini}$.
* **Resultado** (simulación no lineal, escalón $0^\circ \to 35^\circ$ y pulso de 1 mN·m, SIMO vs solo ventilador, `lqr_simo.m`): asentamiento a $\pm 1^\circ$ en 0.70 s en ambos; desviación por perturbación $1.39^\circ$ en ambos; excursión del carro 0.39 cm. Con la planta estable y el ventilador sobrado, el SIMO **no mejora** el caso nominal: su ventaja aparece cuando el ventilador no alcanza (siguiente punto).
* **El carro se mueve cuando hace falta (`Supervisor_d`):** el punto de operación de distancia $d_{nom}$ no es fijo. Si el PWM demandado supera el 70 %, $d_{nom}$ **avanza** (hasta 5 cm/s, proporcional al exceso); si cae por debajo del mínimo (10 %), el ventilador 'sobra' y $d_{nom}$ se **aleja**; si queda bajo el 20 % regresa a 1 cm/s hacia $d_{ini}$. Las ganancias $K$ se programan sobre $(\theta_{ref}, d_{nom})$ (rejilla $15\times 9$, $d\in[5,25]\text{ cm}$). La orden de PWM se satura en $[10\%, 90\%]$ y la integral del error solo acumula con $|e| < 5^\circ$ y con el PWM sin saturar (anti-windup), lo que evita el sobrepico. Con $\theta_{ref} = 8^\circ$ (por debajo de lo que alcanza el 10 % mínimo a 10 cm, $\approx 11.8^\circ$) el carro se aleja a 13.9 cm y la compuerta llega a $8^\circ$; con el carro arrancando a 18–22 cm el ventilador solo no puede levantar la compuerta: el carro se acerca a unos 14.6 cm y el PWM llega al 90 % mientras sube.
* **Rendimiento en el `.slx`** (escalón $0^\circ \to 35^\circ$, carro a 10 cm, referencia prefiltrada con $\tau_{ref}=0.15\text{ s}$): subida (10–90 %) 0.41 s, sobrepico $0.9^\circ$, asentamiento a $\pm 1^\circ$ en 0.59 s, error final $0.01^\circ$, PWM máximo 65 %, rizado del PWM de $\approx 4\%$ pico a pico por la cuantización del ADC. Pulsos de $\pm 5\text{ mN·m}$ durante 3 s se absorben (error final $0.01^\circ$, desviación máxima hasta $12.7^\circ$). **Reservas:** el modelo de primer orden con $\tau_{car}$ de `lqr_simo.m` es optimista a la escala de la banda muerta de $\pm 5\text{ mm}$; con un ventilador real mucho más débil que el supuesto ($\le 40\%$) la recuperación es lenta (decenas de segundos) porque la integral no actúa con errores mayores de $5^\circ$ y el supervisor solo avanza cuando el PWM pasa del 70 %. Sigue pendiente validar con el carro y la IMU reales.

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
* **Estado:** la síntesis de sensibilidad mixta de orden completo (`mixsyn`/`hinfsyn`) requiere Robust Control Toolbox, que no está instalada. Con la compuerta colgante la planta es estable y el compensador de loop shaping (`sintonizar_loop_shaping.m`, optimización directa con Control System Toolbox: mín. $M_s + 2\,\text{ITAE}$ con $PM \ge 55^\circ$) degenera en un PI filtrado, $C(s)=2.88\,(s+27.5)^2/[s(s+30)]$: $PM = 70^\circ$, $M_s = 1.5$, sin sobrepico, pero asienta en 1.5 s frente a 0.6 s del PID. `systune` sin una meta de desempeño converge a un controlador casi puramente integral (sobrepico 25 %), por eso no se usa. El compensador anterior ($K=293.9$, ceros en 4.2 y 0.2) estaba sintonizado para la planta inestable y ya no aplica.
* **Robustez a la distancia del carro:** la ganancia efectiva del ventilador varía como $(d_0/d)^2$. El PID rápido sigue estable con el carro entre 5 y 20 cm (ganancia $\times 4$ a $\times 0.25$) y el compensador robusto entre unos 6 y 20 cm; con la planta estable el margen de ganancia inferior deja de ser un problema (al alejarse el carro solo se pierde velocidad).

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
1. **Gráfica de la señal PWM:** Muestra la evolución temporal del PWM ($10-90\%$) enviado al actuador en el subplot superior derecho.
2. **Indicador de Estado PWM:** Muestra en texto el valor final y los picos mínimos/máximos de PWM aplicados.
3. **Validación IMU:** Grafica la distancia real frente a la estimada por la IMU a partir de la condición inicial de $10\text{ cm}$.
