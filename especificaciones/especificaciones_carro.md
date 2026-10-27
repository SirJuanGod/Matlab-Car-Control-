# Especificaciones del Carro (Plataforma Móvil con IMU y Control Independiente)

## Enlaces del Grafo

* **Subsistema Padre:** [[arquitectura_sistema]]
* **Soporte Físico del Ventilador:** [[especificaciones_ventilador]]
* **Interacción física por distancia con:** [[especificaciones_compuerta]]
* **Estrategias de Control del Carro:** [[estrategias_control]]

## 1. Rol en el Proyecto y Condiciones Iniciales

El carro robótico opera como una **plataforma móvil con lazo interno de posición**, cuya referencia $d_{ref}$ es la segunda salida del controlador SIMO (ver [[arquitectura_sistema]], sección 1.1):

* **Condición Inicial de Arranque:** El vehículo arranca **siempre a una distancia nominal inicial de $10\text{ cm}$** ($d_0 = 0.10\text{ m}$) respecto a la compuerta.
* **Medición de Distancia Mediante Unidad IMU:** Incorpora una unidad de medición inercial (**IMU de 6 ejes**, tipo MPU6050) montada sobre el chasis. La aceleración longitudinal $a_x(t)$ se integra y filtra en tiempo real para estimar la **distancia avanzada o retrocedida ($\Delta x(t)$)** respecto al punto de partida:
  $$d(t) = d_0 - \Delta x(t) = 10\text{ cm} - \Delta x(t)$$
* **Generador de Perturbaciones Controladas:** Al avanzar o retroceder de forma autónoma, el carro varía la distancia física $d(t)$, alterando la fuerza aerodinámica incidente ($\tau_{aero} \propto 1/d^2$). Esto sirve como escenario de prueba para validar la robustez de los controladores de la compuerta (especialmente el $H_\infty$).

---

## 2. Estimación de Distancia Avanzada / Retrocedida con la IMU

### 2.1 Principio Cinemático

El acelerómetro mide la aceleración en el eje longitudinal del carro:
$$a_{x,meas}(t) = a_x(t) + b_{imu} + \eta_{imu}(t)$$
donde $b_{imu}$ es el sesgo (bias) constante del sensor y $\eta_{imu}$ es ruido blanco de medición.

La velocidad lineal estimada $\hat{v}(t)$ y la distancia relativa recorrida $\Delta \hat{x}(t)$ se obtienen mediante:
$$\Delta \hat{x}(t) = \int_0^t \hat{v}(\tau) d\tau = \int_0^t \left( \int_0^\tau a_x(s) ds \right) d\tau$$

### 2.2 Observador de Estados / Filtro de Kalman

Para evitar la deriva de doble integración generada por el sesgo del acelerómetro, el sistema implementa un observador de estados lineal (diseñado mediante `lqe` en MATLAB con las matrices del carro):
$$\begin{aligned}
\dot{\hat{v}} &= \frac{F_{trac}(u) - b_{car}\hat{v}}{m_{car}} + L_1 (a_{meas} - \hat{a}) \\
\dot{\hat{b}} &= L_2 (a_{meas} - \hat{a}) \\
\Delta \dot{\hat{x}} &= \hat{v}
\end{aligned}$$
* Distancia estimada hacia la compuerta en todo instante:
  $$\hat{d}(t) = 10\text{ cm} - \Delta \hat{x}(t)$$

---

## 3. Parámetros Físicos y Geométricos del Vehículo
* **Masa Total del Carro ($m_{car}$):** 0.421 kg (incluyendo ventilador, baterías, IMU y chasis).
* **Longitud del Chasis:** 178.03 mm (0.178 m).
* **Ancho del Chasis:** 132.68 mm (0.133 m).
* **Trocha / Ancho de Vía ($W_{track}$):** $\approx 120\text{ mm}$ (separación entre ruedas motrices).
* **Configuración Cinemática:** Tracción diferencial:
  * 2 ruedas motrices traseras ($D = 6.5\text{ cm}$, $r = 3.25\text{ cm}$, ancho 2.7 cm).
  * 1 rueda delantera loca (castor wheel).

---

## 4. Motorreductores DC (Modelo TGP01D-A130)
* **Voltaje de Operación:** 3.0 VDC ~ 6.0 VDC.
* **Corriente de Operación:** 100 mA (sin carga), 200 mA (con carga típica).
* **Caja Reductora:** Relación 1:48.
* **Velocidad de salida en el eje:** 220 RPM (con carga).
* **Velocidad lineal teórica máxima:** $v_{max} = \omega_{eje} \cdot r \approx 0.75\text{ m/s}$.
* **Torque nominal:** 0.5 kg·cm ($\approx 0.049\text{ N}\cdot\text{m}$) por motor.
* **Empuje máximo total de tracción:** $F_{trac} \approx 3.0\text{ N}$.

---

## 5. Electrónica del Sistema del Carro
* **Controlador Principal:** Arduino UNO R3 (o segundo hilo/núcleo del ESP32).
* **Unidad IMU:** MPU6050 (Acelerómetro + Giroscopio I2C, rango $\pm 2g$).
* **Driver de Motores de Tracción:** Módulo puente H L298N dedicado para los 2 motores de tracción (alimentación 7.4V / 9V).
* **Alimentación:** Batería recargable Li-Ion (7.4V, 2 celdas 18650).

---

## 6. Lazo de Control de Posición Longitudinal del Carro

El vehículo regula su posición longitudinal mediante un lazo PID independiente:

$$e_d(t) = d_{ref}(t) - \hat{d}_{IMU}(t)$$
$$u_{car}(t) = \text{sat}_{[-100, 100]}\left( K_{p,d} \cdot e_d(t) + K_{i,d} \int_0^t e_d(\tau) d\tau + K_{d,d} \frac{d e_d(t)}{dt} \right)$$

* **$u_{car} > 0$:** Motores giran en sentido de avance hacia la compuerta (reduce $d$).
* **$u_{car} < 0$:** Motores giran en sentido de retroceso alejándose de la compuerta (aumenta $d$).
* **Banda Muerta:** $\pm 5\text{ mm}$ para amortiguar conmutaciones por fricción estática del suelo.
