# Arquitectura del Sistema de Control Angular y Plataforma Móvil

## Enlaces del Grafo
* **Lazo 1 - Actuador Controlado (Ventilador + L298N 24V):** [[especificaciones_ventilador]]
* **Lazo 1 - Planta Pasiva y Sensor (ESP32 ADC Directo 0-3.3V):** [[especificaciones_compuerta]]
* **Lazo 2 - Plataforma Móvil (IMU Odometría, Inicio 10cm):** [[especificaciones_carro]]
* **Estrategias de Control (PID, LQR, H-infinita y Control del Carro):** [[estrategias_control]]

## Etiquetas
#sistema #arquitectura #control-simo #esp32 #adc-12bits #0-3.3v #l298n #24vdc #imu-odometria #inicio-10cm #gui-pwm #control-automatico

---

## 1. Descripción General de la Arquitectura

El proyecto está concebido como un **controlador SIMO** (entrada: ángulo $\theta$ del potenciómetro; salidas: PWM del ventilador y distancia $d_{ref}$ del carro) sobre una **planta MISO** (ver 1.1). Internamente se organiza en dos lazos:

1. **Lazo 1: Control Angular de la Compuerta (Lazo Principal):**
   * **Objetivo:** Regular con alta precisión el ángulo $\theta(t) \in [0^\circ, 180^\circ]$ de la compuerta.
   * **Planta:** Compuerta mecánica pasiva (subactuada, sin motor propio en el pivote).
   * **Actuador rápido del lazo:** Ventilador industrial Sanyo Denki San Ace 40 accionado mediante un **módulo puente H L298N alimentado a 24 VDC**.
   * **Sensor de Salida (Conexión Directa 0 - 3.3V):** Potenciómetro analógico lineal alimentado directamente a **$3.3\text{ VDC}$** desde el ESP32, entregando:
     * $0.00\text{ V} \Longleftrightarrow 0^\circ$
     * $1.65\text{ V} \Longleftrightarrow 90^\circ$
     * $3.30\text{ V} \Longleftrightarrow 180^\circ$
     * **Sin acondicionamiento externo ni divisores de tensión**: Conexión directa al ADC de 12 bits del ESP32 ($0 - 4095$ cuentas, $\Delta \theta \approx 0.044^\circ/\text{LSB}$).
   * **Controladores disponibles:** PID con feedforward y anti-windup, Regulador Óptimo LQR con acción integral, y Control Robusto $H_\infty$.

2. **Lazo 2: Control Independiente del Carro (Plataforma Móvil con IMU):**
   * **Condición Inicial:** El carro arranca **siempre a una distancia inicial fija de $10\text{ cm}$** ($d_0 = 0.10\text{ m}$) respecto a la compuerta.
   * **Sensor de Distancia:** Unidad de medición inercial (**IMU**) que registra la aceleración longitudinal $a_x(t)$ para calcular la distancia avanzada o retrocedida ($\Delta x(t)$) mediante integración y filtro de Kalman:
     $$d(t) = 10\text{ cm} - \Delta x(t)$$
   * **Naturaleza:** Lazo interno de posición del carro (PID con IMU). Su referencia $d_{ref}$ es la segunda salida del controlador SIMO, por lo que el carro actúa como actuador lento del ángulo de la compuerta.
   * **Interacción física:** Los desplazamientos del carro varían la distancia física $d(t)$ y con ella la ganancia aerodinámica ($\tau_{aero} \propto 1/d^2$): $d$ es a la vez un canal de actuación (comandado por el controlador) y una fuente de incertidumbre para validar la robustez ($H_\infty$).

3. **Supervisión y GUI:**
   * La interfaz gráfica (GUI) visualiza en tiempo real la **señal PWM que se está enviando al actuador** (tanto en gráfica temporal como en panel de estado numérico), permitiendo monitorear saturaciones y esfuerzo de control junto con el ángulo de la compuerta y la distancia estimada por la IMU.

---

## 1.1 Estructura SIMO del Controlador (Planta MISO)

El mismo lazo se describe desde dos lados:

| Vista | Entradas | Salidas |
| :--- | :--- | :--- |
| **Controlador (SIMO)** | $\theta_{meas}$ (potenciómetro / ADC), con error $e = \theta_{ref} - \theta_{meas}$ | $u_{PWM}$ (ventilador) y $d_{ref}$ (distancia del carro) |
| **Planta (MISO)** | $u_{PWM}$ y $d$ | $\theta$ (modelo `modelo_miso_compuerta`) |

* **Por qué SIMO:** hay una sola variable medida y regulada ($\theta$), pero dos actuadores que producen el mismo efecto físico, el torque aerodinámico $\tau_{aero} = k_m \, v_{fan} \, (d_0/d)^2$: el PWM entra de forma lineal y la distancia de forma cuadrática inversa.
* **Redundancia de actuación:** existen infinitas combinaciones $(u_{PWM}, d)$ que dan el mismo torque de equilibrio, por lo que el controlador necesita una regla de reparto del esfuerzo.
* **Regla de reparto (propuesta por defecto, pendiente de confirmar):** separación por velocidad.
  * $u_{PWM}$: canal rápido; corrige la dinámica de $\theta$ (limitada por $\tau_{fan} \approx 0.15\text{ s}$).
  * $d_{ref}$: canal lento; el lazo interno del carro (IMU) debe ser bastante más lento que el del ventilador y su función es mantener el PWM lejos de la saturación (0 % / 100 %).

```
                          +-------------------+--- u_PWM ---> [ Ventilador ] --+
 theta_ref -->(+)-- e --> | Controlador SIMO  |                                 +--> tau_aero --> [ Compuerta ] --> theta
              ^ -         | (ESP32)           |--- d_ref ---> [ Carro + IMU ] --+   (d: 1/d^2)              |
              |           +-------------------+                                                            |
              +------------------------------ theta_meas (potenciometro / ADC) <--------------------------+
```

---

## 2. Diagrama de Bloques de los Lazos Internos

```
=====================================================================================================
LAZO 1: CONTROL ANGULAR DE LA COMPUERTA (CANAL PWM -> THETA)
=====================================================================================================

                       +-------------------------------------------------+
                       |     Acoplamiento Físico por Distancia           |
                       |       Distancia del Carro: d(t) [Perturbación]  |
                       +-------------------------------------------------+
                                                |
                                                v
[ theta_ref ]      e_th     +------------+ PWM (3.3V) +---------------+ 24V PWM  +---------------+ tau_aero  +------------------+  theta(t)
   (deg)    --->(+)-------> | Controlador|----------->| Driver L298N  |---------->|  Ventilador   |---------->| Compuerta Pasiva |----+--->
                 ^ -        | (ESP32)    |   |        | (Alim. 24VDC) |           |  San Ace 40   |           | (Masa, Inercia)  |    |
                 |          +------------+   |        +---------------+           +---------------+           +------------------+    |
                 |                           v                                                                                        |
                 |                  [ MONITOREO GUI: ]                                                                                |
                 |                  [ SEÑAL PWM (%)  ]                                                                                |
                 |                                                                                                                    |
                 |                         +-----------------------------+                     V_pot (0 - 3.3V)                       |
                 +-------------------------| ADC ESP32 (12 bits: 0-4095) |<-----------------------------------------------------------+
                                           | Res: ~0.044°/LSB            |          (CONEXIÓN DIRECTA SIN DIVISOR)
                                           +-----------------------------+

=====================================================================================================
LAZO 2: CONTROL INDEPENDIENTE DEL CARRO (PLATAFORMA MÓVIL CON IMU)
=====================================================================================================

                       (Arranque siempre en d0 = 10 cm)
[ d_ref ]          e_d      +------------+  PWM_motores +---------------+  V_motores  +------------------+    d(t)
 (cm)       --->(+)-------> | Controlador|------------> | Driver Puente |------------>| 2x Motorreductores|---> (Distancia hacia
                 ^ -        | del Carro  |              | H (L298N)     |             | TGP01D-A130       |      la compuerta)
                 |          +------------+              +---------------+             +------------------+          |
                 |                                                                                                  |
                 |                         +-----------------------------+     a_x(t) [Aceleración]                 |
                 +-------------------------| Filtro Kalman / Observador  |<-----------------------------------------+
                                           | d(t) = 10 cm - Delta_x(t)   |     (Unidad IMU MPU6050)
                                           +-----------------------------+
```

---

## 3. Especificaciones Clave de las Interfaces

| Elemento | Subsistema | Parámetro / Tecnología | Valores de Operación |
| :--- | :--- | :--- | :--- |
| **Microcontrolador** | Lazo Compuerta | ESP32 (Xtensa Dual-Core 240 MHz) | Lógica 3.3V, periférico LEDC PWM (1 a 10 kHz). |
| **Convertidor ADC** | Sensor Compuerta | SAR ADC1 del ESP32 (GPIO 34/35/36) | **Resolución de 12 bits** ($4096$ cuentas), rango nativo $0 - 3.3\text{ V}$. |
| **Potenciómetro** | Sensor Angular | Alimentado a $3.3\text{ VDC}$ del ESP32 | $0\text{V}=0^\circ$, $1.65\text{V}=90^\circ$, $3.3\text{V}=180^\circ$. **Conexión directa**. |
| **Driver Ventilador** | Actuador Aerodinámico | Módulo Puente H **L298N** | **Alimentación: 24 VDC**, jumper de 5V retirado, conmutación por PWM a 3.3V. |
| **Ventilador** | Lazo Compuerta | Sanyo Denki San Ace 40 | 24VDC, 11500 RPM, 0.42 m³/min, consumo 4.56 W. |
| **Posición Inicial Carro** | Plataforma Móvil | Distancia nominal $d_0$ | **$10\text{ cm}$ fijos al arrancar** ($0.10\text{ m}$). |
| **Odometría Carro** | Sensor de Locomoción | Unidad **IMU** (MPU6050) | Medición de $a_x(t)$ y estimación de $\Delta x(t)$ para calcular $d(t) = 10\text{ cm} - \Delta x$. |
| **Driver Motores Carro**| Locomoción Carro | Segundo módulo L298N | Alimentación 7.4V Li-Ion para los 2 motores TGP01D-A130. |
| **Visualización GUI** | Interfaz MATLAB | Panel de Gráficas y Métricas | **Gráfica continua de la señal PWM enviada**, indicador numérico de duty cycle e IMU. |