# Arquitectura del Sistema de Control Angular y Plataforma Móvil

## Enlaces del Grafo

* **Lazo 1 - Actuador Controlado (Ventilador + L298N 24V):** [[especificaciones_ventilador]]
* **Lazo 1 - Planta Pasiva y Sensor (ESP32 ADC Directo 0-1.65V):** [[especificaciones_compuerta]]
* **Lazo 2 - Plataforma Móvil (IMU Odometría, Inicio 10cm):** [[especificaciones_carro]]
* **Estrategias de Control (LQR+I y Control PID del Carro):** [[estrategias_control]]

## 1. Descripción General de la Arquitectura

El proyecto está concebido como un **controlador SIMO** (entrada: ángulo $\theta$ del potenciómetro; salidas: PWM del ventilador y distancia $d_{ref}$ del carro):

1. **Lazo 1: Control Angular de la Compuerta (Lazo Principal):**
   * **Objetivo:** Regular con alta precisión el ángulo $\theta(t) \in [0^\circ, 90^\circ]$ de la compuerta.
   * **Planta:** Compuerta mecánica pasiva (subactuada, sin motor propio en el pivote).
   * **Actuador rápido del lazo:** Ventilador industrial Sanyo Denki San Ace 40 accionado mediante un **módulo puente H L298N alimentado a 24 VDC**.
   * **Sensor de Salida (Conexión Directa 0 - 3.3V):** Potenciómetro analógico lineal alimentado directamente a **$3.3\text{ VDC}$** desde el ESP32, entregando:
     * $0.00\text{ V} \Longleftrightarrow 0^\circ$
     * $0.825\text{ V} \Longleftrightarrow 45^\circ$
     * $1.65\text{ V} \Longleftrightarrow 90^\circ$
     * **Sin acondicionamiento externo ni divisores de tensión**: Conexión directa al ADC de 12 bits del ESP32 ($0 - 4095$ cuentas).
   * **Controladores disponibles:** anti-windup para Regulador Óptimo LQR + I con acción integral.

2. **Lazo 2: Control Independiente del Carro (Plataforma Móvil con IMU):**
   * **Condición Inicial:** El carro arranca **siempre a una distancia inicial fija de $10\text{ cm}$** ($d_0 = 0.10\text{ m}$) respecto a la compuerta.
   * **Sensor de Distancia:** Unidad de medición inercial (**IMU**) que registra la aceleración longitudinal $a_x(t)$ para calcular la distancia avanzada o retrocedida ($\Delta x(t)$) mediante integración y filtro de Kalman:
     $$d(t) = 10\text{ cm} - \Delta x(t)$$
   * **Naturaleza:** Lazo interno de posición del carro (PID con IMU). Su referencia $d_{ref}$ es la segunda salida del controlador SIMO, por lo que el carro actúa como actuador lento del ángulo de la compuerta.
   * **Interacción física:** Los desplazamientos del carro varían la distancia física $d(t)$ y con ella la ganancia aerodinámica ($\tau_{aero} \propto 1/d^2$): $d$ es a la vez un canal de actuación (comandado por el controlador).

3. **Supervisión y GUI:**
   * La interfaz gráfica (GUI) visualiza en tiempo real la **señal PWM que se está enviando al actuador** (tanto en gráfica temporal como en panel de estado numérico), permitiendo monitorear saturaciones y esfuerzo de control junto con el ángulo de la compuerta y la distancia estimada por la IMU.

---

## 1.1 Estructura SIMO del Controlador

El mismo lazo se describe desde dos lados:

| Vista | Entradas | Salidas |
| :--- | :--- | :--- |
| **Controlador (SIMO)** | $\theta_{meas}$ (potenciómetro / ADC), con error $e = \theta_{ref} - \theta_{meas}$ | $u_{PWM}$ (ventilador) y $d_{ref}$ (distancia del carro) |

* **Por qué SIMO:** hay una sola variable medida y regulada ($\theta$), pero dos actuadores que producen el mismo efecto físico, el torque aerodinámico $\tau_{aero} = k_m \, v_{fan} \, (d_0/d)^2$: el PWM entra de forma lineal y la distancia de forma cuadrática inversa.
* **Redundancia de actuación:** existen infinitas combinaciones $(u_{PWM}, d)$ que dan el mismo torque de equilibrio, por lo que el controlador necesita una regla de reparto del esfuerzo.
* **Regla de reparto:** separación por velocidad.
  * $u_{PWM}$: canal rápido; corrige la dinámica de $\theta$ (limitada por $\tau_{fan} \approx 0.15\text{ s}$).
  * $d_{ref}$: canal lento; el lazo interno del carro (IMU) debe ser bastante más lento que el del ventilador y su función es mantener el PWM lejos de la saturación (límites de 10 % / 90 %).

graph LR
    %% Entradas y Sumador
    Ref((theta_ref)) --> Sum{+}
    Sum -->|e| Ctrl[Controlador SIMO<br>ESP32]

    %% Salidas del Controlador
    Ctrl -->|u_PWM| Fan[Ventilador]
    Ctrl -->|d_ref| Cart[Carro + IMU]
    
    %% Interacción Física (Planta)
    Fan -->|tau_aero| Flap[Compuerta]
    Cart -.->|Efecto de distancia<br>d: 1/d^2| Flap
    
    %% Salida y Retroalimentación
    Flap -->|theta| Out(((theta)))
    Flap --> Sensor[theta_meas<br>Potenciómetro / ADC]
    Sensor -->|-| Sum

## 2. Diagrama de Bloques de los Lazos Internos

graph LR
    %% LAZO 1: CONTROL ANGULAR
    subgraph Lazo1 [Lazo 1: Control Angular de la Compuerta]
        direction LR
        Ref1((theta_ref)) --> Sum1{+}
        Sum1 -->|e_th| Ctrl1[Controlador ESP32]
        Ctrl1 -->|PWM 3.3V| Drv1[Driver L298N<br>Alim. 24VDC]
        Drv1 -->|24V PWM| Fan[Ventilador<br>San Ace 40]
        Fan -->|tau_aero| Plant1[Compuerta Pasiva<br>Masa, Inercia]
        Plant1 -->|theta_t| Out1(((theta)))

        %% Retroalimentación Lazo 1
        Plant1 -->|V_pot 0-3.3V| ADC[ADC ESP32 12-bit<br>Res: ~0.044°/LSB]
        ADC -->|-| Sum1
        
        %% Monitoreo
        Ctrl1 -.->|Señal PWM %| GUI[Monitoreo GUI]
    end

    %% LAZO 2: CONTROL DEL CARRO
    subgraph Lazo2 [Lazo 2: Control Independiente del Carro]
        direction LR
        Ref2((d_ref)) --> Sum2{+}
        Sum2 -->|e_d| Ctrl2[Controlador del Carro]
        Ctrl2 -->|PWM_motores| Drv2[Driver Puente H<br>L298N]
        Drv2 -->|V_motores| Motores[2x Motorreductores<br>TGP01D-A130]
        Motores -->|d_t| Out2(((d_t)))
        
        %% Retroalimentación Lazo 2
        Out2 -->|a_x| IMU[Unidad IMU MPU6050]
        IMU -->|a_x_t| Kalman[Filtro Kalman / Observador<br>d = 10cm - Delta_x]
        Kalman -->|-| Sum2
    end

    %% ACOPLAMIENTO FÍSICO (Perturbación)
    Out2 -.->|Acoplamiento Físico por Distancia<br>Perturbación d_t| Plant1
    
    %% Nota de condición inicial
    classDef note fill:#f9f,stroke:#333,stroke-width:1px;
    NotaD0>Arranque Lazo 2 siempre en [d0 = 10 cm] :::note
    NotaD0 -.-> Out2
---

## 3. Especificaciones Clave de las Interfaces

| Elemento | Subsistema | Parámetro / Tecnología | Valores de Operación |
| :--- | :--- | :--- | :--- |
| **Microcontrolador** | Lazo Compuerta | ESP32 (Xtensa Dual-Core 240 MHz) | Lógica 3.3V, periférico LEDC PWM (1 a 10 kHz). |
| **Convertidor ADC** | Sensor Compuerta | SAR ADC1 del ESP32 (GPIO 34/35/36) | **Resolución de 12 bits** ($4096$ cuentas), rango nativo $0 - 3.3\text{ V}$. |
| **Potenciómetro** | Sensor Angular | Alimentado a $3.3\text{ VDC}$ del ESP32 | $0\text{V}=0^\circ$, $0.825\text{V}=45^\circ$, $1.65\text{V}=90^\circ$. **Conexión directa**. |
| **Driver Ventilador** | Actuador Aerodinámico | Módulo Puente H **L298N** | **Alimentación: 24 VDC**, jumper de 5V retirado, conmutación por PWM a 3.3V. |
| **Ventilador** | Lazo Compuerta | Sanyo Denki San Ace 40 | 24VDC, 11500 RPM, 0.42 m³/min, consumo 4.56 W. |
| **Posición Inicial Carro** | Plataforma Móvil | Distancia nominal $d_0$ | **$10\text{ cm}$ fijos al arrancar** ($0.10\text{ m}$). |
| **Odometría Carro** | Sensor de Locomoción | Unidad **IMU** (MPU6050) | Medición de $a_x(t)$ y estimación de $\Delta x(t)$ para calcular $d(t) = 10\text{ cm} - \Delta x$. |
| **Driver Motores Carro** | Locomoción Carro | Segundo módulo L298N | Alimentación 7.4V Li-Ion para los 2 motores TGP01D-A130. |
| **Visualización GUI** | Interfaz MATLAB | Panel de Gráficas y Métricas | **Gráfica continua de la señal PWM enviada**, indicador numérico de duty cycle e IMU. |
