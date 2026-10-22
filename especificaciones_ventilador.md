# Especificaciones del Ventilador y Etapa de Potencia L298N (24V)

## Enlaces del Grafo
* **Subsistema Padre:** [[arquitectura_sistema]]
* **Montado sobre plataforma física:** [[especificaciones_carro]]
* **Genera torque aerodinámico sobre:** [[especificaciones_compuerta]]
* **Estrategias de Control:** [[estrategias_control]]

## Etiquetas
#actuador #ventilador #l298n #24vdc #pwm #esp32 #etapa-potencia #sanyo-denki #aerodinamica

---

## 1. Rol en la Arquitectura de Control
El ventilador es el **actuador rápido** del lazo de control angular de la compuerta; el carro, a través de la distancia $d$, es el actuador lento (ver [[arquitectura_sistema]], sección 1.1).
* El microcontrolador ESP32 genera una señal PWM (utilizando su periférico hardware LEDC) que modula la velocidad del flujo de aire.
* Dicha señal se amplifica en tensión y corriente a través de un **módulo puente H L298N alimentado con una fuente de 24 VDC**.

---

## 2. Parámetros de Identificación del Ventilador
* **Fabricante:** Sanyo Denki Co., Ltd.
* **Línea:** San Ace 40
* **Modelo:** 109P0424G302 (Doble rodamiento de bolas)
* **Dimensiones Físicas:** 40 mm x 40 mm x 28 mm
* **Peso:** 52 gramos

---

## 3. Parámetros Eléctricos y Desempeño
* **Voltaje Nominal:** 24 VDC
* **Rango de Tensión de Operación:** 12.0 VDC a 26.4 VDC
* **Corriente Nominal:** 0.19 A (Típica) / 0.22 A (Máxima)
* **Consumo de Potencia:** 4.56 Watts
* **Velocidad Nominal Máxima:** 11,500 RPM
* **Flujo de Aire Máximo:** 0.42 m³/min (14.8 CFM $\approx 0.007\text{ m}^3/\text{s}$)
* **Presión Estática Máxima:** 178.8 Pa

---

## 4. Etapa de Potencia: Módulo Puente H L298N a 24 VDC

El accionamiento del ventilador se realiza a través de un canal del módulo L298N (dual full-bridge driver).

```
   Fuente 24VDC
   (+) -------> [ VMS / 12V ]  Módulo L298N
   (-) -------> [ GND       ]  (Jumper 5V RETIRADO)
                      |
   ESP32 (3.3V)       |
   GPIO PWM --> [ ENA       ]  (O control por IN1)
   GPIO Dir --> [ IN1 / IN2 ]
   GND -------> [ GND       ]  (GND común unificado)
                      |
                [ OUT1, OUT2] ========> Ventilador San Ace 40 (24VDC)
```

### 4.1 Configuración de Hardware Crítica del L298N
1. **Retiro del Jumper de 5V (OBLIGATORIO):**
   * El módulo L298N incluye un regulador lineal 78M05 integrado. Cuando la tensión de alimentación del motor ($V_s$) es mayor a $12\text{ V}$ (en este caso **$24\text{ VDC}$**), **se debe retirar el jumper de 5V** para evitar sobrecalentar y quemar el regulador interno.
   * La lógica interna del L298N debe alimentarse conectando $5.0\text{ V}$ externos al borne de $+5\text{V}$ del módulo (provenientes de un convertidor DC-DC buck step-down o fuente auxiliar de 5V).
2. **Margen de Seguridad de Corriente:**
   * El L298N soporta hasta $2.0\text{ A}$ continuos por canal. El ventilador consume únicamente $0.19\text{ A} - 0.22\text{ A}$ ($\approx 10\%$ de la capacidad del driver), por lo que opera con temperatura muy baja y alta confiabilidad.
3. **Diodos de Protección Flyback:**
   * Las placas comerciales L298N integran 8 diodos de conmutación rápida (tipo 1N4007 o Schottky) que protegen los transistores del puente contra las sobretensiones inductivas generadas por el motor del ventilador durante el corte del PWM.

### 4.2 Lógica de Control desde el ESP32
* **Conexión:**
  * Terminal `OUT1` $\to$ Cable positivo (+24V) del ventilador.
  * Terminal `OUT2` $\to$ Cable negativo (GND) del ventilador.
  * `IN1` = Nivel ALTO (o señal PWM del ESP32).
  * `IN2` = Nivel BAJO (GND).
  * `ENA` = Nivel ALTO o entrada de modulación PWM directa.
* **Generación de PWM en ESP32 (Periférico LEDC):**
  * Frecuencia sugerida: $1\text{ kHz}$ a $10\text{ kHz}$ (garantiza conmutación limpia en el L298N sin silbidos molestos).
  * Resolución de duty cycle: 8 a 10 bits ($0 - 255$ o $0 - 1023$).

---

## 5. Dinámica del Actuador Aerodinámico
La respuesta del flujo frente al ciclo de trabajo del PWM se modela como un sistema de primer orden:

$$\tau_{fan} \frac{d v_{aire}(t)}{dt} + v_{aire}(t) = K_v \cdot u_{PWM}(t)$$

* $\tau_{fan} \approx 0.15 - 0.20\text{ s}$ (inercia del rodete y respuesta eléctrica).
* $u_{PWM} \in [0\%, 100\%]$ (gobernado por el controlador PID, LQR o $H_\infty$).
* Fuerza aerodinámica neta a distancia $d$:
  $$F_{aero}(u, d) = \frac{1}{2} \rho_{aire} A_{eff} C_d v_{aire}^2 \approx k_m(d) \cdot u_{PWM}(t)$$
  con $k_m(d) \approx k_0 \left(\frac{d_0}{d}\right)^2$.