# Graph Report - review-compile-c-code-45a0eb  (2026-10-26)

## Corpus Check
- 40 files · ~364,490 words
- Verdict: corpus is large enough that graph structure adds value.
- Unclassified: 11 file(s) not represented in the graph (top: (none) 4, .slx 3, .defaults 2)

## Summary
- 213 nodes · 318 edges · 14 communities (12 shown, 2 thin omitted)
- Extraction: 92% EXTRACTED · 8% INFERRED · 0% AMBIGUOUS · INFERRED: 25 edges (avg confidence: 0.85)
- Token cost: 0 input · 0 output

## Graph Freshness
- Built from commit: `c3ad301f`
- Run `git rev-parse HEAD` and compare to check if the graph is stale.
- Run `graphify update .` after code changes (no API cost).

## Community Hubs (Navigation)
- mpu6050.c
- 5. Estrategia de control LQR
- CLAUDE.md
- espnow_base.c
- Especificaciones del Ventilador y Etapa de Potencia L298N (24V)
- Estrategias de Control: Compuerta (PID, LQR, H-infinita) y Carro Independiente
- adc_reader.c
- Firmware ESP32 en C / ESP-IDF con FreeRTOS (Control Carro & Compuerta)
- Especificaciones del Carro (Plataforma Móvil con IMU y Control Independiente)
- Especificaciones de la Compuerta (Planta Pasiva y Sensor Directo 0-3.3V con ESP32)
- espnow_car_send_telemetry
- 2. Ecuaciones primitivas
- 4. Cómo se llegó al LQR como mejor opción

## God Nodes (most connected - your core abstractions)
1. `5. Estrategia de control LQR` - 11 edges
2. `Documentación técnica: modelado, funciones de transferencia y control LQR` - 10 edges
3. `app_main()` - 8 edges
4. `Especificaciones del Carro (Plataforma Móvil con IMU y Control Independiente)` - 8 edges
5. `car_control_task()` - 7 edges
6. `2. Ecuaciones primitivas` - 7 edges
7. `Especificaciones del Ventilador y Etapa de Potencia L298N (24V)` - 7 edges
8. `3. Linealización y funciones de transferencia` - 6 edges
9. `4. Cómo se llegó al LQR como mejor opción` - 6 edges
10. `Estrategias de Control: Compuerta (PID, LQR, H-infinita) y Carro Independiente` - 6 edges

## Surprising Connections (you probably didn't know these)
- `app_main()` --calls--> `espnow_base_init()`  [INFERRED]
  code/C/esp32_base_potenciometro/main/main.c → code/C/esp32_base_potenciometro/main/espnow_base.c
- `app_main()` --calls--> `espnow_car_init()`  [INFERRED]
  code/C/esp32_carro_control/main/main.c → code/C/esp32_carro_control/main/espnow_car.c
- `car_telemetry_task()` --calls--> `espnow_car_send_telemetry()`  [INFERRED]
  code/C/esp32_carro_control/main/main.c → code/C/esp32_carro_control/main/espnow_car.c
- `car_control_task()` --calls--> `mpu6050_is_ready()`  [INFERRED]
  code/C/esp32_carro_control/main/main.c → code/C/esp32_carro_control/main/mpu6050.c
- `app_main()` --calls--> `adc_reader_init()`  [INFERRED]
  code/C/esp32_base_potenciometro/main/main.c → code/C/esp32_base_potenciometro/main/adc_reader.c

## Import Cycles
- None detected.

## Communities (14 total, 2 thin omitted)

### Community 0 - "mpu6050.c"
Cohesion: 0.11
Nodes (19): car_controller_init(), car_controller_reset_integrators(), car_controller_step(), kalman_observer_get_state(), kalman_observer_init(), kalman_observer_reset(), kalman_observer_update(), app_main() (+11 more)

### Community 1 - "5. Estrategia de control LQR"
Cohesion: 0.07
Nodes (29): 1.1 Montaje físico, 1.2 Convenciones, 1.3 Parámetros del modelo, 1. Descripción del sistema y convenciones, 3.1 Punto de equilibrio, 3.2 Linealización, 3.3 Modelo en espacio de estados (5 estados, 2 entradas), 3.4 Funciones de transferencia de cada subsistema (+21 more)

### Community 3 - "espnow_base.c"
Cohesion: 0.10
Nodes (9): adc_reader_get_last_angle(), espnow_base_init(), espnow_base_send(), espnow_recv_cb(), espnow_send_cb(), base_hub_task(), protocol_checksum(), uart_matlab_send_telemetry() (+1 more)

### Community 5 - "Especificaciones del Ventilador y Etapa de Potencia L298N (24V)"
Cohesion: 0.13
Nodes (15): 1.1 Estructura SIMO del Controlador, 1. Descripción General de la Arquitectura, 2. Diagrama de Bloques de los Lazos Internos, 3. Especificaciones Clave de las Interfaces, Arquitectura del Sistema de Control Angular y Plataforma Móvil, Enlaces del Grafo, 1. Rol en la Arquitectura de Control, 2. Parámetros de Identificación del Ventilador (+7 more)

### Community 6 - "Estrategias de Control: Compuerta (PID, LQR, H-infinita) y Carro Independiente"
Cohesion: 0.13
Nodes (15): 1.1 Función de Conversión Directa del Potenciómetro, 1.2 Ecuación Dinámica No Lineal, 1.3 Punto de Operación y Equilibrio ($\theta_{eq}$), 1.4 Modelo Linealizado (Pequeña Señal), 1. Modelo Matemático del Lazo Angular de la Compuerta (canal PWM → θ), 2. Estrategia: Control Óptimo LQR con Acción Integral ($LQR+I$), 3.1 Espacio de Estados Aumentado, 3.2 Optimización Cuadrática (+7 more)

### Community 7 - "adc_reader.c"
Cohesion: 0.18
Nodes (5): adc_reader_init(), adc_reader_start(), adc_reader_stop(), app_main(), uart_matlab_init()

### Community 8 - "Firmware ESP32 en C / ESP-IDF con FreeRTOS (Control Carro & Compuerta)"
Cohesion: 0.18
Nodes (10): 1. Asignación de Pines Hardware (Pinout), 2.1 Enlace Serial PC / MATLAB $\leftrightarrow$ Estación Base (115200 Baudios), 2. Protocolo de Comunicación, 3. Instrucciones de Compilación y Flasheo con ESP-IDF, Estructura de Directorios, Firmware ESP32 en C / ESP-IDF con FreeRTOS (Control Carro & Compuerta), Para compilar y subir el Nodo del Carro:, Para compilar y subir la Estación Base: (+2 more)

### Community 9 - "Especificaciones del Carro (Plataforma Móvil con IMU y Control Independiente)"
Cohesion: 0.20
Nodes (10): 1. Rol en el Proyecto y Condiciones Iniciales, 2.1 Principio Cinemático, 2.2 Observador de Estados / Filtro de Kalman, 2. Estimación de Distancia Avanzada / Retrocedida con la IMU, 3. Parámetros Físicos y Geométricos del Vehículo, 4. Motorreductores DC (Modelo TGP01D-A130), 5. Electrónica del Sistema del Carro, 6. Lazo de Control de Posición Longitudinal del Carro (+2 more)

### Community 10 - "Especificaciones de la Compuerta (Planta Pasiva y Sensor Directo 0-3.3V con ESP32)"
Cohesion: 0.20
Nodes (10): 1. Naturaleza del Sistema Dinámico, 2.1 Calibración Tensión vs. Ángulo (0V a 3.3V), 2.2 Función de Transferencia Directa del Sensor, 2.3 Adquisición Directa con ADC de 12 Bits del ESP32, 2. Sensor de Salida: Potenciómetro Directo (0V - 1.65V $\to$ 0° - 90°), 3. Parámetros Físicos y Mecánicos, 4. Ecuación Dinámica Pasiva de la Compuerta, Enlaces del Grafo (+2 more)

### Community 11 - "espnow_car_send_telemetry"
Cohesion: 0.25
Nodes (5): espnow_car_init(), espnow_car_recv_cb(), espnow_car_send_telemetry(), car_telemetry_task(), protocol_checksum()

### Community 12 - "2. Ecuaciones primitivas"
Cohesion: 0.29
Nodes (7): 2.1 Compuerta (péndulo colgante): derivación por Lagrange, 2.2 Torque aerodinámico, 2.3 Ventilador y saturación, 2.4 Carro, 2.5 Sensores y estimadores, 2.6 Sistema no lineal completo, 2. Ecuaciones primitivas

### Community 13 - "4. Cómo se llegó al LQR como mejor opción"
Cohesion: 0.33
Nodes (6): 4.1 Requisitos que debía cumplir el controlador, 4.2 Estrategias evaluadas, 4.3 Resultados comparativos, 4.4 Por qué el LQR resultó la mejor opción, 4.5 Árbol de decisión, 4. Cómo se llegó al LQR como mejor opción

## Knowledge Gaps
- **2 thin communities (<3 nodes) omitted from report** — run `graphify query` to explore isolated nodes.

## Suggested Questions
_Questions this graph is uniquely positioned to answer:_

- **Why does `Documentación técnica: modelado, funciones de transferencia y control LQR` connect `5. Estrategia de control LQR` to `2. Ecuaciones primitivas`, `4. Cómo se llegó al LQR como mejor opción`?**
  _High betweenness centrality (0.033) - this node is a cross-community bridge._
- **Are the 7 inferred relationships involving `app_main()` (e.g. with `car_controller_init()` and `espnow_car_init()`) actually correct?**
  _`app_main()` has 7 INFERRED edges - model-reasoned connections that need verification._
- **Should `mpu6050.c` be split into smaller, more focused modules?**
  _Cohesion score 0.11182795698924732 - nodes in this community are weakly interconnected._
- **Why does `Estrategias de Control: Compuerta (PID, LQR, H-infinita) y Carro Independiente` connect `Estrategias de Control: Compuerta (PID, LQR, H-infinita) y Carro Independiente` to `Especificaciones del Ventilador y Etapa de Potencia L298N (24V)`?**
  _High betweenness centrality (0.028) - this node is a cross-community bridge._
- **Should `5. Estrategia de control LQR` be split into smaller, more focused modules?**
  _Cohesion score 0.06666666666666667 - nodes in this community are weakly interconnected._
- **Why does `app_main()` connect `mpu6050.c` to `espnow_car_send_telemetry`, `esp32_carro_control/main/main.c`?**
  _High betweenness centrality (0.023) - this node is a cross-community bridge._
- **Should `espnow_base.c` be split into smaller, more focused modules?**
  _Cohesion score 0.10052910052910052 - nodes in this community are weakly interconnected._