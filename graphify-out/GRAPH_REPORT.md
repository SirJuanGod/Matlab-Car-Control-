# Graph Report - esp32-car-base-codes-f15829  (2026-11-04)

## Corpus Check
- 48 files · ~374,000 words
- Verdict: corpus is large enough that graph structure adds value.
- Unclassified: 12 file(s) not represented in the graph (top: (none) 5, .slx 3, .defaults 2)

## Summary
- 265 nodes · 425 edges · 13 communities (11 shown, 2 thin omitted)
- Extraction: 89% EXTRACTED · 11% INFERRED · 0% AMBIGUOUS · INFERRED: 46 edges (avg confidence: 0.85)
- Token cost: 0 input · 0 output

## Graph Freshness
- Built from commit: `958b84c8`
- Run `git rev-parse HEAD` and compare to check if the graph is stale.
- Run `graphify update .` after code changes (no API cost).

## Community Hubs (Navigation)
- mpu6050.c
- 8. Puesta en marcha de los ESP32
- CLAUDE.md
- adc_reader.c
- Especificaciones del Ventilador y Etapa de Potencia L298N (24V)
- Estrategias de Control: Compuerta (PID, LQR, H-infinita) y Carro Independiente
- car_control_task
- Firmware ESP32 en C / ESP-IDF con FreeRTOS (Control Carro & Compuerta)
- Especificaciones del Carro (Plataforma Móvil con IMU y Control Independiente)
- Especificaciones de la Compuerta (Planta Pasiva y Sensor Directo 0-3.3V con ESP32)
- espnow_base.c
- 5. Estrategia de control LQR

## God Nodes (most connected - your core abstractions)
1. `car_control_task()` - 12 edges
2. `Documentación técnica: modelado, funciones de transferencia y control LQR` - 11 edges
3. `5. Estrategia de control LQR` - 11 edges
4. `8. Puesta en marcha de los ESP32` - 11 edges
5. `app_main()` - 10 edges
6. `main()` - 8 edges
7. `Especificaciones del Carro (Plataforma Móvil con IMU y Control Independiente)` - 8 edges
8. `car_controller_step()` - 7 edges
9. `2. Ecuaciones primitivas` - 7 edges
10. `Especificaciones del Ventilador y Etapa de Potencia L298N (24V)` - 7 edges

## Surprising Connections (you probably didn't know these)
- `app_main()` --calls--> `car_controller_init()`  [INFERRED]
  code/C/esp32_carro_control/main/main.c → code/C/esp32_carro_control/main/car_controller.c
- `app_main()` --calls--> `espnow_car_init()`  [INFERRED]
  code/C/esp32_carro_control/main/main.c → code/C/esp32_carro_control/main/espnow_car.c
- `car_telemetry_task()` --calls--> `espnow_car_send_telemetry()`  [INFERRED]
  code/C/esp32_carro_control/main/main.c → code/C/esp32_carro_control/main/espnow_car.c
- `app_main()` --calls--> `kalman_observer_init()`  [INFERRED]
  code/C/esp32_carro_control/main/main.c → code/C/esp32_carro_control/main/kalman_observer.c
- `car_control_task()` --calls--> `mpu6050_is_ready()`  [INFERRED]
  code/C/esp32_carro_control/main/main.c → code/C/esp32_carro_control/main/mpu6050.c

## Import Cycles
- None detected.

## Communities (13 total, 2 thin omitted)

### Community 0 - "mpu6050.c"
Cohesion: 0.13
Nodes (17): app_main(), mpu6050_calibrate_bias(), mpu6050_init(), mpu6050_read_accel_x(), mpu6050_read_gyro_z(), read_accel_xyz(), read_bytes(), read_gyro_z_raw() (+9 more)

### Community 1 - "8. Puesta en marcha de los ESP32"
Cohesion: 0.05
Nodes (42): 1.1 Montaje físico, 1.2 Convenciones, 1.3 Parámetros del modelo, 1. Descripción del sistema y convenciones, 2.1 Compuerta (péndulo colgante): derivación por Lagrange, 2.2 Torque aerodinámico, 2.3 Ventilador y saturación, 2.4 Carro (+34 more)

### Community 3 - "adc_reader.c"
Cohesion: 0.07
Nodes (14): adc_reader_get_last_angle(), adc_reader_init(), adc_reader_start(), adc_reader_stop(), adc_sample_task(), angle_kalman_update(), espnow_base_init(), espnow_base_send() (+6 more)

### Community 5 - "Especificaciones del Ventilador y Etapa de Potencia L298N (24V)"
Cohesion: 0.13
Nodes (15): 1.1 Estructura SIMO del Controlador, 1. Descripción General de la Arquitectura, 2. Diagrama de Bloques de los Lazos Internos, 3. Especificaciones Clave de las Interfaces, Arquitectura del Sistema de Control Angular y Plataforma Móvil, Enlaces del Grafo, 1. Rol en la Arquitectura de Control, 2. Parámetros de Identificación del Ventilador (+7 more)

### Community 6 - "Estrategias de Control: Compuerta (PID, LQR, H-infinita) y Carro Independiente"
Cohesion: 0.13
Nodes (15): 1.1 Función de Conversión Directa del Potenciómetro, 1.2 Ecuación Dinámica No Lineal, 1.3 Punto de Operación y Equilibrio ($\theta_{eq}$), 1.4 Modelo Linealizado (Pequeña Señal), 1. Modelo Matemático del Lazo Angular de la Compuerta (canal PWM → θ), 2. Estrategia: Control Óptimo LQR con Acción Integral ($LQR+I$), 3.1 Espacio de Estados Aumentado, 3.2 Optimización Cuadrática (+7 more)

### Community 7 - "car_control_task"
Cohesion: 0.10
Nodes (21): car_controller_init(), car_controller_step(), kalman_observer_get_state(), kalman_observer_init(), kalman_observer_reset(), kalman_observer_update(), clampf(), ganancia() (+13 more)

### Community 8 - "Firmware ESP32 en C / ESP-IDF con FreeRTOS (Control Carro & Compuerta)"
Cohesion: 0.15
Nodes (12): 1. Asignación de Pines Hardware (Pinout), 2.1 Enlace Serial PC / MATLAB $\leftrightarrow$ Estación Base (115200 Baudios), 2.2 Emparejamiento ESP-NOW (Base $\leftrightarrow$ Carro), 2. Protocolo de Comunicación, 3. Instrucciones de Compilación y Flasheo con ESP-IDF, Estructura de Directorios, Firmware ESP32 en C / ESP-IDF con FreeRTOS (Control Carro & Compuerta), Para compilar y subir el Nodo del Carro: (+4 more)

### Community 9 - "Especificaciones del Carro (Plataforma Móvil con IMU y Control Independiente)"
Cohesion: 0.20
Nodes (10): 1. Rol en el Proyecto y Condiciones Iniciales, 2.1 Principio Cinemático, 2.2 Observador de Estados / Filtro de Kalman, 2. Estimación de Distancia Avanzada / Retrocedida con la IMU, 3. Parámetros Físicos y Geométricos del Vehículo, 4. Motorreductores DC (Modelo TGP01D-A130), 5. Electrónica del Sistema del Carro, 6. Lazo de Control de Posición Longitudinal del Carro (+2 more)

### Community 10 - "Especificaciones de la Compuerta (Planta Pasiva y Sensor Directo 0-3.3V con ESP32)"
Cohesion: 0.20
Nodes (10): 1. Naturaleza del Sistema Dinámico, 2.1 Calibración Tensión vs. Ángulo (0V a 3.3V), 2.2 Función de Transferencia Directa del Sensor, 2.3 Adquisición Directa con ADC de 12 Bits del ESP32, 2. Sensor de Salida: Potenciómetro Directo (0V - 1.65V $\to$ 0° - 90°), 3. Parámetros Físicos y Mecánicos, 4. Ecuación Dinámica Pasiva de la Compuerta, Enlaces del Grafo (+2 more)

### Community 11 - "espnow_base.c"
Cohesion: 0.12
Nodes (7): espnow_send_cb(), espnow_car_init(), espnow_car_recv_cb(), espnow_car_send_cb(), espnow_car_send_telemetry(), car_telemetry_task(), protocol_checksum()

### Community 12 - "5. Estrategia de control LQR"
Cohesion: 0.18
Nodes (11): 5.10 Resumen de parámetros de diseño, 5.1 Formulación, 5.2 Estados y entradas, 5.3 Selección de los pesos $Q$ y $R$, 5.4 *Gain scheduling* en $(\theta_{ref}, d_{nom})$, 5.5 Ley de control completa, 5.6 Estimación de los estados no medidos, 5.7 Saturación y anti-windup (+3 more)

## Knowledge Gaps
- **2 thin communities (<3 nodes) omitted from report** — run `graphify query` to explore isolated nodes.

## Suggested Questions
_Questions this graph is uniquely positioned to answer:_

- **Why does `Documentación técnica: modelado, funciones de transferencia y control LQR` connect `8. Puesta en marcha de los ESP32` to `5. Estrategia de control LQR`?**
  _High betweenness centrality (0.035) - this node is a cross-community bridge._
- **Are the 11 inferred relationships involving `car_control_task()` (e.g. with `car_controller_step()` and `kalman_observer_get_state()`) actually correct?**
  _`car_control_task()` has 11 INFERRED edges - model-reasoned connections that need verification._
- **Should `mpu6050.c` be split into smaller, more focused modules?**
  _Cohesion score 0.13333333333333333 - nodes in this community are weakly interconnected._
- **Why does `app_main()` connect `mpu6050.c` to `espnow_base.c`, `esp32_carro_control/main/main.c`, `car_control_task`?**
  _High betweenness centrality (0.026) - this node is a cross-community bridge._
- **Are the 9 inferred relationships involving `app_main()` (e.g. with `car_controller_init()` and `espnow_car_init()`) actually correct?**
  _`app_main()` has 9 INFERRED edges - model-reasoned connections that need verification._
- **Should `8. Puesta en marcha de los ESP32` be split into smaller, more focused modules?**
  _Cohesion score 0.046511627906976744 - nodes in this community are weakly interconnected._
- **Why does `car_control_task()` connect `car_control_task` to `mpu6050.c`, `esp32_carro_control/main/main.c`?**
  _High betweenness centrality (0.026) - this node is a cross-community bridge._