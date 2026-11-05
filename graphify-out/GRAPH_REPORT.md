# Graph Report - esp32-car-base-codes-f15829  (2026-11-04)

## Corpus Check
- 48 files · ~374,878 words
- Verdict: corpus is large enough that graph structure adds value.
- Unclassified: 12 file(s) not represented in the graph (top: (none) 5, .slx 3, .defaults 2)

## Summary
- 271 nodes · 438 edges · 14 communities (12 shown, 2 thin omitted)
- Extraction: 89% EXTRACTED · 11% INFERRED · 0% AMBIGUOUS · INFERRED: 46 edges (avg confidence: 0.85)
- Token cost: 0 input · 0 output

## Graph Freshness
- Built from commit: `4be5bd8e`
- Run `git rev-parse HEAD` and compare to check if the graph is stale.
- Run `graphify update .` after code changes (no API cost).

## Community Hubs (Navigation)
- mpu6050.c
- 5. Estrategia de control LQR
- CLAUDE.md
- adc_reader.c
- Especificaciones del Ventilador y Etapa de Potencia L298N (24V)
- Estrategias de Control: Compuerta (PID, LQR, H-infinita) y Carro Independiente
- uart_matlab.c
- Firmware ESP32 en C / ESP-IDF con FreeRTOS (Control Carro & Compuerta)
- Especificaciones del Carro (Plataforma Móvil con IMU y Control Independiente)
- Especificaciones de la Compuerta (Planta Pasiva y Sensor Directo 0-3.3V con ESP32)
- espnow_base.c
- esp32_prueba_motores/main/main.c
- 8. Puesta en marcha de los ESP32

## God Nodes (most connected - your core abstractions)
1. `car_control_task()` - 12 edges
2. `Documentación técnica: modelado, funciones de transferencia y control LQR` - 11 edges
3. `5. Estrategia de control LQR` - 11 edges
4. `8. Puesta en marcha de los ESP32` - 11 edges
5. `app_main()` - 10 edges
6. `main()` - 8 edges
7. `Especificaciones del Carro (Plataforma Móvil con IMU y Control Independiente)` - 8 edges
8. `car_controller_step()` - 7 edges
9. `app_main()` - 7 edges
10. `2. Ecuaciones primitivas` - 7 edges

## Surprising Connections (you probably didn't know these)
- `app_main()` --calls--> `espnow_base_init()`  [INFERRED]
  code/C/esp32_base_potenciometro/main/main.c → code/C/esp32_base_potenciometro/main/espnow_base.c
- `base_hub_task()` --calls--> `espnow_base_send()`  [INFERRED]
  code/C/esp32_base_potenciometro/main/main.c → code/C/esp32_base_potenciometro/main/espnow_base.c
- `app_main()` --calls--> `car_controller_init()`  [INFERRED]
  code/C/esp32_carro_control/main/main.c → code/C/esp32_carro_control/main/car_controller.c
- `car_control_task()` --calls--> `car_controller_step()`  [INFERRED]
  code/C/esp32_carro_control/main/main.c → code/C/esp32_carro_control/main/car_controller.c
- `app_main()` --calls--> `espnow_car_init()`  [INFERRED]
  code/C/esp32_carro_control/main/main.c → code/C/esp32_carro_control/main/espnow_car.c

## Import Cycles
- None detected.

## Communities (14 total, 2 thin omitted)

### Community 0 - "mpu6050.c"
Cohesion: 0.11
Nodes (23): kalman_observer_get_state(), kalman_observer_init(), kalman_observer_reset(), kalman_observer_update(), app_main(), car_control_task(), actuators_emergency_stop(), car_motor_effective_percent() (+15 more)

### Community 1 - "5. Estrategia de control LQR"
Cohesion: 0.05
Nodes (42): 1.1 Montaje físico, 1.2 Convenciones, 1.3 Parámetros del modelo, 1. Descripción del sistema y convenciones, 2.1 Compuerta (péndulo colgante): derivación por Lagrange, 2.2 Torque aerodinámico, 2.3 Ventilador y saturación, 2.4 Carro (+34 more)

### Community 3 - "adc_reader.c"
Cohesion: 0.14
Nodes (10): adc_reader_get_last_angle(), adc_reader_init(), adc_reader_start(), adc_reader_stop(), adc_sample_task(), angle_kalman_update(), app_main(), base_hub_task() (+2 more)

### Community 5 - "Especificaciones del Ventilador y Etapa de Potencia L298N (24V)"
Cohesion: 0.13
Nodes (15): 1.1 Estructura SIMO del Controlador, 1. Descripción General de la Arquitectura, 2. Diagrama de Bloques de los Lazos Internos, 3. Especificaciones Clave de las Interfaces, Arquitectura del Sistema de Control Angular y Plataforma Móvil, Enlaces del Grafo, 1. Rol en la Arquitectura de Control, 2. Parámetros de Identificación del Ventilador (+7 more)

### Community 6 - "Estrategias de Control: Compuerta (PID, LQR, H-infinita) y Carro Independiente"
Cohesion: 0.13
Nodes (15): 1.1 Función de Conversión Directa del Potenciómetro, 1.2 Ecuación Dinámica No Lineal, 1.3 Punto de Operación y Equilibrio ($\theta_{eq}$), 1.4 Modelo Linealizado (Pequeña Señal), 1. Modelo Matemático del Lazo Angular de la Compuerta (canal PWM → θ), 2. Estrategia: Control Óptimo LQR con Acción Integral ($LQR+I$), 3.1 Espacio de Estados Aumentado, 3.2 Optimización Cuadrática (+7 more)

### Community 7 - "uart_matlab.c"
Cohesion: 0.13
Nodes (9): car_controller_init(), car_controller_step(), clampf(), ganancia(), lqr_simo_reset(), lqr_simo_step(), app_main(), lazo_cerrado() (+1 more)

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
Cohesion: 0.09
Nodes (11): espnow_base_init(), espnow_base_send(), espnow_recv_cb(), espnow_send_cb(), protocol_checksum(), espnow_car_init(), espnow_car_recv_cb(), espnow_car_send_cb() (+3 more)

### Community 12 - "esp32_prueba_motores/main/main.c"
Cohesion: 0.21
Nodes (10): app_main(), barrido_ventilador(), comparar_frecuencias_ventilador(), fan_timer_freq(), prueba_direccion(), pulso(), pwm_channel(), set_dir() (+2 more)

### Community 13 - "8. Puesta en marcha de los ESP32"
Cohesion: 0.18
Nodes (11): 8.10 Problemas comunes, 8.1 Qué se necesita, 8.2 Conexiones, 8.3 Configuración del ESP32, 8.4 Instalar ESP-IDF (una sola vez), 8.5 Activar ESP-IDF (cada vez que abras PowerShell), 8.6 Encontrar el puerto COM de cada placa, 8.7 Compilar y cargar el carro (+3 more)

## Knowledge Gaps
- **2 thin communities (<3 nodes) omitted from report** — run `graphify query` to explore isolated nodes.

## Suggested Questions
_Questions this graph is uniquely positioned to answer:_

- **Why does `Documentación técnica: modelado, funciones de transferencia y control LQR` connect `5. Estrategia de control LQR` to `8. Puesta en marcha de los ESP32`?**
  _High betweenness centrality (0.033) - this node is a cross-community bridge._
- **Are the 11 inferred relationships involving `car_control_task()` (e.g. with `car_controller_step()` and `kalman_observer_get_state()`) actually correct?**
  _`car_control_task()` has 11 INFERRED edges - model-reasoned connections that need verification._
- **Should `mpu6050.c` be split into smaller, more focused modules?**
  _Cohesion score 0.1111111111111111 - nodes in this community are weakly interconnected._
- **Why does `app_main()` connect `mpu6050.c` to `espnow_base.c`, `esp32_carro_control/main/main.c`, `uart_matlab.c`?**
  _High betweenness centrality (0.025) - this node is a cross-community bridge._
- **Are the 9 inferred relationships involving `app_main()` (e.g. with `car_controller_init()` and `espnow_car_init()`) actually correct?**
  _`app_main()` has 9 INFERRED edges - model-reasoned connections that need verification._
- **Should `5. Estrategia de control LQR` be split into smaller, more focused modules?**
  _Cohesion score 0.046511627906976744 - nodes in this community are weakly interconnected._
- **Why does `car_control_task()` connect `mpu6050.c` to `esp32_carro_control/main/main.c`, `uart_matlab.c`?**
  _High betweenness centrality (0.024) - this node is a cross-community bridge._