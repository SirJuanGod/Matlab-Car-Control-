# Graph Report - Car_Control  (2026-10-26)

## Corpus Check
- 14 files · ~257,480 words
- Verdict: corpus is large enough that graph structure adds value.
- Unclassified: 3 file(s) not represented in the graph (top: (none) 1, .bak 1, .slx 1)

## Summary
- 110 nodes · 113 edges · 11 communities (9 shown, 2 thin omitted)
- Extraction: 100% EXTRACTED · 0% INFERRED · 0% AMBIGUOUS
- Token cost: 0 input · 0 output

## Graph Freshness
- Built from commit: `045ad3a9`
- Run `git rev-parse HEAD` and compare to check if the graph is stale.
- Run `graphify update .` after code changes (no API cost).

## Community Hubs (Navigation)
- Estrategias de Control: Compuerta (PID, LQR, H-infinita) y Carro Independiente
- Documentación técnica: modelado, funciones de transferencia y control LQR
- CLAUDE.md
- 5. Estrategia de control LQR
- 2. Ecuaciones primitivas
- Especificaciones del Carro (Plataforma Móvil con IMU y Control Independiente)
- Especificaciones de la Compuerta (Planta Pasiva y Sensor Directo 0-3.3V con ESP32)
- Especificaciones del Ventilador y Etapa de Potencia L298N (24V)
- Arquitectura del Sistema de Control Angular y Plataforma Móvil
- 3. Linealización y funciones de transferencia

## God Nodes (most connected - your core abstractions)
1. `5. Estrategia de control LQR` - 11 edges
2. `Documentación técnica: modelado, funciones de transferencia y control LQR` - 10 edges
3. `Especificaciones del Carro (Plataforma Móvil con IMU y Control Independiente)` - 9 edges
4. `Estrategias de Control: Compuerta (PID, LQR, H-infinita) y Carro Independiente` - 9 edges
5. `Especificaciones del Ventilador y Etapa de Potencia L298N (24V)` - 8 edges
6. `2. Ecuaciones primitivas` - 7 edges
7. `Arquitectura del Sistema de Control Angular y Plataforma Móvil` - 7 edges
8. `Especificaciones de la Compuerta (Planta Pasiva y Sensor Directo 0-3.3V con ESP32)` - 7 edges
9. `3. Linealización y funciones de transferencia` - 6 edges
10. `4. Cómo se llegó al LQR como mejor opción` - 6 edges

## Surprising Connections (you probably didn't know these)
- None detected - all connections are within the same source files.

## Communities (11 total, 2 thin omitted)

### Community 0 - "Estrategias de Control: Compuerta (PID, LQR, H-infinita) y Carro Independiente"
Cohesion: 0.10
Nodes (21): 1.1 Función de Conversión Directa del Potenciómetro, 1.2 Ecuación Dinámica No Lineal, 1.3 Punto de Operación y Equilibrio ($\theta_{eq}$), 1.4 Modelo Linealizado (Pequeña Señal), 1. Modelo Matemático del Lazo Angular de la Compuerta (canal PWM → θ), 2.1 Componentes de Implementación, 2. Estrategia 1: Control PID de la Compuerta con Feedforward y Anti-Windup, 3.1 Espacio de Estados Aumentado (+13 more)

### Community 1 - "Documentación técnica: modelado, funciones de transferencia y control LQR"
Cohesion: 0.11
Nodes (18): 1.1 Montaje físico, 1.2 Convenciones, 1.3 Parámetros del modelo, 1. Descripción del sistema y convenciones, 4.1 Requisitos que debía cumplir el controlador, 4.2 Estrategias evaluadas, 4.3 Resultados comparativos, 4.4 Por qué el LQR resultó la mejor opción (+10 more)

### Community 3 - "5. Estrategia de control LQR"
Cohesion: 0.18
Nodes (11): 5.10 Resumen de parámetros de diseño, 5.1 Formulación, 5.2 Estados y entradas, 5.3 Selección de los pesos $Q$ y $R$, 5.4 *Gain scheduling* en $(\theta_{ref}, d_{nom})$, 5.5 Ley de control completa, 5.6 Estimación de los estados no medidos, 5.7 Saturación y anti-windup (+3 more)

### Community 4 - "2. Ecuaciones primitivas"
Cohesion: 0.29
Nodes (7): 2.1 Compuerta (péndulo colgante): derivación por Lagrange, 2.2 Torque aerodinámico, 2.3 Ventilador y saturación, 2.4 Carro, 2.5 Sensores y estimadores, 2.6 Sistema no lineal completo, 2. Ecuaciones primitivas

### Community 5 - "Especificaciones del Carro (Plataforma Móvil con IMU y Control Independiente)"
Cohesion: 0.18
Nodes (11): 1. Rol en el Proyecto y Condiciones Iniciales, 2.1 Principio Cinemático, 2.2 Observador de Estados / Filtro de Kalman, 2. Estimación de Distancia Avanzada / Retrocedida con la IMU, 3. Parámetros Físicos y Geométricos del Vehículo, 4. Motorreductores DC (Modelo TGP01D-A130), 5. Electrónica del Sistema del Carro, 6. Lazo de Control de Posición Longitudinal del Carro (+3 more)

### Community 6 - "Especificaciones de la Compuerta (Planta Pasiva y Sensor Directo 0-3.3V con ESP32)"
Cohesion: 0.18
Nodes (11): 1. Naturaleza del Sistema Dinámico, 2.1 Calibración Tensión vs. Ángulo (0V a 3.3V), 2.2 Función de Transferencia Directa del Sensor, 2.3 Adquisición Directa con ADC de 12 Bits del ESP32, 2. Sensor de Salida: Potenciómetro Directo (0V - 3.3V $\to$ 0° - 180°), 3. Parámetros Físicos y Mecánicos, 4. Ecuación Dinámica Pasiva de la Compuerta, Enlaces del Grafo (+3 more)

### Community 7 - "Especificaciones del Ventilador y Etapa de Potencia L298N (24V)"
Cohesion: 0.20
Nodes (10): 1. Rol en la Arquitectura de Control, 2. Parámetros de Identificación del Ventilador, 3. Parámetros Eléctricos y Desempeño, 4.1 Configuración de Hardware Crítica del L298N, 4.2 Lógica de Control desde el ESP32, 4. Etapa de Potencia: Módulo Puente H L298N a 24 VDC, 5. Dinámica del Actuador Aerodinámico, Enlaces del Grafo (+2 more)

### Community 8 - "Arquitectura del Sistema de Control Angular y Plataforma Móvil"
Cohesion: 0.29
Nodes (7): 1.1 Estructura SIMO del Controlador (Planta MISO), 1. Descripción General de la Arquitectura, 2. Diagrama de Bloques de los Lazos Internos, 3. Especificaciones Clave de las Interfaces, Arquitectura del Sistema de Control Angular y Plataforma Móvil, Enlaces del Grafo, Etiquetas

### Community 10 - "3. Linealización y funciones de transferencia"
Cohesion: 0.33
Nodes (6): 3.1 Punto de equilibrio, 3.2 Linealización, 3.3 Modelo en espacio de estados (5 estados, 2 entradas), 3.4 Funciones de transferencia de cada subsistema, 3.5 Planta SIMO completa en $35^\circ$ y $10\text{ cm}$, 3. Linealización y funciones de transferencia

## Knowledge Gaps
- **2 thin communities (<3 nodes) omitted from report** — run `graphify query` to explore isolated nodes.

## Suggested Questions
_Questions this graph is uniquely positioned to answer:_

- **Why does `Estrategias de Control: Compuerta (PID, LQR, H-infinita) y Carro Independiente` connect `Estrategias de Control: Compuerta (PID, LQR, H-infinita) y Carro Independiente` to `arquitectura_sistema.md`?**
  _High betweenness centrality (0.178) - this node is a cross-community bridge._
- **Should `Estrategias de Control: Compuerta (PID, LQR, H-infinita) y Carro Independiente` be split into smaller, more focused modules?**
  _Cohesion score 0.09523809523809523 - nodes in this community are weakly interconnected._
- **Why does `Documentación técnica: modelado, funciones de transferencia y control LQR` connect `Documentación técnica: modelado, funciones de transferencia y control LQR` to `3. Linealización y funciones de transferencia`, `5. Estrategia de control LQR`, `2. Ecuaciones primitivas`?**
  _High betweenness centrality (0.126) - this node is a cross-community bridge._
- **Should `Documentación técnica: modelado, funciones de transferencia y control LQR` be split into smaller, more focused modules?**
  _Cohesion score 0.10526315789473684 - nodes in this community are weakly interconnected._
- **Why does `Especificaciones del Carro (Plataforma Móvil con IMU y Control Independiente)` connect `Especificaciones del Carro (Plataforma Móvil con IMU y Control Independiente)` to `arquitectura_sistema.md`?**
  _High betweenness centrality (0.099) - this node is a cross-community bridge._