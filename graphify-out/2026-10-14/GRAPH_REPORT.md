# Graph Report - Car_Control  (2026-10-14)

## Corpus Check
- 9 files · ~9,969 words
- Verdict: corpus is large enough that graph structure adds value.
- Unclassified: 5 file(s) not represented in the graph (top: .mat 2, .slx 1, .slxc 1)

## Summary
- 65 nodes · 69 edges · 8 communities (6 shown, 2 thin omitted)
- Extraction: 100% EXTRACTED · 0% INFERRED · 0% AMBIGUOUS
- Token cost: 0 input · 0 output

## Community Hubs (Navigation)
- Estrategias de Control: Compuerta (PID, LQR, H-infinita) y Carro Independiente
- 3. Estrategia 2: Control Óptimo LQR con Acción Integral ($LQR+I$)
- CLAUDE.md
- Especificaciones del Carro (Plataforma Móvil con Control Independiente)
- Especificaciones de la Compuerta (Planta Pasiva y Sensor con ADC ESP32)
- Especificaciones del Ventilador y Etapa de Potencia L298N (24V)
- Arquitectura del Sistema de Control Angular y Plataforma Móvil

## God Nodes (most connected - your core abstractions)
1. `Especificaciones del Carro (Plataforma Móvil con Control Independiente)` - 9 edges
2. `Estrategias de Control: Compuerta (PID, LQR, H-infinita) y Carro Independiente` - 9 edges
3. `Especificaciones del Ventilador y Etapa de Potencia L298N (24V)` - 8 edges
4. `Especificaciones de la Compuerta (Planta Pasiva y Sensor con ADC ESP32)` - 7 edges
5. `Arquitectura del Sistema de Control Angular y Plataforma Móvil` - 6 edges
6. `2. Sensor de Salida: Potenciómetro y Conversión ADC en ESP32` - 4 edges
7. `1. Modelo Matemático del Lazo Angular de la Compuerta (SISO)` - 4 edges
8. `3. Estrategia 2: Control Óptimo LQR con Acción Integral ($LQR+I$)` - 4 edges
9. `2.3 Adquisición y Resolución del ADC de 12 Bits del ESP32` - 3 edges
10. `4. Etapa de Potencia: Módulo Puente H L298N a 24 VDC` - 3 edges

## Surprising Connections (you probably didn't know these)
- None detected - all connections are within the same source files.

## Communities (8 total, 2 thin omitted)

### Community 0 - "Estrategias de Control: Compuerta (PID, LQR, H-infinita) y Carro Independiente"
Cohesion: 0.12
Nodes (16): 1.1 Ecuación Dinámica No Lineal, 1.2 Punto de Operación y Equilibrio ($\theta_{eq}$), 1.3 Modelo Linealizado (Pequeña Señal), 1. Modelo Matemático del Lazo Angular de la Compuerta (SISO), 2.1 Mejoras de Implementación en ESP32, 2. Estrategia 1: Control PID de la Compuerta con Feedforward y Anti-Windup, 4.1 Justificación Frente a la Plataforma Móvil, 4.2 Modelo de Incertidumbre y Sensibilidad Mixta (+8 more)

### Community 1 - "3. Estrategia 2: Control Óptimo LQR con Acción Integral ($LQR+I$)"
Cohesion: 0.50
Nodes (4): 3.1 Espacio de Estados Aumentado, 3.2 Optimización Cuadrática, 3.3 Ley de Control, 3. Estrategia 2: Control Óptimo LQR con Acción Integral ($LQR+I$)

### Community 5 - "Especificaciones del Carro (Plataforma Móvil con Control Independiente)"
Cohesion: 0.20
Nodes (10): 1. Rol en el Proyecto: Sistema con Control Independiente, 2. Parámetros Físicos y Geométricos del Vehículo, 3. Ruedas y Conjunto de Tracción, 4. Motorreductores DC (Modelo TGP01D-A130), 5. Electrónica del Sistema del Carro, 6.1 Ley de Control PID de Distancia, 6. Arquitectura del Control Independiente del Carro, Enlaces del Grafo (+2 more)

### Community 6 - "Especificaciones de la Compuerta (Planta Pasiva y Sensor con ADC ESP32)"
Cohesion: 0.17
Nodes (12): 1. Naturaleza del Sistema Dinámico, 2.1 Calibración Tensión vs. Ángulo, 2.2 Acondicionamiento de Señal para ESP32 (Divisor de Tensión), 2.3 Adquisición y Resolución del ADC de 12 Bits del ESP32, 2. Sensor de Salida: Potenciómetro y Conversión ADC en ESP32, 3. Parámetros Físicos y Mecánicos, 4. Ecuación Dinámica Pasiva de la Compuerta, Cálculo de la Resolución Angular (+4 more)

### Community 7 - "Especificaciones del Ventilador y Etapa de Potencia L298N (24V)"
Cohesion: 0.20
Nodes (10): 1. Rol en la Arquitectura de Control, 2. Parámetros de Identificación del Ventilador, 3. Parámetros Eléctricos y Desempeño, 4.1 Configuración de Hardware Crítica del L298N, 4.2 Lógica de Control desde el ESP32, 4. Etapa de Potencia: Módulo Puente H L298N a 24 VDC, 5. Dinámica del Actuador Aerodinámico, Enlaces del Grafo (+2 more)

### Community 8 - "Arquitectura del Sistema de Control Angular y Plataforma Móvil"
Cohesion: 0.33
Nodes (6): 1. Descripción General de la Arquitectura, 2. Diagrama de Bloques General de los Lazos Desacoplados, 3. Especificaciones Clave de las Interfaces, Arquitectura del Sistema de Control Angular y Plataforma Móvil, Enlaces del Grafo, Etiquetas

## Knowledge Gaps
- **2 thin communities (<3 nodes) omitted from report** — run `graphify query` to explore isolated nodes.

## Suggested Questions
_Questions this graph is uniquely positioned to answer:_

- **Why does `Estrategias de Control: Compuerta (PID, LQR, H-infinita) y Carro Independiente` connect `Estrategias de Control: Compuerta (PID, LQR, H-infinita) y Carro Independiente` to `arquitectura_sistema.md`, `3. Estrategia 2: Control Óptimo LQR con Acción Integral ($LQR+I$)`?**
  _High betweenness centrality (0.481) - this node is a cross-community bridge._
- **Should `Estrategias de Control: Compuerta (PID, LQR, H-infinita) y Carro Independiente` be split into smaller, more focused modules?**
  _Cohesion score 0.125 - nodes in this community are weakly interconnected._
- **Why does `Especificaciones de la Compuerta (Planta Pasiva y Sensor con ADC ESP32)` connect `Especificaciones de la Compuerta (Planta Pasiva y Sensor con ADC ESP32)` to `arquitectura_sistema.md`?**
  _High betweenness centrality (0.298) - this node is a cross-community bridge._
- **Why does `Especificaciones del Carro (Plataforma Móvil con Control Independiente)` connect `Especificaciones del Carro (Plataforma Móvil con Control Independiente)` to `arquitectura_sistema.md`?**
  _High betweenness centrality (0.254) - this node is a cross-community bridge._