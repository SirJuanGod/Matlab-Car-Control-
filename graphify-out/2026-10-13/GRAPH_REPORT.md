# Graph Report - Car_Control  (2026-10-13)

## Corpus Check
- 10 files · ~1,657 words
- Verdict: corpus is large enough that graph structure adds value.

## Summary
- 54 nodes · 51 edges · 10 communities (8 shown, 2 thin omitted)
- Extraction: 100% EXTRACTED · 0% INFERRED · 0% AMBIGUOUS
- Token cost: 0 input · 0 output

## Community Hubs (Navigation)
- Especificaciones del Actuador Aerodinámico (Ventilador)
- Especificaciones de la Compuerta (Planta a Controlar)
- CLAUDE.md
- Especificaciones del Vehículo Móvil (Carro)
- Arquitectura de Control e Interacción del Sistema
- Especificaciones del Carro (Plataforma Móvil)
- Especificaciones de la Compuerta (Planta)
- Especificaciones del Ventilador (Actuador Aerodinámico)
- Arquitectura del Sistema de Control Angular

## God Nodes (most connected - your core abstractions)
1. `Especificaciones del Carro (Plataforma Móvil)` - 7 edges
2. `Especificaciones de la Compuerta (Planta)` - 6 edges
3. `Especificaciones del Actuador Aerodinámico (Ventilador)` - 6 edges
4. `Especificaciones del Ventilador (Actuador Aerodinámico)` - 6 edges
5. `Arquitectura del Sistema de Control Angular` - 5 edges
6. `Especificaciones de la Compuerta (Planta a Controlar)` - 5 edges
7. `Especificaciones del Vehículo Móvil (Carro)` - 5 edges
8. `Arquitectura de Control e Interacción del Sistema` - 4 edges
9. `graphify` - 1 edges
10. `Descripción del Acople` - 1 edges

## Surprising Connections (you probably didn't know these)
- None detected - all connections are within the same source files.

## Communities (10 total, 2 thin omitted)

### Community 0 - "Especificaciones del Actuador Aerodinámico (Ventilador)"
Cohesion: 0.29
Nodes (6): Características Geométricas, Especificaciones del Actuador Aerodinámico (Ventilador), Especificaciones Eléctricas, Identificación, Rendimiento Aerodinámico, Ubicación en el Sistema

### Community 1 - "Especificaciones de la Compuerta (Planta a Controlar)"
Cohesion: 0.33
Nodes (5): Especificaciones de la Compuerta (Planta a Controlar), Fijación y Cinemática, Geometría y Material, Identificación, Propiedades Físicas Estimadas

### Community 3 - "Especificaciones del Vehículo Móvil (Carro)"
Cohesion: 0.33
Nodes (5): Actuadores de Tracción (Motorreductores), Especificaciones del Vehículo Móvil (Carro), Geometría de las Ruedas Motrices, Geometría del Chasis, Identificación General

### Community 4 - "Arquitectura de Control e Interacción del Sistema"
Cohesion: 0.40
Nodes (4): Arquitectura de Control e Interacción del Sistema, Descripción del Acople, Estrategias de Control Propuestas (MISO), Variables Críticas de Interacción

### Community 5 - "Especificaciones del Carro (Plataforma Móvil)"
Cohesion: 0.29
Nodes (7): Electrónica de Potencia y Control, Enlaces del Grafo, Especificaciones del Carro (Plataforma Móvil), Etiquetas, Motorreductores (Modelo TGP01D-A130), Parámetros Físicos y Geométricos, Ruedas y Tracción

### Community 6 - "Especificaciones de la Compuerta (Planta)"
Cohesion: 0.33
Nodes (6): Enlaces del Grafo, Especificaciones de la Compuerta (Planta), Etiquetas, Geometría y Material, Mecánica y Anclaje, Parámetros Físicos Estimados

### Community 7 - "Especificaciones del Ventilador (Actuador Aerodinámico)"
Cohesion: 0.33
Nodes (6): Enlaces del Grafo, Especificaciones del Ventilador (Actuador Aerodinámico), Etiquetas, Parámetros Aerodinámicos, Parámetros de Identificación, Parámetros Eléctricos

### Community 8 - "Arquitectura del Sistema de Control Angular"
Cohesion: 0.40
Nodes (5): Arquitectura del Sistema de Control Angular, Descripción General, Enlaces del Grafo, Etiquetas, Modos de Operación

## Knowledge Gaps
- **2 thin communities (<3 nodes) omitted from report** — run `graphify query` to explore isolated nodes.

## Suggested Questions
_Questions this graph is uniquely positioned to answer:_

- **Why does `Especificaciones del Carro (Plataforma Móvil)` connect `Especificaciones del Carro (Plataforma Móvil)` to `arquitectura_sistema.md`?**
  _High betweenness centrality (0.102) - this node is a cross-community bridge._
- **Why does `Especificaciones de la Compuerta (Planta)` connect `Especificaciones de la Compuerta (Planta)` to `arquitectura_sistema.md`?**
  _High betweenness centrality (0.087) - this node is a cross-community bridge._
- **Why does `Especificaciones del Ventilador (Actuador Aerodinámico)` connect `Especificaciones del Ventilador (Actuador Aerodinámico)` to `arquitectura_sistema.md`?**
  _High betweenness centrality (0.087) - this node is a cross-community bridge._