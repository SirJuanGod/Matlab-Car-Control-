# Graph Report - Car_Control  (2026-10-11)

## Corpus Check
- 6 files · ~3,643 words
- Verdict: corpus is large enough that graph structure adds value.
- Unclassified: 15 file(s) not represented in the graph (top: .mat 10, .slx 2, .slxc 1)

## Summary
- 10 nodes · 8 edges · 3 communities (2 shown, 1 thin omitted)
- Extraction: 100% EXTRACTED · 0% INFERRED · 0% AMBIGUOUS
- Token cost: 0 input · 0 output

## Community Hubs (Navigation)
- Contexto_Proyecto.md
- Entregables Requeridos
- CLAUDE.md

## God Nodes (most connected - your core abstractions)
1. `Entregables Requeridos` - 3 edges
2. `graphify` - 1 edges
3. `Contexto y Rol` - 1 edges
4. `Descripción del Sistema` - 1 edges
5. `Ecuación Diferencial No Lineal (Planta)` - 1 edges
6. `1. Archivo de Parámetros (`parametros_sistema.m`)` - 1 edges
7. `2. Generador del Modelo de Simulink (`crear_simulink.m`)` - 1 edges
8. `Instrucciones de Salida` - 1 edges

## Surprising Connections (you probably didn't know these)
- None detected - all connections are within the same source files.

## Communities (3 total, 1 thin omitted)

### Community 0 - "Contexto_Proyecto.md"
Cohesion: 0.40
Nodes (4): Contexto y Rol, Descripción del Sistema, Ecuación Diferencial No Lineal (Planta), Instrucciones de Salida

### Community 1 - "Entregables Requeridos"
Cohesion: 0.67
Nodes (3): 1. Archivo de Parámetros (`parametros_sistema.m`), 2. Generador del Modelo de Simulink (`crear_simulink.m`), Entregables Requeridos

## Knowledge Gaps
- **1 thin communities (<3 nodes) omitted from report** — run `graphify query` to explore isolated nodes.

## Suggested Questions
_Questions this graph is uniquely positioned to answer:_

- **Why does `Entregables Requeridos` connect `Entregables Requeridos` to `Contexto_Proyecto.md`?**
  _High betweenness centrality (0.306) - this node is a cross-community bridge._