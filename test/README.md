# Placa de Montaje Paramétrica - FreeCAD

Modelo paramétrico de placa de montaje diseñado utilizando operaciones nativas de FreeCAD (PartDesign y Spreadsheet).

## 1. Especificaciones de Diseño
- **Dimensiones Nominales**: $100 \times 60 \times 5\text{ mm}$ (Largo $\times$ Ancho $\times$ Espesor).
- **Sistema de Coordenadas**: Centrado en el origen tridimensional $(0, 0, 0)$.
  - Rango en X: $[-50.00, +50.00]\text{ mm}$
  - Rango en Y: $[-30.00, +30.00]\text{ mm}$
  - Rango en Z: $[-2.50, +2.50]\text{ mm}$
  - Centro de masas: $(0.00, 0.00, 0.00)\text{ mm}$
- **Agujeros de Montaje**: 4 agujeros pasantes con diámetro $\varnothing 5.00\text{ mm}$ (radio $2.50\text{ mm}$).
- **Posición de los Agujeros**: Ubicados a $10.00\text{ mm}$ de cada borde exterior.
  - Coordenadas de los centros: $(\pm 40.00, \pm 20.00, 0.00)\text{ mm}$.

## 2. Estructura del Árbol de Operaciones Nativas
- **`Parametros` (`Spreadsheet::Sheet`)**: Hoja de cálculo con celdas y alias editables:
  - `Largo`: Longitud en el eje X ($100\text{ mm}$).
  - `Ancho`: Ancho en el eje Y ($60\text{ mm}$).
  - `Espesor`: Grosor de la placa en el eje Z ($5\text{ mm}$).
  - `DiametroAgujero`: Diámetro de los agujeros pasantes ($5\text{ mm}$).
  - `DistanciaBorde`: Separación desde cada borde exterior al centro del agujero ($10\text{ mm}$).
- **`PlacaMontaje` (`PartDesign::Body`)**:
  - **`SketchBase`**: Rectángulo con simetría de esquinas opuestas respecto al origen $(0,0)$. Restricciones dimensionales asociadas a `=Parametros.Largo` y `=Parametros.Ancho`.
  - **`PadBase`**: Extrusión simétrica (`SideType: 'Symmetric'`) con longitud vinculada a `=Parametros.Espesor`.
  - **`SketchAgujeros`**: Cuatro circunferencias con simetrías respecto a los planos principales X e Y, igualdad de radios, diámetro vinculado a `=Parametros.DiametroAgujero` y distancias respecto al origen fijadas dinámicamente mediante las expresiones `=Parametros.Largo / 2 - Parametros.DistanciaBorde` y `=Parametros.Ancho / 2 - Parametros.DistanciaBorde`.
  - **`VaciadoAgujeros`**: Operación Pocket tipo `ThroughAll` simétrica (`SideType: 'Symmetric'`), garantizando un corte pasante limpio para cualquier espesor.

## 3. Resultados de Verificación
| Parámetro | Valor Teórico | Valor Obtenido | Discrepancia |
| :--- | :--- | :--- | :--- |
| **Validez Geométrica** | Válido (`Shape.isValid()`) | `True` | 0 |
| **Conteo de Sólidos** | 1 sólido cerrado | 1 | 0 |
| **Dimensiones BoundingBox** | $100.00 \times 60.00 \times 5.00\text{ mm}$ | $100.00 \times 60.00 \times 5.00\text{ mm}$ | $0.00\text{ mm}$ |
| **Volumen** | $29607.3009\text{ mm}^3$ | $29607.3009\text{ mm}^3$ | $< 10^{-11}\text{ mm}^3$ |
| **Centro de Masa** | $(0.00, 0.00, 0.00)\text{ mm}$ | $(-3.07\times 10^{-16}, 0.00, 4.86\times 10^{-17})\text{ mm}$ | $< 10^{-15}\text{ mm}$ |
| **Verificación STEP** | Sólido idéntico | Válido (1 sólido, $29607.3009\text{ mm}^3$) | $0.00\text{ mm}^3$ |

## 4. Archivos Generados
- `placa_montaje.FCStd`: Documento nativo de FreeCAD con árbol de operaciones y hoja de parámetros editables.
- `placa_montaje.step`: Copia de exportación en formato estándar neutro STEP (AP214/AP203).
- `generar_placa.py`: Script reproducible en Python ejecutado en el entorno de FreeCAD.
