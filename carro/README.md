# Modelo CAD 3D - Kit Carro Robot Smart 2WD

Modelo CAD 3D completo y paramétrico generado a partir de los planos y especificaciones técnicas de las imágenes de [`D:\USC\Proyectos\Car_Control\carro`](file:///D:/USC/Proyectos/Car_Control/carro).

---

## 1. Referencias y Planos Analizados

1. **`modelo_carro.jpeg`**: Plano acotado de las piezas de acrílico cortadas por láser:
   - **Chasis Base**: Longitud total $178.03\text{ mm}$, ancho de cuerpo/frontal $96.08\text{ mm}$, ancho en aletas traseras $132.68\text{ mm}$.
   - **Placa Superior Trasera**: $114.17\text{ mm}$ de ancho $\times$ $84.88\text{ mm}$ de longitud.
   - **Placa Superior Delantera**: $97.97\text{ mm}$ de ancho $\times$ $49.73\text{ mm}$ de longitud.
   - **Soporte de Interruptor**: $19.31\text{ mm}$ de ancho $\times$ $43.87\text{ mm}$ de longitud.
   - **Soportes Verticales (2x)**: $21.46\text{ mm}$ de ancho $\times$ $30.22\text{ mm}$ de altura.
   - **Espesor de Lámina Acrílica**: $3.00\text{ mm}$.
2. **`Motores_carro.jpeg`**: Ficha técnica y dimensiones del motorreductor TT (TGP01D-A130):
   - Dimensiones: $69.2\text{ mm} \times 22.5\text{ mm} \times 18.8\text{ mm}$ (caja reductora $44.2\text{ mm} +$ motor DC metálico $\varnothing 20\text{ mm} \times 25.0\text{ mm}$).
   - Eje biaxial con doble plano: $\varnothing 5.4\text{ mm}$ exterior, $3.6\text{ mm}$ entre caras planas, sobresale $9.0\text{ mm}$ por lado (ancho total con eje $40.50\text{ mm}$).
3. **`ruedas_carro.jpeg`**: Ficha técnica de las ruedas de tracción:
   - Diámetro exterior total: $\varnothing 65.0\text{ mm}$.
   - Ancho de la llanta: $27.0\text{ mm}$.
   - Maza con acople para eje TT de doble plano ($5.0\text{ mm} \times 3.0\text{ mm}$).
   - Rin amarillo de 5 radios y neumático exterior de goma con relieve.
4. **`Componentes.jpeg` y `Carro_ lateral.jpeg`**:
   - Rueda loca frontal metálica de bola (Ball Caster tipo CY-15A, esfera $\varnothing 15.8\text{ mm}$, brida $50 \times 32\text{ mm}$, distancia entre agujeros $40\text{ mm}$).
   - 8 separadores hexagonales M3 de nylon (altura $25.0\text{ mm}$, ancho entre caras $5.5\text{ mm}$).

---

## 2. Árbol de Componentes del Ensamblaje 3D

El modelo cinemático y geométrico ubica todas las piezas en su cota de ensamble real:

- **Plano de Suelo ($Z = 0.00\text{ mm}$)**:
  - Las ruedas de tracción ($\varnothing 65\text{ mm}$) tocan el suelo en $Z = 0$, situando el eje de giro a $Z = 32.50\text{ mm}$.
  - La rueda loca frontal apoya su esfera rodante en el suelo a $Z = 0.00\text{ mm}$.
- **Tren de Tracción ($Z = 32.50\text{ mm}$)**:
  - Dos motores TT TGP01D-A130 fijados en los laterales inferiores del chasis.
  - Dos ruedas montadas concéntricamente a los ejes de los motores en $X = \pm 76.04\text{ mm}$.
- **Plataforma Inferior ($Z = 41.90\text{ a } 44.90\text{ mm}$)**:
  - Chasis base acrílico montado sobre los motores a $41.90\text{ mm}$ de despeje sobre el suelo.
  - Vaciado central pasacables, ranura trasera de batería, ranuras de fijación de motores y patrón de agujeros para Arduino Uno / Leonardo.
- **Columna de Separadores ($Z = 44.90\text{ a } 69.90\text{ mm}$)**:
  - 4 separadores delanteros M3 $\times 25\text{ mm}$.
  - 4 separadores traseros M3 $\times 25\text{ mm}$.
- **Plataforma Superior ($Z = 69.90\text{ mm}$)**:
  - Placa superior delantera con ventana de interruptor.
  - Marco soporte de interruptor basculante fijado en $Z = 72.90\text{ mm}$.
  - Placa superior trasera tipo alerón con calado ornamental.
  - Dos soportes verticales montados a $90^\circ$ en las ranuras del alerón ($Z = 72.90\text{ a } 100.12\text{ mm}$).

---

## 3. Tabla Metrológica de Control de Calidad

| Componente | Validez Geométrica | Dimensiones Medidas ($X \times Y \times Z$) | Dimensiones Plano ($X \times Y \times Z$) | Estado |
| :--- | :--- | :--- | :--- | :--- |
| **Chasis Base** | Válido (1 sólido) | $132.68 \times 178.03 \times 3.00\text{ mm}$ | $132.68 \times 178.03 \times 3.00\text{ mm}$ | **APROBADO** |
| **Placa Superior Trasera** | Válido (1 sólido) | $114.17 \times 84.88 \times 3.00\text{ mm}$ | $114.17 \times 84.88 \times 3.00\text{ mm}$ | **APROBADO** |
| **Placa Superior Delantera** | Válido (1 sólido) | $97.97 \times 49.73 \times 3.00\text{ mm}$ | $97.97 \times 49.73 \times 3.00\text{ mm}$ | **APROBADO** |
| **Soporte Interruptor** | Válido (1 sólido) | $19.31 \times 43.87 \times 3.00\text{ mm}$ | $19.31 \times 43.87 \times 3.00\text{ mm}$ | **APROBADO** |
| **Soportes Verticales (2x)** | Válido (1 sólido) | $21.46 \times 30.22 \times 3.00\text{ mm}$ | $21.46 \times 30.22 \times 3.00\text{ mm}$ | **APROBADO** |
| **Motorreductor TT (2x)** | Válido (1 sólido) | $69.20 \times 40.50 \times 20.00\text{ mm}$ | $69.20 \times 40.50 \times 20.00\text{ mm}$ | **APROBADO** |
| **Rueda de Tracción (2x)** | Válido (1 sólido) | $65.00 \times 65.00 \times 27.00\text{ mm}$ | $65.00 \times 65.00 \times 27.00\text{ mm}$ | **APROBADO** |
| **Rueda Loca (Ball Caster)** | Válido (1 sólido) | $50.00 \times 32.00 \times 16.70\text{ mm}$ | Esfera $\varnothing 15.8\text{ mm}$, brida $50 \times 32\text{ mm}$ | **APROBADO** |
| **Ensamblaje Completo STEP** | Válido (19 sólidos) | $185.08 \times 186.46 \times 100.12\text{ mm}$ | Altura $100.12\text{ mm}$, contacto suelo $Z = 0$ | **APROBADO** |

---

## 4. Archivos Entregados

- **Modelo Maestro FreeCAD**: [`carro_robot.FCStd`](file:///D:/USC/Proyectos/Car_Control/carro/carro_robot.FCStd)
- **Ensamblaje Completo STEP**: [`carro_robot.step`](file:///D:/USC/Proyectos/Car_Control/carro/carro_robot.step)
- **Piezas Individuales STEP (Corte Láser / Impresión 3D)**:
  - [`chasis_base.step`](file:///D:/USC/Proyectos/Car_Control/carro/chasis_base.step)
  - [`placa_superior_delantera.step`](file:///D:/USC/Proyectos/Car_Control/carro/placa_superior_delantera.step)
  - [`placa_superior_trasera.step`](file:///D:/USC/Proyectos/Car_Control/carro/placa_superior_trasera.step)
  - [`soporte_interruptor.step`](file:///D:/USC/Proyectos/Car_Control/carro/soporte_interruptor.step)
  - [`soporte_vertical.step`](file:///D:/USC/Proyectos/Car_Control/carro/soporte_vertical.step)
  - [`motor_tt.step`](file:///D:/USC/Proyectos/Car_Control/carro/motor_tt.step)
  - [`rueda.step`](file:///D:/USC/Proyectos/Car_Control/carro/rueda.step)
  - [`rueda_loca.step`](file:///D:/USC/Proyectos/Car_Control/carro/rueda_loca.step)
- **Script Generador Reproducible**: [`generar_modelo_carro.py`](file:///D:/USC/Proyectos/Car_Control/carro/generar_modelo_carro.py)
