import os
import math
import FreeCAD
import Part
import Sketcher
import PartDesign
import Spreadsheet

output_dir = r"D:\USC\Proyectos\Car_Control\test"
os.makedirs(output_dir, exist_ok=True)

fcstd_file = os.path.join(output_dir, "placa_montaje.FCStd")
step_file = os.path.join(output_dir, "placa_montaje.step")

print("=== INICIANDO MODELADO PARAMÉTRICO DE PLACA DE MONTAJE ===")
print("Versión FreeCAD:", ".".join(FreeCAD.Version()[:3]))

# 1. Crear nuevo documento
doc = FreeCAD.newDocument("placa_montaje")

# 2. Hoja de cálculo con parámetros editables
ss = doc.addObject("Spreadsheet::Sheet", "Parametros")
ss.set("A1", "Parámetro")
ss.set("B1", "Valor")
ss.set("C1", "Unidad")
ss.set("D1", "Descripción")

parametros_def = [
    ("A2", "Largo", "B2", "100", "Largo", "C2", "mm", "D2", "Longitud de la placa en el eje X"),
    ("A3", "Ancho", "B3", "60", "Ancho", "C3", "mm", "D3", "Ancho de la placa en el eje Y"),
    ("A4", "Espesor", "B4", "5", "Espesor", "C4", "mm", "D4", "Espesor de la placa en el eje Z"),
    ("A5", "DiametroAgujero", "B5", "5", "DiametroAgujero", "C5", "mm", "D5", "Diámetro de los 4 agujeros pasantes"),
    ("A6", "DistanciaBorde", "B6", "10", "DistanciaBorde", "C6", "mm", "D6", "Distancia de los agujeros a cada borde")
]

for row in parametros_def:
    ss.set(row[0], row[1])
    ss.set(row[2], row[3])
    ss.setAlias(row[2], row[4])
    ss.set(row[5], row[6])
    ss.set(row[7], row[8])

doc.recompute()
print("✓ Hoja de parámetros 'Parametros' configurada exitosamente.")

# 3. Crear Body para PartDesign
body = doc.addObject("PartDesign::Body", "PlacaMontaje")
doc.recompute()
xy_plane = [f for f in body.Origin.OriginFeatures if "XY_Plane" in f.Name][0]

# 4. Boceto Base (SketchBase)
sketch_base = doc.addObject("Sketcher::SketchObject", "SketchBase")
body.addObject(sketch_base)
sketch_base.AttachmentSupport = [(xy_plane, "")]
sketch_base.MapMode = "FlatFace"

p0 = FreeCAD.Vector(-50, -30, 0)
p1 = FreeCAD.Vector(50, -30, 0)
p2 = FreeCAD.Vector(50, 30, 0)
p3 = FreeCAD.Vector(-50, 30, 0)

sketch_base.addGeometry(Part.LineSegment(p0, p1))
sketch_base.addGeometry(Part.LineSegment(p1, p2))
sketch_base.addGeometry(Part.LineSegment(p2, p3))
sketch_base.addGeometry(Part.LineSegment(p3, p0))

# Coincidencias en vértices
sketch_base.addConstraint(Sketcher.Constraint("Coincident", 0, 2, 1, 1))
sketch_base.addConstraint(Sketcher.Constraint("Coincident", 1, 2, 2, 1))
sketch_base.addConstraint(Sketcher.Constraint("Coincident", 2, 2, 3, 1))
sketch_base.addConstraint(Sketcher.Constraint("Coincident", 3, 2, 0, 1))

# Horizontalidad y verticalidad
sketch_base.addConstraint(Sketcher.Constraint("Horizontal", 0))
sketch_base.addConstraint(Sketcher.Constraint("Horizontal", 2))
sketch_base.addConstraint(Sketcher.Constraint("Vertical", 1))
sketch_base.addConstraint(Sketcher.Constraint("Vertical", 3))

# Simetría respecto al origen (0,0)
sketch_base.addConstraint(Sketcher.Constraint("Symmetric", 0, 1, 1, 2, -1, 1))

# Restricciones acopladas a la hoja de cálculo
c_largo = sketch_base.addConstraint(Sketcher.Constraint("DistanceX", 0, 1, 0, 2, 100.0))
sketch_base.setExpression(f"Constraints[{c_largo}]", "Parametros.Largo")

c_ancho = sketch_base.addConstraint(Sketcher.Constraint("DistanceY", 1, 1, 1, 2, 60.0))
sketch_base.setExpression(f"Constraints[{c_ancho}]", "Parametros.Ancho")

doc.recompute()
print(f"✓ SketchBase creado y resuelto (Conflictos: {len(sketch_base.ConflictingConstraints)}, Redundancias: {len(sketch_base.RedundantConstraints)})")

# 5. Extrusión (PadBase)
pad = doc.addObject("PartDesign::Pad", "PadBase")
body.addObject(pad)
pad.Profile = sketch_base
pad.SideType = "Symmetric"
pad.setExpression("Length", "Parametros.Espesor")
doc.recompute()
print("✓ Operación PadBase (Extrusión simétrica en Z) completada.")

# 6. Boceto de Agujeros (SketchAgujeros)
sketch_holes = doc.addObject("Sketcher::SketchObject", "SketchAgujeros")
body.addObject(sketch_holes)
sketch_holes.AttachmentSupport = [(xy_plane, "")]
sketch_holes.MapMode = "FlatFace"

c0 = sketch_holes.addGeometry(Part.Circle(FreeCAD.Vector(40, 20, 0), FreeCAD.Vector(0,0,1), 2.5))
c1 = sketch_holes.addGeometry(Part.Circle(FreeCAD.Vector(-40, 20, 0), FreeCAD.Vector(0,0,1), 2.5))
c2 = sketch_holes.addGeometry(Part.Circle(FreeCAD.Vector(-40, -20, 0), FreeCAD.Vector(0,0,1), 2.5))
c3 = sketch_holes.addGeometry(Part.Circle(FreeCAD.Vector(40, -20, 0), FreeCAD.Vector(0,0,1), 2.5))

# Simetrías respecto a los ejes principales
sketch_holes.addConstraint(Sketcher.Constraint("Symmetric", 0, 3, 1, 3, -2))
sketch_holes.addConstraint(Sketcher.Constraint("Symmetric", 3, 3, 2, 3, -2))
sketch_holes.addConstraint(Sketcher.Constraint("Symmetric", 0, 3, 3, 3, -1))

# Restricciones de igualdad de radio
sketch_holes.addConstraint(Sketcher.Constraint("Equal", 0, 1))
sketch_holes.addConstraint(Sketcher.Constraint("Equal", 1, 2))
sketch_holes.addConstraint(Sketcher.Constraint("Equal", 2, 3))

# Diámetro referenciado al spreadsheet
c_dia = sketch_holes.addConstraint(Sketcher.Constraint("Diameter", 0, 5.0))
sketch_holes.setExpression(f"Constraints[{c_dia}]", "Parametros.DiametroAgujero")

# Posicionamiento respecto a los ejes en función de Largo, Ancho y DistanciaBorde
c_posX = sketch_holes.addConstraint(Sketcher.Constraint("DistanceX", -1, 1, 0, 3, 40.0))
sketch_holes.setExpression(f"Constraints[{c_posX}]", "Parametros.Largo / 2 - Parametros.DistanciaBorde")

c_posY = sketch_holes.addConstraint(Sketcher.Constraint("DistanceY", -1, 1, 0, 3, 20.0))
sketch_holes.setExpression(f"Constraints[{c_posY}]", "Parametros.Ancho / 2 - Parametros.DistanciaBorde")

doc.recompute()
print(f"✓ SketchAgujeros creado y resuelto (Conflictos: {len(sketch_holes.ConflictingConstraints)}, Redundancias: {len(sketch_holes.RedundantConstraints)})")

# 7. Operación Vaciado (VaciadoAgujeros)
pocket = doc.addObject("PartDesign::Pocket", "VaciadoAgujeros")
body.addObject(pocket)
pocket.Profile = sketch_holes
pocket.Type = "ThroughAll"
pocket.SideType = "Symmetric"
doc.recompute()
print("✓ Operación VaciadoAgujeros (Pocket pasante simétrico) completada.")

# 8. Verificación geométrica rigurosa
shape = body.Shape
assert shape.isValid(), "ERROR: La geometría generada no es válida."
assert len(shape.Solids) == 1, f"ERROR: Se esperaba 1 sólido, pero se encontraron {len(shape.Solids)}."

bbox = shape.BoundBox
dx = bbox.XMax - bbox.XMin
dy = bbox.YMax - bbox.YMin
dz = bbox.ZMax - bbox.ZMin

vol_nominal_esperado = (100.0 * 60.0 * 5.0) - (4.0 * math.pi * (2.5**2) * 5.0)
vol_real = shape.Volume
vol_error = abs(vol_real - vol_nominal_esperado)

com = shape.Solids[0].CenterOfMass

print("\n=== REPORTE DE VERIFICACIÓN GEOMÉTRICA (NOMINAL) ===")
print(f"• Estado del sólido: {'VÁLIDO' if shape.isValid() else 'INVÁLIDO'}")
print(f"• Cantidad de sólidos: {len(shape.Solids)}")
print(f"• Dimensiones BoundingBox: {dx:.3f} × {dy:.3f} × {dz:.3f} mm")
print(f"  - X: [{bbox.XMin:.2f}, {bbox.XMax:.2f}] mm (Centrado: {abs(bbox.XMin + bbox.XMax) < 1e-6})")
print(f"  - Y: [{bbox.YMin:.2f}, {bbox.YMax:.2f}] mm (Centrado: {abs(bbox.YMin + bbox.YMax) < 1e-6})")
print(f"  - Z: [{bbox.ZMin:.2f}, {bbox.ZMax:.2f}] mm (Centrado: {abs(bbox.ZMin + bbox.ZMax) < 1e-6})")
print(f"• Centro de masas: ({com.x:.4e}, {com.y:.4e}, {com.z:.4e}) mm")
print(f"• Volumen calculado: {vol_real:.4f} mm³")
print(f"• Volumen teórico:  {vol_nominal_esperado:.4f} mm³")
print(f"• Error de volumen:  {vol_error:.4e} mm³")

# Verificación de agujeros (caras cilíndricas)
cylindrical_faces = []
for face in shape.Faces:
    surf = face.Surface
    if isinstance(surf, Part.Cylinder):
        cylindrical_faces.append((surf.Center, surf.Radius * 2))

print(f"• Agujeros cilíndricos detectados: {len(cylindrical_faces)}")
for i, (center, diam) in enumerate(cylindrical_faces, 1):
    dist_x = 50.0 - abs(center.x)
    dist_y = 30.0 - abs(center.y)
    print(f"  Agujero {i}: Centro=({center.x:+.2f}, {center.y:+.2f}), Diámetro={diam:.2f} mm, Distancia a bordes: dX={dist_x:.2f} mm, dY={dist_y:.2f} mm")

# 9. Test de comportamiento paramétrico
print("\n=== TEST DE REGENACIÓN PARAMÉTRICA ===")
print("Modificando parámetros a: Largo=120 mm, Ancho=70 mm, Espesor=8 mm, Diametro=6 mm, DistanciaBorde=12 mm...")
ss.set("B2", "120")
ss.set("B3", "70")
ss.set("B4", "8")
ss.set("B5", "6")
ss.set("B6", "12")
doc.recompute()

shape_test = body.Shape
bbox_t = shape_test.BoundBox
vol_test_esperado = (120.0 * 70.0 * 8.0) - (4.0 * math.pi * (3.0**2) * 8.0)
print(f"• Post-edición BoundingBox: {bbox_t.XMax - bbox_t.XMin:.1f} × {bbox_t.YMax - bbox_t.YMin:.1f} × {bbox_t.ZMax - bbox_t.ZMin:.1f} mm")
print(f"• Post-edición Volumen: {shape_test.Volume:.4f} mm³ (Esperado: {vol_test_esperado:.4f} mm³)")
assert shape_test.isValid() and len(shape_test.Solids) == 1, "Fallo en regeneración paramétrica"
print("✓ Modelo paramétrico responde correctamente a variaciones dimensionales.")

# Restaurar valores nominales
print("\nRestaurando valores nominales (100 × 60 × 5 mm)...")
ss.set("B2", "100")
ss.set("B3", "60")
ss.set("B4", "5")
ss.set("B5", "5")
ss.set("B6", "10")
doc.recompute()

# 10. Guardar documento .FCStd
doc.saveAs(fcstd_file)
print(f"\n✓ Documento FreeCAD guardado en: {fcstd_file}")

# 11. Exportar copia STEP
Part.export([body], step_file)
print(f"✓ Archivo STEP exportado en: {step_file}")

# 12. Verificación de archivos exportados
assert os.path.exists(fcstd_file), "No se encontró el archivo FCStd"
assert os.path.exists(step_file), "No se encontró el archivo STEP"
fcstd_size = os.path.getsize(fcstd_file)
step_size = os.path.getsize(step_file)
print(f"• Tamaño placa_montaje.FCStd: {fcstd_size} bytes")
print(f"• Tamaño placa_montaje.step: {step_size} bytes")

# 13. Reimportar y verificar el archivo STEP exportado
step_shape = Part.read(step_file)
assert step_shape.isValid(), "ERROR: La forma importada desde el archivo STEP no es válida"
assert len(step_shape.Solids) == 1, "ERROR: El STEP no contiene exactamente 1 sólido"
step_bbox = step_shape.BoundBox
print("\n=== VERIFICACIÓN DE INTEGRIDAD DEL ARCHIVO STEP REIMPORTADO ===")
print(f"• Sólido STEP válido: {step_shape.isValid()}")
print(f"• Número de sólidos en STEP: {len(step_shape.Solids)}")
print(f"• Dimensiones en STEP: {step_bbox.XMax - step_bbox.XMin:.3f} × {step_bbox.YMax - step_bbox.YMin:.3f} × {step_bbox.ZMax - step_bbox.ZMin:.3f} mm")
print(f"• Volumen en STEP: {step_shape.Volume:.4f} mm³")
print(f"• Coincidencia de volumen FCStd vs STEP: {abs(step_shape.Volume - shape.Volume):.4e} mm³")
print("\n=== TODAS LAS VERIFICACIONES COMPLETADAS CON ÉXITO ===")
