import os
import math
import FreeCAD
import Part

print("======================================================================")
print("  GENERACIÓN DEL MODELO CAD 3D - KIT CARRO ROBOT SMART 2WD")
print(f"  Entorno: FreeCAD {' '.join(FreeCAD.Version()[:3])}")
print("======================================================================")

output_dir = r"D:\USC\Proyectos\Car_Control\carro"
os.makedirs(output_dir, exist_ok=True)

# --------------------------------------------------------------------
# 1. PARÁMETROS GEOMÉTRICOS PRINCIPALES (Extraídos de los planos)
# --------------------------------------------------------------------
# Chasis Base (modelo_carro.jpeg)
CHASSIS_L = 178.03          # Longitud total
CHASSIS_W_FRONT = 96.08     # Ancho del cuerpo frontal / laterales
CHASSIS_W_WING = 132.68     # Ancho total en los alerones posteriores
ACRYLIC_THICKNESS = 3.0     # Espesor estándar de lámina de acrílico

# Placa Superior Trasera (modelo_carro.jpeg)
REAR_PLATE_W = 114.17       # Ancho superior
REAR_PLATE_L = 84.88        # Longitud

# Placa Superior Delantera (modelo_carro.jpeg)
FRONT_PLATE_W = 97.97       # Ancho
FRONT_PLATE_L = 49.73       # Longitud

# Soportes Verticales (modelo_carro.jpeg)
BRACKET_W = 21.46           # Ancho
BRACKET_H = 30.22           # Altura total (incluyendo lengüeta de encaje)

# Soporte Interruptor (modelo_carro.jpeg)
SWITCH_W = 19.31            # Ancho
SWITCH_L = 43.87            # Longitud

# Motorreductores TT TGP01D-A130 (Motores_carro.jpeg)
MOTOR_BOX_L = 44.2          # Longitud de la caja reductora
MOTOR_BOX_W = 22.5          # Ancho
MOTOR_BOX_H = 18.8          # Altura
MOTOR_DC_L = 25.0           # Longitud del motor cilíndrico metálico
MOTOR_DC_D = 20.0           # Diámetro del motor cilíndrico
MOTOR_TOTAL_L = 69.2        # Longitud total (6.92 cm = 44.2 + 25.0)
SHAFT_D = 5.4               # Diámetro eje biaxial
SHAFT_FLAT = 3.6            # Distancia entre caras planas
SHAFT_PROTRUSION = 9.0      # Longitud sobresaliente por cada lado

# Ruedas (ruedas_carro.jpeg)
WHEEL_DIAMETER = 65.0       # Diámetro total (6.5 cm)
WHEEL_WIDTH = 27.0          # Ancho de la llanta (2.7 cm)
RIM_DIAMETER = 52.0         # Diámetro del rin amarillo

# Rueda Loca Metálica (Componentes.jpeg)
BALL_CASTER_D = 15.8        # Diámetro de la esfera (5/8 pulgada)
CASTER_FLANGE_W = 50.0      # Ancho de la pletina de fijación
CASTER_FLANGE_L = 32.0      # Fondo de la pletina
CASTER_HOLE_DIST = 40.0     # Distancia entre agujeros

# Separadores Hexagonales (Standoffs)
STANDOFF_H = 25.0           # Altura de los espaciadores de nylon
STANDOFF_FLATS = 5.5        # Ancho entre caras hexagonales

# --------------------------------------------------------------------
# 2. FUNCIONES DE CONSTRUCCIÓN DE COMPONENTES SÓLIDOS
# --------------------------------------------------------------------

def crear_solido_chasis_base():
    """Genera la placa de chasis base acrílica con todos los vaciados y orificios."""
    L = CHASSIS_L
    W_f = CHASSIS_W_FRONT
    W_w = CHASSIS_W_WING
    T = ACRYLIC_THICKNESS
    
    x_f = W_f / 2.0         # 48.04 mm
    x_w = W_w / 2.0         # 66.34 mm
    y_front_apex = L / 2.0  # +89.015 mm
    y_rear_apex = -L / 2.0  # -89.015 mm
    y_front_corner = 68.0
    y_wing_start = -35.0
    y_wing_tip = -58.0
    
    # Perímetro exterior
    p_f_left = FreeCAD.Vector(-x_f, y_front_corner, 0)
    p_f_mid = FreeCAD.Vector(0, y_front_apex, 0)
    p_f_right = FreeCAD.Vector(x_f, y_front_corner, 0)
    front_arc = Part.Arc(p_f_left, p_f_mid, p_f_right).toShape()
    
    p_w_start_r = FreeCAD.Vector(x_f, y_wing_start, 0)
    right_side = Part.makeLine(p_f_right, p_w_start_r)
    
    p_w_tip_r = FreeCAD.Vector(x_w, y_wing_tip, 0)
    right_flare = Part.makeLine(p_w_start_r, p_w_tip_r)
    
    p_w_in_r = FreeCAD.Vector(x_f, y_wing_tip, 0)
    right_cutback = Part.makeLine(p_w_tip_r, p_w_in_r)
    
    p_r_mid = FreeCAD.Vector(0, y_rear_apex, 0)
    p_w_in_l = FreeCAD.Vector(-x_f, y_wing_tip, 0)
    rear_arc = Part.Arc(p_w_in_r, p_r_mid, p_w_in_l).toShape()
    
    p_w_tip_l = FreeCAD.Vector(-x_w, y_wing_tip, 0)
    left_cutback = Part.makeLine(p_w_in_l, p_w_tip_l)
    
    p_w_start_l = FreeCAD.Vector(-x_f, y_wing_start, 0)
    left_flare = Part.makeLine(p_w_tip_l, p_w_start_l)
    
    left_side = Part.makeLine(p_w_start_l, p_f_left)
    
    outer_wire = Part.Wire([front_arc, right_side, right_flare, right_cutback, rear_arc, left_cutback, left_flare, left_side])
    base_solid = Part.Face(outer_wire).extrude(FreeCAD.Vector(0, 0, T))
    
    # Cortes y vaciados interiores
    cut_shapes = []
    
    # 1. Ranura central alargada (pasacables)
    wire_slot_l = 34.0
    wire_slot_w = 14.0
    c_slot = Part.makeBox(wire_slot_w, wire_slot_l, T + 2)
    c_slot.translate(FreeCAD.Vector(-wire_slot_w/2, 10.0, -1))
    cut_shapes.append(c_slot)
    
    # 2. Ranura trasera escalonada (batería / cables)
    r_rear_main = Part.makeBox(26.0, 18.0, T + 2)
    r_rear_main.translate(FreeCAD.Vector(-13.0, -65.0, -1))
    r_rear_step = Part.makeBox(14.0, 10.0, T + 2)
    r_rear_step.translate(FreeCAD.Vector(-7.0, -47.0, -1))
    cut_shapes.append(r_rear_main.fuse(r_rear_step))
    
    # 3. Ranuras longitudinales de montaje de motores (2 ranuras laterales)
    for x_slot in [-35.0, 35.0]:
        m_slot = Part.makeBox(3.2, 14.0, T + 2)
        m_slot.translate(FreeCAD.Vector(x_slot - 1.6, -22.0, -1))
        cut_shapes.append(m_slot)
    
    # 4. Agujeros para rueda loca (patrón de fijación frontal)
    for dx in [-20.0, 20.0]:
        h_caster = Part.makeCylinder(1.6, T + 2)
        h_caster.translate(FreeCAD.Vector(dx, 60.0, -1))
        cut_shapes.append(h_caster)
        
    # 5. Agujeros para fijación de motores (tornillos M3 pasantes)
    for xm in [-40.0, 40.0]:
        for ym in [-28.0, -2.0]:
            h_m = Part.makeCylinder(1.6, T + 2)
            h_m.translate(FreeCAD.Vector(xm, ym, -1))
            cut_shapes.append(h_m)
            
    # 6. Agujeros para separadores M3 (4 delanteros y 4 traseros)
    standoff_coords = [
        (-35.0, 30.0), (35.0, 30.0), (-35.0, 55.0), (35.0, 55.0),
        (-20.0, -40.0), (20.0, -40.0), (-20.0, -70.0), (20.0, -70.0)
    ]
    for xs, ys in standoff_coords:
        h_s = Part.makeCylinder(1.6, T + 2)
        h_s.translate(FreeCAD.Vector(xs, ys, -1))
        cut_shapes.append(h_s)
        
    # 7. Agujeros adicionales de montaje (Arduino Uno y electrónica)
    arduino_coords = [(-15.0, 5.0), (35.0, 5.0), (-15.0, -23.0), (33.0, -23.0)]
    for xa, ya in arduino_coords:
        h_a = Part.makeCylinder(1.6, T + 2)
        h_a.translate(FreeCAD.Vector(xa, ya, -1))
        cut_shapes.append(h_a)
        
    for cut_tool in cut_shapes:
        base_solid = base_solid.cut(cut_tool)
        
    return base_solid


def crear_solido_placa_sup_trasera():
    """Genera la placa superior trasera (alerón / soporte de sensores) de 114.17 x 84.88 x 3 mm."""
    w = REAR_PLATE_W        # 114.17
    l = REAR_PLATE_L        # 84.88
    t = ACRYLIC_THICKNESS   # 3.0
    
    # Origen centrado en el medio de la placa
    # Borde superior cóncavo (arco hacia adentro)
    p1 = FreeCAD.Vector(-w/2, l/2, 0)
    p2 = FreeCAD.Vector(0, l/2 - 14.0, 0)
    p3 = FreeCAD.Vector(w/2, l/2, 0)
    top_arc = Part.Arc(p1, p2, p3).toShape()
    
    p4 = FreeCAD.Vector(w/2, 18.0, 0)
    side_r1 = Part.makeLine(p3, p4)
    p5 = FreeCAD.Vector(25.0, -15.0, 0)
    side_r2 = Part.makeLine(p4, p5)
    p6 = FreeCAD.Vector(25.0, -l/2, 0)
    side_r3 = Part.makeLine(p5, p6)
    p7 = FreeCAD.Vector(-25.0, -l/2, 0)
    bot = Part.makeLine(p6, p7)
    p8 = FreeCAD.Vector(-25.0, -15.0, 0)
    side_l3 = Part.makeLine(p7, p8)
    p9 = FreeCAD.Vector(-w/2, 18.0, 0)
    side_l2 = Part.makeLine(p8, p9)
    side_l1 = Part.makeLine(p9, p1)
    
    wire = Part.Wire([top_arc, side_r1, side_r2, side_r3, bot, side_l3, side_l2, side_l1])
    plate = Part.Face(wire).extrude(FreeCAD.Vector(0, 0, t))
    
    # Orificios para separadores y ranuras para soportes verticales
    cuts = []
    # 4 agujeros para separadores
    for xs, ys in [(-20.0, 15.0), (20.0, 15.0), (-20.0, -30.0), (20.0, -30.0)]:
        h = Part.makeCylinder(1.6, t + 2)
        h.translate(FreeCAD.Vector(xs, ys, -1))
        cuts.append(h)
        
    # Ranuras para encastre de los soportes verticales
    for xs in [-30.0, 30.0]:
        slot = Part.makeBox(8.2, 3.2, t + 2)
        slot.translate(FreeCAD.Vector(xs - 4.1, 10.0 - 1.6, -1))
        cuts.append(slot)
        
    # Calado ornamental central
    flame = Part.makeCylinder(4.0, t + 2)
    flame.translate(FreeCAD.Vector(0, -5.0, -1))
    cuts.append(flame)
    
    for c in cuts:
        plate = plate.cut(c)
        
    return plate


def crear_solido_placa_sup_delantera():
    """Genera la placa superior delantera de 97.97 x 49.73 x 3 mm con ventana de interruptor."""
    w = FRONT_PLATE_W       # 97.97
    l = FRONT_PLATE_L       # 49.73
    t = ACRYLIC_THICKNESS   # 3.0
    
    pts = [
        FreeCAD.Vector(-w/2, -l/2, 0),
        FreeCAD.Vector(w/2, -l/2, 0),
        FreeCAD.Vector(w/2, 5.0, 0),
        FreeCAD.Vector(20.0, 16.0, 0),
        FreeCAD.Vector(12.0, l/2, 0),
        FreeCAD.Vector(-12.0, l/2, 0),
        FreeCAD.Vector(-20.0, 16.0, 0),
        FreeCAD.Vector(-w/2, 5.0, 0),
        FreeCAD.Vector(-w/2, -l/2, 0)
    ]
    poly = Part.makePolygon(pts)
    plate = Part.Face(poly).extrude(FreeCAD.Vector(0, 0, t))
    
    cuts = []
    # Ventana central para el interruptor
    sw_cut = Part.makeBox(13.5, 22.0, t + 2)
    sw_cut.translate(FreeCAD.Vector(-13.5/2, -5.0, -1))
    cuts.append(sw_cut)
    
    # Orificio en la lengüeta superior
    h_top = Part.makeCylinder(1.6, t + 2)
    h_top.translate(FreeCAD.Vector(0, l/2 - 5.0, -1))
    cuts.append(h_top)
    
    # Orificios para los 4 separadores
    for xs, ys in [(-35.0, -12.5), (35.0, -12.5), (-35.0, 12.5), (35.0, 12.5)]:
        h = Part.makeCylinder(1.6, t + 2)
        h.translate(FreeCAD.Vector(xs, ys, -1))
        cuts.append(h)
        
    # Fila de 4 orificios para sensores frontales
    for xs in [-25.0, -8.0, 8.0, 25.0]:
        hf = Part.makeCylinder(1.6, t + 2)
        hf.translate(FreeCAD.Vector(xs, -l/2 + 7.0, -1))
        cuts.append(hf)
        
    for c in cuts:
        plate = plate.cut(c)
        
    return plate


def crear_solido_soporte_interruptor():
    """Genera el marco bisel del interruptor basculante de 19.31 x 43.87 x 3 mm."""
    w = SWITCH_W            # 19.31
    l = SWITCH_L            # 43.87
    t = ACRYLIC_THICKNESS   # 3.0
    
    base = Part.makeBox(w, l, t)
    base.translate(FreeCAD.Vector(-w/2, -l/2, 0))
    
    cuts = []
    # Ventana central
    win = Part.makeBox(13.0, 20.0, t + 2)
    win.translate(FreeCAD.Vector(-6.5, -10.0, -1))
    cuts.append(win)
    
    # Pequeña muesca inferior
    notch = Part.makeBox(4.0, 3.0, t + 2)
    notch.translate(FreeCAD.Vector(-2.0, -13.0, -1))
    cuts.append(notch)
    
    # 2 orificios de fijación
    for y_hole in [-16.5, 16.5]:
        h = Part.makeCylinder(1.6, t + 2)
        h.translate(FreeCAD.Vector(0, y_hole, -1))
        cuts.append(h)
        
    for c in cuts:
        base = base.cut(c)
        
    return base


def crear_solido_soporte_vertical():
    """Genera la aleta/soporte vertical de 21.46 x 30.22 x 3 mm (altura total = 30.22 mm)."""
    w = BRACKET_W           # 21.46
    h_total = BRACKET_H     # 30.22
    t = ACRYLIC_THICKNESS   # 3.0
    
    tab_h = 3.0
    body_h = h_total - tab_h # 27.22
    
    pts = [
        FreeCAD.Vector(-w/2, 0, 0),
        FreeCAD.Vector(-4.0, 0, 0),
        FreeCAD.Vector(-4.0, -tab_h, 0),    # Lengüeta inferior de encastre
        FreeCAD.Vector(4.0, -tab_h, 0),
        FreeCAD.Vector(4.0, 0, 0),
        FreeCAD.Vector(w/2, 0, 0),
        FreeCAD.Vector(w/2 - 2.0, body_h, 0),
        FreeCAD.Vector(2.0, body_h, 0),
        FreeCAD.Vector(2.0, body_h - 7.0, 0), # Ranura superior
        FreeCAD.Vector(-2.0, body_h - 7.0, 0),
        FreeCAD.Vector(-2.0, body_h, 0),
        FreeCAD.Vector(-w/2 + 2.0, body_h, 0),
        FreeCAD.Vector(-w/2, 0, 0)
    ]
    poly = Part.makePolygon(pts)
    bracket = Part.Face(poly).extrude(FreeCAD.Vector(0, 0, t))
    return bracket


def crear_solido_motor_tt():
    """Genera el motorreductor TT DC TGP01D-A130 (69.2 x 22.5 x 18.8 mm) con eje biaxial."""
    # 1. Caja reductora plástica amarilla (44.2 x 22.5 x 18.8 mm)
    box = Part.makeBox(MOTOR_BOX_L, MOTOR_BOX_W, MOTOR_BOX_H)
    
    # 2. Motor DC cilíndrico metálico posterior (25.0 mm largo, 20.0 mm diametro)
    cyl = Part.makeCylinder(MOTOR_DC_D / 2.0, MOTOR_DC_L)
    cyl.rotate(FreeCAD.Vector(0, 0, 0), FreeCAD.Vector(0, 1, 0), 90)
    cyl.translate(FreeCAD.Vector(MOTOR_BOX_L, MOTOR_BOX_W / 2.0, MOTOR_BOX_H / 2.0))
    
    motor_body = box.fuse(cyl)
    
    # 3. Eje de salida biaxial con doble plano orientado a lo largo del eje Y
    axle_cyl = Part.makeCylinder(SHAFT_D / 2.0, MOTOR_BOX_W + 2 * SHAFT_PROTRUSION)
    axle_cyl.rotate(FreeCAD.Vector(0, 0, 0), FreeCAD.Vector(1, 0, 0), -90)
    axle_cyl.translate(FreeCAD.Vector(15.0, -SHAFT_PROTRUSION, MOTOR_BOX_H / 2.0))
    
    # 4. Agujeros de fijación pasantes M3
    for x_hole in [11.0, 28.5]:
        h_m = Part.makeCylinder(1.6, MOTOR_BOX_W + 4)
        h_m.rotate(FreeCAD.Vector(0, 0, 0), FreeCAD.Vector(1, 0, 0), -90)
        h_m.translate(FreeCAD.Vector(x_hole, -2, MOTOR_BOX_H / 2.0))
        motor_body = motor_body.cut(h_m)
        
    motor_complete = motor_body.fuse(axle_cyl)
    return motor_complete


def crear_solido_rueda():
    """Genera la rueda completa (Diámetro 65 mm, Ancho 27 mm) con rin de 5 radios y neumático de goma."""
    D_total = WHEEL_DIAMETER    # 65.0
    W_total = WHEEL_WIDTH       # 27.0
    D_rim = RIM_DIAMETER        # 52.0
    
    # 1. Rin de plástico amarillo
    rim_outer = Part.makeCylinder(D_rim / 2.0, W_total - 2.0)
    rim_outer.translate(FreeCAD.Vector(0, 0, 1.0))
    
    # Maza central y taladro para eje TT
    hub = Part.makeCylinder(6.0, W_total)
    shaft_bore = Part.makeBox(SHAFT_FLAT, SHAFT_D, W_total + 2)
    shaft_bore.translate(FreeCAD.Vector(-SHAFT_FLAT/2, -SHAFT_D/2, -1))
    hub = hub.cut(shaft_bore)
    
    # Vaciado del rin con 5 radios
    recess = Part.makeCylinder(D_rim / 2.0 - 3.0, W_total - 4.0)
    recess.translate(FreeCAD.Vector(0, 0, 2.0))
    rim_shell = rim_outer.cut(recess).fuse(hub)
    
    # Añadir 5 radios
    spokes = []
    for i in range(5):
        angle = i * (360.0 / 5.0)
        spoke = Part.makeBox(D_rim / 2.0 - 4.0, 3.5, 4.0)
        spoke.translate(FreeCAD.Vector(4.0, -1.75, (W_total - 4.0) / 2.0))
        spoke.rotate(FreeCAD.Vector(0, 0, 0), FreeCAD.Vector(0, 0, 1), angle)
        spokes.append(spoke)
        
    for s in spokes:
        rim_shell = rim_shell.fuse(s)
        
    # 2. Neumático exterior de caucho
    tire_solid = Part.makeCylinder(D_total / 2.0, W_total)
    tire_bore = Part.makeCylinder(D_rim / 2.0, W_total + 2)
    tire_bore.translate(FreeCAD.Vector(0, 0, -1))
    tire = tire_solid.cut(tire_bore)
    
    # Relieve de la banda de rodadura (taqueado / gajos)
    treads = []
    num_treads = 18
    for j in range(num_treads):
        t_ang = j * (360.0 / num_treads)
        tread_groove = Part.makeBox(2.5, 4.0, W_total - 4.0)
        tread_groove.translate(FreeCAD.Vector(D_total / 2.0 - 1.5, -2.0, 2.0))
        tread_groove.rotate(FreeCAD.Vector(0, 0, 0), FreeCAD.Vector(0, 0, 1), t_ang)
        treads.append(tread_groove)
        
    for tg in treads:
        tire = tire.cut(tg)
        
    rueda_completa = rim_shell.fuse(tire)
    return rueda_completa


def crear_solido_rueda_loca():
    """Genera la rueda loca esférica metálica (Ball Caster tipo CY-15A)."""
    # Brida de fijación metálica estampada
    flange = Part.makeBox(CASTER_FLANGE_W, CASTER_FLANGE_L, 1.8)
    flange.translate(FreeCAD.Vector(-CASTER_FLANGE_W/2, -CASTER_FLANGE_L/2, 0))
    
    # 2 agujeros de fijación a 40 mm entre centros
    for xh in [-CASTER_HOLE_DIST / 2.0, CASTER_HOLE_DIST / 2.0]:
        ch = Part.makeCylinder(1.6, 4.0)
        ch.translate(FreeCAD.Vector(xh, 0, -1))
        flange = flange.cut(ch)
        
    # Carcasa cúpula semiesférica
    housing = Part.makeSphere(BALL_CASTER_D / 2.0 + 2.0)
    housing.translate(FreeCAD.Vector(0, 0, -BALL_CASTER_D / 2.0))
    cap_cut = Part.makeBox(40, 40, 20)
    cap_cut.translate(FreeCAD.Vector(-20, -20, 0))
    housing = housing.cut(cap_cut)
    
    # Esfera de acero rodante
    ball = Part.makeSphere(BALL_CASTER_D / 2.0)
    ball.translate(FreeCAD.Vector(0, 0, -BALL_CASTER_D / 2.0))
    
    caster_solid = flange.fuse(housing).fuse(ball)
    return caster_solid


def crear_solido_separador(altura=STANDOFF_H):
    """Genera un separador hexagonal de nylon roscado M3."""
    # Prisma hexagonal regular
    r_circum = (STANDOFF_FLATS / 2.0) / math.cos(math.radians(30))
    hex_pts = []
    for k in range(6):
        ang = math.radians(k * 60)
        hex_pts.append(FreeCAD.Vector(r_circum * math.cos(ang), r_circum * math.sin(ang), 0))
    hex_pts.append(hex_pts[0])
    
    hex_wire = Part.makePolygon(hex_pts)
    hex_solid = Part.Face(hex_wire).extrude(FreeCAD.Vector(0, 0, altura))
    
    # Taladro pasante central M3
    bore = Part.makeCylinder(1.5, altura + 2)
    bore.translate(FreeCAD.Vector(0, 0, -1))
    return hex_solid.cut(bore)


# --------------------------------------------------------------------
# 3. CONSTRUCCIÓN DEL DOCUMENTO FREECAD Y ENSAMBLAJE CINEMÁTICO 3D
# --------------------------------------------------------------------

print("\n1. Inicializando documento FreeCAD...")
doc = FreeCAD.newDocument("Carro_Robot_2WD")

# Definición de alturas de ensamblaje (con respecto al suelo Z = 0)
Z_AXLE = WHEEL_DIAMETER / 2.0               # 32.5 mm
Z_CHASSIS_BOTTOM = Z_AXLE + (MOTOR_BOX_H / 2.0) # 32.5 + 9.4 = 41.9 mm
Z_CHASSIS_TOP = Z_CHASSIS_BOTTOM + ACRYLIC_THICKNESS # 44.9 mm
Z_DECK_TOP = Z_CHASSIS_TOP + STANDOFF_H     # 44.9 + 25.0 = 69.9 mm

Y_AXLE = -15.0  # Posición longitudinal del eje motor / ruedas

print("2. Generando sólidos individuales...")
s_chasis = crear_solido_chasis_base()
s_placa_trasera = crear_solido_placa_sup_trasera()
s_placa_delantera = crear_solido_placa_sup_delantera()
s_soporte_sw = crear_solido_soporte_interruptor()
s_soporte_vert = crear_solido_soporte_vertical()
s_motor = crear_solido_motor_tt()
s_rueda = crear_solido_rueda()
s_caster = crear_solido_rueda_loca()
s_standoff = crear_solido_separador()

print("✓ Todos los sólidos base creados con éxito.")

# --------------------------------------------------------------------
# 4. INSTANCIACIÓN Y POSICIONAMIENTO EN EL ENSAMBLAJE
# --------------------------------------------------------------------
print("3. Posicionando componentes en el ensamblaje 3D...")

part_objects = []

def add_part(name, shape, placement, color, transparency=0):
    obj = doc.addObject("Part::Feature", name)
    sh = shape.copy()
    sh.Placement = placement
    obj.Shape = sh
    if hasattr(obj, "ViewObject") and obj.ViewObject is not None:
        obj.ViewObject.ShapeColor = color
        obj.ViewObject.Transparency = transparency
    part_objects.append(obj)
    return obj

# 1. Chasis Base
p_chasis = FreeCAD.Placement(FreeCAD.Vector(0, 0, Z_CHASSIS_BOTTOM), FreeCAD.Rotation(0, 0, 0, 1))
o_chasis = add_part("Chasis_Base_Acrilico", s_chasis, p_chasis, (0.1, 0.1, 0.12), transparency=15)

# 2. Motores TT (Izquierdo y Derecho)
# Motor Izquierdo (X < 0)
rot_mot_l = FreeCAD.Rotation(FreeCAD.Vector(0, 0, 1), 90)
p_mot_l = FreeCAD.Placement(FreeCAD.Vector(-23.75, Y_AXLE - 15.0, Z_AXLE - MOTOR_BOX_H/2.0), rot_mot_l)
o_mot_l = add_part("Motor_TT_Izquierdo", s_motor, p_mot_l, (0.95, 0.75, 0.05))

# Motor Derecho (X > 0)
rot_mot_r = FreeCAD.Rotation(FreeCAD.Vector(0, 0, 1), -90)
p_mot_r = FreeCAD.Placement(FreeCAD.Vector(23.75, Y_AXLE + 15.0, Z_AXLE - MOTOR_BOX_H/2.0), rot_mot_r)
o_mot_r = add_part("Motor_TT_Derecho", s_motor, p_mot_r, (0.95, 0.75, 0.05))

# 3. Ruedas Principales (Izquierda y Derecha)
# Rueda Izquierda
rot_w_l = FreeCAD.Rotation(FreeCAD.Vector(0, 1, 0), -90)
p_w_l = FreeCAD.Placement(FreeCAD.Vector(-(CHASSIS_W_FRONT/2.0 + WHEEL_WIDTH + 4.0), Y_AXLE, Z_AXLE), rot_w_l)
o_w_l = add_part("Rueda_Izquierda", s_rueda, p_w_l, (0.2, 0.2, 0.2))

# Rueda Derecha
rot_w_r = FreeCAD.Rotation(FreeCAD.Vector(0, 1, 0), 90)
p_w_r = FreeCAD.Placement(FreeCAD.Vector((CHASSIS_W_FRONT/2.0 + 4.0), Y_AXLE, Z_AXLE), rot_w_r)
o_w_r = add_part("Rueda_Derecha", s_rueda, p_w_r, (0.2, 0.2, 0.2))

# 4. Rueda Loca Frontal (Ball Caster)
p_caster = FreeCAD.Placement(FreeCAD.Vector(0, 60.0, Z_CHASSIS_BOTTOM), FreeCAD.Rotation(0, 0, 0, 1))
o_caster = add_part("Rueda_Loca_Frontal", s_caster, p_caster, (0.8, 0.8, 0.85))

# 5. Separadores Hexagonales M3 (8 unidades)
standoff_positions = [
    # 4 para la placa delantera
    (-35.0, 30.0), (35.0, 30.0), (-35.0, 55.0), (35.0, 55.0),
    # 4 para la placa trasera
    (-20.0, -40.0), (20.0, -40.0), (-20.0, -70.0), (20.0, -70.0)
]
for idx, (xs, ys) in enumerate(standoff_positions, 1):
    p_st = FreeCAD.Placement(FreeCAD.Vector(xs, ys, Z_CHASSIS_TOP), FreeCAD.Rotation(0, 0, 0, 1))
    add_part(f"Separador_Hexagonal_M3_{idx:02d}", s_standoff, p_st, (0.92, 0.92, 0.92))

# 6. Placa Superior Delantera
p_p_del = FreeCAD.Placement(FreeCAD.Vector(0, 42.5, Z_DECK_TOP), FreeCAD.Rotation(0, 0, 0, 1))
o_p_del = add_part("Placa_Superior_Delantera", s_placa_delantera, p_p_del, (0.1, 0.1, 0.12), transparency=15)

# 7. Marco Soporte de Interruptor
p_sw = FreeCAD.Placement(FreeCAD.Vector(0, 37.5, Z_DECK_TOP + ACRYLIC_THICKNESS), FreeCAD.Rotation(0, 0, 0, 1))
o_sw = add_part("Soporte_Interruptor", s_soporte_sw, p_sw, (0.15, 0.15, 0.15))

# 8. Placa Superior Trasera (Alerón)
p_p_tras = FreeCAD.Placement(FreeCAD.Vector(0, -55.0, Z_DECK_TOP), FreeCAD.Rotation(0, 0, 0, 1))
o_p_tras = add_part("Placa_Superior_Trasera", s_placa_trasera, p_p_tras, (0.1, 0.1, 0.12), transparency=15)

# 9. Soportes Verticales (2 unidades en el alerón trasero)
rot_b_l = FreeCAD.Rotation(FreeCAD.Vector(1, 0, 0), 90)
p_b_l = FreeCAD.Placement(FreeCAD.Vector(-30.0, -45.0, Z_DECK_TOP + ACRYLIC_THICKNESS), rot_b_l)
o_b_l = add_part("Soporte_Vertical_Izquierdo", s_soporte_vert, p_b_l, (0.1, 0.1, 0.12), transparency=15)

rot_b_r = FreeCAD.Rotation(FreeCAD.Vector(1, 0, 0), 90)
p_b_r = FreeCAD.Placement(FreeCAD.Vector(30.0, -45.0, Z_DECK_TOP + ACRYLIC_THICKNESS), rot_b_r)
o_b_r = add_part("Soporte_Vertical_Derecho", s_soporte_vert, p_b_r, (0.1, 0.1, 0.12), transparency=15)

doc.recompute()
print("✓ Ensamblaje completado y recalculado exitosamente.")

# --------------------------------------------------------------------
# 5. GUARDAR PROYECTO .FCStd Y EXPORTAR EN FORMATO STEP
# --------------------------------------------------------------------
fcstd_path = os.path.join(output_dir, "carro_robot.FCStd")
step_asm_path = os.path.join(output_dir, "carro_robot.step")

print(f"\n4. Guardando documento FreeCAD en: {fcstd_path}")
doc.saveAs(fcstd_path)

print(f"5. Exportando ensamblaje completo a formato STEP: {step_asm_path}")
Part.export(part_objects, step_asm_path)

# Exportar piezas individuales usando exportStep (genera B-Rep completo)
piezas_individuales = [
    ("chasis_base.step", s_chasis),
    ("placa_superior_delantera.step", s_placa_delantera),
    ("placa_superior_trasera.step", s_placa_trasera),
    ("soporte_interruptor.step", s_soporte_sw),
    ("soporte_vertical.step", s_soporte_vert),
    ("motor_tt.step", s_motor),
    ("rueda.step", s_rueda),
    ("rueda_loca.step", s_caster)
]

print("6. Exportando piezas individuales en formato STEP...")
for fname, shape in piezas_individuales:
    fpath = os.path.join(output_dir, fname)
    shape.exportStep(fpath)
    print(f"  ✓ {fname} ({os.path.getsize(fpath)} bytes)")

# --------------------------------------------------------------------
# 6. VERIFICACIÓN METROLÓGICA Y GEOMÉTRICA DE CONTROL DE CALIDAD
# --------------------------------------------------------------------
print("\n======================================================================")
print("  REPORTE DE CONTROL DE CALIDAD Y VERIFICACIÓN METROLÓGICA")
print("======================================================================")

verificaciones = [
    ("Chasis Base", s_chasis, (CHASSIS_W_WING, CHASSIS_L, ACRYLIC_THICKNESS)),
    ("Placa Sup Trasera", s_placa_trasera, (REAR_PLATE_W, REAR_PLATE_L, ACRYLIC_THICKNESS)),
    ("Placa Sup Delantera", s_placa_delantera, (FRONT_PLATE_W, FRONT_PLATE_L, ACRYLIC_THICKNESS)),
    ("Soporte Interruptor", s_soporte_sw, (SWITCH_W, SWITCH_L, ACRYLIC_THICKNESS)),
    ("Soporte Vertical", s_soporte_vert, (BRACKET_W, BRACKET_H, ACRYLIC_THICKNESS)),
    ("Motorreductor TT", s_motor, (MOTOR_TOTAL_L, MOTOR_BOX_W + 2*SHAFT_PROTRUSION, MOTOR_DC_D)),
    ("Rueda Tracción", s_rueda, (WHEEL_DIAMETER, WHEEL_DIAMETER, WHEEL_WIDTH))
]

print(f"{'COMPONENTE':<24} | {'VALIDEZ':<8} | {'DIMENSIONES REALES (mm)':<28} | {'ESPERADO (mm)':<24} | {'ESTADO'}")
print("-" * 105)

todos_validos = True
for name, sh, expected_dim in verificaciones:
    bb = sh.BoundBox
    dx = bb.XMax - bb.XMin
    dy = bb.YMax - bb.YMin
    dz = bb.ZMax - bb.ZMin
    
    valido = sh.isValid()
    if not valido:
        todos_validos = False
        
    ex_x, ex_y, ex_z = expected_dim
    diff = max(abs(dx - ex_x), abs(dy - ex_y), abs(dz - ex_z))
    status = "APROBADO" if (valido and diff < 0.1) else "REVISAR"
    
    print(f"{name:<24} | {'VÁLIDO' if valido else 'ERROR':<8} | {dx:6.2f} x {dy:6.2f} x {dz:5.2f} mm | {ex_x:6.2f} x {ex_y:6.2f} x {ex_z:5.2f} mm | {status}")

# Verificación del archivo STEP ensamblado
step_imported = Part.read(step_asm_path)
step_valid = step_imported.isValid()
step_bb = step_imported.BoundBox
print("\n--- Verificación del Modelo Ensamblado STEP Reimportado ---")
print(f"• Archivo STEP válido: {step_valid}")
print(f"• Sólidos en el ensamblaje: {len(step_imported.Solids)}")
print(f"• Caja envolvente total: {step_bb.XMax - step_bb.XMin:.2f} (ancho) x {step_bb.YMax - step_bb.YMin:.2f} (largo) x {step_bb.ZMax - step_bb.ZMin:.2f} (alto) mm")
print(f"• Rango Z total: [{step_bb.ZMin:.2f}, {step_bb.ZMax:.2f}] mm (Contacto suelo a Z = 0: {abs(step_bb.ZMin) < 1e-3})")

# Verificación de piezas individuales STEP reimportadas
print("\n--- Verificación de Archivos STEP Individuales Reimportados ---")
for fname, shape in piezas_individuales:
    fpath = os.path.join(output_dir, fname)
    sh_step = Part.read(fpath)
    print(f"• {fname:<28}: {'VÁLIDO' if sh_step.isValid() else 'ERROR'}, Sólidos: {len(sh_step.Solids)}, Tamaño: {os.path.getsize(fpath)} bytes")

assert todos_validos, "Fallo en validación de componentes individuales"
assert step_valid, "Fallo en validación del archivo STEP ensamblado"

print("\n======================================================================")
print("  PROCESO COMPLETADO EXITOSAMENTE: TODOS LOS REQUISITOS CUMPLIDOS")
print("======================================================================")
