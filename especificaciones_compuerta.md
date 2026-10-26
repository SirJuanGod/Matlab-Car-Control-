# Especificaciones de la Compuerta (Planta Pasiva y Sensor Directo 0-3.3V con ESP32)

## Enlaces del Grafo
* **Subsistema Padre:** [[arquitectura_sistema]]
* **Recibe flujo de aire de:** [[especificaciones_ventilador]]
* **Interacción física por distancia con:** [[especificaciones_carro]]
* **Estrategias de Control:** [[estrategias_control]]

## Etiquetas
#planta #compuerta #potenciometro #sensor-angular #esp32 #adc-12bits #conexion-directa #sin-divisor #dinamica-pasiva

---

## 1. Naturaleza del Sistema Dinámico
La compuerta es la **planta física** del lazo de control angular:
* **Sistema Pasivo / No Controlado Directamente:** Carece de servomotores o motores acoplados a su eje.
* Su desplazamiento angular $\theta(t)$ resulta del equilibrio dinámico entre el **torque aerodinámico** transmitido por el chorro del ventilador, el **torque gravitacional restaurador** y las **fuerzas de amortiguamiento y fricción** en el soporte.
* **Montaje mecánico:** la compuerta cuelga de un **soporte en L invertida** (poste vertical con un brazo horizontal); el pivote, con el potenciómetro, está en la punta del brazo, de modo que la compuerta queda **suspendida** sin tocar el suelo. El **ventilador va sobre el chasis del carro** y sopla horizontalmente hacia la compuerta.
* **Convención de ángulo:** $\theta = 0^\circ$ es la compuerta **colgando en reposo** (vertical, sin aire). El aire la empuja alejándola del carro y la gravedad la devuelve hacia $\theta = 0^\circ$ (torque restaurador $\propto \sin\theta$): la planta es **estable**. El potenciómetro de 0–180° cubre desde el reposo colgante hasta la vertical hacia arriba.

---

## 2. Sensor de Salida: Potenciómetro Directo (0V - 3.3V $\to$ 0° - 180°)

Para simplificar el hardware y **eliminar por completo cualquier etapa de acondicionamiento de señal o divisor de tensión resistivo**, el potenciómetro se alimenta directamente desde el riel de **$3.3\text{ VDC}$** regulado del ESP32.

```
       ESP32 (3.3V)
           3V3 o--------+ (Pin 1 Potenciómetro)
                        |
                       [ ] Potenciómetro Lineal 10 kΩ
      GPIO ADC o<-------+ (Cursor / Pin 2: 0.0V - 3.3V DIRECTO AL ESP32)
      (Pin 34/35)       |
           GND o--------+ (Pin 3 Potenciómetro)
```

### 2.1 Calibración Tensión vs. Ángulo (0V a 3.3V)
| Voltaje Potenciómetro ($V_{pot}$) | Ángulo Físico ($\theta$) | Equivalente en Radianes | Condición Mecánica |
| :---: | :---: | :---: | :--- |
| **$0.00\text{ V}$** | **$0^\circ$** | $0\text{ rad}$ | Compuerta colgando en reposo (vertical, sin aire) |
| **$1.65\text{ V}$** | **$90^\circ$** | $\frac{\pi}{2}\text{ rad}$ | Compuerta horizontal |
| **$3.30\text{ V}$** | **$180^\circ$** | $\pi\text{ rad}$ | Compuerta vertical hacia arriba (máxima apertura) |

### 2.2 Función de Transferencia Directa del Sensor
Sensibilidad angular:
$$K_{pot} = \frac{3.3\text{ V} - 0.0\text{ V}}{180^\circ - 0^\circ} = \frac{3.3}{180}\text{ V/deg} = \frac{11}{600}\text{ V/deg} \approx \mathbf{0.01833\text{ V/deg}} \quad (18.33\text{ mV/deg})$$

Ecuación de tensión de salida:
$$V_{pot}(\theta) = \left(\frac{3.3\text{ V}}{180^\circ}\right) \theta \approx 0.01833 \cdot \theta \quad [\text{V}]$$

Ecuación de reconstrucción angular a partir de la tensión medida:
$$\theta[^\circ] = \left(\frac{180^\circ}{3.3\text{ V}}\right) V_{pot} \approx \mathbf{54.545} \cdot V_{pot}\text{ [V]}$$
$$\theta[\text{rad}] = \left(\frac{\pi}{3.3\text{ V}}\right) V_{pot} \approx 0.952 \cdot V_{pot}\text{ [V]}$$

### 2.3 Adquisición Directa con ADC de 12 Bits del ESP32
* **Conexión de Hardware:** El cursor del potenciómetro se conecta **directamente** a un pin de entrada analógica del ESP32 (recomendado ADC1: `GPIO34`, `GPIO35`, `GPIO36` o `GPIO39`), sin componentes pasivos intermedios.
* **Rango del ADC:** 12 bits $\implies 2^{12} = 4096$ valores discretos ($0$ a $4095$).
* **Resolución Angular Obtenida:**
  $$\Delta \theta = \frac{180^\circ}{4095} \approx \mathbf{0.04396^\circ \text{ / LSB}} \quad (\approx 0.044^\circ)$$

#### Fórmula de Cálculo Directo en Firmware (ESP32 / C++)
$$\theta[^\circ] = \left( \frac{\text{Lectura\_ADC}}{4095} \right) \times 180.0^\circ$$

---

## 3. Parámetros Físicos y Mecánicos

* **Material:** Lámina ligera rígida (cartón prensado o acrílico fino de 1.5 mm).
* **Ancho ($W$):** $0.15\text{ m}$ ($15\text{ cm}$).
* **Largo ($L$):** $0.20\text{ m}$ ($20\text{ cm}$).
* **Área Expuesta Total ($A$):** $A = W \cdot L = 0.03\text{ m}^2$.
* **Masa Total ($m_g$):** $\approx 0.015\text{ kg}$ ($15\text{ g}$).
* **Centro de Masa ($\ell_c$):** $\ell_c = \frac{L}{2} = 0.10\text{ m}$ ($10\text{ cm}$ desde el eje).
* **Momento de Inercia respecto al eje ($J_g$):** 
  $$J_g = \frac{1}{3} m_g L^2 = \frac{1}{3}(0.015)(0.20)^2 = 2.0 \times 10^{-4}\text{ kg}\cdot\text{m}^2$$

---

## 4. Ecuación Dinámica Pasiva de la Compuerta

$$J_g \ddot{\theta}(t) + b_c \dot{\theta}(t) + m_g g \ell_c \sin(\theta(t)) + \tau_{fric}\text{sign}(\dot{\theta}) = \tau_{aero}(t)$$

* $b_c \approx 0.001\text{ N}\cdot\text{m}\cdot\text{s/rad}$ (amortiguamiento viscoso del pivote).
* $\tau_{fric} \approx 0.0001\text{ N}\cdot\text{m}$ (fricción mecánica y de escobilla del potenciómetro).
* $\tau_{aero}(t) \approx k_m(d) \cdot u_{PWM}(t)$ (torque aerodinámico generado por el ventilador gobernado por el L298N).

* **Equilibrio y planta linealizada:** con el aire, $\tau_{aero} = m_g g \ell_c \sin\theta_{eq}$. Linealizada en $35^\circ$: $G_p(s) = \dfrac{1.5}{s^2 + 5s + 60.3}$, con polos $-2.5 \pm 7.35j$ rad/s ($\zeta \approx 0.32$, $\omega_n \approx 7.8$ rad/s): **estable pero oscilatoria**; el controlador debe amortiguarla.