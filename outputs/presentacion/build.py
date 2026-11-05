"""Genera dist/index.html autocontenido: incrusta img/* como data URI.
El visor de artifacts no carga imagenes por ruta relativa de forma fiable."""
import base64, json, mimetypes, pathlib, re

raiz = pathlib.Path(__file__).parent
html = (raiz / "index.html").read_text(encoding="utf-8")

imagenes = {}

def registrar(ruta):
    if ruta not in imagenes:
        archivo = raiz / ruta
        tipo = mimetypes.guess_type(archivo.name)[0]
        imagenes[ruta] = f"data:{tipo};base64," + base64.b64encode(archivo.read_bytes()).decode()
    return f'window.__IMG["{ruta}"]'

# atributos JSX src="img/..." y cadenas JS "img/..." -> referencias a window.__IMG
patron = r'(img/[^"]+\.(?:png|gif|jpg))'
html = re.sub(r'src="' + patron + '"', lambda m: "src={" + registrar(m.group(1)) + "}", html)
html = re.sub(r'(?<!\[)"' + patron + '"', lambda m: registrar(m.group(1)), html)
# las imagenes van en un <script> normal, fuera del codigo que compila Babel
datos = "<script>window.__IMG = " + json.dumps(imagenes) + ";</script>\n"
html = html.replace('<script type="text/babel"', datos + '<script type="text/babel"', 1)
n = len(imagenes)
(raiz / "dist").mkdir(exist_ok=True)
(raiz / "dist" / "index.html").write_text(html, encoding="utf-8")
print(f"{n} imagenes incrustadas, {len(html) / 2**20:.1f} MB")
