"""
Importación de una carretera real desde OpenStreetMap y análisis de su curvatura.
Caso de estudio: Carretera Federal 54, tramo Villanueva – Apozol (Zacatecas).
"""
import math, os, subprocess, time
import numpy as np
import requests
import sumolib

# netconvert necesita la base de proyecciones (proj.db) para convertir lat/lon a metros
if os.environ.get("SUMO_HOME"):
    _proj = os.path.join(os.environ["SUMO_HOME"], "data", "proj")
    if os.path.isdir(_proj):
        os.environ.setdefault("PROJ_DATA", _proj)
        os.environ.setdefault("PROJ_LIB", _proj)

UA = {"User-Agent": "fed54-curve-agent/1.0 (academic research)"}
OVERPASS = ["https://overpass-api.de/api/interpreter",
            "https://overpass.private.coffee/api/interpreter",
            "https://maps.mail.ru/osm/tools/overpass/api/interpreter",
            "https://overpass.kumi.systems/api/interpreter"]

def _overpass(consulta, valida, rondas=3, espera=30):
    """Envía la consulta a varios servidores Overpass, con reintentos. Devuelve la respuesta válida."""
    errores = []
    for ronda in range(rondas):
        for url in OVERPASS:
            try:
                r = requests.post(url, data={"data": consulta}, headers=UA, timeout=240)
                if r.ok and valida(r):
                    return r
                errores.append(f"{url.split('/')[2]}: HTTP {r.status_code}")
            except requests.RequestException as e:
                errores.append(f"{url.split('/')[2]}: {type(e).__name__}")
        if ronda < rondas - 1:
            print(f"Servidores ocupados; reintento en {espera} s...")
            time.sleep(espera)
    raise RuntimeError("No se pudo consultar Overpass. Detalle: " + "; ".join(errores[-len(OVERPASS):]) +
                       ". Intenta en unos minutos o sube el archivo .osm de una corrida anterior.")

# ---------------- 1. Geocodificación ----------------
def geocodificar(lugar):
    """Devuelve (lat, lon) de un lugar usando Nominatim (OpenStreetMap)."""
    r = requests.get("https://nominatim.openstreetmap.org/search",
                     params={"q": lugar, "format": "json", "limit": 1}, headers=UA, timeout=60)
    r.raise_for_status()
    d = r.json()
    if not d:
        raise ValueError(f"No se encontró: {lugar}")
    time.sleep(1)                      # política de uso de Nominatim: 1 consulta por segundo
    return float(d[0]["lat"]), float(d[0]["lon"])

# ---------------- 2. Descarga de la carretera ----------------
def descargar_carretera(origen, destino, ref_regex, archivo, margen=0.08):
    """Descarga solo las vías con la referencia indicada (p. ej. la Federal 54)
    dentro del rectángulo que contiene origen y destino, con un margen en grados."""
    s = min(origen[0], destino[0]) - margen; n = max(origen[0], destino[0]) + margen
    w = min(origen[1], destino[1]) - margen; e = max(origen[1], destino[1]) + margen
    q = f"""[out:xml][timeout:180];
(
  way["highway"]["ref"~"{ref_regex}"]({s},{w},{n},{e});
);
(._;>;);
out body;"""
    if os.path.exists(archivo) and os.path.getsize(archivo) > 1000:
        print(f"Usando el archivo ya existente: {archivo}")
        return archivo
    r = _overpass(q, lambda r: b"<way" in r.content)
    open(archivo, "wb").write(r.content)
    return archivo

# ---------------- 3. Conversión a red de SUMO ----------------
def convertir_a_sumo(archivo_osm, archivo_net):
    if not os.path.exists(archivo_osm):
        raise FileNotFoundError(f"No existe el archivo {archivo_osm}")
    r = subprocess.run(["netconvert", "--osm-files", archivo_osm, "-o", archivo_net,
                        "--geometry.remove", "false",        # conservar la geometría de las curvas
                        "--keep-edges.components", "1",      # solo el componente conectado principal
                        "--no-turnarounds", "true",
                        "--osm.sidewalks", "false",
                        "--tls.discard-simple", "true",
                        "--opposites.guess", "true",          # carril contrario para rebasar
                        "--no-warnings", "true"],
                       capture_output=True, text=True)
    if r.returncode != 0:
        raise RuntimeError("netconvert falló:\n" + (r.stderr or r.stdout)[-2000:])
    return sumolib.net.readNet(archivo_net, withInternal=False)

# ---------------- 4. Ruta entre origen y destino ----------------
def _aristas_cercanas(net, lat, lon, radio=500):
    x, y = net.convertLonLat2XY(lon, lat)
    cand = net.getNeighboringEdges(x, y, radio)
    cand.sort(key=lambda t: t[1])
    return [e for e, _ in cand if e.allows("passenger")]

def construir_ruta(net, origen, destino):
    """Ruta más corta (lista de aristas) de origen a destino sobre la red importada."""
    mejor = None
    for radio in (300, 800, 2000, 5000):
        for e1 in _aristas_cercanas(net, *origen, radio)[:6]:
            for e2 in _aristas_cercanas(net, *destino, radio)[:6]:
                ruta, costo = net.getShortestPath(e1, e2)
                if ruta and (mejor is None or costo < mejor[1]):
                    mejor = (ruta, costo)
        if mejor:
            return mejor[0]
    raise RuntimeError("No se encontró una ruta conectada entre origen y destino.")

# ---------------- 5. Perfil de curvatura ----------------
def geometria_ruta(ruta):
    """Polilínea (x, y) de la ruta (carril 0 de cada arista) y límite de velocidad (km/h) en cada punto."""
    pts, vel = [], []
    for e in ruta:
        for p in e.getLane(0).getShape():
            if not pts or math.dist(p, pts[-1]) > 0.5:
                pts.append(p); vel.append(e.getSpeed() * 3.6)
    return np.array(pts), np.array(vel)

def remuestrear(pts, paso=5.0, vel=None):
    seg = np.hypot(*np.diff(pts, axis=0).T)
    s = np.concatenate([[0], np.cumsum(seg)])
    s_new = np.arange(0, s[-1], paso)
    P = np.column_stack([np.interp(s_new, s, pts[:, 0]), np.interp(s_new, s, pts[:, 1])])
    if vel is None:
        return s_new, P
    idx = np.clip(np.searchsorted(s, s_new, side="right") - 1, 0, len(vel) - 1)
    return s_new, P, vel[idx]

def perfil_curvatura(ruta, paso=5.0, semiventana=20.0, suavizado=7):
    """Curvatura con signo (1/m) cada `paso` metros: círculo por tres puntos separados
    `semiventana` m, suavizado con media móvil. Positiva = curva a la izquierda.
    Devuelve (s, P, kap, vel): distancia, puntos (x, y), curvatura y límite de velocidad (km/h)."""
    pts, vel = geometria_ruta(ruta)
    s, P, v = remuestrear(pts, paso, vel)
    k = max(1, int(semiventana / paso))
    kap = np.zeros(len(P))
    for i in range(k, len(P) - k):
        a, b, c = P[i - k], P[i], P[i + k]
        ab, bc, ca = np.linalg.norm(b - a), np.linalg.norm(c - b), np.linalg.norm(a - c)
        cruz = (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])
        if ab * bc * ca > 0:
            kap[i] = 2 * cruz / (ab * bc * ca)
    kap = np.convolve(kap, np.ones(suavizado) / suavizado, mode="same")
    return s, P, kap, v

def detectar_curvas(s, kap, radio_max=400.0, long_min=20.0):
    """Tramos continuos con radio < radio_max. Devuelve lista de dicts."""
    en_curva = np.abs(kap) > 1.0 / radio_max
    curvas, i = [], 0
    while i < len(s):
        if en_curva[i]:
            j = i
            while j + 1 < len(s) and en_curva[j + 1] and np.sign(kap[j + 1]) == np.sign(kap[i]):
                j += 1
            if s[j] - s[i] >= long_min:
                kmax = np.max(np.abs(kap[i:j + 1]))
                curvas.append({"inicio_m": round(s[i]), "fin_m": round(s[j]),
                               "longitud_m": round(s[j] - s[i]), "radio_min_m": round(1 / kmax),
                               "sentido": "izquierda" if kap[i] > 0 else "derecha"})
            i = j + 1
        else:
            i += 1
    return curvas

def mejores_tramos(curvas, long_total, ventana=5000.0, n=3, radio_critico=250.0,
                   s=None, urbano=None, max_frac_urbana=0.05):
    """Ventanas de `ventana` m con más curvas cerradas (radio mínimo < radio_critico), sin traslape.
    Si se dan `s` y `urbano`, descarta ventanas con más de `max_frac_urbana` de zona urbana."""
    criticas = [c for c in curvas if c["radio_min_m"] < radio_critico]
    candidatos = []
    for c in criticas:
        ini = max(0.0, c["inicio_m"] - 200.0)
        fin = min(long_total, ini + ventana)
        if fin - ini < 0.8 * ventana:
            continue                                  # ventana recortada al final de la ruta
        if urbano is not None:
            m = (s >= ini) & (s <= fin)
            if urbano[m].mean() > max_frac_urbana:
                continue
        dentro = [d for d in criticas if d["inicio_m"] >= ini and d["fin_m"] <= fin]
        candidatos.append((len(dentro), -min(d["radio_min_m"] for d in dentro), ini, fin))
    candidatos.sort(reverse=True)
    elegidos = []
    for cnt, _, ini, fin in candidatos:
        if all(fin <= a or ini >= b for a, b, _ in elegidos):
            elegidos.append((ini, fin, cnt))
        if len(elegidos) == n:
            break
    return [{"inicio_km": round(a / 1000, 2), "fin_km": round(b / 1000, 2), "curvas_cerradas": c}
            for a, b, c in sorted(elegidos)]


# ---------------- 6. Exclusión de zonas urbanas ----------------
RADIO_LOCALIDAD = {"city": 2500.0, "town": 1500.0, "village": 700.0, "hamlet": 300.0}

def descargar_localidades(origen, destino, margen=0.08):
    """Ciudades, pueblos y villas de OpenStreetMap dentro del rectángulo de la ruta."""
    s = min(origen[0], destino[0]) - margen; n = max(origen[0], destino[0]) + margen
    w = min(origen[1], destino[1]) - margen; e = max(origen[1], destino[1]) + margen
    q = f"""[out:json][timeout:120];
node["place"~"^(city|town|village|hamlet)$"]({s},{w},{n},{e});
out;"""
    def valida(r):
        try:
            return "elements" in r.json()
        except ValueError:
            return False
    r = _overpass(q, valida)
    return [{"nombre": el["tags"].get("name", "?"), "tipo": el["tags"]["place"],
             "lat": el["lat"], "lon": el["lon"]} for el in r.json()["elements"]]

def localidades_manuales(lista):
    """Localidades agregadas a mano. Cada elemento: (texto, tipo, radio_m o None) o
    (texto, tipo, radio_m o None, (lat, lon)) si ya conoces sus coordenadas.
    Si una localidad no se encuentra, se avisa y se omite (no detiene el notebook)."""
    res = []
    for item in lista:
        texto, tipo, radio = item[:3]
        coord = item[3] if len(item) > 3 else None
        nombre = texto.split(",")[0].strip()
        if coord is None:
            partes = [p.strip() for p in texto.split(",")]
            variantes = [texto, f"{nombre}, Zacatecas, México", f"{nombre}, Zacatecas", nombre]
            for v in dict.fromkeys(variantes):
                try:
                    coord = geocodificar(v)
                    break
                except ValueError:
                    continue
        if coord is None:
            print(f"AVISO: no se encontró «{texto}». Agrégala con sus coordenadas: "
                  f'("{nombre}", "{tipo}", None, (lat, lon))')
            continue
        res.append({"nombre": nombre, "tipo": tipo, "lat": coord[0], "lon": coord[1],
                    "radio": radio, "manual": True})
    return res

def marcar_urbano(net, s, P, vel, localidades, vel_urbana=60.0, radios=RADIO_LOCALIDAD, alcance_aviso=4000.0):
    """True en los puntos de la ruta dentro de una localidad o con límite <= vel_urbana km/h.
    Devuelve (mascara, cruzadas, cercanas): `cercanas` son localidades a menos de `alcance_aviso` m
    de la ruta que NO se marcaron (útil para revisar si falta alguna)."""
    urbano = vel <= vel_urbana + 1e-6
    cruzadas, cercanas, vistos = [], [], set()
    for loc in localidades:
        x, y = net.convertLonLat2XY(loc["lon"], loc["lat"])
        d = np.hypot(P[:, 0] - x, P[:, 1] - y)
        r = loc.get("radio") or radios.get(loc["tipo"], 700.0)
        i = int(np.argmin(d))
        fila = {"localidad": loc["nombre"], "tipo": loc["tipo"], "km": round(float(s[i]) / 1000, 1),
                "distancia_m": round(float(d[i])), "radio_m": round(r),
                "origen": "manual" if loc.get("manual") else "OSM"}
        clave = (loc["nombre"].lower(), fila["km"])
        if d[i] < r:
            urbano |= d < r
            if clave not in vistos:
                cruzadas.append(fila); vistos.add(clave)
        elif d[i] < alcance_aviso and loc["tipo"] != "hamlet":
            cercanas.append(fila)
    return urbano, sorted(cruzadas, key=lambda c: c["km"]), sorted(cercanas, key=lambda c: c["km"])