"""
v4c — Agente knowledge-driven (con incertidumbre de percepción) sobre una carretera real (Federal 54, Villanueva–Apozol).
- Conocimiento del mapa: perfil de curvatura de la ruta (radio en cada punto).
- Planeación de velocidad anticipada: nunca exceder la velocidad segura de las curvas próximas.
- Obstáculo denso: el agente proyecta un "obstáculo virtual" sobre el carril y decide
  cuándo rebasar por el carril contrario, según el tráfico de frente.
- Obstáculo blando: se ignora.
"""
import math, os, csv, json, random
import numpy as np
import traci, sumolib
import ruta_osm as ro

# ---------------- Parámetros del agente ----------------
A_LAT_MAX        = 3.0     # m/s²  aceleración lateral máxima admisible
DECEL_CONFORT    = 2.0     # m/s²  desaceleración para la planeación de velocidad
MARGEN_CURVA     = 10.0    # m     llegar a la velocidad segura antes de la curva
HORIZONTE        = 400.0   # m     distancia de anticipación sobre el mapa
RANGO_DETECCION  = 200.0   # m     alcance de detección de obstáculos (sensor de largo alcance)
V_REBASE         = 8.0     # m/s   velocidad máxima para rebasar por el carril contrario
DIST_VISIBLE_MIN = 150.0   # m     tramo de carril contrario que debe estar libre
T_MARGEN_REBASE  = 4.0     # s     margen de tiempo adicional para el rebase
# Percepción con incertidumbre (v4c)
DESPEJE_LATERAL  = 5.0     # m     distancia del carril al obstáculo visual (talud, cerro) en curvas
SIGMA_REL        = 0.03    # —     ruido de la distancia medida: σ = SIGMA_REL·d + SIGMA_ABS
SIGMA_ABS        = 0.5     # m
DIST_CONFIRMACION = 50.0   # m     a esta distancia se vuelve a clasificar el obstáculo (segunda mirada)
DECEL_EMERG      = 4.0     # m/s²  desaceleración para detenerse dentro de la distancia visible
REGLA_VISIBILIDAD = True   # nunca circular más rápido de lo que permite detenerse en la distancia visible
CALENTAMIENTO    = 500.0   # m     el vehículo entra antes del tramo; las métricas solo cuentan dentro del tramo
L_EGO, ANCHO_OBST = 4.5, 3.0
EGO = "ego"

# ---------------- Mapa (base de conocimiento geométrica) ----------------
class Mapa:
    def __init__(self, net, aristas, paso=5.0):
        self.net = net
        self.aristas = [e.getID() for e in aristas]
        self.cum, acc = {}, 0.0
        for e in aristas:
            self.cum[e.getID()] = acc
            acc += e.getLane(0).getLength()
        self.longitud = acc
        s, P, kap, vel = ro.perfil_curvatura(aristas, paso=paso)
        # reescalar para que coincida con las posiciones de SUMO (longitudes de carril)
        self.s = s * (self.longitud / max(s[-1], 1.0))
        self.kap = kap
        R = 1.0 / np.maximum(np.abs(kap), 1e-4)
        self.R = R
        self.vseg = np.sqrt(A_LAT_MAX * R)
        self.paso = self.s[1] - self.s[0]
        # arista contraria (mismo par de nodos, sentido opuesto)
        self.contraria = {}
        for e in aristas:
            for o in e.getToNode().getOutgoing():
                if o.getToNode() == e.getFromNode():
                    self.contraria[o.getID()] = e.getID()

    def s_de(self, edge, pos):
        return self.cum[edge] + pos if edge in self.cum else None

    def radio(self, s):
        i = int(np.clip(round(s / self.paso), 0, len(self.R) - 1))
        return float(self.R[i])

    def v_permitida(self, s, v_lim):
        """Máxima velocidad actual que permite llegar a cada curva próxima a su velocidad segura."""
        i0 = int(np.clip(s // self.paso, 0, len(self.s) - 1))
        i1 = int(np.clip((s + HORIZONTE) // self.paso, i0 + 1, len(self.s)))
        d = np.maximum(self.s[i0:i1] - s - MARGEN_CURVA, 0.0)
        v = np.sqrt(self.vseg[i0:i1] ** 2 + 2 * DECEL_CONFORT * d)
        return float(min(v_lim, v.min()))

    def distancia_visible(self, s, hasta=None):
        """Distancia de visibilidad en curva (despeje lateral M): S = 2R·acos(1 − M/R),
        con el radio mínimo entre s y s+hasta. En recta queda limitada por el alcance del sensor."""
        hasta = RANGO_DETECCION if hasta is None else max(hasta, self.paso)
        i0 = int(np.clip(s // self.paso, 0, len(self.R) - 1))
        i1 = int(np.clip((s + hasta) // self.paso + 1, i0 + 1, len(self.R)))
        R = float(self.R[i0:i1].min())
        if R <= DESPEJE_LATERAL:
            return DESPEJE_LATERAL
        return float(min(RANGO_DETECCION, 2 * R * math.acos(1 - DESPEJE_LATERAL / R)))

    def s_contrario(self, edge, pos):
        """Posición equivalente sobre la ruta de un vehículo que circula en sentido contrario."""
        e = self.contraria.get(edge)
        if e is None:
            return None
        return self.cum[e] + self.net.getEdge(e).getLane(0).getLength() - pos

# ---------------- Escenario de un tramo ----------------
def preparar_tramo(carpeta, mapa, ini_m, fin_m, vel_kmh, obstaculos, flujo_vph, t_salida=60.0):
    """obstaculos: lista de (id, s_m, densidad). Escribe rutas y POIs del tramo."""
    os.makedirs(carpeta, exist_ok=True)
    def ubicar(s):
        for e in reversed(mapa.aristas):
            if mapa.cum[e] <= s:
                return e, s - mapa.cum[e]
        return mapa.aristas[0], s
    e0, p0 = ubicar(max(0.0, ini_m - CALENTAMIENTO)); e1, p1 = ubicar(fin_m)
    ruta = mapa.aristas[mapa.aristas.index(e0): mapa.aristas.index(e1) + 1]
    contraria = [k for e in reversed(ruta) for k, v in mapa.contraria.items() if v == e]
    v = vel_kmh / 3.6
    with open(f"{carpeta}/tramo.rou.xml", "w") as f:
        f.write(f'''<routes>
    <vType id="auto_autonomo" accel="2.6" decel="4.5" emergencyDecel="9.0" sigma="0" length="{L_EGO}"
           maxSpeed="{v:.2f}" speedFactor="2" guiShape="passenger" color="0,0.6,1"/>
    <vType id="frente" accel="2.6" decel="4.5" sigma="0.5" length="4.5" maxSpeed="25" speedFactor="1"/>
    <vType id="virtual" length="{ANCHO_OBST}" maxSpeed="0.1" color="1,0,0" guiShape="passenger"/>
    <route id="r_ego" edges="{' '.join(ruta)}"/>
''')
        if flujo_vph > 0 and contraria:
            f.write(f'    <route id="r_frente" edges="{" ".join(contraria)}"/>\n')
            f.write(f'    <flow id="frente" type="frente" route="r_frente" begin="0" end="7200" '
                    f'vehsPerHour="{flujo_vph}" departSpeed="max" departLane="best"/>\n')
        f.write(f'    <vehicle id="{EGO}" type="auto_autonomo" route="r_ego" depart="{t_salida}" '
                f'departPos="{p0:.1f}" arrivalPos="{min(p1, mapa.net.getEdge(e1).getLength()-1):.1f}" '
                f'departSpeed="{v:.2f}"/>\n</routes>\n')
    colores = {"denso": "1,0,0", "blando": "0,1,0"}
    info = []
    with open(f"{carpeta}/obstaculos.add.xml", "w") as f:
        f.write("<additional>\n")
        for oid, s, dens in obstaculos:
            e, p = ubicar(s)
            info.append({"id": oid, "s": s, "edge": e, "pos": p, "densidad": dens})
            f.write(f'    <poi id="{oid}" lane="{e}_0" pos="{p:.1f}" color="{colores[dens]}" type="{dens}" '
                    f'width="{ANCHO_OBST}" height="{ANCHO_OBST}"><param key="densidad" value="{dens}"/></poi>\n')
        f.write("</additional>\n")
    tramo = {"ruta": ruta, "obstaculos": info, "ini_m": ini_m, "fin_m": fin_m, "vel_kmh": vel_kmh,
             "flujo_vph": flujo_vph, "archivo_net": getattr(mapa, "archivo_net", "")}
    with open(f"{carpeta}/tramo.json", "w") as f:
        json.dump(tramo, f, indent=2)
    return tramo

def obstaculos_aleatorios(mapa, curvas, ini_m, fin_m, rng, n_densos=2, separacion=600.0):
    """Coloca obstáculos densos al azar en curvas cerradas del tramo, y uno blando 150 m después de cada uno."""
    cand = [c for c in curvas if ini_m + 300 < c["inicio_m"] and c["fin_m"] < fin_m - 300
            and c["radio_min_m"] < 250]
    rng.shuffle(cand)
    elegidos = []
    for c in cand:
        s = (c["inicio_m"] + c["fin_m"]) / 2
        if all(abs(s - x) > separacion for x in elegidos):
            elegidos.append(s)
        if len(elegidos) == n_densos:
            break
    obs = []
    for k, s in enumerate(sorted(elegidos)):
        obs += [(f"denso_{k}", s, "denso"), (f"blando_{k}", min(s + 150, fin_m - 50), "blando")]
    return obs

# ---------------- Agente ----------------
def frente_libre(mapa, s_ego, v_ego, d_obs):
    """True si no viene tráfico de frente en la distancia necesaria para rebasar."""
    t_rebase = (d_obs + ANCHO_OBST + L_EGO + 20.0) / max(V_REBASE, 1.0) + T_MARGEN_REBASE
    necesaria = max(DIST_VISIBLE_MIN, d_obs + 25.0 * t_rebase)   # 25 m/s: velocidad máx. de frente
    for v in traci.vehicle.getIDList():
        if v == EGO or v.startswith("virtual"):
            continue
        s = mapa.s_contrario(traci.vehicle.getRoadID(v), traci.vehicle.getLanePosition(v))
        if s is not None and 0 < s - s_ego < necesaria:
            return False
    # vehículos que ya van por el carril contrario frente al ego también cuentan
    return True

def _clasificar(real, p_error, rng):
    """Clasificación con error: con probabilidad p_error se confunde denso con blando y viceversa."""
    if rng.random() < p_error:
        return "blando" if real == "denso" else "denso"
    return real

def correr(carpeta, mapa, tramo, con_agente=True, gui=False, seed=42, delay=100, guardar_pasos=False,
           p_error=0.0, politica="conservadora"):
    """politica: 'conservadora' = si alguna mirada dice "denso", se trata como denso (seguridad primero);
                 'ultima'       = se actúa según la clasificación más reciente."""
    try:
        traci.close()
    except Exception:
        pass
    cmd = ["sumo-gui" if gui else "sumo", "-n", mapa.archivo_net, "-r", f"{carpeta}/tramo.rou.xml",
           "-a", f"{carpeta}/obstaculos.add.xml", "--seed", str(seed), "--step-length", "0.1",
           "--no-step-log", "--no-warnings", "--collision.action", "warn", "--lanechange.duration", "2",
           "--end", "3000"]
    if gui:
        cmd += ["--start", "--quit-on-end", "--delay", str(delay)]
    traci.start(cmd)
    dens = {o["id"]: o for o in tramo["obstaculos"]}
    virtuales, pasos = {}, []
    rng = random.Random(seed * 7919 + int(p_error * 1000))
    creencia, mirada_cercana, virtual_blando = {}, set(), set()
    deteccion_d, info_virtual = {}, {}
    colision_obst, colision_veh, rebases, t_espera = set(), 0, 0, 0.0
    s_prev, salio, siguiendo = None, False, False
    if con_agente:
        pass
    while traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()
        t = traci.simulation.getTime()
        ids = traci.vehicle.getIDList()
        # fijar los obstáculos virtuales recién insertados
        for oid, (vid, frente) in list(virtuales.items()):
            if vid in ids and not traci.vehicle.isStopped(vid):
                try:
                    traci.vehicle.setStop(vid, dens[oid]["edge"], pos=frente, laneIndex=0, duration=1e6)
                except traci.TraCIException:
                    pass
        if EGO not in ids:
            if salio:
                break
            continue
        if not salio:
            salio = True
            if con_agente:
                traci.vehicle.setParameter(EGO, "laneChangeModel.lcOpposite", "0")
            if gui:
                traci.gui.trackVehicle("View #0", EGO); traci.gui.setZoom("View #0", 1500)
        edge = traci.vehicle.getRoadID(EGO)
        lane = traci.vehicle.getLaneID(EGO)
        pos = traci.vehicle.getLanePosition(EGO)
        v = traci.vehicle.getSpeed(EGO)
        s = mapa.s_de(edge, pos)
        en_contrario = edge in mapa.contraria
        if en_contrario:
            s = mapa.s_contrario(edge, pos)
        if s is None:                       # dentro de un cruce interno
            s = (s_prev or 0.0) + v * 0.1
        s_prev = s
        R = mapa.radio(s)
        a_lat = v * v / R
        # ---- percepción: visibilidad en curva, clasificación con error y ruido en la distancia
        d_obs, obs_id, d_est = math.inf, None, math.inf
        for oid, o in dens.items():
            d = o["s"] - s
            if d <= 0 or d > RANGO_DETECCION:
                continue
            if not con_agente:
                continue
            if oid not in creencia:
                if d <= mapa.distancia_visible(s, d):          # ¿ya es visible desde aquí?
                    creencia[oid] = _clasificar(o["densidad"], p_error, rng)
                    deteccion_d[oid] = d
                else:
                    continue
            if d <= DIST_CONFIRMACION and oid not in mirada_cercana:
                nueva = _clasificar(o["densidad"], p_error, rng)          # segunda mirada, más cerca
                if politica == "conservadora" and creencia[oid] == "denso":
                    nueva = "denso"                                       # nunca se degrada un "denso"
                creencia[oid] = nueva
                mirada_cercana.add(oid)
            if d < d_obs:
                d_obs, obs_id = d, oid
        if obs_id is not None:
            sigma = SIGMA_REL * d_obs + SIGMA_ABS
            d_est = max(0.5, d_obs + rng.gauss(0.0, sigma))
        accion = "sin_agente"
        if con_agente:
            v_lim = traci.vehicle.getAllowedSpeed(EGO)
            v_obj = mapa.v_permitida(s, max(v_lim, 0.1))
            accion = "reducir_velocidad" if v_obj < v_lim - 0.5 else "mantener_trayectoria"
            if REGLA_VISIBILIDAD:
                v_vis = math.sqrt(2 * DECEL_EMERG * max(mapa.distancia_visible(s) - 15.0, 0.0))
                if v_vis < v_obj:
                    v_obj, accion = v_vis, "reducir_por_visibilidad"
            cre = creencia.get(obs_id)
            # el agente cambió de opinión: era "denso" y ahora cree "blando" → retira el obstáculo virtual
            if obs_id in virtuales and cre == "blando" and not en_contrario:
                vid, _ = virtuales.pop(obs_id)
                if vid in ids:
                    traci.vehicle.remove(vid)
            if obs_id is not None and cre == "denso":
                o = dens[obs_id]
                if obs_id not in virtuales:
                    # obstáculo virtual en la posición ESTIMADA, alargado según la incertidumbre (±2σ)
                    sigma = SIGMA_REL * d_est + SIGMA_ABS
                    largo = ANCHO_OBST + 4 * sigma
                    centro = o["pos"] + (d_est - d_obs)
                    L_edge = mapa.net.getEdge(o["edge"]).getLength()
                    frente = float(np.clip(centro + largo / 2, largo, L_edge - 0.1))
                    vid = f"virtual_{obs_id}"
                    traci.vehicle.add(vid, "r_ego", typeID="virtual", departPos=f"{frente:.1f}",
                                      departSpeed="0", departLane="0")
                    traci.vehicle.setRoute(vid, [o["edge"]])
                    traci.vehicle.setLength(vid, largo)
                    virtuales[obs_id] = (vid, frente)
                    info_virtual[obs_id] = {"s_frente": mapa.cum[o["edge"]] + frente, "largo": largo}
                    if o["densidad"] == "blando":
                        virtual_blando.add(obs_id)
                vid, frente = virtuales[obs_id]
                iv = info_virtual[obs_id]
                if vid not in ids:
                    # aún no está en el carril: detenerse antes de la parte trasera del obstáculo estimado
                    s_trasera = iv["s_frente"] - iv["largo"]
                    v_obj = min(v_obj, math.sqrt(max(0.0, 2 * DECEL_EMERG * (s_trasera - s - 8.0))))
                    if v < 0.5 and vid in traci.simulation.getPendingVehicles():
                        # detenido y el virtual no cabe: recortarlo para que quede delante del vehículo
                        traci.vehicle.remove(vid)
                        nuevo = iv["s_frente"] - (s + 4.0)
                        if nuevo >= ANCHO_OBST:
                            vid2 = f"{vid}_r{int(t*10)}"
                            traci.vehicle.add(vid2, "r_ego", typeID="virtual", departPos=f"{frente:.1f}",
                                              departSpeed="0", departLane="0")
                            traci.vehicle.setRoute(vid2, [o["edge"]])
                            traci.vehicle.setLength(vid2, nuevo)
                            virtuales[obs_id] = (vid2, frente); iv["largo"] = nuevo
                        else:
                            virtuales.pop(obs_id)          # demasiado tarde: el obstáculo está encima
                            creencia[obs_id] = "ignorado"
                if frente_libre(mapa, s, v, d_est):
                    traci.vehicle.setParameter(EGO, "laneChangeModel.lcOpposite", "1")
                    v_obj = min(v_obj, V_REBASE) if d_est < 60 else v_obj
                    accion = "rebasar_contrario"
                else:
                    traci.vehicle.setParameter(EGO, "laneChangeModel.lcOpposite", "0")
                    accion = "esperar_frente_libre"
            elif not en_contrario:
                traci.vehicle.setParameter(EGO, "laneChangeModel.lcOpposite", "0")
            traci.vehicle.setSpeed(EGO, v_obj)
        # métricas
        for oid, o in dens.items():
            if o["densidad"] == "denso" and not en_contrario and lane == f"{o['edge']}_0" \
               and abs((pos - L_EGO / 2) - o["pos"]) < L_EGO / 2 + ANCHO_OBST / 2:
                colision_obst.add(oid)
        if EGO in traci.simulation.getCollidingVehiclesIDList():
            colision_veh += 1
        if v < 1.0:
            t_espera += 0.1
        pasos.append({"t": t, "s_m": round(s, 1), "edge": edge, "contrario": int(en_contrario),
                      "v_kmh": round(v * 3.6, 1), "radio_m": round(R, 1), "a_lat": round(a_lat, 2),
                      "d_obs": round(d_obs, 1) if math.isfinite(d_obs) else None,
                      "d_est": round(d_est, 1) if math.isfinite(d_est) else None,
                      "obs": dens[obs_id]["densidad"] if obs_id else "ninguno",
                      "creencia": creencia.get(obs_id, "ninguno") if obs_id else "ninguno",
                      "accion": accion})
    traci.close()
    if guardar_pasos and pasos:
        nombre = "pasos_agente.csv" if con_agente else "pasos_base.csv"
        with open(f"{carpeta}/{nombre}", "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=pasos[0].keys()); w.writeheader(); w.writerows(pasos)
    # métricas solo dentro del tramo (se excluye la zona de calentamiento)
    pasos_m = [p for p in pasos if tramo["ini_m"] <= p["s_m"] <= tramo["fin_m"]] or pasos
    a = np.array([p["a_lat"] for p in pasos_m]) if pasos_m else np.zeros(1)
    t_espera = sum(0.1 for p in pasos_m if p["v_kmh"] < 3.6)
    cambios = [p["contrario"] for p in pasos]
    rebases = sum(1 for i in range(1, len(cambios)) if cambios[i] == 1 and cambios[i - 1] == 0)
    n_densos = sum(1 for o in dens.values() if o["densidad"] == "denso")
    n_blandos = sum(1 for o in dens.values() if o["densidad"] == "blando")
    det = [deteccion_d[k] for k in deteccion_d if dens[k]["densidad"] == "denso"]
    return {"condicion": "con_agente" if con_agente else "sin_agente", "p_error": p_error,
            "politica": politica if con_agente else "—",
            "obst_densos": n_densos, "obst_blandos": n_blandos,
            "colision_obst_denso": len(colision_obst), "colision_vehiculo": int(colision_veh > 0),
            "maniobra_innecesaria": len(virtual_blando),
            "dist_deteccion_m": round(float(np.mean(det)), 1) if det else None,
            "rebases": rebases, "a_lat_max": round(float(a.max()), 2),
            "t_sobre_umbral_s": round(float((a > A_LAT_MAX + 0.05).sum() * 0.1), 1),
            "t_espera_s": round(t_espera, 1),
            "tiempo_recorrido_s": round(pasos_m[-1]["t"] - pasos_m[0]["t"], 1) if pasos_m else None,
            "v_media_kmh": round(float(np.mean([p["v_kmh"] for p in pasos_m])), 1) if pasos_m else None}