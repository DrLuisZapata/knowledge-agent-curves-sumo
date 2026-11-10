"""
Visualiza en tu PC (sumo-gui) un tramo real de la Federal 54 con el agente.
Coloca este archivo en la carpeta descomprimida del escenario y ejecuta:
    python ver_tramo.py
Requisitos (una sola vez):  pip install numpy requests sumolib==1.27.1 traci==1.27.1
"""
import json, os
os.chdir(os.path.dirname(os.path.abspath(__file__)))
import sumolib
import agente_ruta as ag

CON_AGENTE = True            # False = caso base (sin agente)
P_ERROR    = 0.0             # probabilidad de clasificar mal un obstáculo (0.0 – 1.0)
POLITICA   = "conservadora"  # "conservadora" o "ultima"
SEMILLA    = 1
DELAY_MS   = 50

if __name__ == "__main__":
    cfg = json.load(open("tramo.json"))
    net = sumolib.net.readNet(cfg["archivo_net"], withInternal=False)
    ruta = [net.getEdge(e) for e in open("ruta_aristas.txt").read().split()]
    mapa = ag.Mapa(net, ruta); mapa.archivo_net = cfg["archivo_net"]
    r = ag.correr(".", mapa, cfg, con_agente=CON_AGENTE, gui=True, delay=DELAY_MS,
                  seed=SEMILLA, p_error=P_ERROR, politica=POLITICA)
    print("\nResultado de la corrida:")
    for k, v in r.items():
        print(f"  {k:22}: {v}")
    input("\nListo. Presiona Enter para cerrar...")