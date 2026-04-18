# Programme principal de conduite autonome CoVAPSy
#
# Démarrage : python main_autonomous.py
# Commandes disponibles dans le terminal :
#   a    → démarre la conduite autonome
#   n  → arrête la voiture
#   q  → arrête la voiture et quitte le programme
#
# Conformité règlement CoVAPSy 2026 :
#   - La voiture n'avance pas avant réception de la commande GO
#   - La voiture s'arrête immédiatement sur commande STOP
#   - Marche arrière automatique en cas de blocage
#   - La voiture va dans le bon sens par rapport aux 2 couleurs des murs
#
# contrôleur neuronal virtuel différentiel
# reprojeté en commandes Ackermann
# + filtrage moyenneur des données LiDAR
# + utilisation de 5 points exacts :
#   gauche: 60° et 70°
#   front : 0°
#   droite: -60° et -70°

import logging
import os
import sys
import threading
import time
import importlib

from commun_v1 import (
    filtre_moyenneur,
    analyze_walls,
    calculer_commande_auto,
)

import config
from robot_base import Actionneurs, CapteurLidar


# ============================================================
# Configuration du logging (un logger c'est juste un print amélioré, avec timestamp, niveau de gravité, etc.)
# ============================================================
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    datefmt="%H:%M:%S",
    handlers=[
        logging.StreamHandler(sys.stdout),
        logging.FileHandler("covapsy.log", encoding="utf-8"),
    ]
)
logger = logging.getLogger(__name__)


def initialiser_camera():
    """Initialise la camera Raspberry Pi si disponible.

    Retourne:
        camera_obj | None
        show_window (bool)
    """
    if not bool(config.CAMERA_ACTIVE):
        logger.info("Camera desactivee par config")
        return None, False

    try:
        picamera2_mod = importlib.import_module("picamera2")
        picamera2_cls = getattr(picamera2_mod, "Picamera2", None)
    except Exception:
        picamera2_cls = None

    if picamera2_cls is None:
        logger.warning("Picamera2 indisponible: verification camera desactivee")
        return None, False

    show_window = bool(config.CAMERA_SHOW_WINDOW) and (
        ("DISPLAY" in os.environ) or (os.name == "nt")
    )

    try:
        camera = picamera2_cls()
        width = int(config.CAMERA_WIDTH)
        height = int(config.CAMERA_HEIGHT)

        cfg = camera.create_preview_configuration(
            main={"size": (width, height), "format": "RGB888"},
            queue=False,
        )
        camera.configure(cfg)
        camera.start()
        time.sleep(1.0)
        logger.info("Camera initialisee (%dx%d)", width, height)
        return camera, show_window
    except Exception as exc:
        logger.warning("Echec initialisation camera: %s", exc)
        return None, False


def gestion_commandes_clavier(mode_auto_event: threading.Event, stop_event: threading.Event, actionneurs: Actionneurs):
    """Thread de lecture commandes utilisateur (A/N/Q) sans bloquer la boucle de conduite."""
    while not stop_event.is_set():
        try:
            cmd = input("\nCommande > ").strip().upper() #input est bloquant, mais c'est pas grave car c'est dans un thread séparé
        except EOFError:
            cmd = "Q"
        except KeyboardInterrupt:
            logger.info("Interruption clavier recue (Ctrl+C) — arret du programme")
            stop_event.set()
            break

        if cmd == "A":
            if mode_auto_event.is_set():
                print("  Deja en mode auto.")
            else:
                logger.info("Mode auto active par l'utilisateur")
                print("  Mode auto active.")
                mode_auto_event.set()
                actionneurs.demarrer()

        elif cmd == "N":
            if not mode_auto_event.is_set():
                print("  Deja en mode manuel.")
            else:
                logger.info("Mode auto desactive par l'utilisateur")
                print("  Mode auto desactive.")
                mode_auto_event.clear()
                actionneurs.arreter()

        elif cmd == "Q":
            logger.info("Quitter le programme.")
            stop_event.set()

        elif cmd:
            print("  Commande inconnue. Utiliser A, N ou Q.")


def main():
    # =========================
    # Mode de fonctionnement
    # =========================
    mode_auto_event = threading.Event() # Precedamment appelé modeAuto, mais un Event est plus adapté pour la synchronisation entre threads
    stop_event = threading.Event()
    print("CoVAPSy — Conduite Autonome pour Webots")
    print("Cliquer sur la vue 3D pour commencer")
    print("a : mode auto")
    print("n : stop")
    print("q : quitter")
    
    # =========================
    # Initialisation des actionneurs
    # =========================
    act   = Actionneurs()
    # act.demarrer()
    logger.info("Actionneurs initialises et PWM actives")
    logger.info(
        "Bornes vitesse auto: min=%.3f m/s, max=%.3f m/s",
        float(config.VITESSE_AUTO_MIN_M_S),
        float(config.VITESSE_AUTO_MAX_M_S),
    )
    
    # =========================
    # Initialisation du LiDAR
    # =========================
    lidar = CapteurLidar()
    try:
        # Initialisation du matériel
        logger.info("Connexion au lidar...")
        lidar.connecter()
        lidar.demarrer()
        
        print("\n  Lidar connecté")
    except Exception as e:
        logger.error("Erreur critique : %s", e, exc_info=True)
    
    # =========================
    # Initialisation du sonar arriere
    # =========================
    sonar = None

    # =========================
    # Initialisation clavier
    # =========================
    clavier_thread = threading.Thread(
        target=gestion_commandes_clavier,
        args=(mode_auto_event, stop_event, act),
        daemon=True,
        name="thread_commandes",
    )
    clavier_thread.start()
    
    # =========================
    # Initialisation camera
    # =========================
    camera, show_camera_window = initialiser_camera()

    try:
        while not stop_event.is_set():
            # =========================
            # Acquisition LiDAR et filtre
            # =========================
            if not lidar.lire():
                time.sleep(config.BOUCLE_PERIODE_S)
                continue
            tableau_lidar_filtre = filtre_moyenneur(lidar.tableau_mm)

            # =========================
            # Mode manuel / arret
            # =========================
            if not mode_auto_event.is_set():
                time.sleep(config.BOUCLE_PERIODE_S)
                continue

            # =========================
            # Commande auto navigation seulement
            # =========================
            v_cmd, angle_cmd = calculer_commande_auto(
                tableau_lidar_filtre,
                L_entraxe=config.L_ENTRAXE_M,
                W_empattement=config.W_EMPATTEMENT_M,
                maxangle_degre=config.ANGLE_DEGRE_MAX,
                dmax=config.LIDAR_DMAX_MM,
                v_min=config.VITESSE_AUTO_MIN_M_S,
                v_max=config.VITESSE_AUTO_MAX_M_S,
                debug=bool(config.AUTO_DEBUG),
            )

            # =========================
            # Camera  — traitée uniquement quand la voiture est à l'arrêt
            # =========================
            wall_info = None
            if camera is not None and abs(float(v_cmd)) < 1e-6:
                try:
                    frame_rgb = camera.capture_array("main")
                    frame_bgr = frame_rgb[:, :, ::-1].copy()
                    wall_info = analyze_walls(
                        frame_bgr,
                        band_ratio=float(config.CAMERA_BAND_RATIO),
                        min_ratio=float(config.CAMERA_MIN_RATIO),
                        dominance=float(config.CAMERA_DOMINANCE),
                        unknown_value=int(config.CAMERA_UNKNOWN_VALUE),
                    )

                    if show_camera_window and wall_info is not None:
                        try:
                            cv2_mod = importlib.import_module("cv2")
                            cv2_mod.imshow("Camera murs rouge/vert", wall_info["annotated"])
                            cv2_mod.waitKey(1)
                        except Exception:
                            pass
                except Exception as exc:
                    logger.warning("Erreur lecture camera: %s", exc)

            act.set_direction_degre(float(angle_cmd))
            act.set_vitesse_m_s(float(v_cmd))

            if bool(config.DEBUG_ACTIONNEURS):
                logger.info(
                    "NAV v=%.2f ang=%.1f",
                    float(v_cmd),
                    float(angle_cmd),
                )

            time.sleep(config.BOUCLE_PERIODE_S)
    except KeyboardInterrupt:
        logger.info("Interruption clavier recue (Ctrl+C) — arret propre")
        stop_event.set()
    
    finally:
        # =========================
        # Arrêt propre
        # =========================
        logger.info("Arrêt propre en cours...")
        try:
            time.sleep(0.5)
        except KeyboardInterrupt:
            pass
        try:
            act.arreter()
        except Exception:
            pass
        if camera is not None:
            try:
                camera.stop()
            except Exception:
                pass
        if show_camera_window:
            try:
                cv2_mod = importlib.import_module("cv2")
                cv2_mod.destroyAllWindows()
            except Exception:
                pass
        lidar.arreter()
        logger.info("Programme terminé.")
        print("Au revoir.")

if __name__ == "__main__":
    main()