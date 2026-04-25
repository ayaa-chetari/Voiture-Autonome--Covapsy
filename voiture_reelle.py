import logging
import os
import sys
import threading
import time
import importlib

from commun import (
    filtre_moyenneur,
    analyze_walls,
    calculer_commande_automate,
    reset_automate,
    get_automate_state,
)

import config
from robot_base import Actionneurs, CapteurLidar


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

ETAT_NAMES = {0: "NAVIGATION", 1: "BLOCAGE"}
SOUS_ETAT_NAMES = {0: "BACKWARD", 2: "TURN_LEFT", 3: "TURN_RIGHT"}


def initialiser_camera():
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

        cfg = camera.create_preview_configuration(
            main={
                "size": (int(config.CAMERA_WIDTH), int(config.CAMERA_HEIGHT)),
                "format": "RGB888",
            },
            queue=False,
        )

        camera.configure(cfg)
        camera.start()
        time.sleep(1.0)

        logger.info(
            "Camera initialisee (%dx%d)",
            int(config.CAMERA_WIDTH),
            int(config.CAMERA_HEIGHT),
        )

        return camera, show_window

    except Exception as exc:
        logger.warning("Echec initialisation camera: %s", exc)
        return None, False


def gestion_commandes_clavier(mode_auto_event, stop_event, actionneurs):
    while not stop_event.is_set():
        try:
            cmd = input("\nCommande > ").strip().upper()
        except EOFError:
            cmd = "Q"
        except KeyboardInterrupt:
            logger.info("Interruption clavier recue")
            stop_event.set()
            break

        if cmd == "A":
            if mode_auto_event.is_set():
                print("  Deja en mode auto.")
            else:
                logger.info("Mode auto active")
                print("  Mode auto active.")
                reset_automate()
                mode_auto_event.set()
                actionneurs.demarrer()

        elif cmd == "N":
            if not mode_auto_event.is_set():
                print("  Deja en mode manuel.")
            else:
                logger.info("Mode auto desactive")
                print("  Mode auto desactive.")
                mode_auto_event.clear()
                reset_automate()
                actionneurs.arreter()

        elif cmd == "Q":
            logger.info("Quitter le programme")
            stop_event.set()

        elif cmd:
            print("  Commande inconnue. Utiliser A, N ou Q.")


def main():
    mode_auto_event = threading.Event()
    stop_event = threading.Event()

    print("CoVAPSy — Conduite Autonome")
    print("a : mode auto")
    print("n : stop")
    print("q : quitter")

    act = Actionneurs()

    logger.info("Actionneurs initialises")
    logger.info(
        "Bornes vitesse auto: min=%.3f m/s, max=%.3f m/s",
        float(config.VITESSE_AUTO_MIN_M_S),
        float(config.VITESSE_AUTO_MAX_M_S),
    )

    lidar = CapteurLidar()

    try:
        logger.info("Connexion au lidar...")
        lidar.connecter()
        lidar.demarrer()
        print("\n  Lidar connecté")
    except Exception as exc:
        logger.error("Erreur critique lidar : %s", exc, exc_info=True)

    sonar = None

    clavier_thread = threading.Thread(
        target=gestion_commandes_clavier,
        args=(mode_auto_event, stop_event, act),
        daemon=True,
        name="thread_commandes",
    )
    clavier_thread.start()

    camera, show_camera_window = initialiser_camera()

    reset_automate()
    last_read_time = 0.0
    wall_info_memo = None

    try:
        while not stop_event.is_set():
            now = time.time()

            if (now - last_read_time) <= float(config.BOUCLE_PERIODE_S):
                time.sleep(0.001)
                continue

            last_read_time = now

            if not lidar.lire():
                time.sleep(config.BOUCLE_PERIODE_S)
                continue

            tableau_lidar_filtre = filtre_moyenneur(lidar.tableau_mm)

            if not mode_auto_event.is_set():
                reset_automate()
                wall_info_memo = None
                time.sleep(config.BOUCLE_PERIODE_S)
                continue

            fsm_before = get_automate_state()
            etat_before = int(fsm_before.get("etat", 0))

            wall_info = wall_info_memo

            if camera is not None and etat_before == 1:
                try:
                    frame_bgr = camera.capture_array("main").copy()

                    wall_info = analyze_walls(
                        frame_bgr,
                        band_ratio=float(config.CAMERA_BAND_RATIO),
                        min_ratio=float(config.CAMERA_MIN_RATIO),
                        dominance=float(config.CAMERA_DOMINANCE),
                        unknown_value=int(config.CAMERA_UNKNOWN_VALUE),
                    )

                    wall_info_memo = wall_info

                    if show_camera_window and wall_info is not None:
                        try:
                            cv2_mod = importlib.import_module("cv2")
                            cv2_mod.imshow("Camera murs rouge/vert", wall_info["annotated"])
                            cv2_mod.waitKey(1)
                        except Exception:
                            pass

                except Exception as exc:
                    logger.warning("Erreur lecture camera: %s", exc)

            else:
                wall_info = None
                wall_info_memo = None

            wall_values = wall_info["value"] if wall_info else None

            v_cmd, angle_cmd = calculer_commande_automate(
            tableau_lidar_filtre,
            L_entraxe=config.L_ENTRAXE_M,
            W_empattement=config.W_EMPATTEMENT_M,
            maxangle_degre=config.ANGLE_DEGRE_MAX,
            dmax=config.LIDAR_DMAX_MM,
            v_min=config.VITESSE_AUTO_MIN_M_S,
            v_max=config.VITESSE_AUTO_MAX_M_S,
            seuil_v_blocage=config.SEUIL_V_BLOCAGE_M_S,
            wall_values=wall_values,
            sec_front_fenetre_deg=config.SECURITE_FRONT_FENETRE_DEG,
            sec_front_min_points=config.SECURITE_FRONT_MIN_POINTS,
            seuil_front_degagement_mm=config.SEUIL_FRONT_DEGAGEMENT_MM,
            blocage_action_duration_s=config.BLOCAGE_ACTION_DURATION_S,
            boucle_periode_s=config.BOUCLE_PERIODE_S,
            vitesse_blocage_m_s=config.VITESSE_BLOCAGE_M_S,
            vitesse_turn_blocage_m_s=config.VITESSE_TURN_BLOCAGE_M_S,
            angle_recul_fixe_deg=config.ANGLE_RECUL_FIXE_DEG,
            debug=bool(config.AUTO_DEBUG),
         )
            act.set_direction_degre(float(angle_cmd))
            act.set_vitesse_m_s(float(v_cmd))
            

            if bool(config.DEBUG_ACTIONNEURS):
                fsm = get_automate_state()
                etat = int(fsm.get("etat", 0))
                sous_etat = int(fsm.get("sous_etat", 0))

                d_front = fsm.get("last_front_mm", None)
                d_rear = fsm.get("last_rear_mm", None)

                front_txt = "NA" if d_front is None else f"{float(d_front):.0f}"
                rear_txt = "NA" if d_rear is None else f"{float(d_rear):.0f}"
                wall_txt = "None" if wall_values is None else str(wall_values)

                logger.info(
                    "FSM [%s|%s] v=%.2f ang=%.1f dF=%s dR=%s back=%d turn_right=%s walls=%s",
                    ETAT_NAMES.get(etat, "?"),
                    SOUS_ETAT_NAMES.get(sous_etat, "-") if etat == 1 else "-",
                    float(v_cmd),
                    float(angle_cmd),
                    front_txt,
                    rear_txt,
                    int(fsm.get("counter_etat_backward", 0)),
                    str(bool(fsm.get("flag_turn_right", False))),
                    wall_txt,
                )

    except KeyboardInterrupt:
        logger.info("Interruption clavier recue")
        stop_event.set()

    finally:
        logger.info("Arrêt propre en cours...")

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

        try:
            lidar.arreter()
        except Exception:
            pass

        logger.info("Programme terminé.")
        print("Au revoir.")


if __name__ == "__main__":
    main()