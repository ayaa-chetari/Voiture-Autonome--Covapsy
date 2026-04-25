# robot_base.py — Classes d'abstraction matérielle : actionneurs et lidar
import logging
import threading
import time

from rpi_hardware_pwm import HardwarePWM
from rplidar import RPLidar

import config

logger = logging.getLogger(__name__)


class Actionneurs:
    """Pilote la propulsion et la direction via HardwarePWM."""

    def __init__(self):
        self._pwm_prop = HardwarePWM(pwm_channel=0, hz=50)
        self._pwm_dir  = HardwarePWM(pwm_channel=1, hz=50)
        self._actif    = False
        self._last_v_cmd = 0.0
        self._last_angle_cmd = 0.0
        self._last_pwm_prop = config.PWM_STOP_PROP
        self._last_pwm_dir = config.ANGLE_PWM_CENTRE
        self._last_debug_print = 0.0

    def _print_debug(self, force=False):
        if not getattr(config, "DEBUG_ACTIONNEURS", False):
            return
        now = time.time()
        period = float(getattr(config, "DEBUG_PRINT_PERIOD_S", 0.5))
        if (not force) and (now - self._last_debug_print < period):
            return
        self._last_debug_print = now
        print(
            "[ACT] "
            f"actif={self._actif} "
            f"v_cmd={self._last_v_cmd:.3f} m/s "
            f"angle_cmd={self._last_angle_cmd:.2f} deg "
            f"pwm_prop={self._last_pwm_prop:.3f} "
            f"pwm_dir={self._last_pwm_dir:.3f}"
        )

    def demarrer(self):
        """Active les deux PWM et place la voiture à l'arrêt, roues droites."""
        self._pwm_prop.start(config.PWM_STOP_PROP)
        self._pwm_dir.start(config.ANGLE_PWM_CENTRE)
        self._actif = True
        self._last_pwm_prop = config.PWM_STOP_PROP
        self._last_pwm_dir = config.ANGLE_PWM_CENTRE
        logger.info("Actionneurs démarrés")
        self._print_debug(force=True)

    def arreter(self):
        """Remet à l'arrêt puis désactive les PWM."""
        if self._actif:
            try:
                self.set_vitesse_m_s(0)
                self.set_direction_degre(0)
            except Exception:
                pass
        self._pwm_prop.stop()
        self._pwm_dir.stop()
        self._actif = False
        self._last_pwm_prop = config.PWM_STOP_PROP
        self._last_pwm_dir = config.ANGLE_PWM_CENTRE
        logger.info("Actionneurs arrêtés")
        self._print_debug(force=True)

    # Commandes de base

    def _appliquer_vitesse_pwm(self, vitesse_m_s: float):
        """Applique directement la consigne vitesse -> PWM propulsion."""
        if vitesse_m_s == 0:
            pwm_prop = config.PWM_STOP_PROP
        elif vitesse_m_s > 0:
            # pwm_stop + direction * (point_mort + fraction_vitesse)
            v = vitesse_m_s * config.DELTA_PWM_MAX_PROP / config.VITESSE_MAX_M_S_HARD
            pwm_prop = config.PWM_STOP_PROP + config.DIRECTION_PROP * (config.POINT_MORT_PROP + v)
        else:
            # Marche arrière : soustraction symétrique
            v = vitesse_m_s * config.DELTA_PWM_MAX_PROP / config.VITESSE_MAX_M_S_HARD
            pwm_prop = config.PWM_STOP_PROP - config.DIRECTION_PROP * (config.POINT_MORT_PROP - v)

        self._last_v_cmd = float(vitesse_m_s)
        self._last_pwm_prop = float(pwm_prop)
        self._pwm_prop.change_duty_cycle(self._last_pwm_prop)

    def _engager_marche_arriere(self):
        """Séquence d'armement ESC pour autoriser la marche arrière."""
        self._appliquer_vitesse_pwm(0.0)
        self._appliquer_vitesse_pwm(-float(config.VITESSE_MAX_M_S_HARD))
        time.sleep(0.2)
        self._appliquer_vitesse_pwm(0.0)
        time.sleep(0.1)

    def set_vitesse_m_s(self, vitesse_m_s: float):
        """Commande la vitesse en m/s. Positif = avant, négatif = arrière."""
        prev_v_cmd = float(self._last_v_cmd)
        # Saturation logicielle
        vitesse_m_s = max(-config.VITESSE_MAX_M_S_HARD,
                          min(config.VITESSE_MAX_M_S_SOFT, vitesse_m_s))

        # Passage vers une consigne negative: armer d'abord le recul du variateur.
        if vitesse_m_s < 0 and prev_v_cmd >= 0:
            self._engager_marche_arriere()

        self._appliquer_vitesse_pwm(vitesse_m_s)
        self._print_debug()

    def set_direction_degre(self, angle_degre: float):
        """Commande l'angle de braquage en degrés. 0 = tout droit.
        +angle_degre_max = gauche, -angle_degre_max = droite.
        """
        self._last_angle_cmd = float(angle_degre)
        # Conversion degrés -> duty cycle robuste même si ANGLE_PWM_MIN/MAX sont inversés.
        low = min(config.ANGLE_PWM_MIN, config.ANGLE_PWM_MAX)
        high = max(config.ANGLE_PWM_MIN, config.ANGLE_PWM_MAX)
        span = high - low
        angle_pwm = (
            config.ANGLE_PWM_CENTRE
            + config.DIRECTION_DIR
            * span
            * angle_degre / (2 * config.ANGLE_DEGRE_MAX)
        )
        angle_pwm = max(low, min(high, angle_pwm))
        self._last_pwm_dir = float(angle_pwm)
        self._pwm_dir.change_duty_cycle(self._last_pwm_dir)
        self._print_debug()

    def recule(self):
        """Séquence de recul : impulsion courte arrière, pause, puis recul lent."""
        logger.info("Séquence de recul déclenchée")
        v_recul = -abs(float(getattr(config, "VITESSE_RECUL_M_S", -4.0)))
        self.set_vitesse_m_s(v_recul)
        time.sleep(config.DUREE_RECUL_S)
        self.set_vitesse_m_s(0)


class CapteurLidar:
   

    def __init__(self):
        self.tableau_mm: list = [0] * 360   # tableau public lu par la logique de conduite
        self._acqui_mm: list  = [0] * 360   # tampon d'acquisition (thread lidar)
        self._nouveau_scan    = False
        self._run             = False
        self._thread          = None
        self._lidar           = None
        self._lock            = threading.Lock()
        self._last_debug_print = 0.0

    def connecter(self):
      
        self._lidar = RPLidar(config.LIDAR_PORT, baudrate=config.LIDAR_BAUDRATE)
        self._lidar.connect()
        logger.info("Lidar connecté : %s", self._lidar.get_info())
        self._lidar.start_motor()
        time.sleep(1)

    def demarrer(self):
        self._run    = True
        self._thread = threading.Thread(target=self._scan_loop, daemon=True, name="lidar_scan")
        self._thread.start()
        logger.info("Thread lidar démarré")

    def arreter(self):
        self._run = False
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=3)
        if self._lidar:
            try:
                self._lidar.stop_motor()
                self._lidar.stop()
                time.sleep(1)
                self._lidar.disconnect()
            except Exception as e:
                logger.warning("Erreur à la déconnexion lidar : %s", e)
        logger.info("Lidar arrêté")

    def lire(self) -> bool:
      
        with self._lock:
            if self._nouveau_scan:
                self.tableau_mm = self._acqui_mm.copy()
                self._nouveau_scan = False
                return True
        return False

    def _scan_loop(self):
        while self._run:
            try:
                ignore_sector = bool(getattr(config, "LIDAR_IGNORE_INTERIOR_SECTOR", False))
                interior_min = int(getattr(config, "LIDAR_INTERIOR_MIN_DEG", 90))
                interior_max = int(getattr(config, "LIDAR_INTERIOR_MAX_DEG", 270))

                # Appel valide sur cette version de rplidar (teste en production locale).
                for scan in self._lidar.iter_scans(scan_type='express'):
                    # Repart d'un tableau vide a chaque scan pour eviter les valeurs stale.
                    scan_mm = [0.0] * 360
                    # Chaque 'scan' est une liste de tuples (quality, angle, distance_mm)
                    for _, angle, distance in scan:
                        if distance <= 0:
                            continue
                        # Optionnel: ignorer un secteur (interieur voiture) selon la config.
                        a = int(angle) % 360
                        if ignore_sector and (interior_min <= a <= interior_max):
                            continue
                        # Correction : décalage de 180° pour que tableau_mm[0] = devant.
                        idx = a % 360 #(180 - a) % 360
                        distance = min(float(distance), float(config.LIDAR_DMAX_MM))
                        if scan_mm[idx] == 0.0:
                            scan_mm[idx] = distance
                        else:
                            # Conserve la mesure la plus proche en cas de doublon d'index.
                            scan_mm[idx] = min(scan_mm[idx], distance)
                    with self._lock:
                        self._acqui_mm = scan_mm
                        self._nouveau_scan = True

                    if getattr(config, "DEBUG_LIDAR_RAW", False):
                        now = time.time()
                        period = float(getattr(config, "DEBUG_PRINT_PERIOD_S", 0.5))
                        if now - self._last_debug_print >= period:
                            self._last_debug_print = now
                            nb_valides = sum(1 for d in scan_mm if d > 0)
                            d0 = scan_mm[0]
                            d_l60 = scan_mm[60]
                            d_l70 = scan_mm[70]
                            d_r60 = scan_mm[300]
                            d_r70 = scan_mm[290]
                            print(
                                "[LIDAR RAW] "
                                f"pts={nb_valides}/360 "
                                f"d0={d0:.0f} d60={d_l60:.0f} d70={d_l70:.0f} "
                                f"d-60={d_r60:.0f} d-70={d_r70:.0f}"
                            )

                    time.sleep(0.005)
                    if not self._run:
                        break
            except Exception as e:
                logger.warning("Erreur acquisition lidar : %s", e)
                time.sleep(0.1)



if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO, format="%(levelname)s %(name)s: %(message)s")

    lidar = CapteurLidar()
    lidar.connecter()
    lidar.demarrer()

    act = Actionneurs()
    act.demarrer()

    try:
        print("Test robot_base.py — Ctrl+C pour arrêter")
        print("Affichage des obstacles à < 2000 mm sur 360°\n")
        while 1:
            if lidar.lire():
                for angle, dist in enumerate(lidar.tableau_mm) :
                    if 0 < dist < 500:
                        print(f"Angle {angle}° : {dist:.0f} mm")
            time.sleep(2)
    except KeyboardInterrupt:
        pass
    finally:
        act.arreter()
        lidar.arreter()
        print("Arrêt propre")
