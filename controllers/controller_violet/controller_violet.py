# Copyright 1996-2022 Cyberbotics Ltd.
#
# Controle de la voiture TT-02 simulateur CoVAPSy pour Webots 2023b
# Adaptation : contrôleur neuronal virtuel différentiel
# reprojeté en commandes Ackermann
# + filtrage moyenneur des données LiDAR
# + logique de suivi murs conservée
# + évitement obstacle : déclencheur + mémoire récurrente
#   côté d'évitement fixe (toujours à droite)
# + détection arrêt via accéléromètre
# + automate de recul déclenché si arrêt confirmé

from vehicle import Driver
from controller import Lidar
import numpy as np
import cv2

from imageProcessing import webots_image_to_bgr, analyze_walls

driver = Driver()

basicTimeStep = int(driver.getBasicTimeStep())
sensorTimeStep = 4 * basicTimeStep

# =========================
# Initialisation caméra
# =========================
camera = driver.getDevice("pi_camera")
camera_ok = False

if camera is None:
    print("Camera non trouvée : pi_camera")
else:
    camera.enable(sensorTimeStep)
    camera_ok = True
    print("Camera trouvée :", camera.getName())
    print("Resolution camera :", camera.getWidth(), "x", camera.getHeight())

# =========================
# Initialisation LiDAR
# =========================
lidar = Lidar("RpLidarA2")
lidar.enable(sensorTimeStep)
lidar.enablePointCloud()

# =========================
# Initialisation accéléromètre
# =========================
accelerometer = driver.getDevice("accelerometer")
accel_ok = False

if accelerometer is None:
    print("Accéléromètre non trouvé")
else:
    accelerometer.enable(sensorTimeStep)
    accel_ok = True
    print("Accéléromètre initialisé")

# =========================
# Initialisation clavier
# =========================
keyboard = driver.getKeyboard()
keyboard.enable(sensorTimeStep)

# =========================
# Paramètres véhicule
# =========================
maxSpeed       = 50
maxangle_degre = 18
L_entraxe      = 0.180
W_empattement  = 0.250

driver.setSteeringAngle(0)
driver.setCruisingSpeed(0)

tableau_lidar_mm = [0] * 360

# =========================
# Mémoire récurrente évitement
# =========================
p_fL_brut_memo = 0.0
p_fR_brut_memo = 0.0
alpha          = 0.80

# =========================
# Détection arrêt accéléromètre
# =========================
SEUIL_ARRET     = 0.07
NB_CYCLES_ARRET = 10
compteur_arret  = 0
voiture_arretee = False
norme_horiz     = 0.0

# =========================
# Automate de recul
# =========================
_NAVIGATION = 0
_BLOCAGE    = 1
_BACKWARD   = 0
_TURN_LEFT  = 2
_TURN_RIGHT = 3

BLOCAGE_ACTION_DURATION_S = 0.5
BOUCLE_PERIODE_S          = basicTimeStep * 4 / 1000.0
VITESSE_BLOCAGE_M_S       = 0.6
VITESSE_TURN_BLOCAGE_M_S  = 0.5
ANGLE_RECUL_FIXE_DEG      = 18.0
SEUIL_FRONT_DEGAGEMENT_MM = 500.0

action_steps = max(1, int(BLOCAGE_ACTION_DURATION_S / max(1e-4, BOUCLE_PERIODE_S)))

fsm = {
    "etat":                  _NAVIGATION,
    "sous_etat":             _BACKWARD,
    "action_counter":        0,
    "counter_etat_backward": 0,
    "flag_turn_right":       False,
}

def reset_automate():
    fsm["etat"]                  = _NAVIGATION
    fsm["sous_etat"]             = _BACKWARD
    fsm["action_counter"]        = 0
    fsm["counter_etat_backward"] = 0
    fsm["flag_turn_right"]       = False

def choisir_rotation_camera(wall_colors):
    if not isinstance(wall_colors, (list, tuple)) or len(wall_colors) < 3:
        return fsm["flag_turn_right"]
    red_count   = sum(1 for v in wall_colors if v == 0)
    green_count = sum(1 for v in wall_colors if v == 1)
    if red_count > green_count:
        return True
    if green_count > red_count:
        return False
    return fsm["flag_turn_right"]

# =========================
# Modes
# =========================
modeAuto       = False
cameraTestMode = False

# =========================
# Fonctions véhicule
# =========================
def set_vitesse_m_s(vitesse_m_s):
    speed = vitesse_m_s * 3.6
    speed = max(-maxSpeed, min(maxSpeed, speed))
    driver.setCruisingSpeed(speed)

def set_direction_degre(angle_degre):
    angle_degre = float(np.clip(angle_degre, -maxangle_degre, maxangle_degre))
    driver.setSteeringAngle(-angle_degre * np.pi / 180.0)

# =========================
# Fonctions traitement LiDAR
# =========================
def filtre_moyenneur(tab, fenetre=2):
    n = len(tab)
    tab_filtre = [0.0] * n
    for i in range(n):
        somme = 0.0
        count = 0
        for k in range(-fenetre, fenetre + 1):
            idx = (i + k) % n
            val = tab[idx]
            if val > 0:
                somme += val
                count += 1
        tab_filtre[i] = somme / count if count > 0 else 0.0
    return tab_filtre

def lire_point_lidar(tab, angle_deg, valeur_defaut=3000.0):
    idx = int(angle_deg)
    if idx < -180:
        idx += 360
    elif idx > 179:
        idx -= 360
    valeur = tab[idx]
    if valeur <= 0:
        return valeur_defaut
    return valeur

def normaliser_distance(d, dmax):
    return max(0.0, min(d, dmax)) / dmax

# =========================
# Conversion différentiel -> Ackermann
# — modèle validé Webots —
# vitesse réduite proportionnellement à l'angle
# =========================
def differentiel_vers_ackermann(u_g, u_d, L, W, v_min, v_max, angle_max_deg):
    v_norm = (u_g + u_d) / 2.0
    omega  = (u_d - u_g) / L

    if abs(omega) < 1e-4:
        angle_deg = 0.0
    else:
        R         = v_norm / omega if abs(v_norm) > 1e-4 else 1e6
        angle_rad = np.arctan(W / R)
        angle_deg = np.degrees(angle_rad)

    angle_deg = float(np.clip(angle_deg, -angle_max_deg, angle_max_deg))

    # Modèle Webots validé : vitesse diminue avec l'angle
    ratio = abs(angle_deg) / angle_max_deg
    v_cmd = v_max - (v_max - v_min) * ratio

    return v_cmd, angle_deg

# =========================
# Commande ANN navigation
# — poids et angles validés Webots —
# =========================
def calculer_commande_navigation(tableau_lidar_filtre,
                                  p_fL_memo, p_fR_memo, alpha):
    dmax     = 3000.0
    dmax_obs = 1000.0

    # Angles LiDAR murs
    d_l1 = lire_point_lidar(tableau_lidar_filtre,  63)
    d_l2 = lire_point_lidar(tableau_lidar_filtre,  73)
    d_f0 = lire_point_lidar(tableau_lidar_filtre,   0)
    d_r1 = lire_point_lidar(tableau_lidar_filtre, -63)
    d_r2 = lire_point_lidar(tableau_lidar_filtre, -73)
    # Angles LiDAR obstacle proche ±6°
    d_fL = lire_point_lidar(tableau_lidar_filtre,   6)
    d_fR = lire_point_lidar(tableau_lidar_filtre,  -6)

    # Proximité murs
    p_l1 = 1.0 - normaliser_distance(d_l1, dmax)
    p_l2 = 1.0 - normaliser_distance(d_l2, dmax)
    p_f0 = 1.0 - normaliser_distance(d_f0, dmax)
    p_r1 = 1.0 - normaliser_distance(d_r1, dmax)
    p_r2 = 1.0 - normaliser_distance(d_r2, dmax)

    # Déclencheur évitement + mémoire de pic
    p_f0_obs  = 1.0 - normaliser_distance(d_f0, dmax_obs)
    p_fL_brut = 1.0 - normaliser_distance(d_fL, dmax_obs)
    p_fR_brut = 1.0 - normaliser_distance(d_fR, dmax_obs)

    p_fL_memo = max(p_fL_brut, alpha * p_fL_memo)
    p_fR_memo = max(p_fR_brut, alpha * p_fR_memo)

    declencheur  = max(p_f0_obs, p_fL_memo, p_fR_memo)
    p_fL_combine = declencheur   # évitement toujours à droite
    p_fR_combine = 0.0

    # Vecteur entrée ANN validé Webots
    # [biais, l1, l2, f0, fLc, fRc, r1, r2]
    x = np.array([1.0, p_l1, p_l2, p_f0, p_fL_combine, p_fR_combine, p_r1, p_r2])

    # Poids validés Webots
    w_g = np.array([ 1.2,  1.00,  0.70, -1.2, -1.20,  1.20, -0.65, -0.90])
    w_d = np.array([ 1.2, -0.90, -0.65, -1.2,  1.20, -1.20,  0.70,  1.00])

    u_g = np.tanh(np.dot(x, w_g))
    u_d = np.tanh(np.dot(x, w_d))

    # Ackermann validé Webots : v_min=0.4, v_max=1.2
    v_cmd, angle_cmd = differentiel_vers_ackermann(
        u_g, u_d,
        L=L_entraxe, W=W_empattement,
        v_min=0.4, v_max=1.2,
        angle_max_deg=maxangle_degre
    )

    debug_vals = {
        "u_g": u_g, "u_d": u_d,
        "declencheur": declencheur,
        "p_fL_combine": p_fL_combine,
        "p_fL_memo": p_fL_memo,
        "p_fR_memo": p_fR_memo,
        "d_fL": d_fL, "d_fR": d_fR, "d_f0": d_f0,
    }

    return v_cmd, angle_cmd, p_fL_memo, p_fR_memo, debug_vals

# =========================
# Automate de recul : une étape
# =========================
def step_automate(voiture_arretee, wall_colors, d_front):
    can_exit = (d_front is not None and d_front > SEUIL_FRONT_DEGAGEMENT_MM)

    if wall_colors is not None:
        fsm["flag_turn_right"] = choisir_rotation_camera(wall_colors)

    v_out     = 0.0
    angle_out = 0.0

    if fsm["etat"] == _NAVIGATION:
        if voiture_arretee:
            fsm["etat"]           = _BLOCAGE
            fsm["sous_etat"]      = _BACKWARD
            fsm["action_counter"] = 0
            print(">>> BLOCAGE DÉTECTÉ — Automate de recul activé")

    elif fsm["etat"] == _BLOCAGE:

        if fsm["sous_etat"] == _BACKWARD:
            if fsm["action_counter"] >= action_steps:
                fsm["action_counter"] = 0
                if fsm["flag_turn_right"]:
                    fsm["sous_etat"] = _TURN_RIGHT
                    print(">>> Recul terminé → TURN RIGHT")
                else:
                    fsm["sous_etat"] = _TURN_LEFT
                    print(">>> Recul terminé → TURN LEFT")
            else:
                fsm["action_counter"] += 1
                v_out     = -VITESSE_BLOCAGE_M_S
                angle_out = ANGLE_RECUL_FIXE_DEG if fsm["flag_turn_right"] else -ANGLE_RECUL_FIXE_DEG

        elif fsm["sous_etat"] == _TURN_LEFT:
            if fsm["action_counter"] >= action_steps:
                fsm["action_counter"] = 0
                if can_exit:
                    print(">>> Dégagement OK → retour NAVIGATION")
                    reset_automate()
                else:
                    fsm["sous_etat"]             = _BACKWARD
                    fsm["counter_etat_backward"] += 1
                    fsm["flag_turn_right"]        = False
                    print(">>> Pas dégagé → nouveau recul")
            else:
                fsm["action_counter"] += 1
                v_out     = -VITESSE_TURN_BLOCAGE_M_S
                angle_out = ANGLE_RECUL_FIXE_DEG

        elif fsm["sous_etat"] == _TURN_RIGHT:
            if fsm["action_counter"] >= action_steps:
                fsm["action_counter"] = 0
                if can_exit:
                    print(">>> Dégagement OK → retour NAVIGATION")
                    reset_automate()
                else:
                    fsm["sous_etat"]             = _BACKWARD
                    fsm["counter_etat_backward"] += 1
                    fsm["flag_turn_right"]        = True
                    print(">>> Pas dégagé → nouveau recul")
            else:
                fsm["action_counter"] += 1
                v_out     = -VITESSE_TURN_BLOCAGE_M_S
                angle_out = -ANGLE_RECUL_FIXE_DEG

    return v_out, angle_out

# =========================
# Programme principal
# =========================
print("Cliquer sur la vue 3D pour commencer")
print("a : mode auto")
print("n : stop")
print("t : test traitement camera")

while driver.step() != -1:

    # =========================
    # Lecture clavier
    # =========================
    while True:
        currentKey = keyboard.getKey()
        if currentKey == -1:
            break
        elif currentKey == ord('n') or currentKey == ord('N'):
            if modeAuto:
                modeAuto = False
                reset_automate()
                print("-------- Mode Auto Désactivé -------")
        elif currentKey == ord('a') or currentKey == ord('A'):
            if not modeAuto:
                modeAuto = True
                reset_automate()
                print("-------- Mode Auto Activé -------")
        elif currentKey == ord('t') or currentKey == ord('T'):
            cameraTestMode = not cameraTestMode
            if cameraTestMode:
                print("-------- Test Camera Activé -------")
            else:
                print("-------- Test Camera Désactivé -------")
                try:
                    cv2.destroyWindow("Camera TT02")
                    cv2.destroyWindow("Bande analyse")
                except:
                    pass

    # =========================
    # Lecture accéléromètre + détection arrêt
    # =========================
    if accel_ok:
        values      = accelerometer.getValues()
        ax          = float(values[0])
        ay          = float(values[1])
        norme_horiz = np.sqrt(ax**2 + ay**2)

        if norme_horiz < SEUIL_ARRET:
            compteur_arret += 1
        else:
            compteur_arret = 0

        voiture_arretee = (compteur_arret >= NB_CYCLES_ARRET)

    # =========================
    # Lecture caméra + analyse murs
    # wall_colors = [gauche, centre, droite]
    #   1=vert  0=rouge  -1=inconnu
    # =========================
    wall_colors = None

    if camera_ok:
        image = camera.getImage()
        if image is not None:
            width     = camera.getWidth()
            height    = camera.getHeight()
            image_bgr = webots_image_to_bgr(image, width, height)
            wall_info = analyze_walls(image_bgr)

            if wall_info is not None:
                wall_colors = wall_info["colors"]

            if cameraTestMode and wall_info is not None:
                cv2.imshow("Camera TT02",   wall_info["original_annotated"])
                cv2.imshow("Bande analyse", wall_info["band"])
                cv2.waitKey(1)

    # =========================
    # Acquisition LiDAR
    # =========================
    donnees_lidar_brutes = lidar.getRangeImage()

    for i in range(360):
        if 0 < donnees_lidar_brutes[-i] < 20:
            tableau_lidar_mm[i - 180] = 1000 * donnees_lidar_brutes[-i]
        else:
            tableau_lidar_mm[i - 180] = 0

    tableau_lidar_filtre = filtre_moyenneur(tableau_lidar_mm, fenetre=2)
    d_front = lire_point_lidar(tableau_lidar_filtre, 0)

    # =========================
    # Mode manuel
    # =========================
    if not modeAuto:
        set_direction_degre(0)
        set_vitesse_m_s(0)
        print(f"CAMERA={wall_colors}  ARRET={voiture_arretee}  norme={norme_horiz:.4f}")
        continue

    # =========================
    # Décision : navigation ANN ou automate de recul
    # =========================
    ETAT_NOMS      = {0: "NAVIGATION", 1: "BLOCAGE"}
    SOUS_ETAT_NOMS = {0: "BACKWARD", 2: "TURN_LEFT", 3: "TURN_RIGHT"}

    if fsm["etat"] == _NAVIGATION:
        v_cmd, angle_cmd, p_fL_brut_memo, p_fR_brut_memo, dbg = calculer_commande_navigation(
            tableau_lidar_filtre, p_fL_brut_memo, p_fR_brut_memo, alpha
        )
        # Vérifie déclenchement blocage
        step_automate(voiture_arretee, wall_colors, d_front)
        # Si basculement immédiat
        if fsm["etat"] == _BLOCAGE:
            v_cmd     = 0.0
            angle_cmd = 0.0

    else:
        # Automate actif : gèle le compteur pour ne pas re-déclencher
        compteur_arret  = 0
        voiture_arretee = False
        v_cmd, angle_cmd = step_automate(False, wall_colors, d_front)
        dbg = {}

    set_direction_degre(angle_cmd)
    set_vitesse_m_s(v_cmd)

    # =========================
    # Debug
    # =========================
    print("--------------------------------------------------")
    print(f"FSM [{ETAT_NOMS.get(fsm['etat'],'?')}|"
          f"{SOUS_ETAT_NOMS.get(fsm['sous_etat'],'-')}]  "
          f"counter={fsm['action_counter']}/{action_steps}  "
          f"turn_right={fsm['flag_turn_right']}")
    print(f"v_cmd={v_cmd:.3f} m/s  |  angle={angle_cmd:.3f} deg")
    print(f"ARRET={voiture_arretee}  compteur_arret={compteur_arret}  "
          f"norme={norme_horiz:.4f} m/s²")
    print(f"d_front={d_front:.0f} mm")
    print(f"CAMERA={wall_colors}")
    if dbg:
        print(f"u_g={dbg['u_g']:.3f}  u_d={dbg['u_d']:.3f}  "
              f"declencheur={dbg['declencheur']:.2f}  "
              f"p_fL_memo={dbg['p_fL_memo']:.2f}  p_fR_memo={dbg['p_fR_memo']:.2f}")
        print(f"d_fL={dbg['d_fL']:.0f}mm  d_fR={dbg['d_fR']:.0f}mm  "
              f"d_f0={dbg['d_f0']:.0f}mm")