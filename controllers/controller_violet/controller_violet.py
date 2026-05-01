
from vehicle import Driver
from controller import Lidar
import numpy as np
import cv2

from imageProcessing import webots_image_to_bgr, analyze_walls

driver = Driver()

basicTimeStep = int(driver.getBasicTimeStep())
sensorTimeStep = 4 * basicTimeStep

# Initialisation caméra
camera = driver.getDevice("pi_camera")
camera_ok = False

if camera is None:
    print("Camera non trouvée : pi_camera")
else:
    camera.enable(sensorTimeStep)
    camera_ok = True
    print("Camera trouvée :", camera.getName())
    print("Resolution camera :", camera.getWidth(), "x", camera.getHeight())

# Initialisation LiDAR
lidar = Lidar("RpLidarA2")
lidar.enable(sensorTimeStep)
lidar.enablePointCloud()

# Initialisation accéléromètre
accelerometer = driver.getDevice("accelerometer")
accel_ok = False

if accelerometer is None:
    print("Accéléromètre non trouvé")
else:
    accelerometer.enable(sensorTimeStep)
    accel_ok = True
    print("Accéléromètre initialisé")

# Initialisation clavier
keyboard = driver.getKeyboard()
keyboard.enable(sensorTimeStep)

# Paramètres véhicule
maxSpeed = 50
maxangle_degre = 18

L_entraxe = 0.180
W_empattement = 0.250

driver.setSteeringAngle(0)
driver.setCruisingSpeed(0)

tableau_lidar_mm = [0] * 360

# Mémoire récurrente évitement
p_fL_brut_memo = 0.0
p_fR_brut_memo = 0.0
alpha          = 0.8

# Détection arrêt accéléromètre
SEUIL_ARRET     = 0.06   # m/s²
NB_CYCLES_ARRET = 10     # cycles consécutifs sous le seuil
compteur_arret  = 0
voiture_arretee = False
norme_horiz     = 0.0

# Modes
modeAuto = False
cameraTestMode = False

# Fonctions véhicule
def set_vitesse_m_s(vitesse_m_s):
    speed = vitesse_m_s * 3.6
    if speed > maxSpeed:
        speed = maxSpeed
    if speed < 0:
        speed = 0
    driver.setCruisingSpeed(speed)

def set_direction_degre(angle_degre):
    if angle_degre > maxangle_degre:
        angle_degre = maxangle_degre
    elif angle_degre < -maxangle_degre:
        angle_degre = -maxangle_degre
    angle_rad = -angle_degre * np.pi / 180.0
    driver.setSteeringAngle(angle_rad)

# Fonctions traitement LiDAR
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
    d = max(0.0, min(d, dmax))
    return d / dmax

# Conversion différentiel -> Ackermann
def differentiel_vers_ackermann(u_g, u_d, L, W, v_min, v_max, angle_max_deg):
    v_norm = (u_g + u_d) / 2.0
    omega  = (u_d - u_g) / L

    v_cmd = v_max * max(0.0, v_norm)

    omega_seuil = 1e-4
    if abs(omega) < omega_seuil:
        angle_deg = 0.0
    else:
        R = v_norm / omega if abs(v_norm) > 1e-4 else 1e6
        angle_rad = np.arctan(W / R)
        angle_deg = np.degrees(angle_rad)

    angle_deg = float(np.clip(angle_deg, -angle_max_deg, angle_max_deg))
    return v_cmd, angle_deg

print("Cliquer sur la vue 3D pour commencer")
print("a : mode auto")
print("n : stop")
print("t : test traitement camera")

while driver.step() != -1:

    # Lecture clavier
    while True:
        currentKey = keyboard.getKey()
        if currentKey == -1:
            break

        elif currentKey == ord('n') or currentKey == ord('N'):
            if modeAuto:
                modeAuto = False
                print("-------- Mode Auto Désactivé -------")

        elif currentKey == ord('a') or currentKey == ord('A'):
            if not modeAuto:
                modeAuto = True
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

    # Lecture accéléromètre + détection arrêt
    if accel_ok:
        values = accelerometer.getValues()
        ax = float(values[0])
        ay = float(values[1])

        norme_horiz = np.sqrt(ax**2 + ay**2)

        if norme_horiz < SEUIL_ARRET:
            compteur_arret += 1
        else:
            compteur_arret = 0

        voiture_arretee = (compteur_arret >= NB_CYCLES_ARRET)

    # Lecture caméra + analyse murs

    wall_colors = [-1, -1, -1]

    if camera_ok:
        image = camera.getImage()
        if image is not None:
            width  = camera.getWidth()
            height = camera.getHeight()

            image_bgr = webots_image_to_bgr(image, width, height)
            wall_info = analyze_walls(image_bgr)
            wall_colors = wall_info["colors"]

            if cameraTestMode:
                cv2.imshow("Camera TT02",   wall_info["original_annotated"])
                cv2.imshow("Bande analyse", wall_info["band"])
                cv2.waitKey(1)

    # Acquisition LiDAR
    donnees_lidar_brutes = lidar.getRangeImage()

    for i in range(360):
        if (donnees_lidar_brutes[-i] > 0) and (donnees_lidar_brutes[-i] < 20):
            tableau_lidar_mm[i - 180] = 1000 * donnees_lidar_brutes[-i]
        else:
            tableau_lidar_mm[i - 180] = 0

    tableau_lidar_filtre = filtre_moyenneur(tableau_lidar_mm, fenetre=2)

    # =========================
    # Mode manuel
    # =========================
    if not modeAuto:
        set_direction_degre(0)
        set_vitesse_m_s(0)
        print(f"CAMERA vecteur={wall_colors}  (1=vert  0=rouge  -1=inconnu)")
        print(f"ARRET={voiture_arretee}  compteur={compteur_arret}  norme={norme_horiz:.4f} m/s²")
        continue

    # Angles LiDAR 
    angle_l1 =  63
    angle_l2 =  73
    angle_f0 =   0
    angle_r1 = -63
    angle_r2 = -73

    # Angles LiDAR — obstacle proche
    angle_fL =  6
    angle_fR = -6

    # Lecture distances
    d_l1 = lire_point_lidar(tableau_lidar_filtre, angle_l1)
    d_l2 = lire_point_lidar(tableau_lidar_filtre, angle_l2)
    d_f0 = lire_point_lidar(tableau_lidar_filtre, angle_f0)
    d_r1 = lire_point_lidar(tableau_lidar_filtre, angle_r1)
    d_r2 = lire_point_lidar(tableau_lidar_filtre, angle_r2)

    d_fL = lire_point_lidar(tableau_lidar_filtre, angle_fL)
    d_fR = lire_point_lidar(tableau_lidar_filtre, angle_fR)

    dmax = 3000.0
    p_l1 = 1.0 - normaliser_distance(d_l1, dmax)
    p_l2 = 1.0 - normaliser_distance(d_l2, dmax)
    p_f0 = 1.0 - normaliser_distance(d_f0, dmax)
    p_r1 = 1.0 - normaliser_distance(d_r1, dmax)
    p_r2 = 1.0 - normaliser_distance(d_r2, dmax)

   
    dmax_obs = 1000.0

    p_f0_obs  = 1.0 - normaliser_distance(d_f0, dmax_obs)
    p_fL_brut = 1.0 - normaliser_distance(d_fL, dmax_obs)
    p_fR_brut = 1.0 - normaliser_distance(d_fR, dmax_obs)

    p_fL_brut_memo = max(p_fL_brut, alpha * p_fL_brut_memo)
    p_fR_brut_memo = max(p_fR_brut, alpha * p_fR_brut_memo)

    declencheur  = max(p_f0_obs, p_fL_brut_memo, p_fR_brut_memo)
    p_fL_combine = declencheur
    p_fR_combine = 0.0

    # Entrée réseau
    # [biais, l1, l2, f0, fLc, fRc, r1, r2]
    x = np.array([
        1.0,
        p_l1,
        p_l2,
        p_f0,
        p_fL_combine,
        p_fR_combine,
        p_r1,
        p_r2
    ])

    # Réseau virtuel différentiel
    w_g = np.array([ 1.2,  1.00,  0.70, -1.2, -1.20,  1.20, -0.65, -0.90])
    w_d = np.array([ 1.2, -0.90, -0.65, -1.2,  1.20, -1.20,  0.70,  1.00])

    u_g = np.tanh(np.dot(x, w_g))
    u_d = np.tanh(np.dot(x, w_d))

    # Conversion vers Ackermann
    v_cmd, angle_cmd = differentiel_vers_ackermann(
        u_g, u_d,
        L=L_entraxe,
        W=W_empattement,
        v_min=0.4,
        v_max=1.2,
        angle_max_deg=maxangle_degre
    )

    set_direction_degre(angle_cmd)
    set_vitesse_m_s(v_cmd)

    # Debug
    print("--------------------------------------------------")
    print(f"u_g={u_g:.3f}  u_d={u_d:.3f}")
    print(f"v_cmd={v_cmd:.3f} m/s  |  angle={angle_cmd:.3f} deg")
    print(f"declencheur={declencheur:.2f}  p_fL_combine={p_fL_combine:.2f}")
    print(f"p_fL_memo={p_fL_brut_memo:.2f}  p_fR_memo={p_fR_brut_memo:.2f}")
    print(f"d_fL={d_fL:.0f}mm  d_fR={d_fR:.0f}mm  d_f0={d_f0:.0f}mm")
    print(f"CAMERA vecteur={wall_colors}  (1=vert  0=rouge  -1=inconnu)")
    print(f"ARRET={voiture_arretee}  compteur={compteur_arret}  norme={norme_horiz:.4f} m/s²")