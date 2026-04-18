# Copyright 1996-2022 Cyberbotics Ltd.
#
# Controle de la voiture TT-02 simulateur CoVAPSy pour Webots 2023b
# Adaptation : contrôleur neuronal virtuel différentiel
# reprojeté en commandes Ackermann
# + filtrage moyenneur des données LiDAR
# + utilisation de 5 points exacts :
#   gauche: 60° et 70°
#   front : 0°
#   droite: -60° et -70°

from vehicle import Driver
from controller import Lidar
import numpy as np

driver = Driver()

basicTimeStep = int(driver.getBasicTimeStep())
sensorTimeStep = 4 * basicTimeStep

# =========================
# Initialisation du LiDAR
# =========================
lidar = Lidar("RpLidarA2")
lidar.enable(sensorTimeStep)
lidar.enablePointCloud()

# =========================
# Initialisation clavier
# =========================
keyboard = driver.getKeyboard()
keyboard.enable(sensorTimeStep)

# =========================
# Paramètres véhicule
# =========================
maxSpeed = 28  # km/h
maxangle_degre = 30

driver.setSteeringAngle(0)
driver.setCruisingSpeed(0)

tableau_lidar_mm = [0] * 360

# =========================
# Fonctions véhicule
# =========================
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

def recule():
    driver.setCruisingSpeed(-1)

# =========================
# Fonctions traitement LiDAR
# =========================
def filtre_moyenneur(tab, fenetre=2):
    """
    Filtre moyenneur circulaire.
    fenetre=2 => moyenne sur 5 points.
    Ignore les valeurs nulles.
    """
    n = len(tab)
    tab_filtre = [0] * n

    for i in range(n):
        somme = 0.0
        count = 0

        for k in range(-fenetre, fenetre + 1):
            idx = (i + k) % n
            val = tab[idx]

            if val > 0:
                somme += val
                count += 1

        if count > 0:
            tab_filtre[i] = somme / count
        else:
            tab_filtre[i] = 0

    return tab_filtre

def lire_point_lidar(tab, angle_deg, valeur_defaut=3000.0):
    """
    Lit un point exact du lidar pour un angle donné.
    Les indices sont accessibles dans [-180, 179].
    """
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

# =========================
# Mode de fonctionnement
# =========================
modeAuto = False

print("Cliquer sur la vue 3D pour commencer")
print("a : mode auto")
print("n : stop")

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
                print("-------- Mode Auto Désactivé -------")

        elif currentKey == ord('a') or currentKey == ord('A'):
            if not modeAuto:
                modeAuto = True
                print("-------- Mode Auto Activé -------")

    # =========================
    # Acquisition LiDAR brut
    # =========================
    donnees_lidar_brutes = lidar.getRangeImage()

    for i in range(360):
        if (donnees_lidar_brutes[-i] > 0) and (donnees_lidar_brutes[-i] < 20):
            tableau_lidar_mm[i - 180] = 1000 * donnees_lidar_brutes[-i]
        else:
            tableau_lidar_mm[i - 180] = 0

    # =========================
    # Filtre moyenneur AVANT normalisation
    # =========================
    tableau_lidar_filtre = filtre_moyenneur(tableau_lidar_mm, fenetre=2)

    # =========================
    # Mode manuel / arrêt
    # =========================
    if not modeAuto:
        set_direction_degre(0)
        set_vitesse_m_s(0)
        continue

    # =========================================================
    # Programme auto : 5 points exacts + réseau + Ackermann
    # =========================================================

    # Angles des points pertinents
    angle_l1 = 60
    angle_l2 = 70
    angle_front = 0
    angle_r1 = -60
    angle_r2 = -70

    # 1) Lecture des 5 points exacts
    d_l1 = lire_point_lidar(tableau_lidar_filtre, angle_l1)
    d_l2 = lire_point_lidar(tableau_lidar_filtre, angle_l2)
    d_front = lire_point_lidar(tableau_lidar_filtre, angle_front)
    d_r1 = lire_point_lidar(tableau_lidar_filtre, angle_r1)
    d_r2 = lire_point_lidar(tableau_lidar_filtre, angle_r2)

    # 2) Normalisation
    dmax = 3000.0  # mm
    l1 = normaliser_distance(d_l1, dmax)
    l2 = normaliser_distance(d_l2, dmax)
    f = normaliser_distance(d_front, dmax)
    r1 = normaliser_distance(d_r1, dmax)
    r2 = normaliser_distance(d_r2, dmax)

    # 3) Conversion en proximité
    p_l1 = 1.0 - l1
    p_l2 = 1.0 - l2
    p_f = 1.0 - f
    p_r1 = 1.0 - r1
    p_r2 = 1.0 - r2

    # 4) Vecteur d'entrée du réseau
    x = np.array([1.0, p_l1, p_l2, p_f, p_r1, p_r2])

    # 5) Réseau virtuel différentiel
    # Poids proposés pour comportement d'évitement
    w_g = np.array([1.2,  0.8,  0.8, -1.6, -0.6, -0.6])
    w_d = np.array([1.2, -0.6, -0.6, -1.6,  0.8,  0.8])

    u_g = np.tanh(np.dot(x, w_g))
    u_d = np.tanh(np.dot(x, w_d))

    # 6) Projection vers Ackermann (approche B)
    v_virtual = (u_g + u_d) / 2.0
    delta_virtual = (u_d - u_g)

    # Si le sens de rotation n'est pas correct, utiliser à la place :
    # delta_virtual = (u_g - u_d)

    # 7) Vitesse
    v_min = 0.4
    v_max = 1.2
    v_cmd = v_min + (v_max - v_min) * max(0.0, v_virtual)

    # 8) Direction
    K_delta = 12.0
    angle_cmd = K_delta * delta_virtual
    angle_cmd = np.clip(angle_cmd, -maxangle_degre, maxangle_degre)

    # 9) Commande véhicule
    set_direction_degre(angle_cmd)
    set_vitesse_m_s(v_cmd)

    # =========================
    # Debug
    # =========================
    print("--------------------------------------------------")
    print(f"d_l1     = {d_l1:.1f} mm")
    print(f"d_l2     = {d_l2:.1f} mm")
    print(f"d_front  = {d_front:.1f} mm")
    print(f"d_r1     = {d_r1:.1f} mm")
    print(f"d_r2     = {d_r2:.1f} mm")
    print(f"p_l1     = {p_l1:.3f}")
    print(f"p_l2     = {p_l2:.3f}")
    print(f"p_f      = {p_f:.3f}")
    print(f"p_r1     = {p_r1:.3f}")
    print(f"p_r2     = {p_r2:.3f}")
    print(f"u_g      = {u_g:.3f}")
    print(f"u_d      = {u_d:.3f}")
    print(f"v_cmd    = {v_cmd:.3f} m/s")
    print(f"angle    = {angle_cmd:.3f} deg")