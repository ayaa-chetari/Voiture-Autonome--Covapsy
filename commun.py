import numpy as np
import cv2


def filtre_moyenneur(tab, fenetre=2):
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

        tab_filtre[i] = somme / count if count > 0 else 0

    return tab_filtre


def lire_point_lidar(tab, angle_deg, valeur_defaut=3000.0, fenetre_deg=10, min_points=2):
    idx = int(angle_deg)

    if idx < -180:
        idx += 360
    elif idx > 179:
        idx -= 360

    valeurs = []
    for delta in range(-fenetre_deg, fenetre_deg + 1):
        k = (idx + delta) % 360
        valeur = tab[k]
        if 0 < valeur <= valeur_defaut:
            valeurs.append(valeur)

    if len(valeurs) < int(min_points):
        return valeur_defaut

    return float(np.median(valeurs))


def normaliser_distance(d, dmax):
    d = max(0.0, min(d, dmax))
    return d / dmax


def distance_secteur_lidar(tab_mm, centre_deg, demi_fenetre_deg=15,
                           dmax=3000.0, min_points=5, quantile=20):
    centre = int(centre_deg) % 360
    fen = int(max(0, demi_fenetre_deg))

    valeurs = []

    for delta in range(-fen, fen + 1):
        idx = (centre + delta) % 360
        d = float(tab_mm[idx])

        if 0.0 < d <= float(dmax):
            valeurs.append(d)

    if len(valeurs) < int(min_points):
        return None

    q = float(np.clip(quantile, 0.0, 100.0))
    return float(np.percentile(valeurs, q))


def detect_color_hsv(roi_bgr, min_ratio=0.03, dominance=1.15, unknown_value=-1):
    hsv = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2HSV)

    lower_red1 = np.array([0, 90, 60], dtype=np.uint8)
    upper_red1 = np.array([10, 255, 255], dtype=np.uint8)
    lower_red2 = np.array([170, 90, 60], dtype=np.uint8)
    upper_red2 = np.array([180, 255, 255], dtype=np.uint8)

    lower_green = np.array([40, 70, 50], dtype=np.uint8)
    upper_green = np.array([95, 255, 255], dtype=np.uint8)

    mask_red1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask_red2 = cv2.inRange(hsv, lower_red2, upper_red2)
    mask_red = cv2.bitwise_or(mask_red1, mask_red2)
    mask_green = cv2.inRange(hsv, lower_green, upper_green)

    kernel = np.ones((3, 3), np.uint8)
    mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_OPEN, kernel)
    mask_green = cv2.morphologyEx(mask_green, cv2.MORPH_OPEN, kernel)

    total = max(1, roi_bgr.shape[0] * roi_bgr.shape[1])
    red_ratio = cv2.countNonZero(mask_red) / total
    green_ratio = cv2.countNonZero(mask_green) / total

    if red_ratio >= float(min_ratio) and red_ratio > green_ratio * float(dominance):
        return "rouge", 0, red_ratio, green_ratio

    if green_ratio >= float(min_ratio) and green_ratio > red_ratio * float(dominance):
        return "vert", 1, red_ratio, green_ratio

    return "inconnu", int(unknown_value), red_ratio, green_ratio


def analyze_walls(image_bgr, band_ratio=0.30, min_ratio=0.03,
                  dominance=1.15, unknown_value=-1):
    if image_bgr is None or image_bgr.size == 0:
        return None

    h, w, _ = image_bgr.shape
    out = image_bgr.copy()

    band_h = max(1, int(h * float(band_ratio)))
    y1 = max(0, (h - band_h) // 2)
    y2 = min(h, y1 + band_h)
    band = image_bgr[y1:y2, :].copy()

    third = max(1, w // 3)

    rois = [
        band[:, 0:third],
        band[:, third:2 * third],
        band[:, 2 * third:w],
    ]

    colors = []
    values = []
    stats = []

    for roi in rois:
        color_name, bit_value, red_ratio, green_ratio = detect_color_hsv(
            roi,
            min_ratio=min_ratio,
            dominance=dominance,
            unknown_value=unknown_value,
        )

        colors.append(color_name)
        values.append(bit_value)
        stats.append((red_ratio, green_ratio))

    cv2.rectangle(out, (0, y1), (w - 1, y2 - 1), (255, 255, 255), 2)
    cv2.line(out, (third, y1), (third, y2), (255, 255, 255), 2)
    cv2.line(out, (2 * third, y1), (2 * third, y2), (255, 255, 255), 2)

    labels = ["G", "C", "D"]

    for i, name in enumerate(colors):
        x_text = 10 + i * third
        r_ratio, g_ratio = stats[i]

        cv2.putText(
            out,
            f"{labels[i]}: {name} -> {values[i]}",
            (x_text, max(25, y1 - 30)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255, 255, 255),
            2,
        )

        cv2.putText(
            out,
            f"R={r_ratio:.2f} V={g_ratio:.2f}",
            (x_text, max(50, y1 - 5)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (255, 255, 255),
            1,
        )

    cv2.putText(
        out,
        f"bits={values}",
        (10, h - 20),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (0, 255, 255),
        2,
    )

    return {
        "annotated": out,
        "colors": colors,
        "value": values,
    }


def differentiel_vers_ackermann(u_g, u_d, L, W, v_min, v_max, angle_max_deg):
    v_norm = (u_g + u_d) / 2.0
    omega  = (u_d - u_g) / L


    omega_seuil = 1e-4
    if abs(omega) < omega_seuil:
        angle_deg = 0.0
    else:
        R = v_norm / omega if abs(v_norm) > 1e-4 else 1e6
        angle_rad = np.arctan(W / R)
        angle_deg = np.degrees(angle_rad)

    angle_deg = float(np.clip(angle_deg, -angle_max_deg, angle_max_deg))
    ratio = abs(angle_deg) / angle_max_deg
    v_cmd = v_max - (v_max - v_min) * ratio

    return v_cmd, angle_deg

def calculer_commande_auto(tableau_lidar_filtre, L_entraxe, W_empattement, maxangle_degre,
                           dmax=3000.0, v_min=0.0, v_max=0.6, debug=False):
    angle_l1 = 60
    angle_l2 = 70
    angle_front = 0
    angle_r1 = -60
    angle_r2 = -70

    d_l1 = lire_point_lidar(tableau_lidar_filtre, angle_l1, fenetre_deg=4, min_points=2)
    d_l2 = lire_point_lidar(tableau_lidar_filtre, angle_l2, fenetre_deg=4, min_points=2)
    d_front = lire_point_lidar(tableau_lidar_filtre, angle_front, fenetre_deg=4, min_points=2)
    d_r1 = lire_point_lidar(tableau_lidar_filtre, angle_r1, fenetre_deg=4, min_points=2)
    d_r2 = lire_point_lidar(tableau_lidar_filtre, angle_r2, fenetre_deg=4, min_points=2)

    l1 = normaliser_distance(d_l1, dmax)
    l2 = normaliser_distance(d_l2, dmax)
    f = normaliser_distance(d_front, dmax)
    r1 = normaliser_distance(d_r1, dmax)
    r2 = normaliser_distance(d_r2, dmax)

    p_l1 = 1.0 - l1
    p_l2 = 1.0 - l2
    p_f = 1.0 - f
    p_r1 = 1.0 - r1
    p_r2 = 1.0 - r2

    x = np.array([
        1.0,
        p_l1,
        p_l2,
        p_f,
        p_r1,
        p_r2,
    ])

    w_g = np.array([1.2, 0.85, 0.85, -1.9, -0.65, -0.65])
    w_d = np.array([1.2, -0.65, -0.65, -1.9, 0.85, 0.85])

    u_g = np.tanh(np.dot(x, w_g))
    u_d = np.tanh(np.dot(x, w_d))

    v_cmd, angle_cmd = differentiel_vers_ackermann(
        u_g,
        u_d,
        L=L_entraxe,
        W=W_empattement,
        v_min=v_min,
        v_max=v_max,
        angle_max_deg=maxangle_degre,
    )

    if debug:
        print("--------------------------------------------------")
        print(f"d_l1={d_l1:.1f} mm | d_l2={d_l2:.1f} mm")
        print(f"d_front={d_front:.1f} mm")
        print(f"d_r1={d_r1:.1f} mm | d_r2={d_r2:.1f} mm")
        print(f"p_l1={p_l1:.3f} p_l2={p_l2:.3f} p_f={p_f:.3f} p_r1={p_r1:.3f} p_r2={p_r2:.3f}")
        print(f"u_g={u_g:.3f} | u_d={u_d:.3f}")
        print(f"v_cmd={v_cmd:.3f} m/s")
        print(f"angle={angle_cmd:.3f} deg")

    return v_cmd, angle_cmd


_NAVIGATION = 0
_BLOCAGE = 1

_BACKWARD = 0
_TURN_LEFT = 2
_TURN_RIGHT = 3

_fsm = {
    "etat": _NAVIGATION,
    "sous_etat": _BACKWARD,
    "action_counter": 0,
    "counter_etat_backward": 0,
    "flag_turn_right": False,
    "last_front_mm": None,
    "front_missing_steps": 0,
}


def reset_automate():
    _fsm["etat"] = _NAVIGATION
    _fsm["sous_etat"] = _BACKWARD
    _fsm["action_counter"] = 0
    _fsm["counter_etat_backward"] = 0
    _fsm["flag_turn_right"] = False
    _fsm["last_front_mm"] = None
    _fsm["front_missing_steps"] = 0


def get_automate_state():
    return dict(_fsm)


def _distance_front_robuste(tab, centre_deg, demi_fenetre_deg, dmax, min_points):
    d = distance_secteur_lidar(
        tab,
        centre_deg=centre_deg,
        demi_fenetre_deg=int(demi_fenetre_deg),
        dmax=float(dmax),
        min_points=int(min_points),
        quantile=20,
    )

    if d is not None:
        return float(d)

    d = distance_secteur_lidar(
        tab,
        centre_deg=centre_deg,
        demi_fenetre_deg=max(3, int(demi_fenetre_deg) // 2),
        dmax=float(dmax),
        min_points=max(1, int(min_points) // 3),
        quantile=35,
    )

    if d is not None:
        return float(d)

    d_point = lire_point_lidar(
        tab,
        centre_deg,
        valeur_defaut=-1.0,
        fenetre_deg=max(2, int(demi_fenetre_deg) // 2),
        min_points=1,
    )

    return float(d_point) if d_point > 0 else None


def _choisir_rotation_camera(wall_values, default_turn_right=False):
    if not isinstance(wall_values, (list, tuple)) or len(wall_values) < 3:
        return bool(default_turn_right)

    red_count = sum(1 for v in wall_values if v == 0)
    green_count = sum(1 for v in wall_values if v == 1)

    if red_count > green_count:
        return True

    if green_count > red_count:
        return False

    return bool(default_turn_right)


def calculer_commande_automate(
    tableau_lidar_filtre,
    L_entraxe,
    W_empattement,
    maxangle_degre,
    dmax=3000.0,
    v_min=0.0,
    v_max=0.6,
    seuil_v_blocage=0.02,
    wall_values=None,
    sec_front_fenetre_deg=4,
    sec_front_min_points=5,
    seuil_front_degagement_mm=600.0,
    blocage_action_duration_s=0.04,
    boucle_periode_s=0.01,
    vitesse_blocage_m_s=0.6,
    vitesse_turn_blocage_m_s=0.5,
    angle_recul_fixe_deg=18.0,
    debug=False,
):
    v_base, angle_base = calculer_commande_auto(
        tableau_lidar_filtre,
        L_entraxe=L_entraxe,
        W_empattement=W_empattement,
        maxangle_degre=maxangle_degre,
        dmax=dmax,
        v_min=v_min,
        v_max=v_max,
        debug=debug,
    )

    d_front = _distance_front_robuste(
        tableau_lidar_filtre,
        centre_deg=0,
        demi_fenetre_deg=sec_front_fenetre_deg,
        dmax=dmax,
        min_points=sec_front_min_points,
    )

    if d_front is None:
        _fsm["front_missing_steps"] += 1
        hold_steps = max(1, int(0.30 / max(1e-4, float(boucle_periode_s))))

        if _fsm["front_missing_steps"] <= hold_steps and _fsm["last_front_mm"] is not None:
            d_front = float(_fsm["last_front_mm"])
    else:
        _fsm["front_missing_steps"] = 0

    _fsm["last_front_mm"] = d_front

    blocked_by_stop = float(v_base) <= float(seuil_v_blocage)

    can_exit_blockage = (
        d_front is not None
        and d_front > float(seuil_front_degagement_mm)
    )

    if wall_values is not None:
        _fsm["flag_turn_right"] = _choisir_rotation_camera(
            wall_values,
            default_turn_right=_fsm["flag_turn_right"],
        )

    action_steps = max(
        1,
        int(float(blocage_action_duration_s) / max(1e-4, float(boucle_periode_s))),
    )

    v_backward = abs(float(vitesse_blocage_m_s))
    v_turn = abs(float(vitesse_turn_blocage_m_s))

    v_out = 0.0
    angle_out = 0.0

    match _fsm["etat"]:
        case 0:
            if blocked_by_stop:
                _fsm["etat"] = _BLOCAGE
                _fsm["sous_etat"] = _BACKWARD
                _fsm["action_counter"] = 0
                v_out = 0.0
                angle_out = 0.0
            else:
                v_out = float(v_base)
                angle_out = float(angle_base)

        case 1:
            match _fsm["sous_etat"]:
                case 0:
                    if _fsm["action_counter"] >= action_steps:
                        _fsm["action_counter"] = 0

                        if _fsm["flag_turn_right"]:
                            _fsm["sous_etat"] = _TURN_RIGHT
                            angle_out = -float(angle_recul_fixe_deg)
                        else:
                            _fsm["sous_etat"] = _TURN_LEFT
                            angle_out = float(angle_recul_fixe_deg)

                        v_out = -v_turn
                    else:
                        _fsm["action_counter"] += 1
                        v_out = -v_backward

                        if _fsm["flag_turn_right"]:
                            angle_out = float(angle_recul_fixe_deg)
                        else:
                            angle_out = -float(angle_recul_fixe_deg)

                case 2:
                    if _fsm["action_counter"] >= action_steps:
                        _fsm["action_counter"] = 0

                        if can_exit_blockage:
                            _fsm["etat"] = _NAVIGATION
                            _fsm["sous_etat"] = _BACKWARD
                            _fsm["counter_etat_backward"] = 0
                            v_out = float(v_base)
                            angle_out = float(angle_base)
                        else:
                            _fsm["sous_etat"] = _BACKWARD
                            _fsm["counter_etat_backward"] += 1
                            _fsm["flag_turn_right"] = True
                            v_out = -v_backward
                            angle_out = -float(angle_recul_fixe_deg)

                    else:
                        _fsm["action_counter"] += 1
                        v_out = -v_turn
                        angle_out = float(angle_recul_fixe_deg)

                case 3:
                    if _fsm["action_counter"] >= action_steps:
                        _fsm["action_counter"] = 0

                        if can_exit_blockage:
                            _fsm["etat"] = _NAVIGATION
                            _fsm["sous_etat"] = _BACKWARD
                            _fsm["counter_etat_backward"] = 0
                            v_out = float(v_base)
                            angle_out = float(angle_base)
                        else:
                            _fsm["sous_etat"] = _BACKWARD
                            _fsm["counter_etat_backward"] += 1
                            _fsm["flag_turn_right"] = False
                            v_out = -v_backward
                            angle_out = float(angle_recul_fixe_deg)

                    else:
                        _fsm["action_counter"] += 1
                        v_out = -v_turn
                        angle_out = -float(angle_recul_fixe_deg)

                case _:
                    _fsm["sous_etat"] = _BACKWARD
                    _fsm["action_counter"] = 0
                    v_out = 0.0
                    angle_out = 0.0

        case _:
            reset_automate()
            v_out = 0.0
            angle_out = 0.0

    return float(v_out), float(angle_out)
