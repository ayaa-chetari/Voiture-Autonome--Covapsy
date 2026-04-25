import os
import time
import cv2
from picamera2 import Picamera2

from commun_v1 import analyze_walls

DOSSIER_SORTIE = "camera_debug"


def main():
    os.makedirs(DOSSIER_SORTIE, exist_ok=True)

    cam = Picamera2()
    config = cam.create_preview_configuration(
        main={"size": (128, 96), "format": "RGB888"},
        queue=False,
    )
    cam.configure(config)
    cam.start()
    time.sleep(1.0)

    print("Camera demarree.")
    print("Test avec le meme traitement que dans commun_v1.analyze_walls()")
    print("Une capture toutes les 2 secondes. Ctrl+C pour arreter.\n")

    compteur = 0

    try:
        while True:
            frame_bgr = cam.capture_array("main")

            wall_info = analyze_walls(
                frame_bgr,
                band_ratio=0.30,
                min_ratio=0.03,
                dominance=1.15,
                unknown_value=-1,
            )

            chemin_brut = os.path.join(DOSSIER_SORTIE, f"frame_{compteur:03d}.jpg")
            cv2.imwrite(chemin_brut, frame_bgr)

            if wall_info is not None:
                chemin_annote = os.path.join(DOSSIER_SORTIE, f"frame_{compteur:03d}_annotated.jpg")
                cv2.imwrite(chemin_annote, wall_info["annotated"])

                print(f"[{compteur:03d}] colors={wall_info['colors']} bits={wall_info['value']}")
                print(f"      image brute   : {chemin_brut}")
                print(f"      image annotee : {chemin_annote}")
            else:
                print(f"[{compteur:03d}] analyse impossible")
                print(f"      image brute   : {chemin_brut}")

            compteur += 1
            time.sleep(2.0)

    except KeyboardInterrupt:
        print("\nArret du test.")
    finally:
        cam.stop()


if __name__ == "__main__":
    main()