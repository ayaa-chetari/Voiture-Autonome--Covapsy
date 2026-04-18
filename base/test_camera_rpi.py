"""Script de test pour Raspberry Pi Camera Module 2.

Usage:
    python test_camera_rpi.py

Commandes clavier dans la fenetre OpenCV:
    q : quitter
    s : sauvegarder une image JPG
"""

from datetime import datetime

import cv2

try:
    from picamera2 import Picamera2
except Exception as exc:
    raise SystemExit(
        "picamera2 est introuvable. Installe-le sur Raspberry Pi avec:\n"
        "  sudo apt update\n"
        "  sudo apt install -y python3-picamera2 python3-opencv\n"
        f"\nErreur detaillee: {exc}"
    )


def main():
    camera = Picamera2()

    # Configuration preview simple pour verifier le flux video rapidement.
    config = camera.create_preview_configuration(
        main={"size": (640, 480), "format": "RGB888"}
    )
    camera.configure(config)
    camera.start()

    print("Camera demarree (Raspberry Pi Camera Module 2)")
    print("Touches: 'q' pour quitter, 's' pour sauver une image")

    try:
        while True:
            frame = camera.capture_array()
            cv2.imshow("Test Camera Module 2", frame)

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
            if key == ord("s"):
                filename = f"camera_test_{datetime.now().strftime('%Y%m%d_%H%M%S')}.jpg"
                cv2.imwrite(filename, frame)
                print(f"Image enregistree: {filename}")
    finally:
        camera.stop()
        cv2.destroyAllWindows()
        print("Camera arretee")


if __name__ == "__main__":
    main()
