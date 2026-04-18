# Projet CoVAPSy — Voiture Autonome de A à Z

## Contexte
Tu es l'agent de développement d'une voiture autonome de course pour la compétition CoVAPSy (Paris-Saclay).
La voiture est un châssis Tamiya TT02 piloté par un Raspberry Pi 4 model B.
Tu dois implémenter le logiciel complet de conduite autonome, de la plus simple à la plus avancée.

---

## Matériel cible (Raspberry Pi)

| Composant     | Détails                                                        |
|---------------|----------------------------------------------------------------|
| Lidar         | RPLidar A2M12, port `/dev/ttyUSB0`, **baudrate 115200** (confirmé)  |
| Propulsion    | HardwarePWM channel 0, 50 Hz, duty cycle ~8.1 (stop)          |
| Direction     | HardwarePWM channel 1, 50 Hz, duty cycle ~7.35 (centre)       |
| Librairies    | `rplidar`, `rpi_hardware_pwm`, `numpy`, `threading`, `time`   |

---

## Fichiers existants dans le projet (à lire et réutiliser)

### `commande_PS4.py`
Contrôle manuel complet via manette PS4. À titre de référence pour la logique de contrôle.

> **⚠️ Les paramètres PWM de ce fichier ne sont PAS la référence de calibration finale.**

---

### `conduite_autonome_avec_threads.py`
Architecture avec 2 threads : acquisition lidar et conduite autonome.
Logique de base : évitement murs sur rayons 0°, ±30°, suivi de couloir sur rayons ±60°.

> **→ Utiliser cette architecture multi-thread comme base structurelle.**

---

### `conduite_autonome_basique.py` ✅ RÉFÉRENCE CALIBRATION
Version mono-thread, plus simple. **Paramètres PWM validés sur la voiture réelle :**
- Propulsion : `direction_prop=1`, `pwm_stop_prop=7.36`, `point_mort_prop=0.41`, `delta_pwm_max_prop=1.0`
- Direction : `direction=-1`, `angle_pwm_min=5.5`, `angle_pwm_max=9.3`, `angle_pwm_centre=7.4`

> **→ Ces valeurs sont la référence à utiliser dans `config.py`.**

---

### `pathfinding.py`
Implémentation A* et Dijkstra sur grille 2D avec GUI PySide6.
Classes `Maze`, algorithmes avec coûts et bonus.

> **→ Adapter la logique A* pour la navigation sur carte occupancy grid issue du SLAM.**

---

### `trajectoire_calcul.py`
Contrôleur par linéarisation entrée-sortie pour suivi de trajectoire (x(t), y(t)).
Modèle cinématique unicycle, loi de commande : `u1` (accélération), `u2` (vitesse angulaire).
Variables clés : `lambdax`, `lambday`, `Mux`, `Muy` (gains PD), intégration d'Euler.

> **→ Adapter ce contrôleur pour suivre un chemin calculé par A*.**

---

### `test_lidar.py`, `test_pwm_direction.py`, `test_pwm_propulsion.py`, `raz_lidar.py`
Scripts de test et calibration. Références pour la validation hardware.

---

## Architecture cible — Machine à états

La machine à états doit implémenter les états suivants (inspirée du diagramme fourni, améliorée) :

```
INIT → SET_CMD_ZERO → UPDATE_DIR_VIT → WAIT_10MS
WAIT_10MS --[LIDAR=0, mode facile]---------> CALC_ERREUR_CMD
WAIT_10MS --[LIDAR=1, obstacle détecté]----> EVAL
EVAL      --[MUR=1]-----------------------> SEQUENCE_RECUL → [CMD=0] → WAIT_10MS
EVAL      --[MUR=0, mode moyen/difficile]--> WAIT_10MS
CALC_ERREUR_CMD --[mode difficile]---------> NOUVELLE_TRAJECTOIRE
NOUVELLE_TRAJECTOIRE --> NOUVELLE_CARTOGRAPHIE → [MUR=0] → CALC_ERREUR_CMD
```

### Niveaux de difficulté (implémentation modulaire)

| Niveau      | Description                                                                                         |
|-------------|-----------------------------------------------------------------------------------------------------|
| **FACILE**  | Suivi de couloir réactif (différence rayons ±60°), évitement murs proches, marche arrière auto      |
| **MOYEN**   | Ajout d'une carte d'occupation (occupancy grid 2D), évaluation des obstacles, SLAM basique          |
| **DIFFICILE**| Planification A* sur la carte, suivi de trajectoire avec `trajectoire_calcul.py`, re-planification |

---

## Règlement CoVAPSy — Contraintes critiques à respecter

1. **Démarrage/arrêt à distance** : Implémenter un signal de départ (socket TCP ou GPIO) et un signal d'arrêt. La voiture **NE DOIT PAS** démarrer seule.
2. **Marche arrière obligatoire** : En cas de blocage contre un obstacle >2 s sans voiture derrière, reculer et repartir.
3. **Évitement d'obstacles** : Taille d'une voiture (~400×200 mm). Détection fiable au lidar.
4. **Tours de mise en place** : La voiture peut faire 3 tours de reconnaissance pour construire sa carte (SLAM).
5. **Double sens** : La voiture doit fonctionner dans les deux sens (horaire et trigonométrique).
6. **Détection voiture adverse** : Détecter et éviter (ou doubler) les voitures devant soi sans comportement agressif.
7. **Contre-sens** : Ne pas rouler à contre-sens sur plus de 2 m → arrêt obligatoire si détecté via odométrie.

---

## Plan d'implémentation

### Étape 1 — Refactoring et module de base `robot_base.py`
- Centraliser les paramètres PWM calibrés (depuis `commande_PS4.py`)
- Classes `Actionneurs` (wrap PWM) et `CapteurLidar` (wrap RPLidar avec thread)
- Fonction `recule()` robuste avec timeout
- Démarrage/arrêt via socket TCP (port configurable)

### Étape 2 — Machine à états `state_machine.py`
- Implémenter les états décrits ci-dessus avec un `Enum`
- Boucle principale à 10 ms
- Logique de transition avec drapeaux : `lidar_ok`, `obstacle_detecte`, `mur_detecte`
- Logging des transitions pour debug

### Étape 3 — Module de perception `perception.py`
- Filtrage du tableau lidar (médiane glissante pour réduire le bruit)
- Détection de mur : secteurs avant (±15°), avant-droit (−45°±15°), avant-gauche (+45°±15°)
- Détection obstacle mobile (différence entre deux scans successifs)
- Calcul de la largeur du couloir disponible

### Étape 4 — Conduite réactive `conduite_reactive.py` *(niveau FACILE)*
- Algorithme : `angle = k * (lidar[60] - lidar[-60])`
- Vitesse adaptative : ralentir si obstacle proche (P proportionnel à la distance minimum frontale)
- Séquence de recul : reculer 0.3 s, tourner selon l'espace libre, repartir
- **Doit passer l'homologation CoVAPSy**

### Étape 5 — Cartographie `slam_simple.py` *(niveau MOYEN)*
- Occupancy grid 2D (résolution 50 mm, taille 10 m × 10 m)
- Mise à jour avec les scans lidar (ray casting simple)
- Estimation de la pose par odométrie (intégration vitesse + angle direction)
- Recalage optionnel par ICP simplifié (si temps disponible)
- Visualisation optionnelle via matplotlib (mode debug)

### Étape 6 — Navigation globale `navigation.py` *(niveau DIFFICILE)*
- Extraction du graphe de chemin depuis l'occupancy grid (squelettisation ou grille dilatée)
- Appel à l'algorithme A* (adapter `pathfinding.py`)
- Conversion chemin A* → séquence de waypoints (x, y)
- Suivi de waypoints avec le contrôleur de `trajectoire_calcul.py` (adapter la classe)
- Re-planification si obstacle inattendu détecté

### Étape 7 — Programme principal `main_autonomous.py`
- Instancier tous les modules
- Démarrer les threads (lidar, estimation pose, boucle contrôle)
- Attendre signal de départ TCP
- Lancer la machine à états
- Arrêt propre sur signal TCP ou `KeyboardInterrupt`

### Étape 8 — Tests et validation
- Script `test_integration.py` : simuler les transitions d'états sans matériel (mock lidar)
- Vérifier les cas limites : lidar déconnecté, vitesse nulle coincée, double sens
- Checklist homologation : chaque point du règlement devient un test fonctionnel

---

## Conventions de code

- Python 3.9+, typage avec `typing` uniquement si ça aide la lisibilité
- Pas de dépendances supplémentaires sauf : `rplidar`, `rpi_hardware_pwm`, `numpy`, `threading`, `socket`, `enum`, `logging`
- Les paramètres de calibration doivent être dans un fichier `config.py` séparé
- Chaque module doit fonctionner en standalone avec un `if __name__ == "__main__":` de test
- Utiliser `logging` (pas `print`) pour la production, niveau DEBUG/INFO configurable
- Commenter les formules physiques et les choix algorithmiques non triviaux

---

## Priorités

1. Obtenir d'abord une voiture qui **passe l'homologation** (Étapes 1–4)
2. Ensuite optimiser la vitesse et la robustesse
3. Enfin implémenter le SLAM et A* pour gagner du temps en course

---

## Réponses confirmées — Paramètres définitifs

| Question | Réponse |
|---|---|
| Baudrate lidar | **115200** |
| Calcul déporté | **Non** — RPi 4 Model B seul |
| Calibration valide | **`conduite_autonome_basique.py`** (`direction_prop=1`, `pwm_stop_prop=7.36`, `direction=-1`, `angle_pwm_centre=7.4`) |
| Signal démarrage/arrêt | **`input()` dans le terminal** d'exécution Python |

---

> **Commence par lire tous les fichiers existants, puis propose l'architecture complète avant de coder quoi que ce soit.**
