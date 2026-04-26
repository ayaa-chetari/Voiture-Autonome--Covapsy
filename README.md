# CoVAPSy — Voiture autonome

## Description

Ce projet consiste à développer une voiture autonome miniature basée sur un Raspberry Pi, capable de naviguer dans une piste sans toucher les murs.

Le système repose sur :
- un LiDAR pour la perception de l’environnement
- une caméra Raspberry Pi (module 2) pour l’analyse de la couleur des murs
- des actionneurs (moteur et direction) pilotés en PWM
- un algorithme de navigation basé sur un réseau de neurones artificiel simple

---

## Principe de navigation

L’algorithme principal repose sur un réseau de neurones artificiel :

- 9 entrées issues des mesures du LiDAR
- 2 neurones de sortie :  vitesse droite (v_d) + vitesse gauche (v_g)



Ce réseau est basé sur un modèle de robot différentiel.  
Il produit directement deux vitesses indépendantes (gauche et droite) en fonction de l’environnement.

---

## Conversion vers le modèle Ackermann

La voiture étant de type Ackermann (direction par les roues avant), les vitesses différentielles sont converties en :

- une vitesse linéaire
- un angle de braquage

- si v_g ≈ v_d → la voiture avance tout droit
- si v_g ≠ v_d → un angle de braquage est généré


---

## Gestion des blocages

Une machine à états est utilisée lorsque la voiture est bloquée (face à un mur ou un obstacle).

Dans ce cas :
- la voiture effectue des manœuvres de recul et de rotation

---


## Traitement d'image

La caméra Raspberry Pi (module 2) est utilisée avec OpenCV pour analyser la couleur des murs.

Le traitement suit les étapes suivantes :

1. Conversion de l’image de l’espace RGB vers l’espace HSV  
   → permet de réduire l’impact des variations de luminosité et d’ombres

2. Sélection d’une zone d’intérêt au centre de l’image  

3. Découpage de cette zone en plusieurs parties (gauche, centre, droite)

4. Détection des couleurs rouge et vert à l’aide de seuils HSV

5. Calcul du ratio de pixels de chaque couleur  

Le résultat permet de savoir si les murs sont rouges ou verts et d’utiliser cette information pour choisir le sens de rotation en cas de blocage.

---

## Structure du projet

### voiture-reelle.py
- boucle principale
- gestion du mode autonome
- lecture des capteurs
- envoi des commandes aux actionneurs

### commun.py
- traitement des données LiDAR
- traitement d’image (détection couleur)
- algorithme de navigation
- gestion des situations de blocage

### config.py
- paramètres du système 

### robot_base.py
- interface avec le matériel :
  - moteurs (PWM)
  - direction
  - LiDAR



