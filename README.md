# CoVAPSy — Voiture autonome (Raspberry Pi + LiDAR + Caméra)

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
- 2 neurones de sortie :
  - vitesse gauche (v_g)
  - vitesse droite (v_d)

Ce réseau est basé sur un modèle de robot différentiel.  
Il produit directement deux vitesses indépendantes (gauche et droite) en fonction de l’environnement.

Le principe est le suivant :
- les distances mesurées influencent les neurones
- les obstacles ralentissent un côté du véhicule
- les zones libres favorisent le mouvement

---

## Conversion vers le modèle Ackermann

La voiture étant de type Ackermann (direction par les roues avant), les vitesses différentielles sont converties en :

- une vitesse linéaire
- un angle de braquage

Ainsi :
- si v_g ≈ v_d → la voiture avance tout droit
- si v_g ≠ v_d → un angle de braquage est généré

Cette conversion permet d’adapter le modèle différentiel à un véhicule réel.

---

## Gestion des blocages

Une machine à états est utilisée lorsque la voiture est bloquée (face à un mur ou un obstacle).

Dans ce cas :
- la voiture effectue des manœuvres de recul et de rotation
- la stratégie est simple et réactive pour sortir rapidement de la situation

---

## Rôle de la caméra

La caméra Raspberry Pi (module 2) permet de :

- détecter la couleur des murs (rouge / vert)
- déterminer le sens de rotation en cas de blocage

Cette information permet d’orienter la décision lorsque le LiDAR seul n’est pas suffisant.

---

## Structure du projet
