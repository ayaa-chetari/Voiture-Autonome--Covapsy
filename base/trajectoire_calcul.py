import numpy as np
import matplotlib.pyplot as plt

Masse=1
dt=0.01
tmax=10
temps=np.arange(0, tmax, dt)  # Temps de simulation de 0 à 10 secondes

trajectoire_ref_x=temps  # Trajectoire de référence en x (linéaire)
trajectoire_ref_y=1+np.sin(temps/2)   # Trajectoire de référence en y (sinusoïdale)
#trajectoire_ref_y=temps

Xpos=np.zeros(len(temps))  # Position en x
Ypos=np.zeros(len(temps))  # Position en y

V=np.zeros(len(temps))  # Vitesse en x
V[0]=1 # Vitesse initiale
V[1]=1  # Vitesse initiale pour les différences finies

ray_braq=np.zeros(len(temps))  # Commande de force en x

lambdax=3
lambday=3
Mux=2
Muy=2
u1_plot=np.zeros(len(temps))  # Log pour la commande de force
u2_plot=np.zeros(len(temps))  # Log pour la vitesse

for t in range(2, len(temps)):
    # 1. État actuel du robot (Cinématique)
    # Correction : cos pour x, sin pour y
    xp = V[t-1] * np.cos(ray_braq[t-1]) 
    yp = V[t-1] * np.sin(ray_braq[t-1]) 
    
    # 2. Mise à jour de la position (Intégration d'Euler)
    Xpos[t] = Xpos[t-1] + xp * dt
    Ypos[t] = Ypos[t-1] + yp * dt
    
    # 3. Calcul de la référence et de ses dérivées
    xr = trajectoire_ref_x[t]
    yr = trajectoire_ref_y[t]
    xrp = (trajectoire_ref_x[t] - trajectoire_ref_x[t-1]) / dt
    yrp = (trajectoire_ref_y[t] - trajectoire_ref_y[t-1]) / dt
    
    # 4. Calcul des erreurs
    errx = Xpos[t] - xr
    erry = Ypos[t] - yr
    errxp = xp - xrp
    erryp = yp - yrp
    
    # 5. Loi de commande linéarisante (entrées auxiliaires)
    # Note : on utilise des signes négatifs pour ramener l'erreur à zéro
    v1 = - lambdax * errx - Mux * errxp
    v2 = - lambday * erry - Muy * erryp
    
    # 6. Transformation vers les commandes réelles (u1, u2)
    R = np.array([
        [np.cos(ray_braq[t-1]), np.sin(ray_braq[t-1])],
        [-np.sin(ray_braq[t-1]), np.cos(ray_braq[t-1])]
    ])
    
    # On évite la division par zéro
    v_limit = max(abs(V[t-1]), 0.01)
    
    Vi = np.array([v1, v2])
    
    M=np.array([[Masse*V[t-1],0],
                [0,1]])
    # Le calcul matriciel pour découpler le système
    u =M @ R @ Vi
    
    u1_plot[t] = u[0] / v_limit # Accélération tangentielle
    u2_plot[t] = u[1] / v_limit # Vitesse angulaire
    
    # 7. Mise à jour des états pour t+1
    # Mise à jour de la vitesse linéaire
    V[t] = V[t-1] + u1_plot[t] * dt
    # Mise à jour de l'angle (NE PAS OUBLIER LE * DT)
    ray_braq[t] = ray_braq[t-1] + u2_plot[t] * dt
    
plt.figure()
plt.plot(Xpos, Ypos, label='reel')
plt.plot(trajectoire_ref_x, trajectoire_ref_y, label='ref')
plt.xlabel('Position en x')
plt.ylabel('Position en y')
plt.title('Suivi de trajectoire du robot')
plt.legend()
plt.grid()
plt.show()

plt.figure()
plt.plot(temps, u1_plot, label='reel')
plt.plot(temps, u2_plot, label='ref')
plt.xlabel('Temps')
plt.ylabel('Commande en x')
plt.title('Commande en x du robot')
plt.legend()
plt.grid()
plt.show()