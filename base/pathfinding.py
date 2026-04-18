"""
Application GUI pour la résolution de labyrinthes avec A* et Dijkstra.
"""
import sys
import time
import numpy as np
from PySide6.QtWidgets import (QApplication, QMainWindow, QGraphicsScene, 
                               QGraphicsRectItem, QMessageBox)
from PySide6.QtCore import Qt, QRectF
from PySide6.QtGui import QBrush, QColor, QPen

from ui_maze_window import Ui_MainWindow
from Maze import Maze
from main import create_complete_maze


class MazeWindow(QMainWindow):
    """Fenêtre principale de l'application."""
    
    def __init__(self):
        super().__init__()
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        
        # Variables pour le labyrinthe
        self.maze = None
        self.scene = QGraphicsScene()
        self.ui.mazeView.setScene(self.scene)
        self.cell_size = 30
        self.path_astar = None
        self.path_dijkstra = None
        self.explored_astar = None
        self.explored_dijkstra = None
        
        # Couleurs
        self.color_free = QColor(255, 255, 255)
        self.color_obstacle = QColor(50, 50, 50)
        self.color_start = QColor(0, 255, 0)
        self.color_goal = QColor(255, 0, 0)
        self.color_path_astar = QColor(100, 150, 255)
        self.color_path_dijkstra = QColor(255, 150, 100)
        self.color_bonus = QColor(255, 255, 0)
        self.color_explored_astar = QColor(200, 220, 255)
        self.color_explored_dijkstra = QColor(255, 220, 200)
        
        # Connecter les signaux
        self.ui.generateButton.clicked.connect(self.generate_maze)
        self.ui.solveAStarButton.clicked.connect(self.solve_astar)
        self.ui.solveDijkstraButton.clicked.connect(self.solve_dijkstra)
        self.ui.compareButton.clicked.connect(self.compare_algorithms)
        self.ui.clearButton.clicked.connect(self.clear_path)
        self.ui.showExplorationCheckBox.stateChanged.connect(self.draw_maze)
        self.ui.actionQuitter.triggered.connect(self.close)
        self.ui.actionAPropos.triggered.connect(self.show_about)
        
        # Générer un labyrinthe par défaut
        self.generate_maze()
    
    def generate_maze(self):
        """Génère un nouveau labyrinthe selon les paramètres."""
        width = self.ui.widthSpinBox.value()
        height = self.ui.heightSpinBox.value()
        
        # Déterminer le type d'obstacles
        obstacle_type_index = self.ui.obstacleTypeCombo.currentIndex()
        obstacle_types = ["random", "vertical_walls", "horizontal_walls", 
                         "maze_pattern", "none"]
        obstacle_type = obstacle_types[obstacle_type_index]
        
        density = self.ui.densitySpinBox.value()
        step_cost = self.ui.stepCostSpinBox.value()
        goal_reward = self.ui.goalRewardSpinBox.value()
        add_bonuses = self.ui.bonusCheckBox.isChecked()
        num_bonuses = self.ui.numBonusSpinBox.value()
        bonus_value = self.ui.bonusValueSpinBox.value()
        
        # Créer le labyrinthe
        if obstacle_type == "none":
            self.maze = Maze(width, height, start=(0, 0), 
                           goal=(height-1, width-1))
            # Initialiser les récompenses
            self.maze.rewards = np.full((height, width), step_cost, dtype=float)
            self.maze.set_reward(*self.maze.goal, goal_reward)
        else:
            self.maze = create_complete_maze(
                width, height,
                obstacle_type=obstacle_type,
                obstacle_density=density,
                step_cost=step_cost,
                goal_reward=goal_reward,
                add_bonuses=add_bonuses,
                num_bonuses=num_bonuses,
                bonus_value=bonus_value
            )
        
        # Réinitialiser les chemins
        self.path_astar = None
        self.path_dijkstra = None
        self.explored_astar = None
        self.explored_dijkstra = None
        
        # Afficher le labyrinthe
        self.draw_maze()
        
        # Afficher les infos
        self.ui.statsText.clear()
        self.ui.statsText.append(f"Labyrinthe généré: {width}x{height}")
        self.ui.statsText.append(f"Départ: {self.maze.start}")
        self.ui.statsText.append(f"Arrivée: {self.maze.goal}")
    
    def draw_maze(self):
        """Dessine le labyrinthe dans la vue graphique."""
        if self.maze is None:
            return
        
        self.scene.clear()
        
        # Calculer la taille des cellules
        view_width = self.ui.mazeView.width() - 10
        view_height = self.ui.mazeView.height() - 10
        self.cell_size = min(view_width // self.maze.width, 
                            view_height // self.maze.height, 40)
        
        # Dessiner chaque cellule
        for i in range(self.maze.height):
            for j in range(self.maze.width):
                x = j * self.cell_size
                y = i * self.cell_size
                
                # Déterminer la couleur
                if (i, j) == self.maze.start:
                    color = self.color_start
                elif (i, j) == self.maze.goal:
                    color = self.color_goal
                elif self.maze.grid[i, j] == 1:
                    color = self.color_obstacle
                elif self.maze.rewards[i, j] > 0:
                    color = self.color_bonus
                else:
                    color = self.color_free
                
                # Dessiner le rectangle
                rect = QGraphicsRectItem(QRectF(x, y, self.cell_size, self.cell_size))
                rect.setBrush(QBrush(color))
                rect.setPen(QPen(Qt.black, 1))
                self.scene.addItem(rect)
        
        # Dessiner les cellules explorées si activé
        show_exploration = self.ui.showExplorationCheckBox.isChecked()
        if show_exploration:
            if self.explored_astar is not None:
                self.draw_explored(self.explored_astar, self.color_explored_astar, 0.8)
            
            if self.explored_dijkstra is not None:
                self.draw_explored(self.explored_dijkstra, self.color_explored_dijkstra, 0.8)
        
        # Dessiner les chemins s'ils existent
        if self.path_astar is not None:
            self.draw_path(self.path_astar, self.color_path_astar, 0.3)
        
        if self.path_dijkstra is not None:
            self.draw_path(self.path_dijkstra, self.color_path_dijkstra, 0.3)
    
    def draw_explored(self, explored_set, color, alpha):
        """Dessine les cellules explorées sur le labyrinthe."""
        if explored_set is None or len(explored_set) == 0:
            return
        
        color_with_alpha = QColor(color)
        color_with_alpha.setAlphaF(alpha)
        
        for (i, j) in explored_set:
            # Ne pas redessiner le départ et l'arrivée
            if (i, j) == self.maze.start or (i, j) == self.maze.goal:
                continue
            
            x = j * self.cell_size
            y = i * self.cell_size
            
            rect = QGraphicsRectItem(QRectF(x + 1, y + 1, 
                                           self.cell_size - 2, 
                                           self.cell_size - 2))
            rect.setBrush(QBrush(color_with_alpha))
            rect.setPen(QPen(Qt.NoPen))
            self.scene.addItem(rect)
    
    def draw_path(self, path, color, alpha):
        """Dessine un chemin sur le labyrinthe."""
        if path is None or len(path) == 0:
            return
        
        color_with_alpha = QColor(color)
        color_with_alpha.setAlphaF(alpha)
        
        for (i, j) in path:
            # Ne pas redessiner le départ et l'arrivée
            if (i, j) == self.maze.start or (i, j) == self.maze.goal:
                continue
            
            x = j * self.cell_size
            y = i * self.cell_size
            
            rect = QGraphicsRectItem(QRectF(x + 2, y + 2, 
                                           self.cell_size - 4, 
                                           self.cell_size - 4))
            rect.setBrush(QBrush(color_with_alpha))
            rect.setPen(QPen(color, 2))
            self.scene.addItem(rect)
    
    def solve_astar(self):
        """Résout le labyrinthe avec A*."""
        if self.maze is None:
            return
        
        start_time = time.time()
        result = self.maze.solve(return_explored=True)
        elapsed_time = time.time() - start_time
        
        self.path_astar, self.explored_astar = result
        
        if self.path_astar is None:
            QMessageBox.warning(self, "Aucun chemin", 
                              "Aucun chemin n'a été trouvé avec A* !")
            self.ui.statsText.append("\n❌ A* : Aucun chemin trouvé")
            if self.explored_astar:
                self.ui.statsText.append(f"   Cellules explorées: {len(self.explored_astar)}")
        else:
            # Calculer le coût du chemin
            cost = self.calculate_path_cost(self.path_astar)
            
            self.ui.statsText.append(f"\n✅ A* : Chemin trouvé")
            self.ui.statsText.append(f"   Longueur: {len(self.path_astar)} cellules")
            self.ui.statsText.append(f"   Coût total: {cost:.2f}")
            self.ui.statsText.append(f"   Temps: {elapsed_time*1000:.2f} ms")
            if self.explored_astar:
                self.ui.statsText.append(f"   Cellules explorées: {len(self.explored_astar)}")
            
            self.draw_maze()
    
    def solve_dijkstra(self):
        """Résout le labyrinthe avec Dijkstra."""
        if self.maze is None:
            return
        
        start_time = time.time()
        result = self.maze.solve_dijkstra(return_explored=True)
        elapsed_time = time.time() - start_time
        
        self.path_dijkstra, self.explored_dijkstra = result
        
        if self.path_dijkstra is None:
            QMessageBox.warning(self, "Aucun chemin", 
                              "Aucun chemin n'a été trouvé avec Dijkstra !")
            self.ui.statsText.append("\n❌ Dijkstra : Aucun chemin trouvé")
            if self.explored_dijkstra:
                self.ui.statsText.append(f"   Cellules explorées: {len(self.explored_dijkstra)}")
        else:
            # Calculer le coût du chemin
            cost = self.calculate_path_cost(self.path_dijkstra)
            
            self.ui.statsText.append(f"\n✅ Dijkstra : Chemin trouvé")
            self.ui.statsText.append(f"   Longueur: {len(self.path_dijkstra)} cellules")
            self.ui.statsText.append(f"   Coût total: {cost:.2f}")
            self.ui.statsText.append(f"   Temps: {elapsed_time*1000:.2f} ms")
            if self.explored_dijkstra:
                self.ui.statsText.append(f"   Cellules explorées: {len(self.explored_dijkstra)}")
            
            self.draw_maze()
    
    def compare_algorithms(self):
        """Compare les deux algorithmes."""
        if self.maze is None:
            return
        
        self.ui.statsText.clear()
        self.ui.statsText.append("=== Comparaison A* vs Dijkstra ===\n")
        
        # A*
        start_time = time.time()
        result_astar = self.maze.solve(return_explored=True)
        time_astar = time.time() - start_time
        self.path_astar, self.explored_astar = result_astar
        
        # Dijkstra
        start_time = time.time()
        result_dijkstra = self.maze.solve_dijkstra(return_explored=True)
        time_dijkstra = time.time() - start_time
        self.path_dijkstra, self.explored_dijkstra = result_dijkstra
        
        # Afficher les résultats
        if self.path_astar is None:
            self.ui.statsText.append("❌ A* : Aucun chemin trouvé")
            if self.explored_astar:
                self.ui.statsText.append(f"   Cellules explorées: {len(self.explored_astar)}\n")
        else:
            cost_astar = self.calculate_path_cost(self.path_astar)
            self.ui.statsText.append(f"✅ A* :")
            self.ui.statsText.append(f"   Longueur: {len(self.path_astar)} cellules")
            self.ui.statsText.append(f"   Coût: {cost_astar:.2f}")
            self.ui.statsText.append(f"   Temps: {time_astar*1000:.2f} ms")
            if self.explored_astar:
                self.ui.statsText.append(f"   Cellules explorées: {len(self.explored_astar)}\n")
        
        if self.path_dijkstra is None:
            self.ui.statsText.append("❌ Dijkstra : Aucun chemin trouvé")
            if self.explored_dijkstra:
                self.ui.statsText.append(f"   Cellules explorées: {len(self.explored_dijkstra)}\n")
        else:
            cost_dijkstra = self.calculate_path_cost(self.path_dijkstra)
            self.ui.statsText.append(f"✅ Dijkstra :")
            self.ui.statsText.append(f"   Longueur: {len(self.path_dijkstra)} cellules")
            self.ui.statsText.append(f"   Coût: {cost_dijkstra:.2f}")
            self.ui.statsText.append(f"   Temps: {time_dijkstra*1000:.2f} ms")
            if self.explored_dijkstra:
                self.ui.statsText.append(f"   Cellules explorées: {len(self.explored_dijkstra)}\n")
        
        # Analyse comparative
        if self.path_astar is not None and self.path_dijkstra is not None:
            self.ui.statsText.append("📊 Analyse comparative :")
            
            if len(self.path_astar) == len(self.path_dijkstra):
                self.ui.statsText.append("   Même longueur de chemin")
            else:
                faster = "A*" if len(self.path_astar) < len(self.path_dijkstra) else "Dijkstra"
                self.ui.statsText.append(f"   {faster} trouve un chemin plus court")
            
            if time_astar < time_dijkstra:
                speedup = time_dijkstra / time_astar
                self.ui.statsText.append(f"   A* est {speedup:.2f}x plus rapide")
            else:
                speedup = time_astar / time_dijkstra
                self.ui.statsText.append(f"   Dijkstra est {speedup:.2f}x plus rapide")
            
            # Comparaison de l'exploration
            if self.explored_astar and self.explored_dijkstra:
                self.ui.statsText.append(f"\n   Cellules explorées:")
                self.ui.statsText.append(f"   • A* : {len(self.explored_astar)}")
                self.ui.statsText.append(f"   • Dijkstra : {len(self.explored_dijkstra)}")
                if len(self.explored_astar) < len(self.explored_dijkstra):
                    ratio = len(self.explored_dijkstra) / len(self.explored_astar)
                    self.ui.statsText.append(f"   → A* explore {ratio:.2f}x moins de cellules")
            
            self.ui.statsText.append(f"\n💡 Complexité théorique :")
            self.ui.statsText.append(f"   A* : O((V+E) log V) avec heuristique")
            self.ui.statsText.append(f"   Dijkstra : O((V+E) log V) sans heuristique")
            self.ui.statsText.append(f"\n   A* explore moins de noeuds grâce à l'heuristique")
        
        self.draw_maze()
    
    def calculate_path_cost(self, path):
        """Calcule le coût total d'un chemin."""
        if path is None or len(path) == 0:
            return 0
        
        cost = 0
        for (i, j) in path:
            cost -= self.maze.rewards[i, j]
        
        return cost
    
    def clear_path(self):
        """Efface les chemins affichés."""
        self.path_astar = None
        self.path_dijkstra = None
        self.explored_astar = None
        self.explored_dijkstra = None
        self.draw_maze()
        self.ui.statsText.append("\nChemins et exploration effacés")
    
    def show_about(self):
        """Affiche la boîte de dialogue À propos."""
        QMessageBox.about(self, "À propos",
                         "Résolution de Labyrinthe\n\n"
                         "TP1 - Robotique : Résoudre un labyrinthe avec A*\n\n"
                         "Implémentation des algorithmes :\n"
                         "• Dijkstra : Algorithme de plus court chemin\n"
                         "• A* : Algorithme avec heuristique\n\n"
                         "Quang-Trung Luu\n"
                         "Université Paris-Saclay")


def main():
    """Point d'entrée de l'application."""
    app = QApplication(sys.argv)
    window = MazeWindow()
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
