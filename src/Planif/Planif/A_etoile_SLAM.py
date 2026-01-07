import matplotlib.pyplot as plt
from PIL import Image
import time as t
import os
import numpy as np
import heapq
import csv
import yaml
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path, OccupancyGrid
from std_msgs.msg import Int32
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from nav_msgs.msg import Odometry
from builtin_interfaces.msg import Time

# pip install pyyaml si le module yaml n'est pas trouvé
# Commande pour publier sur le topic lançant le noeud (pour tester) : ros2 topic pub /init_planif std_msgs/Int32 "{data: 1}" -1

# POUR LANCER LE NOEUD DE PLANIFICATION, VOUS AUREZ BESOIN DE 5 TERMINAUX :
# - terminal 1 : build avec 'colcon build --symlink-install' ou 'colcon build --packages-select Planif'
# - terminal 2 : lancer le simulateur après avoir source : '. install/local_setup.bash' puis  'ros2 launch f1tenth_gym_ros gym_bridge_launch.py'
# - terminal 3 : lancer le noeud A* : '. install/local_setup.bash' puis 'ros2 run Planif A_star'
# - terminal 4 : lancer le noeud de guidage : '. install/local_setup.bash' puis 'ros2 run Waypoints pure_pursuit'
# - terminal 5 : déclancher manuellement la planification (attendre que "Initialisation terminée" apparaisse dans le terminal liée à A*) : '. install/local_setup.bash'
#                puis 'ros2 topic pub /init_planif std_msgs/Int32 "{data: 1}" -1' 

# general overall: this code uses ROS2 messages like PoseStamped, Path and AckermannDriveStamped. It reads a map from a .pgm file and its corresponding .yaml file.
# Then calculates the waypoints thanks to A* and publishes them to a ROS2 topic. 

class Astar(Node):
    def __init__(self, pos_topic = '/odom', map_topic = '/map', waypoint_topic='/waypoints'):
        super().__init__('A_star')

        self.goal_poses = Path() # liste des waypoints
        # /!\ A CHANGER : OBTENIR POINT D'ORIGINE DU REPERE AVEC SLAM (IN PROCESS)
        # self.map_info = map_info_location
        self.resolution_carte = 0.05 # 1 pixel = 5 cm
        self.init_pos = None # position initiale du robot
        self.init_pos_dist = (0, 0) # position initiale du robot (dans le repère de la voiture)
        self.origin = (0, 0) #  valeur en distance du pixel d'origine du repère de la map
        

        # Publishers et subscribers
        self.init_plannif_sub = self.create_subscription(Int32, "/init_planif", self.create_waypoints, 10) # Lance l'algorithme dés qu'il reçoit un signal sur ce topic
        self.map_sub = self.create_subscription(OccupancyGrid, map_topic, self.update_map, 10)
        self.odom_sub = self.create_subscription(Odometry, pos_topic, self.update_position, 10)
        self.waypoint_pub = self.create_publisher(Path, waypoint_topic, 10)

        #self.img = np.asarray(self.circuit)
        #self.taille_circuit = len(self.img)
        #self.map_a = np.divide(self.img, 254)

        # Paramétrage initial ################################################################
        self.securite = 5 # 10 pixels de sécurité
        # /!\ CHANGER DYNAMIQUEMENT NB WAYPOINTS
        self.echelle_wp = 30 # Waypoint tous les X pixels
        self.vide = 2 # zone libre
        self.mur = 0 # zone occupée
        # /!\ CHANGER DIRECTION EN FONCTION DE LA POSITION
        self.direction = 4 # 1 = Nord, 2 = Est, 3 = Sud, 4 = Ouest
        # self.map_a = self.securite_map(self.map_a, self.securite) # Ajout des frontières de sécurité
        # self.map_a = self.detect_circuit(self.map_a, self.init_pos, 1, self.vide) # Detection des zones accessibles
        ######################################################################################
        
        self.map_a = None
        print("\nInitialisation terminée, en attente des données du SLAM")

    ##### FONCTIONS UTILISEES POUR GENERER UNE TRAJECTOIRE OPTIMISEE #####
        
    # Dés que une message est publié sur map_topic, le map est enregistré     
    def update_map(self,msg):
        """Transforme OccupancyGrid du SLAM à numpy array et applique les distances de securité"""
        ### Cette partie doit être modifié selon les noms des données extraites de l'OccupancyGrid
        width = msg.info.width # width du OccupancyGrid
        height = msg.info.height # height du OccupancyGrid
        self.resolution_carte = msg.info.resolution # resolution du OccupancyGrid
        self.origin = (msg.info.origin.position.x, msg.info.origin.position.y) # # Offset de distance pour l'origine de la carte (en bas à gauche)
        ### 

        grid = np.array(msg.data).reshape((height,width)) # tranformation d'OccupancyGrid à nparray
        self.map_a = np.where(grid == -1, 0.5, grid)
        self.map_a = self.securite_map(self.map_a, self.securite)
        self.map_a = self.detect_circuit(self.map_a, self.init_pos, 1, self.vide) # Detection des zones accessibles
        print("Map mis à jour et distances de securité ajoutées")
        
    def update_position(self,msg):
        """Position initiale du robot avec données du SLAM"""
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.init_pos = (int((y - self.origin[1]) / self.resolution_carte), 
                         int((x - self.origin[0]) / self.resolution_carte))
        print(f"Position mise à jour: [{x},{y}]")

    def numpy_to_distance(self, X, Y) :
        """A partir des coordonnées (X, Y) dans la matrice numpy, renvoie le waypoint en m dans le repère du robot"""
        X_dist = (Y + self.origin[0]/self.resolution_carte) * self.resolution_carte
        Y_dist = (-X + self.origin[1]/self.resolution_carte + self.taille_circuit) * self.resolution_carte
        return X_dist, Y_dist
    
    # Dans OccupancyGrid, il ne faut pas de fichier YAML car toutes les données de la carte peuvent être extraites à travers de l'OccupancyGrid
    #def get_info_map(self, map_info):
    #    """Permet d'obtenir la résolution et l'origine de la carte"""
    #    with open(map_info) as f:
    #        data = yaml.load(f, Loader = yaml.loader.BaseLoader)
    #        self.resolution_carte = float(data["resolution"])
    #        self.origin = (float(data["origin"][0]), float(data["origin"][1])) # Offset de distance pour l'origine  de la carte (en bas à gauche)

    
    def create_waypoints(self, data, csv_plot = False):
        '''A partir de la trajectoire en pixels du A*, génère les waypoints souhaités et les publie dans un topic dédié + fichier csv'''
        if self.map_a is None or self.init_pos is None:
            print("En attente de map et position...")
            return
        
        print("Calcul de la trajectoire...")
        # /!\ A CHANGER : OBTENIR POSITION DE LA VOITURE AVEC LE NOEUD SLAM (IN PROCESS)
        start = self.init_pos # data[0] # Position actuelle de la voiture        
        self.map_a, goal = self.detect_end(self.map_a, start, self.mur, self.direction) # Detection automatique du point à atteindre (x, y)
        
        csv_plot = True
        if csv_plot :
            # On ouvre le fichier csv
            f = open('waypoints.csv', 'w')
            writer = csv.writer(f)

        # On récupère l'origine et la résolution de la map
        # self.get_info_map(self.map_info)

        # On lance l'algorithme A*
        route = self.astar(self.map_a, start, goal, self.vide)
        print("Calcul terminé !")
        print("Ecriture des résultats...")

         
        for i in (range(0,len(route))):
            # Pour chaque waypoint en coordonnée pixels sur la carte, on associe le waypoint correspondant à la réalité
            if (i+1)%self.echelle_wp == 0:
                goal_pose = PoseStamped()
                goal_pose.header.frame_id = 'Trajectory'
                goal_pose.header.stamp = self.get_clock().now().to_msg()
                x_dist, y_dist = self.numpy_to_distance(route[i][0], route[i][1])
                goal_pose.pose.position.x = x_dist
                goal_pose.pose.position.y = y_dist
                goal_pose.pose.position.z = 0.0
                goal_pose.pose.orientation.x = 0.0
                goal_pose.pose.orientation.y = 0.0
                goal_pose.pose.orientation.z = 0.0
                goal_pose.pose.orientation.w = 0.0
                self.goal_poses.poses.append(goal_pose)
                if csv_plot :
                    # On rempli le fichier csv
                    writer.writerow([x_dist, y_dist])
        
        # On publie la liste de waypoints
        self.waypoint_pub.publish(self.goal_poses)
        if csv_plot :
            # On ferme le fichier csv de waypoints
            f.close()
        print("Résultats transmis !")

    def securite_map(self, map_a, resolution) :
        '''Ajout d'une distance de securité autour des murs'''
        x_max, y_max = map_a.shape
        for i in range(len(map_a)) :
            for j in range(len(map_a[0])) :
                if map_a[i][j] == 0 :# Pour chaque pixel de mur
                    # On ajoute une zone de sécurité
                    mini_x = max(0, i - resolution)
                    mini_y = max(0, j - resolution)
                    maxi_x = min(x_max - 1, i + resolution)
                    maxi_y = min(y_max - 1, j + resolution)
                    for x in range(mini_x, maxi_x + 1) :
                        for y in range(mini_y, maxi_y + 1) :
                            if map_a[x][y] == 1 :
                                map_a[x][y] = 0.5
        return map_a
    
    def heuristique2(self, a, b):
        """Distance de Manhattan"""
        return np.abs(b[0] - a[0]) + np.abs(b[1] - a[1])

    def heuristique1(self, a, b) :
        """Distance euclidienne"""
        return np.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2)
        

    def astar(self, map_a, start, goal, vide):
        """Algorithme A*"""
        action = [(0,1),(0,-1),(1,0),(-1,0),(1,1),(1,-1),(-1,1),(-1,-1)] # Liste des mouvements possibles
        close_set = set() # Ensemble des positions à ne plus considérer (déjà regardées)
        chemin_pos = {} # Ensemble des positions des parents de chaque position
        dist_orig = {start:0} # Score 1 : Distance au point de départ
        score = {start:self.heuristique2(start, goal)} # Score réel : Distance au point de départ + Distance à l'arrivée estimée (via l'heuristique)
        pile_pos = [] # Pile des positions à considérer comme le plus court chemin
        
        # On push la position de départ et son score sur la pile
        heapq.heappush(pile_pos, (score[start], start))
    
        while pile_pos: # On check les positions à regarder tant qu'il y en a
            
            # On récupère la position du voisin avec le plus petit score possible
            current = heapq.heappop(pile_pos)[1]
            
            if current == goal: # Si on a atteint l'arrivée, on récupère le plus court chemin en remontant la liste des parents des positions considérées
                data = []
                while current in chemin_pos:
                    data.append(current)
                    current = chemin_pos[current]
                return data

            # Sinon, on met cette position dans la liste des positions à ne plus regarder
            close_set.add(current)

            for i, j in action: # Pour tous les voisins
                neighbor = current[0] + i, current[1] + j # On calcule sa position (pos actuel + mouvement)           
                tentative_g_score = dist_orig[current] + self.heuristique2(current, neighbor)

                if 0 <= neighbor[0] < map_a.shape[0]: # Si le voisin est en dehors de la grille (effet de bord), on ne le prend pas en compte
                    if 0 <= neighbor[1] < map_a.shape[1]:                
                        if map_a[neighbor[0]][neighbor[1]] < vide:
                            continue # l'instruction continue permet de passer à l'itération suivante de la boucle for 
                    else:
                        # limite en y (horizontal)(vertical)
                        continue
                else:
                    # limite en x (vertical)
                    continue
                
                if neighbor in close_set and tentative_g_score >= dist_orig.get(neighbor, 0):
                    # Si le voisin a déjà été vu et que le score de la nouvelle position avec l'heuristique est moins bon que le score actuel
                    continue
                
                if  tentative_g_score < dist_orig.get(neighbor, 0) or neighbor not in [i[1]for i in pile_pos]:
                    # Meilleur score d'heuristique ou position non rencontrée : on l'ajoute à la pile des positions à considérer
                    chemin_pos[neighbor] = current
                    dist_orig[neighbor] = tentative_g_score
                    score[neighbor] = tentative_g_score + self.heuristique2(neighbor, goal)
                    heapq.heappush(pile_pos, (score[neighbor], neighbor)) 

        def distance(a, b, c, d) :
            '''distance euclidienne (au carré) entre (a,b) et (c,d)'''
            return ((c - a)**2 + (d - b)**2 )

        def detect_goal(self, map_a, start, vide):
            '''Fonction qui détecte les frontières et indique ou aller à l'algorithme A*'''
            
            bordure_map = []
            x_max, y_max = map_a.shape
            bordure_map.append([[(0, j), map_a[0, j]] for j in range(len(map_a[0]))]) # haut
            bordure_map.append([[(x_max - 1, j), map_a[x_max - 1, j]] for j in range(len(map_a[0]))]) # bas
            bordure_map.append([[(i, 0), map_a[i, 0]] for i in range(len(map_a))]) # gauche
            bordure_map.append([[(i, y_max - 1), map_a[i, y_max - 1]] for i in range(len(map_a))]) # droite
            frontieres = [] # liste des frontières

            # Detection des frontières (par leurs points extrêmes)
            for bord in bordure_map :
                for i in range(len(bord)) :
                    if bord[i][1] == vide and i == 0 : # début de frontière
                        frontieres.append([bord[i][0]])
                    elif bord[i][1] == vide and bord[i - 1][1] != vide : # début de frontière
                        frontieres.append([bord[i][0]])
                    if bord[i][1] == vide and i == len(bord) -1 : # fin de frontière
                        frontieres[-1].append(bord[i][0])
                    elif bord[i][1] == vide and bord[i + 1][1] != vide : # fin de frontière
                        frontieres[-1].append(bord[i][0])
            
            dist = [] # distance aux frontières
            
            for front in frontieres :
                centre = ( 0.5 * (front[0][0] + front[1][0]), 0.5 * (front[0][1] + front[1][1]) )
                dist.append(distance(centre[0], centre[1], start[0], start[1]))
            
            ind_best_front = dist.index(max(dist)) # On veut la frontière la plus plus lointaine
            
            d1 = distance(frontieres[ind_best_front][0][0], frontieres[ind_best_front][0][1], start[0], start[1])
            d2 = distance(frontieres[ind_best_front][1][0], frontieres[ind_best_front][1][1], start[0], start[1])
            
            if d1 < d2 : # On retourne le point le plus proche de la frontière la plus lointaine
                return frontieres[ind_best_front][0]
            else :
                return frontieres[ind_best_front][1]
        
    def detect_circuit(self, map_a, start, vide_avant, vide_apres) :
        """Fonction qui détecte les points accessibles et précise le meilleur point à atteindre pour A*.
        Map_a est la carte avec frontières plus larges (par sécurité)"""
        
        circuit = set() # Ensemble des points à traiter du circuit
        x_max = len(map_a) - 1
        y_max = len(map_a[0]) - 1
        map_a[start[0]][start[1]] = vide_apres # 2 = zone accessible
        action = [(0,1),(0,-1),(1,0),(-1,0),(1,1),(1,-1),(-1,1),(-1,-1)] # Liste des mouvements possibles
        
        # On push la position de départ sur la pile
        circuit.add(start)
        
        while len(circuit) != 0  :
            # On récupère la position d'un élément de l'ensemble
            current = circuit.pop()
            for i, j in action : # Pour chaque voisin
                neighbor = current[0] + i, current[1] + j # On calcule sa position (pos actuel + mouvement) 
                if (neighbor[0] >= 0 and neighbor[0] <= x_max) and (neighbor[1] >= 0 and neighbor[1] <= y_max) :
                    if map_a[neighbor[0]][neighbor[1]] == vide_avant :
                        map_a[neighbor[0]][neighbor[1]] = vide_apres
                        circuit.add(neighbor)
        return map_a 

    def detect_end(self, map_a, start, mur, direction) :
        """Créé une frontière sur la map derrière la voiture et donne le but à atteindre"""
        x_voit, y_voit = start
        #-------> axe Y
        #|
        #|
        #v axe X
        ind1 = 1
        ind2 = 1
        if direction == 1 : # NORD --> frontière horizontale au sud de la voiture
            goal = (x_voit + 2, y_voit)
            map_a[x_voit + 1, y_voit] = mur
            while (ind1>= 0) or (ind2>=0) :
                if ind1 >= 0 :
                    if map_a[x_voit + 1][y_voit + ind1] != mur :
                        map_a[x_voit + 1][y_voit + ind1] = mur
                        ind1 = ind1 + 1
                    else :
                        ind1 = -1
                if ind2 >= 0 :
                    if map_a[x_voit + 1][y_voit - ind2] != mur :
                        map_a[x_voit + 1][y_voit - ind2] = mur
                        ind2 = ind2 + 1
                    else :
                        ind2 = -1
        elif direction == 2 : # EST --> frontière verticale à l'ouest de la voiture
            goal = (x_voit, y_voit - 2)
            map_a[x_voit, y_voit - 1] = mur
            while (ind1>= 0) or (ind2>=0) :
                if ind1 >= 0 :
                    if map_a[x_voit + ind1][y_voit - 1] != mur :
                        map_a[x_voit + ind1][y_voit - 1] = mur
                        ind1 = ind1 + 1
                    else :
                        ind1 = -1
                if ind2 >= 0 :
                    if map_a[x_voit - ind2][y_voit - 1] != mur :
                        map_a[x_voit - ind2][y_voit - 1] = mur
                        ind2 = ind2 + 1
                    else :
                        ind2 = -1
        elif direction == 3 : # SUD --> frontière horizontale au nord de la voiture
            goal = (x_voit - 2, y_voit)
            map_a[x_voit - 1, y_voit] = mur
            while (ind1>= 0) or (ind2>=0) :
                if ind1 >= 0 :
                    if map_a[x_voit - 1][y_voit + ind1] != mur :
                        map_a[x_voit - 1][y_voit + ind1] = mur
                        ind1 = ind1 + 1
                    else :
                        ind1 = -1
                if ind2 >= 0 :
                    if map_a[x_voit - 1][y_voit - ind2] != mur :
                        map_a[x_voit - 1][y_voit - ind2] = mur
                        ind2 = ind2 + 1
                    else :
                        ind2 = -1
        elif direction == 4 : # OUEST --> frontière verticale à l'est de la voiture
            goal = (x_voit, y_voit + 2)
            map_a[x_voit, y_voit + 1] = mur
            while (ind1>= 0) or (ind2>=0) :
                if ind1 >= 0 :
                    if map_a[x_voit + ind1][y_voit + 1] != mur :
                        map_a[x_voit + ind1][y_voit + 1] = mur
                        ind1 = ind1 + 1
                    else :
                        ind1 = -1
                if ind2 >= 0 :
                    if map_a[x_voit - ind2][y_voit + 1] != mur :
                        map_a[x_voit - ind2][y_voit + 1] = mur
                        ind2 = ind2 + 1
                    else :
                        ind2 = -1
        
        return map_a, goal
     
    

def main(args=None):
    rclpy.init(args=args)
    a = Astar() # Astar('/scan', '/drive')
    rclpy.spin(a)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
