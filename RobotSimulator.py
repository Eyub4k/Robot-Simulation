import pybullet as p
import pybullet_data
import time
import math

# Initialiser l'environnement PyBullet
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
planeId = p.loadURDF("plane.urdf")

# Créer un corps principal pour le robot aspirateur
rayon_corps = 0.15
hauteur_corps = 0.05
masse_corps = 1.0

# Créer le corps principal
corps_collision_id = p.createCollisionShape(p.GEOM_CYLINDER, radius=rayon_corps, height=hauteur_corps)
corps_visuel_id = p.createVisualShape(p.GEOM_CYLINDER, radius=rayon_corps, length=hauteur_corps, rgbaColor=[0.8, 0.8, 0.8, 1])

# Position initiale plus haute pour éviter la pénétration avec le sol
position_base = [0, 0, hauteur_corps + 0.05]  # +0.05 pour élever un peu

# Paramètres des roues
rayon_roue = 0.025
largeur_roue = 0.02
masse_roue = 0.1

# Créer les formes pour les roues
roue_collision_id = p.createCollisionShape(p.GEOM_CYLINDER, radius=rayon_roue, height=largeur_roue)
roue_visuel_id = p.createVisualShape(p.GEOM_CYLINDER, radius=rayon_roue, length=largeur_roue, rgbaColor=[0.2, 0.2, 0.2, 1])

# Configuration des liens
linkMasses = [] 
linkCollisionShapeIndices = []
linkVisualShapeIndices = []
linkPositions = []
linkOrientations = []
linkInertialFramePositions = []
linkInertialFrameOrientations = []
linkParentIndices = []
linkJointTypes = []
linkJointAxis = []

# Position des roues
pos_roue_gauche = [-0.1, 0, -hauteur_corps/2 - rayon_roue]  # Ajuster pour que les roues touchent le sol
pos_roue_droite = [0.1, 0, -hauteur_corps/2 - rayon_roue]   # Ajuster pour que les roues touchent le sol
orientation_roue = p.getQuaternionFromEuler([math.pi/2, 0, 0])

# Ajouter la roue gauche
linkMasses.append(masse_roue)
linkCollisionShapeIndices.append(roue_collision_id)
linkVisualShapeIndices.append(roue_visuel_id)
linkPositions.append(pos_roue_gauche)
linkOrientations.append(orientation_roue)
linkInertialFramePositions.append([0, 0, 0])
linkInertialFrameOrientations.append([0, 0, 0, 1])
linkParentIndices.append(0)
linkJointTypes.append(p.JOINT_REVOLUTE)
linkJointAxis.append([0, 0, 1])

# Ajouter la roue droite
linkMasses.append(masse_roue)
linkCollisionShapeIndices.append(roue_collision_id)
linkVisualShapeIndices.append(roue_visuel_id)
linkPositions.append(pos_roue_droite)
linkOrientations.append(orientation_roue)
linkInertialFramePositions.append([0, 0, 0])
linkInertialFrameOrientations.append([0, 0, 0, 1])
linkParentIndices.append(0)
linkJointTypes.append(p.JOINT_REVOLUTE)
linkJointAxis.append([0, 0, 1])

# Créer le robot complet avec les roues
robot_id = p.createMultiBody(
    baseMass=masse_corps,
    baseCollisionShapeIndex=corps_collision_id,
    baseVisualShapeIndex=corps_visuel_id,
    basePosition=position_base,
    linkMasses=linkMasses,
    linkCollisionShapeIndices=linkCollisionShapeIndices,
    linkVisualShapeIndices=linkVisualShapeIndices,
    linkPositions=linkPositions,
    linkOrientations=linkOrientations,
    linkInertialFramePositions=linkInertialFramePositions,
    linkInertialFrameOrientations=linkInertialFrameOrientations,
    linkParentIndices=linkParentIndices,
    linkJointTypes=linkJointTypes,
    linkJointAxis=linkJointAxis
)


# Fonction pour contrôler le mouvement du robot avec plus de force
def controle_robot(vitesse_gauche, vitesse_droite):
    # Utiliser maxForce plus élevée pour surmonter la friction
    p.setJointMotorControl2(bodyUniqueId=robot_id, jointIndex=0, controlMode=p.VELOCITY_CONTROL, 
                           targetVelocity=vitesse_gauche, force=50)
    p.setJointMotorControl2(bodyUniqueId=robot_id, jointIndex=1, controlMode=p.VELOCITY_CONTROL, 
                           targetVelocity=vitesse_droite, force=50)

# Régler la force de friction au niveau du sol
p.changeDynamics(planeId, -1, lateralFriction=0.8)

# Vitesse plus élevée pour le mouvement
vitesse_base = 20

# Boucle de simulation
for i in range(10000):
    if i == 0:
        print("Démarrage du robot...")
        # Commencer immédiatement avec vitesse maximale pour les deux roues
        controle_robot(10, 10)
    
    p.stepSimulation()
    time.sleep(1./240.)
    
    # Afficher la position du robot toutes les 100 étapes
    if i % 100 == 0:
        pos, _ = p.getBasePositionAndOrientation(robot_id)
        print(f"Position du robot: {pos}")
    
p.disconnect()