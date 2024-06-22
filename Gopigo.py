import RPi.GPIO as GPIO
import time
import rospy
from geometry_msgs.msg import Twist
import sys, select, termios, tty

# Désactiver les avertissements
GPIO.setwarnings(False)

# Configuration des broches
GPIO.setmode(GPIO.BCM)
TRIG1 = 5
ECHO1 = 6
TRIG2 = 16
ECHO2 = 12
TRIG3 = 20
ECHO3 = 21
LED = 25

threshold = 5
threshold_distance1 = 15

GPIO.setup(TRIG1, GPIO.OUT)
GPIO.setup(ECHO1, GPIO.IN)
GPIO.setup(TRIG2, GPIO.OUT)
GPIO.setup(ECHO2, GPIO.IN)
GPIO.setup(TRIG3, GPIO.OUT)
GPIO.setup(ECHO3, GPIO.IN)
GPIO.setup(LED, GPIO.OUT)

GPIO.output(LED, False)
time.sleep(0.5)

def mesure_distance(TRIG, ECHO):
    """
    Fonction pour mesurer la distance à l'aide d'un capteur ultrason HC-SR04.
    Retourne la distance mesurée en centimètres.
    """
    # S'assurer que le TRIG est désactivé
    GPIO.output(TRIG, False)
    time.sleep(0.05)

    # Envoyer une impulsion ultrasonique
    GPIO.output(TRIG, True)
    time.sleep(0.00001)
    GPIO.output(TRIG, False)

    # Mesurer le temps de retour de l'impulsion
    pulse_start = time.time()
    while GPIO.input(ECHO) == 0:
        pulse_start = time.time()
    while GPIO.input(ECHO) == 1:
        pulse_end = time.time()

    # Calculer la durée de l'impulsion
    pulse_duration = pulse_end - pulse_start

    # Convertir la durée en distance
    distance = pulse_duration * 17150
    distance = round(distance, 2)

    return distance

def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def tourner_2_degres(pub, direction, vitesse_angulaire=1.0):
    """
    Fonction pour faire tourner le robot de 2 degrés.
    direction: 'gauche' ou 'droite'
    vitesse_angulaire: vitesse de rotation (en radians par seconde)
    """
    twist = Twist()
    twist.linear.x = 0
    twist.angular.z = vitesse_angulaire if direction == 'gauche' else -vitesse_angulaire
    pub.publish(twist)
    time.sleep(0.045)  # Ajustez cette valeur pour obtenir une rotation de 2 degrés
    twist.angular.z = 0
    pub.publish(twist)

def tourner_69_degres(pub, direction, vitesse_angulaire=1.0):
    """
    Fonction pour faire tourner le robot de 69 degrés.
    direction: 'gauche' ou 'droite'
    vitesse_angulaire: vitesse de rotation (en radians par seconde)
    """
    twist = Twist()
    twist.linear.x = 0
    twist.angular.z = vitesse_angulaire if direction == 'gauche' else -vitesse_angulaire
    pub.publish(twist)
    time.sleep(1.53)  # Ajustez cette valeur pour obtenir une rotation de 69 degrés
    twist.angular.z = 0
    pub.publish(twist)

def stabiliser(pub):
    """
    Fonction pour stabiliser le robot en ligne droite après une mini-rotation.
    """
    twist = Twist()
    twist.linear.x = -0.1  # Reculer très lentement
    twist.angular.z = 0
    pub.publish(twist)
    time.sleep(0.1)
    twist.linear.x = 0
    pub.publish(twist)

def reculer(pub, duration):
    """
    Fonction pour faire reculer le robot pendant une durée donnée.
    """
    twist = Twist()
    twist.linear.x = -0.2  # Vitesse de recul
    twist.angular.z = 0
    pub.publish(twist)
    time.sleep(duration)
    twist.linear.x = 0
    pub.publish(twist)

if __name__ == "__main__":
    settings = termios.tcgetattr(sys.stdin)
    rospy.init_node('robot_ultrason_teleop')
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

    speed = -0.2  # Vitesse négative pour reculer lentement
    turn = 0.0    # Assurer que la vitesse angulaire est nulle pour reculer en ligne droite
    status = 0
    running = False
    rotation_count = 0  # Compteur de rotations

    print("Appuyez sur 'a' pour commencer à téléopérer le robot.")
    print("Appuyez sur 'q' pour quitter.")
    print("Appuyez sur 'Ctrl + C' pour arrêter le programme.")

    try:
        while True:
            key = getKey()

            if key == 'a':
                running = True
                print("Téléopération démarrée.")

            if key == 'q':
                running = False
                print("Téléopération arrêtée.")
                twist = Twist()
                twist.linear.x = 0
                twist.angular.z = 0
                pub.publish(twist)

            if key == 'l':
                print("Tourner à droite.")
                tourner_69_degres(pub, 'droite')
                rotation_count += 1

            if key == 'k':
                print("Tourner à gauche.")
                tourner_69_degres(pub, 'gauche')
                rotation_count += 1

            if rotation_count >= 5:
                print("5 rotations atteintes, reculer pendant 2 secondes et tourner à droite.")
                reculer(pub, 2.5)  # Reculer pendant 2 secondes
                tourner_69_degres(pub, 'droite')
                rotation_count = 0  # Réinitialiser le compteur

            if key == '\x03':  # Ctrl + C pour quitter
                raise KeyboardInterrupt

            if running:
                distance1 = mesure_distance(TRIG1, ECHO1)
                distance2 = mesure_distance(TRIG2, ECHO2)
                distance3 = mesure_distance(TRIG3, ECHO3)

                print(f"Distance1: {distance1} cm")
                print(f"Distance2: {distance2} cm")
                print(f"Distance3: {distance3} cm")

                twist = Twist()

                if distance3 < threshold_distance1:
                    print("Obstacle détecté par Distance 3, s'arrêter et évaluer les distances.")
                    twist.linear.x = 0
                    twist.angular.z = 0
                    pub.publish(twist)
                    time.sleep(1)  # Pause pour évaluer les distances

                    # Réévaluer les distances
                    distance1 = mesure_distance(TRIG1, ECHO1)
                    distance2 = mesure_distance(TRIG2, ECHO2)
                    distance3 = mesure_distance(TRIG3, ECHO3)

                    max_distance = max(distance1, distance2, distance3)
                    if max_distance == distance1:
                        print("Tourner de 69 degrés à gauche.")
                        direction = 'gauche'
                    elif max_distance == distance2:
                        print("Tourner de 69 degrés à droite.")
                        direction = 'droite'
                    else:
                        print("Tourner de 69 degrés vers la direction de la plus grande distance.")
                        direction = 'gauche' if distance3 >= threshold else 'droite'

                    tourner_69_degres(pub, direction)
                    rotation_count += 1
                elif distance1 < threshold or distance2 < threshold:
                    print("Obstacle détecté à moins de 5 cm à gauche ou à droite, recalibrer pour maintenir la distance.")
                    if distance1 < threshold:
                        print("Trop proche à gauche, tourner légèrement à droite.")
                        tourner_2_degres(pub, 'droite')  # Tourner légèrement à droite
                        stabiliser(pub)
                    if distance2 < threshold:
                        print("Trop proche à droite, tourner légèrement à gauche.")
                        tourner_2_degres(pub, 'gauche')  # Tourner légèrement à gauche
                        stabiliser(pub)
                else:
                    print("Aucun obstacle détecté à gauche ou à droite.")
                    twist.linear.x = speed  # Reculer lentement
                    twist.angular.z = turn  # S'assurer que la vitesse angulaire est nulle

                pub.publish(twist)
                GPIO.output(LED, distance1 < threshold or distance2 < threshold or distance3 < threshold)

                time.sleep(0.1)  # Augmenter le délai pour réduire les interruptions

    except KeyboardInterrupt:
        print("Programme interrompu par l'utilisateur")
    finally:
        twist = Twist()
        twist.linear.x = 0
        twist.angular.z = 0
        pub.publish(twist)
        GPIO.cleanup()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
