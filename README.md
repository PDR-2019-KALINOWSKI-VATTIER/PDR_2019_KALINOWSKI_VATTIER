# PDR_2019_KALINOWSKI_VATTIER

Pojet de découverte de recherche: "Localiser des QR-Codes dans l'espace à l'aide d'un robot muni d'une caméra 3D puis le robot dans son environnement."

I - Présentation du projet

Le dévelopement de la robotique impose une nécessité de plus en plus importante d'interragir avec l'environnement. Pour cela, il faut que le robot ait une bonne reconnaissance des objets qui l'entourent.
Le but de ce PDR est de permettre au robot d'éviter les obstacles au moyen d'une caméra et de QR-Codes placés dans l'environnement.
Pour réaliser ce projet, nous avons à notre disposition:
-Une caméra Realsense 2
-Un robot Kobuki
-Un PC sous Linux



II - Prise en main de l'environnement

  1) Linux

N'ayant jamais jamais eu de cours, ni utilisé Linux, il nous a fallu une période d'apprentissage et de familiarisation avec ce système d'exploitation, notament pour naviguer, télécharger, créer des répertoires, fichiers etc.

  2) ROS

ROS est le point de départ de notre projet. En effet, c'est lui qui nous permet de contrôler la caméra et le robot, de les interconnecter et de les piloter.
De ce fait, nous avons suivi le tutoriel ROS wiki (http://wiki.ros.org/ROS/Tutorials), afin d'installer ROS (ainsi que plusieurs librairies) et de nous familiariser avec ses fonctionnalités principales. Nous avons l'organisation de ROS sous forme de graphe et les concepts de de noeud, message, topic, master. Nous avons également appris à utiliser des subscribers et publishers.
Afin de mettre en oeuvre ces connaissances, nous avons suivi l'exemple de "Turtlesim", qui consiste à faire déplacer une petite tortue dans son environnement virtuel. Grâce à cet exemple, nous avons pu appliquer ces principes sur le robot Kobuki et le faire ainsi déplacer dans l'espace. La commande "teleop" permet de prendre le contrôle du robot via une commande manuelle ce qui permet une première approche du déplacement du robot.

  3) OpenCV (cv2)
  
OpenCV est une bibliothèque graphique libre. Nous avons utilisé la biliothèque python cv2 afin de mettre en oeuvre les analyses d'images (obtention des images sous forme de matrice et application d'algorithme de détection de QR-Code via la bibliothèque pyzbar.pyzbar).
  
  

III - Réalisation du projet

Les scripts python que nous avons écrits ainsi qu'un script bash se trouvent dans le répertoire */PDR_2019_KALINOWSKI_VATTIER/pdr2019_ck_av/src/*. Tous scripts python ont en-tête qui détaille leur utilisation et sont commentés.

  1) Détection de QR-Codes
  
La première étape consiste à détecter et lire l'information présente sur un QR-Code à l'aide de la caméra realsense 2.
Pour se faire, nous avons réalisé deux méthodes:
 - La première consiste à utiliser zbar-ros. Grâce au topic */barcode* et au programme *detection.py* que nous avons écrit, au passage d'un QR-code devant la caméra, sa présence est immédiatement détectée et un message renvoie l'information de ce QR-Code sur le terminal. Cependant, notre but est d'analyser plus en détails l'image afin d'avoir accès par exemple à la position du QR-Code sur cette dernière ou encore la distance qui sépare le QR-Code de la caméra. C'est pourquoi nous avons implémenté une deuxième méthode de lecture.
 -L'autre méthode, consiste à utiliser la librairie OpenCV et plus particulièrement "pyzbar.pyzbar". Le code *camera_color.py* permet de lire l'image fournie par la camera RVB et de publier dans un topic les informations relatives à la position du QR-Code détecté ainsi que son nom.

  2) Contrôle du robot
  
Après avoir traité la détection et l'analyse de QR-Codes, il convient de déplacer le robot à l'aide de programmes afin de pouvoir relier par la suite les mouvements du robot à la détection de QR-Codes.
Le code *goforward.py* permet au robot d'avancer à une vitesse voulue à son lancement.

  3) Mise en relation de la caméra et du robot

Nous avons commencé par écrire le script python *move_and_stop.py* qui fait avancer le robot et l'arrête à la détection du premier QR-Code. Les programmes sont structurés en séparant l'analyse d'image et la commande du robot qui s'appuie sur cette dernière.

Nous avons ensuite écrit le script python *move_and_dodge.py* qui fait tourner le robot à 90° lorsqu'un QR-Code est détecté. Afin de donner les bonnes instructions de mouvement au robot (par exemple tourner de 90°), nous avons au préalable fait quelques calculs et réalisé des tests afin d'ajuster vitesse angulaire de rotation et durée de virage.

Il reste à présent réaliser le projet final et répondre au besoin du robot, à savoir éviter les obstacles lors de son déplacement.

Pour ce faire, nous avons tout d'abord imprimé puis collé différents QR-Codes sur des obstacles que le robot est susceptible de rencontrer (chaise, mur...)
Puis nous avons créé deux programmes, *move_2_codes.py* et *maze.py*, qui en fonction du QR-Code détecté, reconnaissent l'obstacle et donnent une consigne de mouvement au robot pour l'éviter. Le premier programme impose au robot une rotation de 90°  vers la gauche à la détection du premier type d'obstacle et d'un demi-tour à la détection du second type. Le deuxième programme remplace la deuxième action par une rotation de 90° vers la droite et permet ainsi au robot de sortir d'un labyrinthe en étant guidé par des QR-Codes judicieusement placés.

Enfin nous avons testé l'execution de ce programme dans un environnement rempli d'obstacles identifiés par des QR-Codes. Le robot évite bien l'ensemble des obstacles, ce qui valide le résultat que nous souhaitions.

IV - Propositions d'améliorations

Les programmes que nous avons écrit ne prennent pas en compte la notion de distance à l'obstacle. Nous avons cherché à faire fonctionner la caméra de profondeur (*depth camera*) mais n'avons réussi à l'exploiter. Le code *depth_camera.py* et les autres versions présentent nos tentatives d'exploiter la profondeur à partir de la détection d'un QR-Code.
Il pourrait être intéressant de donner au robot un ordre pour esquiver un obstacle en fonction de la distance qui le sépare de celui-ci.
L'utilisation de la notion de profondeur ainsi que de l'odométrie pourrait permettre de cartographier l'environnement à l'aide de *Markers*. On pourrait associer à chaque QR-Code les dimensions de l'objet sur lequel il est fixé pour que le robot soit capable de l'éviter (bounding box).
On pourrait également mettre en place un scan grâce à la caméra de profondeur de la même manière qu'un laser.
