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
De ce fait, nous avons suivi le tutoriel ROS wiki, afin d'installer ROS (ainsi que plusieures librairies) et de nous familiariser avec ses fonctionnalités principales. Nous avons donc appris à utiliser des subscribers, publishers de manière à interragir sur les noeuds etc.
Afin de comprendre plus concrêtement le rôle de tous ces paramètres, nous avons suivi l'exemple de "Turtlesim", qui consiste à faire déplacer une petite tortue dans son environnement virtuel. Grâce à cet exemple, nous avons pu appliquer ces principes sur le robot Kobuki et le faire ainsi déplacer dans l'espace. La commande "teleop" permet entre autres de prendre le contrôle du robot assez aisément.

  3) Open CV (cv2)
  
  
  


III - Réalisation du projet

  1) Détection de QR-Codes
  
La première étape consiste à détecter et lire l'information présente sur un QR-Code à l'aide de la caméra realsense 2.
Pour se faire, nous avons réalisé deux méthodes:
 - La première consiste à utiliser zbar-ros. De ce fait, au passage d'un QR-code devant la caméra, sa présence est immédiatement détectée et un message renvoie l'information de ce QR-Code sur le terminal. Cependant, notre but est d'analyser plus en détails l'image afin d'avoir accès par exemple à la position du QR-Code sur cette dernière ou encore la distance qui sépare le QR-Code de la caméra. C'est pourquoi nous avons implémenté une deuxième méthode de lecture.
 -L'autre méthode, consiste à utiliser la librairie OpenCV et plus particulièrement "pyzbar". Nous avons alors accès à différents topics qui nous permettent d'étudier et d'exploiter l'image.
Le code "" permet... et ...

  2) Contrôle du robot
  
Après avoir traité la détection et l'analyse de QR-Codes, il convient de déplacer le robot à l'aide de programmes (ici sous python) afin de pouvoir relier par la suite les mouvements du robot à la détection de QR-Codes.
Le code "move_forward" permet au robot d'avancer à une vitesse voulue à son lancement.
Afin de l'implémenter nous avons dû publier sur tel topic et faire...

  3) Mise en relation de la caméra et du robot
  
Il reste à présent réaliser le projet final et répondre au besoin du robot, à savoir éviter les obstacles lors de son déplacement.
Pour ce faire, nous avons tout d'abord imprimmé puis collé différents QR-Codes sur des obstacles que le robot est susceptible de rencontrer (chaise, mur...)
Puis nous avons créé un programme, qui en fonction du QR-Code détecté, reconnait l'obstacle et donne une consigne de mouvement au robot pour l'éviter.
Afin de donner les bonnes instructions de mouvement au robot (par exemple tourner de 90°), nous avons au préalable fait quelques calculs et réalisé des tests afin d'ajuster vitesse angulaire de rotation et durée de virage.

Voici en détail les actions du programme "":

Enfin nous avons testé l'execution de ce programme dans un environnement rempli d'obstacles identifiés par des QR-Codes. Le robot évite bien l'ensemble des obstacles, ce qui valide le résultat que nous souhaitions.
