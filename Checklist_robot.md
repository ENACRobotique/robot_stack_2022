## Arrière scène

1. ALlumer le bas niveau

2. Brancher le serial entre le bas niveau et la raspberry pi

3. booter la raspberry pi

4. attendre ~2min (attendre que le docker se lance automatiquement sur la raspberry avec le port /dev/ttyACM0 BRANCHE, sinon le docker n'est pas capable de s'y connecter ultérieurement)

### 5. Si le **ros2serial ne répond pas** de la raspy :

6. se connecter à la raspy via le cable ethernet ou le wifi

7. tuer club_robot sur un terminal: docker stop club_robot

8. relancer avec la commande dans le readme le docker avec ros2serial

(9) lancer un rosbag?

## Sur scène

1. s'assurer que la raspy est sur on

2. Placer le robot, calage contre la couleur correspondante

3. Mettre la couleur J/V

4. placer la réplique dans le robot #TODO : comment on le met ?

5. Placer la vitrine

6. déverouiller l'arret d'urgence de la vitrine

7. Déverouiller l'arret d'urgence du robot

8. brnacher le cable usb stm/raspberry pi

9. S'assurer que le display affiche 0 et que ros est actif et indique qu'il est branché


## Post combat

1. Arret d'urgence du robot sur ON

2. Arret d'urgence de la vitrine sur ON

3. récupérer le robot, la vitrine et les 3 poteaux