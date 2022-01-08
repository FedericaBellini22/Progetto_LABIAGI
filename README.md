# Progetto_LABIAGI

Repository del progetto "Collision Avoidance" per il corso di LABIAGI a.a. 2020-2021.

Implementazione e test sono stati svolti con ROS Noetic su Ubuntu 20.04 (LTS) Focal Fossa.

### Specifica del progetto

Il progetto implementa un sistema anti-collisione basato su laser.

Input:

    scan, cmd_vel

Output:

    cmd_vel che non va a sbattere

Il sistema produce un comando di velocità che, utilizzando lo scan, previene la collisione con gli oggetti rilevati attorno a sé, preferibilmente "deflettendo" la traiettoria del robot verso uno spazio libero da collisioni.

Parametri di interesse:

    p_i (x,y): posa ostacolo
    t (x,y): posa robot
    t - p_i: direzione forza risultante
    1/norm(t_i - p_i): modulo forza risultante

\sum_i fi + cmd_vel

### Esecuzione e test

Per scaricare il materiale necessario:

- Fare il download di questo repository nella cartella `src` del proprio catkin workspace;
- Posizionarsi in `~/path_to_your_workspace` e compilare con il comando `catkin_make`;
- Se non lo si ha già installato, installare il package del controller remoto `teleop_twist_keyboard` con il seguente comando: `sudo apt-get install ros-noetic-teleop-twist-keyboard`.

Per testare il nodo:

1. In un terminal, avviare il master:

   ```bash
   $ roscore
   ```

2. In un altro terminal, avviare il nodo:

   ```bash
   $ rosrun collision_avoidance collision_avoidance
   ```

3. In un altro terminal, avviare il controller:

   ```bash
   $ rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=avoid_cmd_vel
   ```

4. In un altro terminal, avviare l'ambiente di simulazione:

   ```bash
   $ rosrun stage_ros stageros ~/path_to_your_world_file/your_file.world
   ```

   I test per questo progetto sono stati effettuati sul file cappero_laser_odom_diag_obstacle_2020-05-06-16-26-03.world. Assicurarsi che nella stessa cartella del file .world si trovi anche la relativa immagine.

5. Muoversi liberamente con i comandi di teleop_twist_keyboard.

Per visualizzare in tempo reale il laser_scan con il tool Rviz:

1. Avviare Rviz con il comando:

   ```bash
   $ rosrun rviz rviz
   ```

2. Nelle Global Options del pannello a sinistra, selezionare Fixed Frame $\to$ base_footprint.

3. Premere il tasto Add $\to$ By topic $\to$ /base_scan.
