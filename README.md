# Progetto_LABIAGI

Repository del progetto "Collision Avoidance" per il corso di LABIAGI a.a. 2020-2021.

Il progetto implementa un sistema anti-collisione basato su laser.

Input:

    scan, cmd_vel

Il sistema prende in input:

    uno scan
    un comando di velocita'

Output:

    cmd_vel che non va a sbattere

Il sistema produce un comando di velocita', che utilizzando lo scan previene la collisione con gli oggetti rilevati nel contorno, preferibilmente "deflettendo" la traiettoria del robot verso uno spazio libero da collisioni.

Parametri di interesse:

    p_i (x,y): posa ostacolo
    t (x,y): posa robot
    t - p_i: direzione forza risultante
    1/norm(t_i - p_i): modulo forza risultante

\sum_i fi + cmd_ve
