# Mobile robot project - group 3

Questo progetto realizza un sistema di movimento autonomo mediante il framework **ROS** per il robot **turtlebot3 waffle pi**. Tale applicazione è stata realizzata avendo come mappa di riferimento il **DIEM**. Lo spostamento del robot avviene attraverso dei punti specifici posti sulla mappa noti come waypoint e rappresentati da due coordinate. 

![](src/navigation/landmarks/waypoints.png)

Per poter far muovere il robot autonomamente è necessario fornire dei comandi che esprimono la direzione da prendere una volta raggiunto il prossimo waypoint. I possibili comandi da fornire al robot sono *STRAIGHT ON*, *RIGHT*, *LEFT*, *GO BACK* e *STOP*; questi devono essere codificati in codici QR e sono posti  sulle pareti lungo il percorso.

Il funzionamento dell'intero sistema può essere riprodotto in simulazione nell'ambiente Gazebo oppure nella realtà.



## Requisiti preliminari

Una volta clonata la repository effettuare il `build` del progetto.

```
cd mobrob-gr3
catkin build
```



## Esecuzione in simulazione su Gazebo

I comandi da eseguire per poter avviare la simulazione sono i seguenti:

1. Avviare l'ambiente di simulazione Gazebo e Rviz:

   ```
   source setup.bash
   roslaunch map2gazebo turtlebot3_diem_sim.launch
   ```

2. Avviare la webcam del pc per la decodifica dei QR:

   ```
   source setup.bash
   roslaunch qrscan webcam.launch
   ```

   In alternativa si può pubblicare direttamente il comando in formato `String`:

   ```
   source setup.bash
   rostopic pub /navigation/command std_msgs/String "data: 'command'"
   ```

   dove `command` va sostituito con il comando da dare al robot.

3.  Avviare il sistema di navigazione autonomo:

   ```
   source setup.bash
   rosrun navigation nav.py
   ```



## Esecuzione con turtlebot3 waffle pi

Per avviare un'esecuzione seguire i seguenti passaggi:

1. Avviare Rviz su cui verrà caricata la mappa:

   ```
   source setup.bash
   roslaunch map2gazebo turtlebot3_diem.launch
   ```

2. Avviare le telecamere eseguendo:

   ```
   source setup.bash
   roslaunch qrscan camera.launch
   ```

3. Avviare il sistema di navigazione autonomo:

   ```
   source setup.bash
   rosrun navigation nav.py
   ```



## Branches

Il progetto si articola in due differenti branches: `hybrid` e `repositioning` . Si differenziano nella logica innescata quando il robot raggiunge un punto senza aver letto nessun comando. Nel primo caso, quando il robot raggiunge un waypoint e non ha letto nessun comando, torna indietro al waypoint precedente finché non viene rilevato un QR; nel secondo caso invece il robot attende di essere spostato manualmente all'ultimo waypoint raggiunto correttamente. 



## Contatti

| Nome e cognome       | Matricola  | Email                           |
| -------------------- | ---------- | ------------------------------- |
| Avitabile Margherita | 0622701825 | m.avitabile6@studenti.unisa.it  |
| Grimaldi Andrea      | 0622701830 | a.grimaldi112@studenti.unisa.it |
| Mignone Lorenzo      | 0622701866 | l.mignone@studenti.unisa.it     |
| Sonnessa Francesco   | 0622701672 | f.sonnessa@studenti.unisa.it    |
