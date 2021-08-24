# PICK E DELIVERY
## Hu Alessio 1839472 LABIAGI-2021
### 1. SCOPO DEL PROGETTO:
Offriamo un semplice servizio di pick e delivery online che permette a un utente registrato in una stanza di chiamare un robot,
affidargli un pacco e assegnargli un destinatario al quale consegnare il pacco
### 2. ARCHITETTURA E TECNOLOGIE UTILIZZATE:
![Diagramma](architettura.png)
### 3. ISTRUZIONI PER L' INSTALLAZIONE E UTILIZZO:
Bisogna innanzitutto aver installato i seguenti programmi:
- ROS Melodic
- Nodejs
- Docker

Per far partire il servizio (i punti 5-8 sono equivalenti al punto 9):
1) Fare una copia in locale della cartella git
   <code>
   git clone HTTPS https://gitlab.com/hu.1839472/progetto-labiagi.git
   </code>
2) Fare source della directory catkin_ws e fare build
   <br/><b>IN: <code>progetto-labiagi/catkin_ws</code></b>
   <code>
   source devel/setup.bash
   catkin build
   </code>
3) Installare i package necessari per nodejs nella cartella main
   <br/><b>IN: <code>progetto-labiagi/main</code></b>
   <code>
   npm install
   </code>
4) Fare il make di proc_webctl
   <br/><b>IN: <code>progetto-labiagi/webctl</code></b>
   <code>
   make
   </code>
5) Creare un immagine di couchdb, farlo partire e creare i due database di users e room
   <code>
   docker run -d --name pick_e_delivery_couchdb -p 5984:5984 -e COUCHDB_USER=admin -e COUCHDB_PASSWORD=admin couchdb
   curl -X PUT http://admin:admin@127.0.0.1:5984/users
   curl -X PUT http://admin:admin@127.0.0.1:5984/rooms
   </code>
   
   Popolare quindi il database delle stanza, nella cartella main eseguire
   <br/><b>IN: <code>progetto-labiagi/main</code></b>
   <code>
   node populate_rooms.js
   </code>
   
   Su http://localhost:5984/_utils è possibile monitorare il database, username e password sono entrambi "admin"
6) Far partire lo stack di navigazione
   <br/><b>IN: <code>progetto-labiagi/catkin_ws/src/srrg2_navigation_2d/april_tag</code></b>
   <code>
   ../../../../webctl/proc_webctl run_navigation.webctl
   </code>

   Fare lo start quindi di:
   <ul>
    <li>01_roscore</li>
    <li>02_stage</li>
    <li>03_mapserver</li>
    <li>04_rviz</li>
    <li>05_localize</li>
    <li>06_planner</li>
    <li>07a_follower(static)</li>
   </ul>
   Guardando su stageros, dare la posa del robot su rviz<br/>
7) Far partire il nodo ros principale
   <code>
   rosrun pick_e_delivery MainNode
   </code>
8) Far partire il server
   <br/><b>IN: <code>progetto-labiagi/main</code></b>
   <code>
   node server.js
   </code>
9) Alternativamente, eseguire il bash nella cartella principale che fa partire lo stack di navigazione, il nodo principale e il server
    <br/><b>IN: <code>progetto-labiagi</code></b>
   <code>
   sudo chmod 777 zzstartserver.sh
   ./zzstartserver.sh
   </code>

    Connettersi in localhost:9001 e iniziare i processi necessari allo stack di navigazione elencati nel punto 6 poi guardando su stageros, dare la posa del robot su rviz.<br/>
    Premere poi un qualsiasi tasto sul terminale per continuare
<br/><h2><b>Il servizio è ora disponibile sulla porta 3000!</b></h2>
### 4. NOTE SUL NODO ROS:
Il nodo MainNode controlla gli stati del robot e i vari eventi di timeout.<br/>
Pubblica sui seguenti topic:
<ul>
    <li><b>/move_base_simple/goal</b> per dare un nuovo goal al robot</li>
    <li><b>/pick_e_delivery/Pose</b> per informare al server la posa del robot e il suo stato</li>
    <li><b>/pick_e_delivery/Timeout</b> per informare al server eventi di timeout</li>
</ul>
&Egrave; iscritto ai seguenti topic:
<ul>
    <li><b>/tf</b> per sapere la posa del robot</li>
    <li><b>/pick_e_delivery/NewGoal</b> per sapere il prossimo goal dal server</li>
</ul>
Offre i seguenti servizi:
<ul>
    <li><b>pick_e_delivery/setTooLongInterval</b> per cambiare il periodo di timeout di tooLong (inizialmente a 50)</li>
    <li><b>pick_e_delivery/setWaitPackInterval</b> per cambiare il periodo di timeout di waitPack (inizialmente a 30)</li>
</ul>
Gli stati del robot sono descritti da questo semplice automa creato con JFLAP<br/>
<img src="robot.png">