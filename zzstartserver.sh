#!/bin/bash
#sed -i 's/\r$//' filename
docker run -d --name pick_e_delivery_couchdb -p 5984:5984 -e COUCHDB_USER=admin -e COUCHDB_PASSWORD=admin couchdb
docker start pick_e_delivery_couchdb
echo Couchdb partito sulla porta 5984

cd catkin_ws/src/srrg2_navigation_2d/april_tag/
#../../../../webctl/proc_webctl run_navigation.webctl
gnome-terminal --tab -- ../../../../webctl/proc_webctl run_navigation.webctl
echo Navigation Stack partito sulla porta 9001

read -n 1 -p "Fai partire lo stack di navigazione" mainmenuinput
cd ../../../..

curl -X PUT http://admin:admin@127.0.0.1:5984/users
curl -X PUT http://admin:admin@127.0.0.1:5984/rooms
echo Creato database users e rooms se non esistevano

noder main/populate_rooms.js
echo Popolato il database dei rooms

gnome-terminal --tab -- node main/server.js
echo Server partito sulla porta 3000

gnome-terminal --tab -- rosrun pick_e_delivery MainNode
echo Nodo ros partito