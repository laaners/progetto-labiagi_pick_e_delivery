#sed -i 's/\r$//' filename
docker run -d --name my-couchdb-new -p 5984:5984 -e COUCHDB_USER=admin -e COUCHDB_PASSWORD=admin couchdb
echo Couchdb partito sulla porta 5984

gnome-terminal --tab -- webctl/proc_webctl catkin_ws/src/srrg2_navigation_2d/april_tag/run_navigation.webctl
echo Navigation Stack partito sulla porta 9001

gnome-terminal --tab -- node main/server.js
echo Server partito sulla porta 3000

gnome-terminal --tab -- rosrun pick_e_delivery MainNode
echo Nodo ros partito