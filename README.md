# Progetto-LABIAGI

Progetto LABIAGI: Pick e Delivery

sudo apt-get install curl software-properties-common
curl -sL https://deb.nodesource.com/setup_12.x | sudo -E bash -
sudo apt-get install nodejs

source /home/alessio/Desktop/labiagi/workspaces/srrg2_labiagi/devel/setup.bash
echo Fatto setup.bash di srrg2_labiagi
source /home/alessio/catkin_ws/devel/setup.bash
echo Fatto setup.bash di catkin_ws
source /home/alessio/Desktop/progetto-labiagi/catkin_ws/devel/setup.bash
echo Fatto setup.bash di progetto-labiagi/catkin_ws

docker run -v /tmp/.X11-unix:/tmp/.X11-unix -e "DISPLAY=$DISPLAY:QT_X11_NO_MITSHM=1" -h $HOSTNAME -v $HOME/.Xauthority:/root/.Xauthority --network host -v /dev:/dev --privileged -it giorgiogrisetti/srrg2_navigation_2d:latest