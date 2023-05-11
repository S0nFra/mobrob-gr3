#!/bin/bash

SSHPASS=turtlebot
USER=ubuntu
IP=$1
REALSENSE_SRC=$2

sshpass -p $SSHPASS sftp -o StrictHostKeyChecking=no $USER@$IP << EOF
cd /home/$USER/realsense/catkin_ws/src/realsense/src/
put $REALSENSE_SRC
chmod 777 $REALSENSE_SRC
bye
EOF

echo "Upload completed"