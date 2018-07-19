#!/bin/bash
echo "Hello There, this bash script will help you every time to setup spherex in matrice 100"
echo "Lets start by setting time in Pi if its PC then provide NO else provide date in YYYYMMDD format and time in HR:Min:sec
            1)NO - If its not PI
            2)1  - if date is set
            3)2  - if only want to launch ros and do catkin_make"
read choice
no = "NO"
if ("$choice" != "$no")
then
  echo "Enter date"
  read date
  date +%Y%m%d -s $date
  echo "Enter Time"
  read t
  date +%T -s $t
  cd DJI/matrice_100/src/Onboard-SDK-ROS
  git pull origin master
  cd ..
  cd ..
  catkin_make -l
fi

      
