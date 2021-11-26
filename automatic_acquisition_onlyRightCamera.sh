#!/bin/bash

data_folder_path='/usr/local/src/robot/rgbde-data-acquisition/real_trajectories'
t=139

printf "\nSCRIPT FOR AUTOMATIC DATASET ACQUISITION\n"

while : ; do
	printf "\nPress any key to acquire or 'q' to exit...\n"
	read -n 1 k1 <&1
    	#sleep 6
	if [[ $k1 = q ]] ; then
		printf "\n\nQUITTING THE PROGRAM...\n"
		break
	else
        mkdir /usr/local/src/robot/rgbde-data-acquisition/real_trajectories/real_traj_$t

        binary-dumper --name /events_right --path /usr/local/src/robot/rgbde-data-acquisition/real_trajectories/real_traj_$t > /dev/null 2>&1 &
        sleep 0.5
        yarp connect /zynqGrabber/AE:o /events_right/AE:i
        	
        printf "\nPress any key to stop and save the acquisition...\n"
		read -n 1 k2 <&1
		
		killall -9 binary-dumper

        printf "\nPress any key to keep the acquisition, or 'd' to delete it...\n"
		read -n 1 k2 <&1

	    if [[ $k2 = d ]] ; then
		    printf "\nDeleting folders...\n"
			cd $data_folder_path
			latest_folders=$(ls -t1 | head -n 1)
			echo $latest_folders
			rm -rf $(echo $latest_folders)
			cd
		else
			t=$(expr $t + 1)
		fi
		
		yarp clean > /dev/null 2>&1 &
	fi
done
