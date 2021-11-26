#!/bin/bash

data_folder_path='/usr/local/src/robot/rgbde-data-acquisition/data'

printf "\nSCRIPT FOR AUTOMATIC DATASET ACQUISITION\n"

while : ; do
	printf "\nPress any key to acquire or 'q' to exit...\n"
	read -n 1 k1 <&1
    	#sleep 6
	if [[ $k1 = q ]] ; then
		printf "\n\nQUITTING THE PROGRAM...\n"
		break
	else
		yarpmanager-console --application /usr/local/src/robot/yarp/build/share/yarp/applications/RGBDE_dumper.xml --run --connect --exit --silent
        printf "\nPress any key to stop and save the acquisition or 'd' to delete it...\n"
		read -n 1 k2 <&1
	    if [[ $k2 = d ]] ; then
		    printf "\nDeleting folders...\n"
            cd $data_folder_path
            latest_folders=$(ls -t1 | head -n 6)
            echo $latest_folders
            rm -rf $(echo $latest_folders)
            cd
        fi
	#yarpmanager-console --application /usr/local/src/robot/yarp/build/share/yarp/applications/RGBDE_dumper.xml --disconnect --stop --exit --silent        
        #sleep 2
	    echo "quit" | yarp rpc /rgbde/rgb:o/rpc
	    echo "quit" | yarp rpc /rgbde/depth:o/rpc
	    echo "quit" | yarp rpc /rgbde/events_left:o/rpc
	    echo "quit" | yarp rpc /rgbde/events_right:o/rpc
	    echo "quit" | yarp rpc /rgbde/COMtracker_left:o/rpc
	    echo "quit" | yarp rpc /rgbde/COMtracker_right:o/rpc
        declare -a modules=("yarpdatadumper")
        for module in ${modules[@]}; do
            killall -9 ${module}
        done
        yarp clean
	fi
done
