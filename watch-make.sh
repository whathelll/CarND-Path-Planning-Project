#!/bin/bash

if [ $# -ne 2 ]; then
	echo usage: $0 project_dir command_to_run
	exit
fi

PROJECT_DIR=$1
COMMAND=$2

while true
do
    # inotifywait -e close_write $PROJECT_DIR
    inotifywait $PROJECT_DIR
    echo do stuff
    make
    $COMMAND
done
