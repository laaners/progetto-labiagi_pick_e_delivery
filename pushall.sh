#!/bin/bash
if [ $# -eq 0 ]; then 
	echo "No argomenti"
else
	echo $1	
	git add -A
	git commit -m $1
	git push
fi
