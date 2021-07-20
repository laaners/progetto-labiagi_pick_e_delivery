#!/bin/bash
if [ $# -eq 0 ]; then 
	echo "No argomenti"
else
	echo $1
	echo git add -A
	git add -A
	echo git commit -m "$1"
	git commit -m "$1"
	echo git push
	git push
fi
