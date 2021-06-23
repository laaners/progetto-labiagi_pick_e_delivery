#!/bin/bash
echo Il tuo push: 
read commit_msg
git add -A
git commit -m "$commit_msg"
git push
