#!/bin/bash
grep path .gitmodules | awk '{ print $3 }' > /tmp/study-git-submodule-dirs
 
# read
while read LINE
do
    echo $LINE
    (cd ./$LINE && git checkout master && git pull)
done < /tmp/study-git-submodule-dirs