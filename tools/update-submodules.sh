#!/bin/bash
grep path .gitmodules | awk '{ print $3 }' > F:/ds/FredFoc/
 
# read
while read LINE
do
    echo $LINE
    (cd ./$LINE && git checkout master && git pull)
done < F:/ds/FredFoc/