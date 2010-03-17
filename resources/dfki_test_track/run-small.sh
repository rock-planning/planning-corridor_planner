#! /bin/sh

time $2 ../../build/src/nav terrain-small.tif terrain_classes.txt 38 37 small 440 155 $1
dot -Tsvg small-plan-simplified-corridors.dot > small-plan-simplified-corridors.svg

