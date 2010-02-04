#! /bin/sh

time ../../build/bin/nav terrain-small.tif terrain_classes.txt 38 37 small 440 155 $1
dot -Tsvg small-corridors.dot > small-corridors.svg

