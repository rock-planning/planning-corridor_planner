#! /bin/sh

time ../build/bin/dstar terrain.tif terrain_classes.txt 76 71 full 865 285 $1
dot -Tsvg full-corridors.dot > full-corridors.svg

