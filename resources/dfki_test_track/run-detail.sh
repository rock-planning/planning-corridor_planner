#! /bin/sh

time $2 ../../build/src/plan_corridors terrain-detail terrain_classes.txt 11 33 detail 94 53 $1
dot -Tsvg detail-corridors.dot > detail-corridors.svg

