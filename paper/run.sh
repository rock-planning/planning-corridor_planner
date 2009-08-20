#! /bin/sh

time ../release/bin/dstar terrain terrain_classes.txt 76 71 full 865 285 $1
dot -Tsvg full-corridors.dot > full-corridors.svg
dot -Tsvg full-blocked-corridors.dot > full-blocked-corridors.svg
dot -Tsvg full-merge-corridors.dot > full-merge-corridors.svg

