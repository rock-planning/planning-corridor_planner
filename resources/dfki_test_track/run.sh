#! /bin/sh

time $2 ../../build/src/plan_corridors terrain terrain_classes.txt 76 71 full 865 285 $1
dot -Tsvg full-corridors.dot > full-corridors.svg
dot -Tsvg full-blocked-corridors.dot > full-blocked-corridors.svg
dot -Tsvg full-merge-corridors.dot > full-merge-corridors.svg

if [[ -f full-left-corridors.dot ]]; then
    dot -Tsvg full-left-corridors.dot > full-left-corridors.svg
fi
if [[ -f full-right-corridors.dot ]]; then
    dot -Tsvg full-right-corridors.dot > full-right-corridors.svg
fi

