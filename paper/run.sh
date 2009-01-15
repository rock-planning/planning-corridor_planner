#! /bin/sh

time ../build/bin/dstar terrain.tif terrain_classes.txt 76 71 out 865 285 1
dot -Tpng out-corridors.dot > out-corridors.png

okular out-corridors.png out-corridors.tif &

