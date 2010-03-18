#! /bin/sh

time $2 ../../build/src/nav envire::Grid_105.tiff terrain_classes.txt 29 277 test_track 23 61 $1
dot -Tsvg test_track-plan-corridors.dot > test_track-plan-corridors.svg
dot -Tsvg test_track-plan-simplified-corridors.dot > test_track-plan-simplified-corridors.svg

