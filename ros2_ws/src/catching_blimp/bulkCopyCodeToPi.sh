#!/bin/bash


# Assume we are flashing all blimps 1,...,6
blimp_ids=($(seq 1 6));

# If arguments are passed, assume they are specific blimps to flash
if (($# > 0)); then
    blimp_ids=("$@");
fi

# Get number of blimp ids
num_blimps=${#blimp_ids[@]}

# Print blimp ids to flash
printf "Flashing blimps: ";
for i in $(seq 0 $((num_blimps-1)));
do
    blimp_id=${blimp_ids[$i]}
    printf "%d " $blimp_id
done
printf "\n"

# Flash blimp ids
for i in $(seq 0 $((num_blimps-1)));
do
    blimp_id=${blimp_ids[$i]}
    bash -c "./copyCompileCodeToPi.sh 192.168.0.10$blimp_id" &
done