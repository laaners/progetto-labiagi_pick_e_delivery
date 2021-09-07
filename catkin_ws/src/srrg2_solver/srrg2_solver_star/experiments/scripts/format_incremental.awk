BEGIN {
    EPOCH_NUM=0;
    ITERATION_TIME=0;
    ITERATION_CHI=0;
}

/EPOCH/ {
    EPOCH_NUM=$3;
}
/TOTAL/ {
    ITERATION_TIME=$3;
}
/STATS/ {
    printf "epoch: %d time: %f chi: %f\n", EPOCH_NUM, ITERATION_TIME, $13;
}

END {
}
