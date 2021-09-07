#!/bin/bash
PLGO_DATASETS=`ls *_plgo_incremental.boss`
BA_DATASETS=`ls *_ba_incremental.boss`
SE3_DATASETS=`ls *_se3_incremental.boss`
SIM3_DATASETS=`ls *_sim3_incremental.boss`

additional_flags=$1;
echo "running batch experiments with the following additional flags [${additional_flags}]"
sleep 1
SRRG_SOLVER_INCREMENTAL="~/workspaces/srrg2/devel/lib/srrg2_solver_star/solver_app_star_solver"
SRRG_SOLVER_BATCH="~/workspaces/srrg2/devel/lib/srrg2_solver_star/solver_app_star_solver_batch"

STAR_INCREMENTAL_LIST="run_optinc.sh"
STAR_BATCH_LIST="run_optbat.sh"
PLAIN_INCREMENTAL_LIST="run_optincplain.sh"

rm -f ${GENERATED_LIST} ${STAR_BATCH_LIST} ${PLAIN_INCREMENTAL_LIST}
function solveOneIncremental {
    mydataset="$1_incremental.boss"
    gt_file="$1_ideal.boss"
    output_file="$1_optinc.boss"
    log_file="$1_optinc.log"
    config_type="$2"
    echo "star-incremental-> input: ${mydataset}, output ${output_file}, gt: ${gt_file}, log: ${log_file}"
    COMMAND_ARGS="-c configs/star_solver_${config_type}_incremental.config -i ${mydataset} -o ${output_file}"
    GT_FILE_FOUND=false
    if [ -f "${gt_file}" ]; then
        COMMAND_ARGS="${COMMAND_ARGS} -bench ${gt_file}"
    else
        gt_file="";
    fi
    COMMAND_ARGS="${COMMAND_ARGS} >& ${log_file}"
    printf "%s %s" "${SRRG_SOLVER_INCREMENTAL}" "${COMMAND_ARGS}" >> "${STAR_INCREMENTAL_LIST}"
    printf "\n" >> "${STAR_INCREMENTAL_LIST}"
}

function solveOneIncrementalPlain {
    mydataset="$1_incremental.boss"
    gt_file="$1_ideal.boss"
    output_file="$1_optinc_plain.boss"
    log_file="$1_optinc_plain.log"
    config_type="$2"
    COMMAND_ARGS="-c configs/star_solver_${config_type}_incremental_plain.config -i ${mydataset} -o ${output_file}"
    GT_FILE_FOUND=false
    if [ -f "${gt_file}" ]; then
        COMMAND_ARGS="${COMMAND_ARGS} -bench ${gt_file}"
    else
        gt_file="";
    fi
    echo "plain-incremental-> input: ${mydataset}, output ${output_file}, gt: ${gt_file}, log: ${log_file}"

    COMMAND_ARGS="${COMMAND_ARGS} >& ${log_file}"
    printf "%s %s" "${SRRG_SOLVER_INCREMENTAL}" "${COMMAND_ARGS}" >> "${PLAIN_INCREMENTAL_LIST}"
    printf "\n" >> "${PLAIN_INCREMENTAL_LIST}"
}

function solveOneBatch {
    mydataset="$1_incremental.boss"
    gt_file="$1_ideal.boss"
    output_file="$1_optbat.boss"
    log_file="$1_optbat.log"
    config_type="$2"
    COMMAND_ARGS="-c configs/star_solver_${config_type}_batch.config -i ${mydataset} -o ${output_file} >& ${output_file}"
    echo "star-batch-> input: ${mydataset}, output ${output_file}, gt: ${gt_file}, log: ${log_file}"
    printf "%s %s" "${SRRG_SOLVER_BATCH}" "${COMMAND_ARGS}" >> "${STAR_BATCH_LIST}"
    printf "\n" >> "${STAR_BATCH_LIST}"
}

echo "running shit"
suffix=".boss";


for dataset in ${SE3_DATASETS}
do
    prefix=${dataset%"_incremental$suffix"}
    solveOneIncremental "$prefix" se3
    solveOneIncrementalPlain "$prefix" se3
    solveOneBatch "$prefix" se3
done

for dataset in ${SIM3_DATASETS}
do
    prefix=${dataset%"_incremental$suffix"}
    solveOneIncremental "$prefix" sim3
    solveOneIncrementalPlain "$prefix" sim3
    solveOneBatch "$prefix" sim3
done

for dataset in ${BA_DATASETS}
do
    prefix=${dataset%"_incremental$suffix"}
    solveOneIncremental "$prefix" plgo
    solveOneIncrementalPlain "$prefix" plgo
    solveOneBatch "$prefix" se3
done

for dataset in ${PLGO_DATASETS}
do
    prefix=${dataset%"_incremental$suffix"}
    solveOneIncremental "$prefix" plgo
    solveOneIncrementalPlain "$prefix" plgo
    solveOneBatch "$prefix" se3
done

