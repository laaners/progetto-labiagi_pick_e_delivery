#!/bin/bash
OVERWRITER_CONF="configs/ovt.conf"
GEN_FILE="run_gen.sh"
SIMULATOR_EXEC="~/workspaces/srrg2/devel/lib/srrg2_solver/solver_app_plgo_simulator"
function completeDataset {
    prefix="$1"
    path_type="$2"
    split_diameter="$3"
    ideal_name="${prefix}_ideal.boss"
    ideal_history="${prefix}_ideal_history.boss"
    incremental_name="${prefix}_incremental.boss"
    batch_name="${prefix}_batch.boss"

    # overwrite covariance
    printf "%s" "rosrun srrg2_solver solver_app_info_overwrite -c ${OVERWRITER_CONF}  -i ${ideal_name} -o ${ideal_name}; " >> ${GEN_FILE}
    printf "%s" "rosrun srrg2_solver solver_app_noise_adder -i ${ideal_name} -o ${ideal_name};" >> ${GEN_FILE}
    printf "%s" "rosrun srrg2_solver solver_app_graph_initializer -i ${ideal_name} -o ${incremental_name}; " >> ${GEN_FILE}
    printf "%s\n" "rosrun srrg2_solver solver_app_graph_sorter -t ${path_type} -i ${incremental_name} -o ${incremental_name} " >> ${GEN_FILE}
}

rm -f ${GEN_FILE}

#BA
echo "manhattan"
printf "%s" "${SIMULATOR_EXEC} -gt se3_ba -mo -ml -mt manhattan -lsf 0 -lpp 1 -lsr 3  -o manhattan_ba_ideal.boss; " >> ${GEN_FILE}
completeDataset manhattan_ba VariableSE3QuaternionRightAD 3

printf "%s" "${SIMULATOR_EXEC} -gt sim3 -mo -mp -mt manhattan -lsf 0 -lpp 1 -psr 1.5  -o manhattan_sim3_ideal.boss; " >> ${GEN_FILE}
completeDataset manhattan_sim3 VariableSim3QuaternionRightAD 3

printf "%s" "${SIMULATOR_EXEC} -gt se3 -mo -mp -mt manhattan -lsf 0 -lpp 1 -psr 1.5  -o manhattan_se3_ideal.boss; " >> ${GEN_FILE}
completeDataset manhattan_se3 VariableSE3QuaternionRightAD 3

printf "%s" "${SIMULATOR_EXEC} -gt se3 -mo -ml -mt manhattan -lsf 0 -lpp 1 -lsr 3  -o manhattan_plgo_ideal.boss; " >> ${GEN_FILE}
completeDataset manhattan_plgo VariableSE3QuaternionRightAD 3


echo "torus"
# #BA
printf "%s" "${SIMULATOR_EXEC} -gt se3_ba -mo -ml -mt torus -lsf 0 -lpp 1 -lsr 7 -ma 60 50 10 40 3 -o torus_50_ba_ideal.boss; " >> ${GEN_FILE}
completeDataset torus_50_ba VariableSE3QuaternionRightAD 3

# #PLGO
printf "%s" "${SIMULATOR_EXEC} -gt se3 -mo -ml -mt torus -lsf 0 -lpp 1 -lsr 5.5 -ma 60 50 10 30 3 -o torus_50_plgo_ideal.boss; " >> ${GEN_FILE} 
completeDataset torus_50_plgo VariableSE3QuaternionRightAD 3

# #PGO
printf "%s" "${SIMULATOR_EXEC} -gt se3 -mo -mp -mt torus -psr 10 -ma 60 50 10 30 2 -o torus_50_se3_ideal.boss;" >>${GEN_FILE} 
completeDataset torus_50_se3 VariableSE3QuaternionRightAD 3

# #SIM3
printf "%s" "${SIMULATOR_EXEC} -gt sim3 -mo -mp -mt torus -psr 10 -ma 60 50 10 30 2 -o torus_50_sim3_ideal.boss;" >> ${GEN_FILE} 
completeDataset torus_50_sim3 VariableSim3QuaternionRightAD 3

echo "sphere"
# #BA
printf "%s" "${SIMULATOR_EXEC} -gt se3_ba -mo -ml -mt sphere_uniform -lsf 0 -lpp 3 -lsr 15 -ma 50 50 -o sphere_50_ba_ideal.boss;" >> ${GEN_FILE}
completeDataset sphere_50_ba VariableSE3QuaternionRightAD 3

# #PLGO
printf "%s" "${SIMULATOR_EXEC} -gt se3 -mo -ml -mt sphere_uniform -lsf 0 -lpp 3 -lsr 15 -ma 50 50 -o sphere_50_plgo_ideal.boss;" >> ${GEN_FILE}
completeDataset sphere_50_plgo VariableSE3QuaternionRightAD 3

# #PGO
printf "%s" "${SIMULATOR_EXEC} -gt se3 -mo -mp -mt sphere_uniform -psr 10 -ma 50 50 -o sphere_50_se3_ideal.boss;" >> ${GEN_FILE}
completeDataset sphere_50_se3 VariableSE3QuaternionRightAD 3

# #SIM3
printf "%s" "${SIMULATOR_EXEC} -gt sim3 -mo -mp -mt sphere_uniform -psr 10 -ma 50 50 -o sphere_50_sim3_ideal.boss;" >> ${GEN_FILE}
completeDataset sphere_50_sim3 VariableSim3QuaternionRightAD 3

