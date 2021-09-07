#!/bin/bash
function convertToBoss {
    file="$1"
    prefix=${file%".g2o"}
    echo ${prefix}
    converted_file="${prefix}.boss"
    rosrun srrg2_solver solver_app_graph_converter -i ${file} -o "${converted_file}"
    rosrun srrg2_solver solver_app_graph_sorter -i "${converted_file}" -o "${sorted_file}"
    mv ${converted_file} .
}

compressed_files=`ls g2o_files/*g2o.gz`
for file in ${compressed_files};
do
    uncompressed_file=${file%".gz"}
    echo ${uncompressed_file}
    gzip -d -k "${file}"
    convertToBoss "${uncompressed_file}"
    rm ${uncompressed_file}
done

uncompressed_files=`ls g2o_files/*.g2o`
for file in ${uncompressed_files};
do
    convertToBoss "${file}"
done
