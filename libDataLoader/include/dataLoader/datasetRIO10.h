//
// Created by Julia Kabalar on 21.09.2021.
//

#ifndef LIBSURFELRECONSTRUCTION_DATASETRIO10_H
#define LIBSURFELRECONSTRUCTION_DATASETRIO10_H

#include "dataset_base.h"

namespace PSLAM {
    class RIO10Dataset : public DatasetDefinitionBase {
    public:
        RIO10Dataset(INPUTE_TYPE type, const std::string &path)  {
            datasetType = type;
            is_rescan = true;
            folder = path;
            frame_index_counter = 1; 
            number_length = 1;
            prefix_pose = "/frame-";
            prefix_depth = "/frame-";
            prefix_rgb = "/frame-";

            suffix_depth = ".rendered.depth.png";
            suffix_rgb = ".color.jpg";
            suffix_pose = ".pose.txt";

            min_pyr_level = 3;
            number_pose = 6;
            number_length = 6;
        }
        bool use_aligned_pose = true;
    };
}

#endif //LIBSURFELRECONSTRUCTION_DATASETRIO10_H
