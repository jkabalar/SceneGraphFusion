//
// Created by sc on 1/13/21.
//

#ifndef LIBSURFELRECONSTRUCTION_DATALOADER_RIO10_H
#define LIBSURFELRECONSTRUCTION_DATALOADER_RIO10_H
#include "dataloader_3rscan.h"
namespace PSLAM {
    class DatasetLoader_RIO10 : public DatasetLoader_3RScan {
    public:
        DatasetLoader_RIO10(std::shared_ptr<DatasetDefinitionBase> dataset);
        void Reset() override;
        bool Retrieve() override;

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    private:
        std::string pose_file_name_ = "";

        Eigen::Matrix4f m_poseTransform;
    };
}

#endif //LIBSURFELRECONSTRUCTION_DATALOADER_RIO10_H
