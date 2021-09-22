//
// Created by Julia Kabalar on 21.09.2021.
//
#include "../include/dataLoader/dataloader_rio10.h"
#include "../include/dataLoader/util.h"
#include "../include/dataLoader/Scan3R_json_loader.h"
#include <ORUtils/Logging.h>

#include <utility>
#include <dataLoader/datasetRIO10.h>
#include <ORUtils/PathTool.hpp>

using namespace PSLAM;


DatasetLoader_RIO10::DatasetLoader_RIO10(std::shared_ptr<DatasetDefinitionBase> dataset):
        DatasetLoader_3RScan(std::move(dataset)) {
    if(!PSLAM::LoadInfoIntrinsics(m_dataset->folder+"/_info.txt",true,m_cam_param_d))
        throw std::runtime_error("unable to open _info file");
    if(!PSLAM::LoadInfoIntrinsics(m_dataset->folder+"/_info.txt",false,m_cam_param_rgb))
        throw std::runtime_error("unable to open _info file");
    m_poseTransform.setIdentity();
    if(reinterpret_cast<PSLAM::RIO10Dataset*>(m_dataset.get())->use_aligned_pose) {
        auto seq_folder = tools::PathTool::find_parent_folder(m_dataset->folder,1);
        auto seq_name = tools::PathTool::getFileName(seq_folder);
        auto data_folder = tools::PathTool::find_parent_folder(seq_folder,1);
        auto scan3rLoader = PSLAM::Scan3RLoader(data_folder+"/3RScan.json");
        if(scan3rLoader.IsRescan(seq_name)) {
            // find ref scan ID
            auto ref_id = scan3rLoader.rescanToReference.at(seq_name);
            m_poseTransform = scan3rLoader.scaninfos.at(ref_id)->rescans.at(seq_name)->transformation;
            m_poseTransform.topRightCorner<3,1>()*=1e3;
            m_dataset->is_rescan = true;
        }
    }
}

bool DatasetLoader_RIO10::Retrieve() {
    std::string depthFilename = GetFileName(m_dataset->folder,
                                            m_dataset->folder_depth,
                                            m_dataset->prefix_depth,
                                            m_dataset->suffix_depth,
                                            m_dataset->number_length);
    std::string colorFilename = GetFileName(m_dataset->folder,
                                            m_dataset->folder_rgb,
                                            m_dataset->prefix_rgb,
                                            m_dataset->suffix_rgb,
                                            m_dataset->number_length);
    pose_file_name_ = GetFileName(m_dataset->folder,
                                  m_dataset->folder_pose,
                                  m_dataset->prefix_pose,
                                  m_dataset->suffix_pose,
                                  m_dataset->number_length);
    bool isExist = isFileExist(depthFilename);
    if (!isExist) {
        frame_index = 0;
        SCLOG(VERBOSE) << "Cannot find path:\n" << depthFilename << "\n" << colorFilename;

        return false;
    }
    m_d = cv::imread(depthFilename, -1);
    // mask depth
    for(size_t c=0;c<(size_t)m_d.cols;++c){
        for(size_t r=0;r<(size_t)m_d.rows;++r){
            if(m_d.at<unsigned short>(r,c)>=m_dataset->max_depth)
                m_d.at<unsigned short>(r,c) = 0;
        }
    }

    if (isFileExist(colorFilename.c_str())) {
        m_rgb = cv::imread(colorFilename, -1);
    }
    if (m_dataset->rotate_pose_img) {
        cv::rotate(m_d, m_d, cv::ROTATE_90_COUNTERCLOCKWISE);
    }
    LoadPose(m_pose, pose_file_name_,m_dataset->rotate_pose_img);
    m_pose = m_poseTransform * m_pose;
    frame_index += m_dataset->frame_index_counter;
    return true;
}



void DatasetLoader_RIO10::Reset() {
    frame_index = 0;
}