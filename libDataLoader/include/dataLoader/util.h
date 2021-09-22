//
// Created by sc on 1/13/21.
//

#ifndef LIBSURFELRECONSTRUCTION_UTIL_H
#define LIBSURFELRECONSTRUCTION_UTIL_H
#include <string>
#include <fstream>
#include <Eigen/Core>
namespace PSLAM {
    static inline bool isFileExist(const std::string &filename) {
        std::ifstream file_tmp(filename);
        if (!file_tmp.is_open()) {
            return false;
        }
        file_tmp.close();
        return true;
    }

    static inline void LoadPose(Eigen::Matrix4f &pose, const std::string &path, bool rotate) {
        pose.setIdentity();
        std::ifstream file(path);
        assert(file.is_open());
        if (file.is_open()) {
            for (int i = 0; i < 4; i++)
                for (int j = 0; j < 4; j++)
                    file >> pose(i, j);
            pose.block<3, 1>(0, 3) *= 1000.0f;
            file.close();
        }
//        std::cout << "pose\n"<< pose << "\n";
    }

    static inline const std::vector<std::string> split(const std::string s, const std::string delim) {
    std::vector<std::string> list;
    auto start = 0U;
    auto end = s.find(delim);
    while (true) {
        list.push_back(s.substr(start, end - start));
        if (end == std::string::npos)
            break;
        start = end + delim.length();
        end = s.find(delim, start);
    }
    return list;
    }

    static inline bool LoadInfoIntrinsics(const std::string& filename,
                                    const bool depth_intrinsics,
                            CameraParameters& intrinsics) {
        const std::string search_tag = depth_intrinsics ? "m_calibrationDepthIntrinsic" : "m_calibrationColorIntrinsic";
        const std::string search_tag_w = depth_intrinsics? "m_depthWidth":"m_colorWidth";
        const std::string search_tag_h = depth_intrinsics? "m_depthHeight":"m_colorHeight";
        std::string line{""};
        std::ifstream file(filename);
        int width,height;
        float fx,fy,cx,cy;
        if (file.is_open()) {
            while (std::getline(file,line)) {
                if (line.rfind(search_tag_w, 0) == 0)
                    width = std::stoi(line.substr(line.find("= ")+2, std::string::npos));
                else if (line.rfind(search_tag_h, 0) == 0)
                    height = std::stoi(line.substr(line.find("= ")+2, std::string::npos));
                else if (line.rfind(search_tag, 0) == 0) {
                    const std::string model = line.substr(line.find("= ")+2, std::string::npos);
                    const auto parts = split(model, " ");
                    fx = std::stof(parts[0]);
                    fy = std::stof(parts[5]);
                    cx = std::stof(parts[2]);
                    cy = std::stof(parts[6]);
                }
            }
            file.close();
            intrinsics.Set(width,height,fx,fy,cx,cy,1.f);
            return true;
        }

        return false;
    }
}



#endif //LIBSURFELRECONSTRUCTION_UTIL_H
