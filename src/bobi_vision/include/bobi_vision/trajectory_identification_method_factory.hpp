#ifndef BOBI_TRAJECTORY_IDENTIFICATION_METHOD_FACTORY_HPP
#define BOBI_TRAJECTORY_IDENTIFICATION_METHOD_FACTORY_HPP

#include <bobi_vision/TrajectoryIdentificationConfig.h>

#include <bobi_vision/filtering_method_base.hpp>
#include <bobi_vision/nearest_centroid.hpp>
#include <bobi_vision/replay.hpp>
#include <bobi_vision/hungarian_based.hpp>

namespace bobi {
    class TrajectoryIdentificationMethodFactory {
    public:
        std::pair<std::shared_ptr<FilteringMethodBase>, bool> operator()(std::shared_ptr<ros::NodeHandle> nh, const int choice, bool force_robot_position)
        {
            std::shared_ptr<FilteringMethodBase> new_filter;
            bool success = true;

            switch (choice) {

            case bobi_vision::TrajectoryIdentification_None:
                new_filter = std::make_shared<FilteringMethodBase>(FilteringMethodBase(force_robot_position));
                break;

            case bobi_vision::TrajectoryIdentification_NearestCentroid:
                new_filter = std::make_shared<NearestCentroid>(NearestCentroid(force_robot_position));
                break;

            case bobi_vision::TrajectoryIdentification_HungarianBased:
                new_filter = std::make_shared<HungarianBased>(HungarianBased(force_robot_position));
                break;

            case bobi_vision::TrajectoryIdentification_Replay:
                // TODO: for now the replay is fixed to NearestCentroid but it can be extended
                new_filter = std::make_shared<Replay>(
                    Replay(nh, std::make_shared<NearestCentroid>(NearestCentroid(force_robot_position))));
                break;

                // case bobi_vision::TrajectoryIdentification_MotionModel:
                //     break;

                // case bobi_vision::TrajectoryIdentification_Kalman:
                //     break;

            default:
                new_filter = std::make_shared<FilteringMethodBase>(FilteringMethodBase());
                success = false;
                break;
            }

            return std::pair<std::shared_ptr<FilteringMethodBase>, bool>(new_filter, success);
        }
    };
} // namespace bobi

#endif