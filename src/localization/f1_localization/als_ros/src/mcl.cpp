#include "rclcpp/rclcpp.hpp"
#include <als_ros/MCL.h>
#include <algorithm>
#include <vector>
#include <iostream>

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    auto mcl = std::make_shared<als_ros::MCL>();

    double localizationHz = mcl->getLocalizationHz();
    if (localizationHz <= 0.0) localizationHz = 1.0;

    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / localizationHz)
    );

    auto timer = mcl->create_wall_timer(period, [mcl]() {
        double start_time = mcl->get_clock()->now().seconds();

        mcl->updateParticlesByMotionModel();
        double t1 = mcl->get_clock()->now().seconds();

        mcl->setCanUpdateScan(false);
        double t2 = mcl->get_clock()->now().seconds();

        mcl->calculateLikelihoodsByMeasurementModel();
        double t3 = mcl->get_clock()->now().seconds();

        mcl->calculateLikelihoodsByDecisionModel();
        double t4 = mcl->get_clock()->now().seconds();

        double t5 = mcl->get_clock()->now().seconds();

        mcl->calculateAMCLRandomParticlesRate();
        double t6 = mcl->get_clock()->now().seconds();

        mcl->calculateEffectiveSampleSize();
        double t7 = mcl->get_clock()->now().seconds();

        mcl->estimatePose();
        double t8 = mcl->get_clock()->now().seconds();

        mcl->resampleParticles();
        double t9 = mcl->get_clock()->now().seconds();

        mcl->publishROSMessages();
        double t10 = mcl->get_clock()->now().seconds();

        mcl->broadcastTF();
        double t11 = mcl->get_clock()->now().seconds();

        mcl->setCanUpdateScan(true);
        double t12 = mcl->get_clock()->now().seconds();

        double total_time = t12 - start_time;
        if (total_time >= 0.020) { // 50 Hz
            std::vector<std::pair<std::string, double>> stages = {
                {"updateParticlesByMotionModel", t1 - start_time},
                {"setCanUpdateScan(false)",      t2 - t1},
                {"calculateLikelihoodsByMeasurementModel", t3 - t2},
                {"calculateLikelihoodsByDecisionModel",    t4 - t3},
                {"calculateGLSampledPosesLikelihood",      t5 - t4},
                {"calculateAMCLRandomParticlesRate",       t6 - t5},
                {"calculateEffectiveSampleSize",           t7 - t6},
                {"estimatePose",                           t8 - t7},
                {"resampleParticles",                      t9 - t8},
                {"publishROSMessages",                     t10 - t9},
                {"broadcastTF",                            t11 - t10},
                {"setCanUpdateScan(true)",                 t12 - t11},
            };

            auto max_stage = std::max_element(
                stages.begin(), stages.end(),
                [](auto &a, auto &b) { return a.second < b.second; }
            );

            std::cout << "총 시간: " << total_time << " 초" << std::endl;
            std::cout << "가장 오래 걸린 작업: "
                      << max_stage->first
                      << " (" << max_stage->second << " 초)" << std::endl;
        }
    });

    rclcpp::spin(mcl);
    rclcpp::shutdown();
    return 0;
}
