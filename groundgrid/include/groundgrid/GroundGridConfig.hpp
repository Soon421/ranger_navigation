/*
Copyright 2023 Dahlem Center for Machine Learning and Robotics, Freie Universität Berlin

Redistribution and use in source and binary forms, with or without modification, are permitted
provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions
and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of
conditions and the following disclaimer in the documentation and/or other materials provided
with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to
endorse or promote products derived from this software without specific prior written permission.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#pragma once

namespace groundgrid {

/**
 * Configuration structure for GroundGrid parameters.
 * Replaces ROS1 dynamic_reconfigure with ROS2 parameters.
 */
struct GroundGridConfig {
    // If a cell has at least this many points, the variance of just that cell is used
    // instead of the variance of 3x3 or 5x5 patch
    int point_count_cell_variance_threshold = 10;

    // Maximum velodyne ring for ground detection consideration
    int max_ring = 1024;

    // If the ground patch layer at a cell is below this value,
    // a cell without the minimum point count can be classified as ground
    double groundpatch_detection_minimum_threshold = 0.01;

    // Compensates for the geometric dilution of the point density with the distance
    double distance_factor = 0.00002;

    // Minimum value for the distance factor
    double minimum_distance_factor = 0.0001;

    // Points lower than ground height + threshold are considered ground points
    double miminum_point_height_threshold = 0.05;

    // Minimum obstacle detection threshold [m]
    double minimum_point_height_obstacle_threshold = 0.02;

    // Outlier detection tolerance [m]
    double outlier_tolerance = 0.03;

    // Minimum point count for ground patch detection in percent of expected point count
    double ground_patch_detection_minimum_point_count_threshold = 0.25;

    // Distance from the center from which on the patch size is increased [m]
    double patch_size_change_distance = 20.0;

    // Occupied cells decrease factor [100/x %]
    double occupied_cells_decrease_factor = 5.0;

    // Occupied cells point count factor [100/x %]
    double occupied_cells_point_count_factor = 20.0;

    // Minimum ground confidence to consider lower points an outlier (5x5 patch)
    double min_outlier_detection_ground_confidence = 1.25;

    // Maximum number of threads
    int thread_count = 8;
};

}  // namespace groundgrid
