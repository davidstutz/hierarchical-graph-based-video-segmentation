/**
 * This command line tool demonstrates the usage of hierarchical graph-based video
 * segmentation as implemented in lib_segment.
 * 
 * Implementation of the hierarchical graph-based video segmentation algorithm
 * proposed by Grundmann et al. [1]:
 * 
 *  [1] M. Grundmann, V. Kwatra, M. Han, and I. A. Essa.
 *      Efficient hierarchical graph-based video segmentation.
 *      In Conference on Computer Vision and Pattern Recognition, pages 2141–2148,
 *      San Francisco, 2010, June 2010.
 * 
 * The algorithm is based on the graph-based image segmentation algorithm by
 * Felzenswalb and Huttenlocher [2]:
 * 
 *  [2] P. F. Felzenszwalb and D. P. Huttenlocher.
 *      Efficient graph-based image segmentation.
 *      International Journal of Computer Vision, 59(2):167–181, September 2004.
 * 
 * Usage:
 *  $ ./segment_cli/segment_cli --help
 *  Allowed options:
 *    --help                           produce help message
 *    --input-video arg                directory of input video (provided as 
 *                                     sequence of individual images)
 *    --input-flow arg                 input flow directory (text files in 
 *                                     cv::Storage format, see io.h)
 *    --length arg (=10)               length of video to oversegment (may be lower
 *                                     than the actual sequence length)
 *    --flow-weight arg (=0.200000003) weight on flow angle for edge weight 
 *                                     computation
 *    --threshold arg (=0.0199999996)  threshold (is multiplied by 1.3 for each 
 *                                     additional hierarchy level)
 *    --hierarchies arg (=40)          number of hierarchies
 *    --vis-dir arg                    visualization directory (default will not 
 *                                     visualize the result!)
 *    --output-dir arg (=output)       output directory
 *  
 * 
 * The code is published under the BSD 3-Clause:
 * 
 * Copyright (c) 2014 - 2015, David Stutz
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <fstream>
#include <iostream>
#include <iomanip>
#include <cstdlib>
#include <vector>
#include <opencv2/opencv.hpp>
#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include <boost/timer.hpp>
#include "io.h"
#include "io_util.h"
#include "graph_segmentation.h"
#include "evaluation.h"

int main(int argc, char** argv) {

    boost::program_options::options_description desc("Allowed options");
    desc.add_options()
        ("help", "produce help message")
        ("input-video", boost::program_options::value<std::string>(), "directory of input video (provided as sequence of individual images)")
        ("input-flow", boost::program_options::value<std::string>(), "input flow directory (text files in cv::Storage format, see io.h)")
        ("input-gt", boost::program_options::value<std::string>(), "input ground truth; if given, some metrics will be computed and displayed")
        ("length", boost::program_options::value<int>()->default_value(10), "length of video to oversegment (may be lower than the actual sequence length)")
        ("flow-weight", boost::program_options::value<float>()->default_value(0.2f), "weight on flow angle for edge weight computation")
        ("threshold", boost::program_options::value<float>()->default_value(0.02), "threshold (is multiplied by 1.3 for each additional hierarchy level)")
        ("hierarchies", boost::program_options::value<int>()->default_value(40), "number of hierarchies")
        ("vis-dir", boost::program_options::value<std::string>()->default_value(""), "visualization directory (default will not visualize the result!)")
        ("output-dir", boost::program_options::value<std::string>()->default_value("output"), "output directory");

    boost::program_options::positional_options_description positionals;
    positionals.add("input-video", 1);
    positionals.add("input-flow", 1);
    
    boost::program_options::variables_map parameters;
    boost::program_options::store(boost::program_options::command_line_parser(argc, argv).options(desc).positional(positionals).run(), parameters);
    boost::program_options::notify(parameters);

    if (parameters.find("help") != parameters.end()) {
        std::cout << desc << std::endl;
        return 1;
    }
    
    boost::filesystem::path in_dir(parameters["input-video"].as<std::string>());
    if (!boost::filesystem::is_directory(in_dir)) {
        std::cout << "Input directory does not exist." << std::endl;
        return 1;
    }
    
    boost::filesystem::path flow_dir(parameters["input-flow"].as<std::string>());
    if (!boost::filesystem::is_directory(flow_dir)) {
        std::cout << "Flow directory does not exist." << std::endl;
        return 1;
    }
    
    
    bool groundTruth = false;
    boost::filesystem::path gt_dir(parameters["input-gt"].as<std::string>());
    if (!gt_dir.empty()) {
        if (!boost::filesystem::is_directory(gt_dir)) {
            std::cout << "Specified ground truth directory does not exist." << std::endl;
            return 1;
        }
        
        groundTruth = true;
    }
    
    bool visualize = false;
    boost::filesystem::path vis_dir(parameters["vis-dir"].as<std::string>());
    if (!vis_dir.empty()) {
        if (!boost::filesystem::is_directory(vis_dir)) {
            boost::filesystem::create_directories(vis_dir);
        }
        
        visualize = true;
    }
    
    boost::filesystem::path out_dir(parameters["output-dir"].as<std::string>());
    if (!boost::filesystem::is_directory(out_dir)) {
        boost::filesystem::create_directory(out_dir);
    }
    
    Video video = IO::readVideo(in_dir, parameters["length"].as<int>());
    FlowVideo flowVideo = IO::readFlowVideo(flow_dir, parameters["length"].as<int>());
    
    SegmentationVideo gtVideo;
    if (groundTruth) {
        gtVideo = IO::readSegmentationVideo(gt_dir, parameters["length"].as<int>());
    }
    
    assert(video.getFrameNumber() > 0);
    assert(video.getFrameNumber() == flowVideo.getFrameNumber());
    
    // Parameters:
    int M = 300;
    int L = parameters["hierarchies"].as<int>();
    float c = parameters["threshold"].as<float>();
    float beta = parameters["flow-weight"].as<float>();
    float alpha = 1 - beta;
    
    GraphSegmentationMagic* magic = new GraphSegmentationMagicThreshold(c);
    GraphSegmentationDistance* distance = new GraphSegmentationEuclideanRGBFlowAngle(alpha, beta);
    
    GraphSegmentation segmenter(distance, magic);
    
    boost::timer timer;
    segmenter.buildGraph(video, flowVideo);
    
    segmenter.buildEdges();
    std::cout << "----- Level 0" << std::endl;
    std::cout << "Built graph (" << timer.elapsed() << ")." << std::endl;
    
    timer.restart();
    segmenter.oversegmentGraph();
    std::cout << "Oversegmented graph (" << timer.elapsed() << ")." 
            << std::endl;
    
    timer.restart();
    segmenter.enforceMinimumSegmentSize(M);
    std::cout << "Enforced minimum region size (" << timer.elapsed() 
            << ")." << std::endl;
    
    SegmentationVideo svVideo = segmenter.deriveLabels();
    IO::writeSegmentationVideo(out_dir / boost::filesystem::path("0"), 
                svVideo);
    
    if (groundTruth) {
        float boundary_recall = Evaluation::compute3DBoundaryRecall(svVideo, gtVideo);
        float undersegmentation_error = Evaluation::compute3DNPUndersegmentationError(svVideo, gtVideo);
        float achievable_segmentation_accuracy = Evaluation::compute3DAchievableSegmentationAccuracy(svVideo, gtVideo);
        
        std::cout << "3D Boundary Recall: " << boundary_recall << std::endl;
        std::cout << "3D Undersegmentation Error: " << undersegmentation_error << std::endl;
        std::cout << "3D Achievable Segmentation Accuracy: " << achievable_segmentation_accuracy << std::endl;
    }
    
    if (visualize) {
        IO::writeColoredSegmentationVideo(vis_dir / boost::filesystem::path("0"), 
                svVideo);
    }
    
    GraphSegmentationHierarchyMagic* hmagic = new GraphSegmentationHierarchyMagicThreshold(c, 1.3);
    GraphSegmentationHierarchyDistance* hdistance = 
            new GraphSegmentationHierarchyRGBChiSquareFlowAngle(alpha, beta);
    
    segmenter.setHierarchyMagic(hmagic);
    segmenter.setHierarchyDistance(hdistance);
    
    for (int l = 0; l < L; l++) {
        
        timer.restart();
        segmenter.buildRegionGraph();
        std::cout << "----- Level " << (l + 1) << std::endl;
        std::cout << "Built region graph ("
                << timer.elapsed() << ")." << std::endl;
        
        timer.restart();
        segmenter.addHierarchyLevel();
        std::cout << "Segmented region graph ("
                << timer.elapsed() << ")." << std::endl;
        
        timer.restart();
        segmenter.enforceMinimumSegmentSize(l/2 * M);
        std::cout << "Enforce minimum segment size ("
                << timer.elapsed() << ")." << std::endl;
        
        SegmentationVideo svVideo = segmenter.deriveLabels();
        IO::writeSegmentationVideo(out_dir / boost::filesystem::path(std::to_string(l + 1)), 
                svVideo);
        
        if (groundTruth) {
            float boundary_recall = Evaluation::compute3DBoundaryRecall(svVideo, gtVideo);
            float undersegmentation_error = Evaluation::compute3DNPUndersegmentationError(svVideo, gtVideo);
            float achievable_segmentation_accuracy = Evaluation::compute3DAchievableSegmentationAccuracy(svVideo, gtVideo);

            std::cout << "3D Boundary Recall: " << boundary_recall << std::endl;
            std::cout << "3D Undersegmentation Error: " << undersegmentation_error << std::endl;
            std::cout << "3D Achievable Segmentation Accuracy: " << achievable_segmentation_accuracy << std::endl;
        }
        
        if (visualize) {
            IO::writeColoredSegmentationVideo(vis_dir / boost::filesystem::path(std::to_string(l + 1)), 
                    svVideo);
        }
    }
    
    return 0;
}
