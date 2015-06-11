/**
 * Example for dense optical flow using OpenCV's dense optical flow implementation
 * based on Farneback's algorithms. Can for example be tested on the Middlebury
 * dataset [1] available at http://vision.middlebury.edu/flow/data/.
 * 
 *  [1] S. Baker, D. Scharstein, J. P. Lewis, S. Roth, M. J. Black, R. Szeliski.
 *      A Databse and Evaluation Methodology for Optical Flow.
 *      International Journal of Computer VIsion, 2011.
 * 
 * Usage:
 * $ ./optical_flow_cli/optical_flow_cli --help
 * Allowed options:
 *   --help                                produce help message
 *   --input-dir arg                       input directory
 *   --pyramid-scale arg (=0.5)            pyramid scale
 *   --pyramid-levels arg (=1)             pyramid levels
 *   --window-size arg (=7)                window size
 *   --num-iterations arg (=10)            number of iterations at each pyramid 
 *                                         level
 *   --polynomial-neighborhood-size arg (=5)
 *                                         size of pixel neighborhood to find 
 *                                         polynomial expansion
 *   --polynomial-sigma arg (=1.10000002)  standard deviation of Gaussian used to 
 *                                         smooth derivatives
 *   --gaussian-filter                     use a Gaussian filter isntead of a box 
 *                                         filter
 *   --output-dir arg (=output)            output directory
 *
 * Copyright (c) 2015, David Stutz
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 
 * - Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 * - Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 * - Neither the name of David Stutz nor the names of its contributors may be
 * used to endorse or promote products derived from this software without specific
 * prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#include <cstdlib>
#include <vector>
#include <opencv2/opencv.hpp>
#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include "io.h"

int main(int argc, char** argv) {

    boost::program_options::options_description desc("Allowed options");
    desc.add_options()
        ("help", "produce help message")
        ("input-dir",  boost::program_options::value<std::string>(), "input directory")
        ("pyramid-scale", boost::program_options::value<float>()->default_value(0.5), "pyramid scale")
        ("pyramid-levels", boost::program_options::value<int>()->default_value(1), "pyramid levels")
        ("window-size", boost::program_options::value<int>()->default_value(7), "window size")
        ("num-iterations", boost::program_options::value<int>()->default_value(10), "number of iterations at each pyramid level")
        ("polynomial-neighborhood-size", boost::program_options::value<int>()->default_value(5), "size of pixel neighborhood to find polynomial expansion")
        ("polynomial-sigma", boost::program_options::value<float>()->default_value(1.1), "standard deviation of Gaussian used to smooth derivatives")
        ("gaussian-filter", "use a Gaussian filter isntead of a box filter")
        ("output-dir", boost::program_options::value<std::string>()->default_value("output"), "output directory");

    boost::program_options::positional_options_description positionals;
    positionals.add("input-dir", 1);
    
    boost::program_options::variables_map parameters;
    boost::program_options::store(boost::program_options::command_line_parser(argc, argv).options(desc).positional(positionals).run(), parameters);
    boost::program_options::notify(parameters);

    if (parameters.find("help") != parameters.end()) {
        std::cout << desc << std::endl;
        return 1;
    }
    
    boost::filesystem::path in_dir(parameters["input-dir"].as<std::string>());
    if (!boost::filesystem::is_directory(in_dir)) {
        std::cout << "Input directory does not exist." << std::endl;
        return 1;
    }
    
    boost::filesystem::path out_dir(parameters["output-dir"].as<std::string>());
    if (!boost::filesystem::is_directory(out_dir)) {
        boost::filesystem::create_directory(out_dir);
    }
    
    std::multimap<std::string, boost::filesystem::path> images;
    boost::filesystem::directory_iterator end;
    
    for (boost::filesystem::directory_iterator it(in_dir); it != end; ++it) {
        if (boost::filesystem::is_regular_file(it->path())) {
            images.insert(std::multimap<std::string, 
                    boost::filesystem::path>::value_type(it->path().string(), it->path()));
        }
    }
    
    int t = 0;
    cv::Mat frame;
    cv::Mat prevFrame;
    cv::Mat opticalFlow;
    
    // Optical flow parameters.
    float pyramidScale = parameters["pyramid-scale"].as<float>();
    int pyramidLevels = parameters["pyramid-levels"].as<int>();
    int windowSize = parameters["window-size"].as<int>();
    int iterations = parameters["num-iterations"].as<int>();
    int polynomialNeighborhoodSize = parameters["polynomial-neighborhood-size"].as<int>();
    float polynomialSigma = parameters["polynomial-sigma"].as<float>();
    
    int flags = 0;
    if (parameters.find("gaussian-filter") != parameters.end()) {
        flags = cv::OPTFLOW_FARNEBACK_GAUSSIAN;
    }
    
    for (std::multimap<std::string, boost::filesystem::path>::iterator it = images.begin(); 
            it != images.end(); ++it) {
        frame = cv::imread(it->second.string());
        cv::cvtColor(frame, frame, CV_BGR2GRAY);
        
        if (t > 0) {
            cv::calcOpticalFlowFarneback(prevFrame, frame, opticalFlow, pyramidScale, 
                    pyramidLevels, windowSize, iterations, polynomialNeighborhoodSize, 
                    polynomialSigma, flags);
            
            IO::writeMat(out_dir.string() + "/" + std::to_string(t) + ".txt", opticalFlow);
        }
        
        prevFrame = frame;
        t++;
    }
    
    return 0;
}
