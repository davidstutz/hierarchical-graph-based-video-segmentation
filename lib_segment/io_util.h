/**
 * Input/Output and Utilities for reading/writing videos and saving video
 * segmentations to images.
 * 
 * See graph_segmentation.h for details on the video segmentation algorithm
 * and evaluation.h for details on evaluation metrics.
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

#ifndef IO_UTIL_H
#define	IO_UTIL_H

#include <vector>
#include <map> 
#include <string>
#include <boost/filesystem.hpp>
#include <opencv2/opencv.hpp>

class IOUtil {
public:
    /**
     * Creates an ordered list of all subdirectories.
     * 
     * @param directory
     * @return 
     */
    static std::multimap<std::string, boost::filesystem::path> listSubdirectories(boost::filesystem::path directory);
    /**
     * Creates an ordered multimap of all files in the given directory.
     * 
     * Files must match one of the given extensions.
     * 
     * @param directory
     * @param extensions
     * @return 
     */
    static std::multimap<std::string, boost::filesystem::path> readDirectory(boost::filesystem::path directory, 
        std::vector<std::string> extensions);
    /**
     * Creates an ordered multimap of all files in the given directory.
     * 
     * Can easily be used to read videos as the individual frames are usually
     * named according to the frame number.
     * 
     * @param directory
     * @return 
     */
    static std::multimap<std::string, boost::filesystem::path> readDirectory(boost::filesystem::path directory);
    /**
     * Compute the number of segments of a superpixel segmentation.
     * 
     * Note that the result may be different than the number of supervoxels
     * of the video the image comes from.
     * 
     * @param sp_image
     * @return
     */
    static int computeSegmentNumber(const cv::Mat & sp_image);
    /**
     * Creates a label map given a segmentation image.
     * 
     * The image is assumed to store the labels as 24bit int distributed
     * over all threee color channles:
     *  label = B + 256*G + 256*256*R
     * 
     * @param image
     * @return 
     */
    static cv::Mat segmentationImageToLabelMap(const cv::Mat& image);
    /**
     * Creates a label image from the label map. Colors are assigned as follows:
     *  B = floor(label % 256)
     *  G = floor((label % 256*256)/256)
     *  R = floor(label/(256*256))
     * 
     * @param labels
     * @return
     */
    static cv::Mat labelMapToSegmentationImage(const cv::Mat& labels);
    /**
     * Gets a vector containing common extensions for images
     * 
     * @return 
     */
    static std::vector<std::string> getImageExtensions();
    /**
     * Gets a vector containing common extensions for text files.
     * 
     * @return 
     */
    static std::vector<std::string> getTxtExtensions();
    
};

#endif	/* IO_UTIL_H */

