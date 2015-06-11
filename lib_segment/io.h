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

#ifndef IO_H
#define	IO_H

#include <vector>
#include <opencv2/opencv.hpp>
#include <string>
#include <boost/filesystem.hpp>

#ifndef DIRECTORY_SEPARATOR
    #if defined(WIN32) || defined(_WIN32)
        #define DIRECTORY_SEPARATOR "\\"
    #else
        #define DIRECTORY_SEPARATOR "/"
    #endif
#endif

/**
 * A video consists of several frames which are given as cv::Mat. This class is
 * the basic container for a video providing basic reading/writing capabilities.
 */
class Video {
public:
    /**
     * Constructor.
     */
    Video();
    /**
     * Destructor.
     */
    virtual ~Video();
    /**
     * Add a single frame in the form of a cv::Mat; the container will
     * create a copy.
     * 
     * If this is the first frame, the frame widths, height and type are saved
     * for later comparison.
     * 
     * Frame's height, width and type must match the first frame.
     * 
     * Cannot add empty frames.
     * 
     * @param frame
     * @return 
     */
    int addFrame(const cv::Mat& frame);
    /**
     * Retrieve the frame width; zero if no frames are contained in this video.
     * 
     * @return 
     */
    int getFrameWidth();
    /**
     * Retrieve frame height; zero if no frames are contained in this video.
     * @return 
     */
    int getFrameHeight();
    /**
     * Read frame type (in theform of a OpenCV type, e.g. CV_8UC3); zero if
     * no frames are contained in this video.
     * 
     * @return 
     */
    int getFrameType();
    /**
     * Retrieve the number of chnalles of each frame.
     * 
     * @return 
     */
    int getFrameChannels();
    /**
     * Get the number of frames in this video.
     * 
     * @return 
     */
    int getFrameNumber();
    /**
     * Retrieve a frame at time t.
     * 
     * @param t
     * @return 
     */
    cv::Mat & getFrame(int t);
    /**
     * Get a single element in the video.
     * 
     * @param t
     * @param i
     * @param j
     * @return 
     */
    template<typename T>
    T & get(int t, int i, int j);
    
protected:
    /**
     * Ordered frames.
     */
    std::vector<cv::Mat> frames;
    /**
     * Frame width.
     */
    int frameWidth;
    /**
     * Frame height.
     */
    int frameHeight;
    /**
     * Frame channels.
     */
    int frameChannels;
    /**
     * Frame type (in terms of OpenCV types).
     */
    int frameType;
    
};

/**
 * A segmentation video is represented by color images where the
 * label is encoded in three channels using 2bit, see io_util.h for details
 * on the encoding.
 */
class SegmentationVideo : public Video {
public:
    /**
     * Constructor.
     */
    SegmentationVideo();
    /**
     * Destructor.
     */
    virtual ~SegmentationVideo();
    /**
     * Get the number of segments.
     * 
     * @return
     */
    int getSegmentNumber();
    /**
     * Get the size of the given segment.
     * 
     * @return 
     */
    int getSegmentSize(int label);
    /**
     * Relabels the segmentation video to contain consecutive segments.
     */
    void relabel();
    /**
     * Relabels the video in place to generate consecutive labels.
     * 
     * @return
     */
    void relabelSemantic(std::multimap<int, int>& mapping);
    /**
     * Add to segmentation videos by overlaying the segments.
     * 
     * @param videoA
     * @param videoB
     * @return 
     */
    static SegmentationVideo add(SegmentationVideo& videoA, 
            SegmentationVideo& videoB, bool relabel = true);
    
protected:
    /**
     * The segment (supervoxel) sizes are cached. And only computed once here.
     */
    void initializeSegments();
    /**
     * Cached segment sizes.
     */
    std::vector<int> segments;
    
};

/**
 * Allowing to distinguish regular (1 or 3 channel videos) from flow videos.
 */
class FlowVideo : public Video {
public:
    /**
     * Constructor.
     */
    FlowVideo();
    /**
     * Destructor.
     */
    virtual ~FlowVideo();
    
};

/**
 * Simple Video and File IO class for reading and writing videos from 
 * directories, reading and writing matrices using cv::FileStorage.
 */
class IO {
public:
    /**
     * Read a video from the given directory.
     * 
     * Will read all images in this directory in alphabetical order.
     * 
     * @param directory
     * @param T
     * @return 
     */
    static Video readVideo(boost::filesystem::path directory, int T = 0);
    /**
     * Read a video from the given directory.
     * 
     * The frames are stored using the OpenCV FileStorage.
     * 
     * @param directory
     * @param T
     * @return 
     */
    static Video readVideoMat(boost::filesystem::path directory, int T = 0);
    /**
     * Write a video to the given directory.
     * 
     * If the directory does not exist it will be created. The images will be
     * saved according to their temporal order.
     * 
     * @param directory
     * @param video
     * @return 
     */
    static int writeVideo(boost::filesystem::path directory, Video & video);
    /**
     * Write a video to text files where the frames are stored using
     * OpenCV FileStorage.
     * 
     * @param directory
     * @param video
     * @return 
     */
    static int writeVideoMat(boost::filesystem::path directory, Video & video);
    /**
     * Read a segmentation of a video, that is a video where the frames
     * represent segmentations.
     * 
     * @param directory
     * @param T
     * @return 
     */
    static SegmentationVideo readSegmentationVideo(boost::filesystem::path directory, int T = 0);
    /**
     * Read a flow video where the given directory contains text files where
     * the actual flow is stored using OpenCV's FileStorage.
     * 
     * @param directory
     * @param T
     * @return 
     */
    static FlowVideo readFlowVideo(boost::filesystem::path directory, int T = 0);
    /**
     * Write a segmentation to a given directory.
     * 
     * @param directory
     * @return 
     */
    static int writeSegmentationVideo(boost::filesystem::path directory, 
            SegmentationVideo & video);
    /**
     * Write the segmentation video using random colors for all segments.
     * 
     * @param directory
     * @param video
     * @return 
     */
    static int writeColoredSegmentationVideo(boost::filesystem::path directory, 
            SegmentationVideo & video);
    /**
     * Read a matrix in the given file.
     * 
     * @param file
     * @param field
     * @return 
     */
    static cv::Mat readMat(boost::filesystem::path file, std::string field = "mat");
    /**
     * Write a matrix to the given file.
     * 
     * Will overwrite the file if it already exists.
     * 
     * @param file
     * @param mat
     * @param field
     * @return 
     */
    static int writeMat(boost::filesystem::path file, const cv::Mat& mat, 
            std::string field = "mat");
    /**
     * Write the content of a cv::Mat to a CSV file.
     * 
     * Only works for cv::Mats with one channel!
     * 
     * @param file
     * @param mat
     * @param separator
     * @param precision
     * @return
     */
    template<typename T>
    static int writeMatCSV(boost::filesystem::path file, const cv::Mat& mat, 
            std::string separator = ",", int precision = 6);
    
};

#endif	/* IO_H */

