/**
 * Implementation of several metrics for evaluating superpixel and supervoxel
 * algorithms:
 * 
 *  - 3D Undersegmentation Error by Xu and Corso [1];
 *  - 3D Undersegmentation Error by Neubert and Protzel [2];
 *  - 3D Achievable Segmentation Accuracy [1];
 *  - and Boundary Recall and Boundary Precision (similar to [1]).
 * 
 *  [1] C. Xu and J. J. Corso.
 *      Evaluation of super-voxel methods for early video processing.
 *      In Computer Vision and Pattern Recognition, Conference on,
 *      pages 1202–1209, Providence, RI, June 2012.
 *  [2] P. Neubert and P. Protzel.
 *      Superpixel benchmark and comparison.
 *      In Forum Bildverarbeitung, Regensburg, Germany, November 2012.
 * 
 * Metrics can be used to evaluate the implementation of the hierarchical
 * graph-based video segmentation algorithm proposed by Grundmann et al. [3] and
 * implemented in video_graph.h and graph_segmentation.h:
 * 
 *  [3] M. Grundmann, V. Kwatra, M. Han, and I. A. Essa.
 *      Efficient hierarchical graph-based video segmentation.
 *      In Conference on Computer Vision and Pattern Recognition, pages 2141–2148,
 *      San Francisco, 2010, June 2010.
 * 
 * The algorithm is based on the graph-based image segmentation algorithm by
 * Felzenswalb and Huttenlocher [4]:
 * 
 *  [4] P. F. Felzenszwalb and D. P. Huttenlocher.
 *      Efficient graph-based image segmentation.
 *      International Journal of Computer Vision, 59(2):167–181, September 2004.
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

#include "evaluation.h"
#include "io_util.h"
#include <limits>

inline bool isSpatialBoundaryPixel(SegmentationVideo & video, int t, 
        int i, int j) {
    
    int H = video.getFrameHeight();
    int W = video.getFrameWidth();
    
    if (i < H - 1) {
        if (video.get<int>(t, i, j) != video.get<int>(t, i + 1, j)) {
            return true;
        }
    }
    
    if (j < W - 1) {
        if (video.get<int>(t, i, j) != video.get<int>(t, i, j + 1)) {
            return true;
        }
    }
    
    return false;
}

inline bool isTemporalBoundaryPixel(SegmentationVideo & video, int t, 
        int i, int j) {
    
    int T = video.getFrameNumber();
    
    if (t < T - 1) {
        if (video.get<int>(t, i, j) != video.get<int>(t + 1, i, j)) {
            return true;
        }
    }
    
    return false;
}

template<class V>
float Evaluation::compute3DBoundaryRecall(V & sv_video, 
            SegmentationVideo & gt_video, float d) {
    
    assert(sv_video.getFrameNumber() == gt_video.getFrameNumber());
    assert(sv_video.getFrameHeight() == gt_video.getFrameHeight());
    assert(sv_video.getFrameWidth() == gt_video.getFrameWidth());
    
    int T = gt_video.getFrameNumber();
    int H = gt_video.getFrameHeight();
    int W = gt_video.getFrameWidth();
    
    int r = std::round(d*std::sqrt(H*H + W*W));
    
    float tp = 0;
    float fn = 0;
    for (int t = 0; t < T; t++) {
        for (int i = 0; i < H; i++) {
            for (int j = 0; j < W; j++) {
                // Note that spatial and temporal boundaries are
                // processed separately
                if (isSpatialBoundaryPixel(gt_video, t, i, j)) {
                    
                    bool pos = false;
                    for (int k = std::max(0, i - r); k < std::min(H - 1, i + r) + 1; k++) {
                        for (int l = std::max(0, j - r); l < std::min(W - 1, j + r) + 1; l++) {
                            if (isSpatialBoundaryPixel(sv_video, t, k, l)) {
                                pos = true;
                            }
                        }
                    }
                    
                    if (pos) {
                        tp++;
                    }
                    else {
                        fn++;
                    }
                }
                
                // Temporal boundary.
                if (isTemporalBoundaryPixel(gt_video, t, i ,j)) {
                    
                    bool pos = false;
                    for (int k = std::max(0, i - r); k < std::min(H - 1, i + r) + 1; k++) {
                        for (int l = std::max(0, j - r); l < std::min(W - 1, j + r) + 1; l++) {
                            if (isTemporalBoundaryPixel(sv_video, t, k, l)) {
                                pos = true;
                            }
                        }
                    }
                    
                    if (pos) {
                        tp++;
                    }
                    else {
                        fn++;
                    }
                }
            }
        }
    }
    
    return tp/(tp + fn);
}

template float Evaluation::compute3DBoundaryRecall<SegmentationVideo>(SegmentationVideo&, 
        SegmentationVideo&, float);

template<class V>
float Evaluation::compute3DBoundaryPrecision(V & sv_video, 
            SegmentationVideo & gt_video, float d) {
    
    assert(sv_video.getFrameNumber() == gt_video.getFrameNumber());
    assert(sv_video.getFrameHeight() == gt_video.getFrameHeight());
    assert(sv_video.getFrameWidth() == gt_video.getFrameWidth());
    
    int T = gt_video.getFrameNumber();
    int H = gt_video.getFrameHeight();
    int W = gt_video.getFrameWidth();
    
    int r = std::round(d*std::sqrt(H*H + W*W));
    
    float tp = 0;
    float fp = 0;
    for (int t = 0; t < T; t++) {
        for (int i = 0; i < H; i++) {
            for (int j = 0; j < W; j++) {
                // Note that spatial and temporal boundaries are
                // processed separately
                if (isSpatialBoundaryPixel(gt_video, t, i, j)) {
                    
                    bool pos = false;
                    for (int k = std::max(0, i - r); k < std::min(H - 1, i + r) + 1; k++) {
                        for (int l = std::max(0, j - r); l < std::min(W - 1, j + r) + 1; l++) {
                            if (isSpatialBoundaryPixel(sv_video, t, k, l)) {
                                pos = true;
                            }
                        }
                    }
                    
                    if (pos) {
                        tp++;
                    }
                }
                else if (isSpatialBoundaryPixel(sv_video, t, i, j)) {
                    
                    bool pos = false;
                    for (int k = std::max(0, i - r); k < std::min(H - 1, i + r) + 1; k++) {
                        for (int l = std::max(0, j - r); l < std::min(W - 1, j + r) + 1; l++) {
                            if (isSpatialBoundaryPixel(gt_video, t, k, l)) {
                                pos = true;
                            }
                        }
                    }
                    
                    if (!pos) {
                        fp++;
                    }
                }
                
                // Temporal boundary.
                if (isTemporalBoundaryPixel(gt_video, t, i ,j)) {
                    
                    bool pos = false;
                    for (int k = std::max(0, i - r); k < std::min(H - 1, i + r) + 1; k++) {
                        for (int l = std::max(0, j - r); l < std::min(W - 1, j + r) + 1; l++) {
                            if (isTemporalBoundaryPixel(sv_video, t, k, l)) {
                                pos = true;
                            }
                        }
                    }
                    
                    if (pos) {
                        tp++;
                    }
                }
                else if (isTemporalBoundaryPixel(sv_video, t, i, j)) {
                    
                    bool pos = false;
                    for (int k = std::max(0, i - r); k < std::min(H - 1, i + r) + 1; k++) {
                        for (int l = std::max(0, j - r); l < std::min(W - 1, j + r) + 1; l++) {
                            if (isTemporalBoundaryPixel(gt_video, t, k, l)) {
                                pos = true;
                            }
                        }
                    }
                    
                    if (!pos) {
                        fp++;
                    }
                }
            }
        }
    }
    
    if (tp + fp > 0) {
        return tp/(tp + fp);
    }
    
    return 0;
}

template float Evaluation::compute3DBoundaryPrecision<SegmentationVideo>(SegmentationVideo&, 
        SegmentationVideo&, float);

template<class V>
std::vector<float> Evaluation::compute2DBoundaryRecallPerFrame(V & sv_video, 
            SegmentationVideo & gt_video, float d) {
    
    assert(sv_video.getFrameNumber() == gt_video.getFrameNumber());
    assert(sv_video.getFrameHeight() == gt_video.getFrameHeight());
    assert(sv_video.getFrameWidth() == gt_video.getFrameWidth());
    
    int T = gt_video.getFrameNumber();
    int H = gt_video.getFrameHeight();
    int W = gt_video.getFrameWidth();
    
    int r = std::round(d*std::sqrt(H*H + W*W));
    
    std::vector<float> recall(T, 0);
    for (int t = 0; t < T; t++) {
        
        float tp = 0;
        float fn = 0;
        
        for (int i = 0; i < H; i++) {
            for (int j = 0; j < W; j++) {
                if (isSpatialBoundaryPixel(gt_video, t, i, j)) {
                    
                    bool pos = false;
                    // Search for boundary pixel in the supervoxel segmentation.
                    for (int k = std::max(0, i - r); k < std::min(H - 1, i + r) + 1; k++) {
                        for (int l = std::max(0, j - r); l < std::min(W - 1, j + r) + 1; l++) {
                            if (isSpatialBoundaryPixel(sv_video, t, k, l)) {
                                pos = true;
                            }
                        }
                    }
                    
                    if (pos) {
                        tp++;
                    }
                    else {
                        fn++;
                    }
                }
            }
        }
        
        recall[t] = tp/(tp + fn);
    }
    
    return recall;
}

template std::vector<float> Evaluation::compute2DBoundaryRecallPerFrame<SegmentationVideo>(SegmentationVideo&, 
        SegmentationVideo&, float);

template<class V>
std::vector<float> Evaluation::compute2DBoundaryPrecisionPerFrame(V & sv_video, 
            SegmentationVideo & gt_video, float d) {
    
    assert(sv_video.getFrameNumber() == gt_video.getFrameNumber());
    assert(sv_video.getFrameHeight() == gt_video.getFrameHeight());
    assert(sv_video.getFrameWidth() == gt_video.getFrameWidth());
    
    int T = gt_video.getFrameNumber();
    int H = gt_video.getFrameHeight();
    int W = gt_video.getFrameWidth();
    
    int r = std::round(d*std::sqrt(H*H + W*W));
    
    std::vector<float> precision(T, 0);
    for (int t = 0; t < T; t++) {
        
        float tp = 0;
        float fp = 0;
        
        for (int i = 0; i < H; i++) {
            for (int j = 0; j < W; j++) {
                if (isSpatialBoundaryPixel(gt_video, t, i, j)) {
                    
                    bool pos = false;
                    // Search for boundary pixel in the supervoxel segmentation.
                    for (int k = std::max(0, i - r); k < std::min(H - 1, i + r) + 1; k++) {
                        for (int l = std::max(0, j - r); l < std::min(W - 1, j + r) + 1; l++) {
                            if (isSpatialBoundaryPixel(sv_video, t, k, l)) {
                                pos = true;
                            }
                        }
                    }
                    
                    if (pos) {
                        tp++;
                    }
                }
                else if (isSpatialBoundaryPixel(sv_video, t, i, j)) {
                    bool pos = false;
                    // Search for boundary pixel in the supervoxel segmentation.
                    for (int k = std::max(0, i - r); k < std::min(H - 1, i + r) + 1; k++) {
                        for (int l = std::max(0, j - r); l < std::min(W - 1, j + r) + 1; l++) {
                            if (isSpatialBoundaryPixel(gt_video, t, k, l)) {
                                pos = true;
                            }
                        }
                    }
                    
                    if (!pos) {
                        fp++;
                    }
                }
            }
        }
        
        precision[t] = 0;
        if (tp + fp > 0) {
            precision[t] = tp/(tp + fp);
        }
    }
    
    return precision;
}

template std::vector<float> Evaluation::compute2DBoundaryPrecisionPerFrame<SegmentationVideo>(SegmentationVideo&, 
        SegmentationVideo&, float);

float Evaluation::compute3DXCUndersegmentationError(SegmentationVideo & sv_video, 
            SegmentationVideo & gt_video) {
    
    cv::Mat intersectionMatrix = Evaluation::computeIntersectionMatrix(sv_video, 
            gt_video);
    
    float error = 0;
    
    for (int i = 0; i < intersectionMatrix.rows; ++i) {
        if (gt_video.getSegmentSize(i) > 0) {
            float gtIError = 0;
            for (int j = 0; j < intersectionMatrix.cols; ++j) {

                // Add error if segments overlap:
                if (intersectionMatrix.at<int>(i, j) > 0) {
                    gtIError += sv_video.getSegmentSize(j);
                }
            }

            gtIError -= gt_video.getSegmentSize(i);
            gtIError /= gt_video.getSegmentSize(i);
            
            error += gtIError;
        }
    }
    
    return error/gt_video.getSegmentNumber();
}

float Evaluation::compute3DNPUndersegmentationError(SegmentationVideo& sv_video, 
        SegmentationVideo& gt_video) {
    
    cv::Mat intersectionMatrix = Evaluation::computeIntersectionMatrix(sv_video, gt_video);
    
    float N = sv_video.getFrameHeight()*sv_video.getFrameWidth()*sv_video.getFrameNumber();
    float error = 0;
    
    for (int i = 0; i < intersectionMatrix.rows; ++i) {
        for (int j = 0; j < intersectionMatrix.cols; ++j) {
            if (intersectionMatrix.at<int>(i, j) > 0) {
                error += std::min(intersectionMatrix.at<int>(i, j), 
                        sv_video.getSegmentSize(j) - intersectionMatrix.at<int>(i, j));
            }
        }
    }
    
    return error/N;
}

std::vector<float> Evaluation::compute2DNPUndersegmentationErrorPerFrame(SegmentationVideo& sv_video, 
        SegmentationVideo& gt_video) {
    
    assert(sv_video.getFrameNumber() == gt_video.getFrameNumber());
    
    int T = sv_video.getFrameNumber();
    int N = sv_video.getFrameHeight()*sv_video.getFrameWidth();
    
    std::vector<float> errors(T, 0);
    
    for (int t = 0; t < T; ++t) {
        
        std::vector<int> superpixelSizes;
        std::vector<int> gtSizes;
        
        cv::Mat intersectionMatrix = Evaluation::computeIntersectionMatrix(sv_video.getFrame(t), 
                gt_video.getFrame(t), superpixelSizes, gtSizes);
        
        for (int i = 0; i < intersectionMatrix.rows; ++i) {
            for (int j = 0; j < intersectionMatrix.cols; ++j) {
                if (intersectionMatrix.at<int>(i, j) > 0) {
                    errors[t] += std::min(intersectionMatrix.at<int>(i, j), 
                            superpixelSizes[j] - intersectionMatrix.at<int>(i, j));
                }
            }
        }
        
        // Normalize by number of pixels.
        errors[t] /= N;
    }
    
    return errors;
}

float Evaluation::compute3DAchievableSegmentationAccuracy(SegmentationVideo& sv_video, 
        SegmentationVideo& gt_video) {

    cv::Mat intersectionMatrix = Evaluation::computeIntersectionMatrix(sv_video, 
            gt_video);
    
    float N = sv_video.getFrameNumber()*sv_video.getFrameHeight()*sv_video.getFrameWidth();
    float accuracy = 0;
    
    for (int j = 0; j < intersectionMatrix.cols; ++j) {
        
        int max = 0;
        for (int i = 0; i < intersectionMatrix.rows; ++i) {
            if (intersectionMatrix.at<int>(i, j) > max) {
                max = intersectionMatrix.at<int>(i, j);
            }
        }
        
        accuracy += max;
    }
    
    return accuracy/N;
}

std::vector<float> Evaluation::compute2DAchievableSegmentationAccuracyPerFrame(SegmentationVideo& sv_video, 
        SegmentationVideo& gt_video) {
    
    assert(sv_video.getFrameNumber() == gt_video.getFrameNumber());
    
    int T = sv_video.getFrameNumber();
    int N = sv_video.getFrameHeight()*sv_video.getFrameWidth();
    
    std::vector<float> accuracies(T, 0);
    
    for (int t = 0; t < T; ++t) {
        
        std::vector<int> superpixelSizes;
        std::vector<int> gtSizes;
        
        cv::Mat intersectionMatrix = Evaluation::computeIntersectionMatrix(sv_video.getFrame(t),
                gt_video.getFrame(t), superpixelSizes, gtSizes);
        
        for (int j = 0; j < intersectionMatrix.cols; ++j) {
            
            int max = 0;
            for (int i = 0; i < intersectionMatrix.rows; ++i) {
                if (intersectionMatrix.at<int>(i, j) > max) {
                    max = intersectionMatrix.at<int>(i, j);
                }
            }
            
            accuracies[t] += max;
        }
        
        accuracies[t] /= N;
    }
    
    return accuracies;
}


cv::Mat Evaluation::computeIntersectionMatrix(const cv::Mat & sp_image, 
        const cv::Mat & gt_image, std::vector<int> & superpixelSizes, 
        std::vector<int> & gtSizes) {
    
    assert(sp_image.rows == gt_image.rows);
    assert(sp_image.cols == gt_image.cols);
    assert(sp_image.channels() == 1 && gt_image.channels() == 1);
    assert(sp_image.type() == CV_32S && gt_image.type() == CV_32S);
    
    int rows = sp_image.rows;
    int cols = sp_image.cols;
    
    int superpixels = IOUtil::computeSegmentNumber(sp_image);
    int gtSegments = IOUtil::computeSegmentNumber(gt_image);
    
    assert(superpixels > 0);
    assert(gtSegments > 0);
    
    superpixelSizes.resize(superpixels);
    std::fill(superpixelSizes.begin(), superpixelSizes.end(), 0);
    
    gtSizes.resize(gtSegments);
    std::fill(gtSizes.begin(), gtSizes.end(), 0);
    
    cv::Mat intersectionMatrix(gtSegments, superpixels, CV_32SC1, cv::Scalar(0));
    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            intersectionMatrix.at<int>(gt_image.at<int>(i, j), sp_image.at<int>(i, j))++;
            
            // Count the number of pixels for all segments:
            superpixelSizes[sp_image.at<int>(i, j)]++;
            gtSizes[gt_image.at<int>(i, j)]++;
        }
    }
    
    return intersectionMatrix;
}

cv::Mat Evaluation::computeIntersectionMatrix(SegmentationVideo & sv_video, 
        SegmentationVideo & gt_video) {
    
    assert(sv_video.getFrameNumber() == gt_video.getFrameNumber());
    assert(sv_video.getFrameWidth() == gt_video.getFrameWidth());
    assert(sv_video.getFrameHeight() == gt_video.getFrameHeight());
    assert(sv_video.getFrameChannels() == gt_video.getFrameChannels());
    
    int rows = sv_video.getFrameHeight();
    int cols = sv_video.getFrameWidth();
    
    int supervoxels = sv_video.getSegmentNumber();
    int gtSegments = gt_video.getSegmentNumber();
    
    cv::Mat intersectionMatrix(gtSegments, supervoxels, CV_32SC1, cv::Scalar(0));
    for (int t = 0; t < sv_video.getFrameNumber(); ++t) {
        for (int i = 0; i < rows; ++i) {
            for (int j = 0; j < cols; ++j) {
                intersectionMatrix.at<int>(gt_video.get<int>(t, i, j), sv_video.get<int>(t, i, j))++;
            }
        }
    }
    
    return intersectionMatrix;
}