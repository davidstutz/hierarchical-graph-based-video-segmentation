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

#ifndef EVALUATION_H
#define	EVALUATION_H

#include "io.h"

/**
 * Provides measures to evaluate (over-) segmentations of a video against a
 * given ground truth segmentation.
 */
class Evaluation {
public:
    /**
     * Compute 3D boundary recall (see 2D description). This includes
     * temporal boundaries.
     * 
     * @param sv_video
     * @param gt_video
     * @param d
     */
    template<class V>
    static float compute3DBoundaryRecall(V & sv_video, 
            SegmentationVideo & gt_video, float d = 0.0025);
    /**
     * Compute 3D boundary precision (see 2D description). This includes
     * temporal boundaries.
     * 
     * @param sv_video
     * @param gt_video
     * @param d
     */
    template<class V>
    static float compute3DBoundaryPrecision(V & sv_video, 
            SegmentationVideo & gt_video, float d = 0.0025);
    /**
     * Compute boundary recall (for each frame):
     * 
     *  $BR(S, G) = \frac{TP}{TP + FN}$
     * 
     * where TP is the number of true positives, and FN is the number of 
     * false negatives.
     * 
     * For each boundary pixel in the G, the corresponding boundary
     * pixel in S is allowed to deviate by a euclidean distance of d times the
     * image diagonal.
     * 
     * @param sv_video
     * @param gt_video
     * @param d
     */
    template<class V>
    static std::vector<float> compute2DBoundaryRecallPerFrame(V & sv_video, 
            SegmentationVideo & gt_video, float d = 0.0025);
    /**
     * Compute boundary precision (for each frame):
     * 
     *  $BR(S, G) = \frac{TP}{TP + FP}$
     * 
     * where TP is the number of true positives, and FP is the number of false
     * positives.
     * 
     * For each boundary pixel in the G, the corresponding boundary
     * pixel in S is allowed to deviate by a euclidean distance of d times the
     * image diagonal.
     * 
     * @param sv_video
     * @param gt_video
     * @param d
     */
    template<class V>
    static std::vector<float> compute2DBoundaryPrecisionPerFrame(V & sv_video, 
            SegmentationVideo & gt_video, float d = 0.0025);
    /**
     * 3D Undersegmentation Error as proposed by Xu and Corso in [1]. This is a
     * generalization of the 2D Undersegmentation Error as used by Levinshtein
     * et al. [2] and Lui et al. [3]. The 3D Undersegmentation Error is given as:
     * 
     *  $3DUE(S, G) = \frac{1}{|G|} \sum_{G_i \in G} 
     *          \frac{\left(\sum_{S_j \cap G_i \neq \emptyset} |S_j| \right) - |G_i|}{|G_i|}
     * 
     *  [1] C. Xu, J. J. Corso. 
     *      Evaluation of Super-Voxel Methods for Early Video Processing.
     *      Conference on Computer Vision and Pattern Recognition, 2012.
     *  [2] A. Levinshtein, A. Stere, K. N. Kutulakos. D.J. Fleet, S. J. Dickinson.
     *      TurboPixels: Fast Superpixels Using Geometric Flows.
     *      Conference on Robotics and Automation.
     *  [3] M. Y. Lui, O. Tuzel, S. Ramalingam, R. Chellappa.
     *      Entropy Rate Superpixel Segmentation.
     *      Conference on Computer Vision and Pattern Recognition, 2011.
     * 
     * @param sv_video
     * @param gt_video
     * @return
     */
    static float compute3DXCUndersegmentationError(SegmentationVideo & sv_video, 
            SegmentationVideo & gt_video);
    /**
     * Computes the 3D undersegmentation error (NP) as follows:
     * 
     *  $3DUE(S, G) = \frac{1}{N} \sum_{G_i \in G} \sum_{S_j \cap G_i \neq \emptyset}
     *          \min \{|S_j \cap G_i|, |S_j - G_i|\}$
     * 
     * This is a generalization of the 2D undersemgentation error as proposed by
     * Neubert and Protzel [1]:
     * 
     *  [1] P. Neubert, P. Protzel.
     *      Superpixel Benchmark and COmparison.
     *      Forum Bildverarbeitung,2012.
     * 
     * @param sv_video
     * @param gt_video
     * @return 
     */
    static float compute3DNPUndersegmentationError(SegmentationVideo & sv_video, 
            SegmentationVideo & gt_video);
    /**
     * Computes the 2D undersegmentation error (NP) per frame with the same formula
     * as given above.
     * 
     * @param sv_video
     * @param gt_video
     * @return 
     */
    static std::vector<float> compute2DNPUndersegmentationErrorPerFrame(SegmentationVideo & sv_video, 
            SegmentationVideo & gt_video);
    /**
     * Compute 3D achievable segmentation accuracy as follows:
     * 
     *  $3DASA(G, S) = \frac{1}{N} \sum_{S_j \in S} \max_{G_i} \{|S_j \cap G_i|\}$
     * 
     * @param sv_video
     * @param gt_video
     * @return 
     */
    static float compute3DAchievableSegmentationAccuracy(SegmentationVideo & sv_video, 
            SegmentationVideo & gt_video);
    /**
     * Compute a temporal adaptation of the 2D achievable segmentation
     * accuracy as follows:
     * 
     *  $TASA(G, S, t) = \frac{1}{N_t} \sum_{S_j \in S} |S_j \cap G_{g(j)}|$
     * 
     * where $g(j)$ is the ground truth segment with maximum overlap with $S_j$
     * in a specific frame. That is, we assign each supervoxel to a ground trut
     * segment based on their overlap on the image plane rather than in the
     * video volume.
     * 
     * @param sv_video
     * @param gt_video
     * @return 
     */
    static std::vector<float> computeTemporal2DAchievableSegmentationAccuracyPerFrame(SegmentationVideo & sv_video, 
            SegmentationVideo & gt_video);
    /**
     * Calculate the 2D achievable segmentation accuracy as formulated for 3D
     * in compute3DAchievableSegmentationAccuracy.
     * 
     * @param sv_video
     * @param gt_video
     * @return 
     */
    static std::vector<float> compute2DAchievableSegmentationAccuracyPerFrame(SegmentationVideo & sv_video, 
            SegmentationVideo & gt_video);
    
private:
    /**
     * Compute the intersection matrix for a superpixel and a groudn truth
     * segmentation. Element $(i, j)$ contains the number of pixels in the
     * intersection of $G_i$ and $S_j$.
     * 
     * @param sp_image
     * @param gt_image
     * @return
     */
    static cv::Mat computeIntersectionMatrix(const cv::Mat & sp_image, 
            const cv::Mat & gt_image, std::vector<int> & superpixelSizes, 
            std::vector<int> & gtSizes);
    /**
     * Compute the intersection matrix for a supervoxel and a ground truth
     * video. Element $(i, j)$ contains the number of pixels in the intersection
     * of $G_i$ and $S_j$.
     * 
     * @param sv_video
     * @param gt_video
     * @return 
     */
    static cv::Mat computeIntersectionMatrix(SegmentationVideo & sv_video, 
            SegmentationVideo & gt_video);
};

#endif	/* EVALUATION_H */

