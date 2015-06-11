/**
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

#ifndef GRAPH_SEGMENTATION_H
#define	GRAPH_SEGMENTATION_H

#include <opencv2/opencv.hpp>
#include "video_graph.h"
#include "io.h"

#define RAND() ((float) std::rand() / (RAND_MAX))

/**
 * All distances used for the initial oversegmentation according to the
 * approach by Felzenszwalb and Huttenlocher.
 * 
 * Essentially, derived classes only need to overwrite the () operator.
 */
class GraphSegmentationDistance {
public:
    /**
     * Constructs a distance without weights.
     */
    GraphSegmentationDistance() {};
    /**
     * Destructor.
     */
    virtual ~GraphSegmentationDistance() {};
    /**
     * Assignment operator.
     * 
     * @param distance
     * @return 
     */
    virtual void operator=(const GraphSegmentationDistance & distance) {
        weights = distance.weights;
    }
    /**
     * Evaluate the distance given 2 nodes.
     * 
     * @param n
     * @param m
     */ 
    virtual float operator()(const VideoNode & n, const VideoNode & m) = 0;
    
protected:
    /**
     * Weights to use.
     */
    std::vector<float> weights;
    
};

/**
 * Manhatten (L1) RGB distance.
 * 
 * @return 
 */
class GraphSegmentationManhattenRGB : public GraphSegmentationDistance {
public:
    /**
     * Constructor.
     */
    GraphSegmentationManhattenRGB() {
        // Normalization.
        D = 255 + 255 + 255;
    }
    /**
     * Evaluate the distance given 2 nodes.
     * 
     * @param n
     * @param m
     */ 
    virtual float operator()(const VideoNode & n, const VideoNode & m) {
        float dr = std::abs(n.r - m.r);
        float dg = std::abs(n.g - m.g);
        float db = std::abs(n.b - m.b);
        
        return (dr + dg + db)/D;
    }
    
private:
    /**
     * Normalization term.
     */
    float D;
    
};

/**
 * Euclidean RGB distance.
 */
class GraphSegmentationEuclideanRGB : public GraphSegmentationDistance {
public:
    /**
     * Constructor.
     */
    GraphSegmentationEuclideanRGB() {
        // Normalization.
        D = std::sqrt(255*255 + 255*255 + 255*255);
    }
    /**
     * Evaluate the distance given 2 nodes.
     * 
     * @param n
     * @param m
     */ 
    virtual float operator()(const VideoNode & n, const VideoNode & m) {
        float dr = n.r - m.r;
        float dg = n.g - m.g;
        float db = n.b - m.b;
        
        return std::sqrt(dr*dr + dg*dg + db*db)/D;
    }
    
private:
    /**
     * Normalization term.
     */
    float D;
    
};

/**
 * Euclidean RGB and true RGB distance plus flow angle. The flow angle is only
 * used within frames.
 */
class GraphSegmentationEuclideanRGBFlowAngle : public GraphSegmentationDistance {
public:
    /**
     * Constructor.
     * 
     * @param alpha
     * @param beta
     * @param gamma
     */
    GraphSegmentationEuclideanRGBFlowAngle(float alpha, float beta) {
        weights.push_back(alpha);
        weights.push_back(beta);
        
        // Normalization.
        D = std::sqrt(255*255 + 255*255 + 255*255);
    }
    /**
     * Evaluate the distance given 2 nodes.
     * 
     * @param n
     * @param m
     */ 
    virtual float operator()(const VideoNode & n, const VideoNode & m) {
        assert(weights.size() == 2);
        
        float dr = n.r - m.r;
        float dg = n.g - m.g;
        float db = n.b - m.b;
        
        if (n.t == m.t) {
            // Use flow within frames.
            float n_f = std::sqrt(n.fx*n.fx + n.fy*n.fy);
            float m_f = std::sqrt(m.fx*m.fx + m.fy*m.fy);
            float cos_a = std::min(1.f, std::max(-1.f, (n.fx*m.fx + n.fy*m.fy) / (n_f*m_f)));
            float a = std::acos(cos_a);

            float pi = M_PI + 1e-4;
            assert(a >= 0 && a <= pi);

            return weights[0]*std::sqrt(dr*dr + dg*dg + db*db)/D
                    + weights[1]*a/pi;
        }
        else{
            return std::sqrt(dr*dr + dg*dg + db*db)/D;
        }
    }
    
private:
    /**
     * Normalization term.
     */
    float D;
    
};

/**
 * The magic part of the graph segmentation, that is given tow nodes decide
 * whether to add an edge between them (merge the corresponding segments)
 * 
 * See [2] for the original discussion.
 */
class GraphSegmentationMagic {
public:
    /**
     * Constructor.
     */
    GraphSegmentationMagic() {};
    /**
     * Destructor.
     */
    virtual ~GraphSegmentationMagic() {};
    /**
     * Decide whether to merge these two components or not.
     * 
     * @param S_n
     * @param S_m
     * @param e
     * @return
     */
    virtual bool operator()(const VideoNode & S_n, const VideoNode & S_m, 
            const VideoEdge & e) = 0;
    
};

/**
 * Threshold the edge weight by a fixed value, segments are merged if the
 * corresponding edge weight is below the threshold.
 */
class GraphSegmentationMagicSimpleThreshold : public GraphSegmentationMagic {
public:
    /**
     * Constructor.
     * 
     * @param _c
     */
    GraphSegmentationMagicSimpleThreshold(float _c) : c(_c) {};
    /**
     * Decide whether to merge these two components or not.
     * 
     * @param S_n
     * @param S_m
     * @param e
     * @return
     */
    virtual bool operator()(const VideoNode & S_n, const VideoNode & S_m, 
            const VideoEdge & e) {
        
        if (e.w < c) {
            return true;
        }
        
        return false;
    }
    
private:
    /**
     * Threshold.
     */
    float c;
    
};

/**
 * The original criterion employed by [2].
 */
class GraphSegmentationMagicThreshold : public GraphSegmentationMagic {
public:
    /**
     * Constructor.
     * 
     * @param _c
     */
    GraphSegmentationMagicThreshold(float _c) : c(_c) {};
    /**
     * Decide whether to merge these two components or not.
     * 
     * @param S_n
     * @param S_m
     * @param e
     * @return
     */
    virtual bool operator()(const VideoNode & S_n, const VideoNode & S_m, 
            const VideoEdge & e) {
        
        float threshold = std::min(S_n.max_w + c/S_n.n, S_m.max_w + c/S_m.n);
        
        if (e.w < threshold) {
            return true;
        }
        
        return false;
    }
    
private:
    /**
     * Threshold.
     */
    float c;
    
};

/**
 * Distance used for hierarchical segmentation.
 * 
 * Essentially, derived classes only need to overwrite the () operator.
 */
class GraphSegmentationHierarchyDistance {
public:
    /**
     * Constructs a distance without weights.
     */
    GraphSegmentationHierarchyDistance() {};
    /**
     * Destructor.
     */
    virtual ~GraphSegmentationHierarchyDistance() {};
    /**
     * Assignment operator.
     * 
     * @param distance
     * @return 
     */
    virtual void operator=(const GraphSegmentationHierarchyDistance & hdistance) {
        weights = hdistance.weights;
    }
    /**
     * Evaluate the distance given 2 nodes.
     * 
     * @param n
     * @param m
     */ 
    virtual float operator()(const VideoNode & n, const VideoNode & m) = 0;
    
protected:
    /**
     * Weights to use.
     */
    std::vector<float> weights;
    
};

/**
 * Chi Square distance on RGB histograms.
 */
class GraphSegmentationHierarchyRGBChiSquare : public GraphSegmentationHierarchyDistance {
public:
    /**
     * Evaluate the distance given 2 nodes.
     * 
     * @param n
     * @param m
     */ 
    virtual float operator()(const VideoNode & n, const VideoNode & m) {
        assert(n.H == m.H);
        assert(n.n > 0 && m.n > 0);
        
        float distance = 0;
        float n_n = n.n;
        float m_n = m.n;
        
        for (int i = 0; i < n.H; i++) {
            float n_h = n.h[i]/n_n;
            float m_h = m.h[i]/m_n;
            
            if (n_h > 0 || m_h > 0) {
                distance += (n_h - m_h)*(n_h - m_h)/(n_h + m_h);
            }
        }
        
        return distance;
    };

};

/**
 * Chi Square distance on RGB histogram plus flow angle. Flow angle is only used
 * within frames.
 */
class GraphSegmentationHierarchyRGBChiSquareFlowAngle : public GraphSegmentationHierarchyDistance {
public:
    /**
     * Constructor.
     * 
     * @param alpha
     * @param beta
     */
    GraphSegmentationHierarchyRGBChiSquareFlowAngle(float alpha, float gamma) {
        weights.push_back(alpha);
        weights.push_back(gamma);
    }
    /**
     * Evaluate the distance given 2 nodes.
     * 
     * @param n
     * @param m
     */ 
    virtual float operator()(const VideoNode & n, const VideoNode & m) {
        assert(weights.size() == 2);
        assert(n.H == m.H);
        assert(n.n > 0 && m.n > 0);
        
        float distance = 0;
        float n_n = n.n;
        float m_n = m.n;
        
        for (int i = 0; i < n.H; i++) {
            float n_h = n.h[i]/n_n;
            float m_h = m.h[i]/m_n;
            
            if (n_h > 0 || m_h > 0) {
                distance += (n_h - m_h)*(n_h - m_h)/(n_h + m_h);
            }
        }
        
        if (n.t == m.t) {
            // Use flow cues only within frame.
            float n_f = std::sqrt(n.fx*n.fx + n.fy*n.fy);
            float m_f = std::sqrt(m.fx*m.fx + m.fy*m.fy);
            float cos_a = std::min(1.f, std::max(-1.f, (n.fx*m.fx + n.fy*m.fy) / (n_f*m_f)));
            float a = std::acos(cos_a);

            float pi = M_PI + 1e-4;
            assert(a >= 0 && a <= pi);

            return weights[0]*distance
                    + weights[1]*a/pi;
        }
        else {
            // Only color cues.          
            return distance;
        }
    };
    
};

/**
 * The magic part of the hierarchical segmentation, that is the decision 
 * whether to merge two segments.
 * 
 * This is usually based on a threshold which is raised every hierarchy level
 * to build a hierarchical segmentation (similar to hierarchical clustering).
 */
class GraphSegmentationHierarchyMagic {
public:
    /**
     * Constructor.
     */
    GraphSegmentationHierarchyMagic() {};
    /**
     * Destructor.
     */
    virtual ~GraphSegmentationHierarchyMagic() {};
    /**
     * Go up alevel, that is raise the threshold.
     */
    virtual void raise() = 0;
    /**
     * Decide whether to merge these two components or not.
     * 
     * @param n
     * @param m
     * @param S_n
     * @param S_m
     * @return
     */
    virtual bool operator()(const VideoNode & S_n, const VideoNode & S_m, 
            const VideoEdge & e) = 0;
    
};

/**
 * Use a simple threshold to decide when to merge two segments.
 */
class GraphSegmentationHierarchyMagicSimpleThreshold : public GraphSegmentationHierarchyMagic {
public:
    /**
     * Constructor.
     * 
     * @param _c
     */
    GraphSegmentationHierarchyMagicSimpleThreshold(float _c) : c(_c), r(2) {};
    /**
     * Constructor.
     * 
     * @param _c
     * @param _r
     */
    GraphSegmentationHierarchyMagicSimpleThreshold(float _c, float _r) : c(_c), r(_r) {};
    /**
     * Go up a level, that is raise the threshold.
     */
    virtual void raise() {
        // Color only: 1.4 (threshold 0.08, 30 levels)
        //r *= 0.95;
        c *= r;
    };
    /**
     * Decide whether to merge these two components or not.
     * 
     * @param S_n
     * @param S_m
     * @param e
     * @return
     */
    virtual bool operator()(const VideoNode & S_n, const VideoNode & S_m, 
            const VideoEdge & e) {
        
        if (e.w < c) {
            return true;
        }
        
        return false;
    }
    
private:
    /**
     * Change of threshold multiplier per raise.
     */
    float r;
    /**
     * Threshold change per raise.
     */
    float c;
    
};

/**
 * Use the threshold proposed by [2] for hierarchical segmentation.
 */
class GraphSegmentationHierarchyMagicThreshold : public GraphSegmentationHierarchyMagic {
public:
    /**
     * Constructor.
     * 
     * @param _c
     */
    GraphSegmentationHierarchyMagicThreshold(float _c) : c(_c), r(2) {};
    /**
     * Constructor.
     * 
     * @param _c
     * @param _r
     */
    GraphSegmentationHierarchyMagicThreshold(float _c, float _r) : c(_c), r(_r) {};
    /**
     * Go up a level, that is raise the threshold.
     */
    virtual void raise() {
        // Color only: 1.4 (threshold 0.08, 30 levels)
        //r *= 0.95;
        c *= r;
    };
    /**
     * Decide whether to merge these two components or not.
     * 
     * @param S_n
     * @param S_m
     * @param e
     * @return
     */
    virtual bool operator()(const VideoNode & S_n, const VideoNode & S_m, 
            const VideoEdge & e) {
        
        float threshold = std::min(S_n.max_w + c/S_n.n, S_m.max_w + c/S_m.n);
        
        if (e.w < threshold) {
            return true;
        }
        
        return false;
    }
    
private:
    /**
     * Change of threshold multiplier per raise.
     */
    float r;
    /**
     * Threshold change per raise.
     */
    float c;
    
};

/**
 * Inject randomness into segmentation.
 * 
 * Essentially, the () operator can be overwritten t decide whether to skip
 * a specific edge.
 */
class GraphSegmentationRandomness {
public:
    /**
     * Constructor.
     */
    GraphSegmentationRandomness() {};
    /**
     * Check whether to skipp this edge or not.
     * 
     * @param e
     */
    virtual bool operator()(const VideoEdge & e) = 0;
};

/**
 * No randomness (should be default).
 */
class GraphSegmentationRandomnessNone : public GraphSegmentationRandomness {
public:
    /**
     * Constructor.
     */
    GraphSegmentationRandomnessNone() {};
    /**
     * Check whether to skipp this edge or not.
     * 
     * @param e
     */
    virtual bool operator()(const VideoEdge & e) {
        return true;
    }
};

/**
 * Skip each edge with the probability p.
 */
class GraphSegmentationRandomnessSimple : public GraphSegmentationRandomness {
public:
    /**
     * Constructor.
     */
    GraphSegmentationRandomnessSimple(float _p) : p(_p) {};
    /**
     * Check whether to skip this edge or not.
     * 
     * @param e
     */
    virtual bool operator()(const VideoEdge & e) {
        float r = RAND();
        
        if (r <= p) {
            return true;
        }
        else {
            return false;
        }
    }
    
private:
    /**
     * probability of going ahead.
     */
    float p;
};

/**
 * Graph segmentation class used for oversegmentation and hierarchical segmentation.
 */
class GraphSegmentation {
public:
    /**
     * Constructor.
     * 
     * Uses the Manhatten RGB distance, the threshold by [2] and no randomness
     * as default.
     */
    GraphSegmentation() : distance(new GraphSegmentationManhattenRGB()), 
            magic(new GraphSegmentationMagicThreshold(25)),
            random(new GraphSegmentationRandomnessNone()),
            hdistance(0),
            hmagic(0) {};
    /**
     * Constructs a graph segmentation object using the given distance and magic.
     * 
     * @param _distance
     * @param _magic
     */
    GraphSegmentation(GraphSegmentationDistance* _distance, GraphSegmentationMagic* _magic) : 
            distance(_distance),
            magic(_magic),
            random(new GraphSegmentationRandomnessNone()),
            hdistance(0),
            hmagic(0) {};
    /**
     * Constructs a graph segmentation object using the given distance, magic
     * and randomness injector.
     * 
     * @param _distance
     * @param _magic
     * @param _random
     */
    GraphSegmentation(GraphSegmentationDistance* _distance, GraphSegmentationMagic* _magic, 
            GraphSegmentationRandomness* _random) : 
            distance(_distance),
            magic(_magic),
            random(_random),
            hdistance(0),
            hmagic(0) {};
    /**
     * Destructor.
     */
    virtual ~GraphSegmentation() {};
    /**
     * Set the distance to use.
     * 
     * @param _distance
     */
    void setDistance(GraphSegmentationDistance* _distance) {
        distance = _distance;
    }
    /**
     * Set the magic part of graph segmentation.
     * 
     * @param _magic
     */
    void setMagic(GraphSegmentationMagic* _magic) {
        magic = _magic;
    }
    /**
     * Set the distance to use for hierarchical segmentation.
     * 
     * @param _hdistance
     */
    void setHierarchyDistance(GraphSegmentationHierarchyDistance* _hdistance) {
        hdistance = _hdistance;
    }
    /**
     * Set the magic part of hierarchical segmentation.
     * 
     * @param _hmagic
     */
    void setHierarchyMagic(GraphSegmentationHierarchyMagic* _hmagic) {
        hmagic = _hmagic;
    }
    /**
     * Set randomness injector.
     * 
     * @param _random
     */
    void setRandomness(GraphSegmentationRandomness* _random) {
        random = _random;
    }
    /**
     * Build the graph based on flow and color.
     * 
     * @param video
     * @param flow
     */
    void buildGraph(Video & video, Video & flow);
    /**
     * Build the edges of the graph.
     */
    void buildEdges();
    /**
     * Oversegment the given graph.
     */
    void oversegmentGraph();
    /**
     * Enforces the given minimum segment size.
     * 
     * @param M
     */
    void enforceMinimumSegmentSize(int M);
    /**
     * Builds a region graph based on the created oversegmentation. Should be
     * called after the initial oversegmentation and after each region segmentation.
     */
    void buildRegionGraph();
    /**
     * Go up one level in the hierarchy as specified by the underlying
     * magic.
     */
    void addHierarchyLevel();
    /**
     * Derive labels from the produced oversegmentation.
     */
    SegmentationVideo deriveLabels();
    
private:
    /**
     * Video dimensions.
     */
    int T;
    int H;
    int W;
    /**
     * The constructed and segmented video graph.
     */
    VideoGraph graph;
    /**
     * The underlying distance.
     */
    GraphSegmentationDistance* distance;
    /**
     * The magic part of graph segmentation.
     */
    GraphSegmentationMagic* magic;
    /**
     * Randomness injector.
     */
    GraphSegmentationRandomness* random;
    /**
     * Distance used for hierarchical segmentation.
     */
    GraphSegmentationHierarchyDistance* hdistance;
    /**
     * The magic partof hierarchical segmentation.
     */
    GraphSegmentationHierarchyMagic* hmagic;

};

#endif	/* GRAPH_SEGMENTATION_H */

