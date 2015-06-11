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

#ifndef VIDEO_GRAPH_H
#define	VIDEO_GRAPH_H

#include <assert.h>
#include <vector>
#include <map>
#include <algorithm>

/**
 * Represents an edge between two pixels or supervoxels in a video volume.
 * Each edge is characterized by a weight and the adjacent nodes.
 */
class VideoEdge {
public:
    /**
     * Default constructor. Initialize additional attributes here!
     */
    VideoEdge() : n(0), m(0), w(0), r(false) {};
    /**
     * Destructor.
     */
    virtual ~VideoEdge() {};
    
    /**
     * Index of first node.
     */
    unsigned long int n;
    /**
     * Idex of second node.
     */
    unsigned long int m;
    /**
     * Edge weight.
     */
    float w;
    /**
     * Whether the edge has been removed.
     */
    bool r;
    
};

/**
 * Sorts edges.
 * 
 * @param g
 * @param h
 * @return 
 */
class VideoEdgeSorter {
public:
    inline bool operator()(const VideoEdge & g, const VideoEdge h) {
        return (h.w > g.w);
    }
};

/**
 * Represents a pixel or a supervoxel in a video volume. Each pixel or supervoxel
 * is represented by its color or color histogram used to compute edge weights.
 * 
 * Additional node attributes can easily be added and sued in
 * GraphSegmentationDistance or GraphSegmentationHierarchyDistance to compute the
 * edge weights. The attributes can be set in GraphSegmentation::buildGraph.
 */
class VideoNode {
public:
    /**
     * Default constructor. Initialize additional attributes here!
     */
    VideoNode() : r(0), g(0), b(0), i(0), j(0), t(0), fx(0), fy(0), 
            H(0), max_w(0), l(0), n(1), id(0) {
    };
    /**
     * Destructor.
     */
    virtual ~VideoNode() {};
    
    /**
     * Pixel indices.
     */
    int i;
    int j;
    int t;
    
    /**
     * Color of pixel.
     */
    int r;
    int g;
    int b;
    
    /**
     * Optical flow.
     */
    float fx;
    float fy;
    float f; // Flow magnitude
    
    /**
     * Color histograms.
     */
    std::vector<int> h;
    int H;
    
    /**
     * The label of the pixel.
     */
    unsigned long int l; // label, i.e. the index of the node this node belongs to
    /**
     * Size of node after merging with other nodes.
     */
    unsigned long int n;
    /**
     * Id of the node.
     */
    unsigned long int id;
    /**
     * Maximum weight.
     */
    float max_w;
    
};

/**
 * Represents a video graph, initially consisting of one node per pixel within the
 * video volume. Pixels are 6-connected and are merged successively to form
 * supervoxels.
 * 
 * A graph is represented by its set of edges and nodes.
 */
class VideoGraph {
public:
    /**
     * Default constructor.
     */
    VideoGraph() {};
    /**
     * Constructs a video graph with the given exact number of nodes.
     * 
     * @param N
     */
    VideoGraph(int N) {
        nodes = std::vector<VideoNode>(N);
    }
    /**
     * Destructor.
     */
    virtual ~VideoGraph() {};
    /**
     * Assignment operator.
     * 
     * @param graph
     * @return 
     */
    void operator=(const VideoGraph & graph) {
        nodes = graph.nodes;
        edges = graph.edges;
    }
    /**
     * Set the n-th node.
     * 
     * @param edge
     */
    void setNode(int n, VideoNode & node) {
        nodes[n] = node;
    }
    /**
     * Add a new node.
     * 
     * @param node
     */
    void addNode(VideoNode & node) {
        nodes.push_back(node);
    }
    /**
     * Add a new edge.
     * 
     * @param edge
     */
    void addEdge(VideoEdge & edge) {
        edges.push_back(edge);
    }
    /**
     * Get the n-th node.
     * 
     * @param n
     * @return 
     */
    VideoNode & getNode(int n) {
        assert(n >= 0 && n < static_cast<int>(nodes.size()));
        return nodes[n];
    }
    /**
     * Get the e-th edge in the current sorting.
     * 
     * @param e
     * @return 
     */
    VideoEdge & getEdge(int e) {
        assert(e >= 0 && e < static_cast<int>(edges.size()));
        return edges[e];
    }
    /**
     * Get the number of nodes.
     * 
     * @return 
     */
    int getNumNodes() {
        return nodes.size();
    }
    /**
     * Get the number of edges.
     * 
     * @return 
     */
    int getNumEdges() {
        return edges.size();
    }
    /**
     * Sort the edges by weight.
     */
    void sortEdges() {
        std::sort(edges.begin(), edges.end(), VideoEdgeSorter());
    }
    /**
     * Delete all edges.
     */
    void clearEdges() {
        edges.clear();
    }
    /**
     * When two nodes get merged, the first node is assigned the id of the second
     * node as label. By traversing this labeling, the current component of each
     * node (that is, pixel) can easily be identified and the label can be updated
     * for efficiency.
     * 
     * @param n
     * @return
     */
    VideoNode & findNodeComponent(VideoNode & n) {
        
        // Get component of node n.
        int l = n.l;
        int id = n.id;
        
        while (l != id) {
            id = nodes[l].id;
            l = nodes[l].l;
        }
        
        VideoNode & S = nodes[l];
        assert(S.l == S.id);
        
        // Save latest component.
        n.l = S.id;
        
        return S;
    }
    /**
     * Merge two pixels or supervoxels (that is merge two nodes). 
     * 
     * Depending on the used "Distance", some lines may be commented out
     * to speed up the algorithm.
     * 
     * @param S_n
     * @param S_m
     * @param e
     */
    void merge(VideoNode & S_n, VideoNode & S_m, VideoEdge & e) {
        S_m.l = S_n.id;
        
        // Update histogram.
        for (int i = 0; i < S_n.H; i++) {
            S_n.h[i] += S_m.h[i];
        }
        
        // Flow.
        S_n.fx += S_m.fx;
        S_n.fy += S_m.fy;
        S_n.f += S_m.f;
        
        // Update mean color.
//        S_n.r += S_m.r;
//        S_n.g += S_m.g;
//        S_n.b += S_m.b;
        
        // Update cound.
        S_n.n += S_m.n;
        
        // Update maximum weight.
        S_n.max_w = std::max(std::max(S_n.max_w, S_m.max_w), e.w);
        
        // This edge has already been removed.
        e.r = true;
    }
    
private:
    /**
     * All edges in this graph.
     */
    std::vector<VideoEdge> edges;
    /**
     * All nodes in this graph.
     */
    std::vector<VideoNode> nodes;
    
};

#endif	/* VIDEO_GRAPH_H */

