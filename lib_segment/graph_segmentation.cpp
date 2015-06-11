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

#include "graph_segmentation.h"
#include <limits>

void GraphSegmentation::buildGraph(Video & video, Video & flow) {
    assert(video.getFrameNumber() > 0);
    assert(video.getFrameHeight() > 0);
    assert(video.getFrameWidth() > 0);
    
    assert(video.getFrameNumber() == flow.getFrameNumber());
    assert(video.getFrameHeight() == flow.getFrameHeight());
    assert(video.getFrameWidth() == flow.getFrameWidth());
    
    T = video.getFrameNumber();
    H = video.getFrameHeight();
    W = video.getFrameWidth();
    
    int N = T*H*W;
    graph = VideoGraph(N);
    
    for (int t = 0; t < T; t++) {
        for (int i = 0; i < H; i++) {
            for (int j = 0; j < W; j++) {
                
                int n = H*W*t + W*i + j;
                VideoNode & node = graph.getNode(n);
                
                node.i = i;
                node.j = j;
                node.t = t;
                
                cv::Vec3b & bgr = video.get<cv::Vec3b>(t, i, j);
                node.b = bgr[0];
                node.g = bgr[1];
                node.r = bgr[2];
                
                // Color histogram.
                int denominator = std::ceil(256./((double) (node.H/3)));
                int h_b = node.b/denominator;
                int h_g = node.g/denominator;
                int h_r = node.r/denominator;
                
                node.H = 36;
                node.h = std::vector<int>(node.H, 0);
                
                node.h[h_b]++;
                node.h[h_g]++;
                node.h[h_r]++;
                
                // Flow.
//                if (t < T - 1) {
                    node.fx = flow.get<cv::Vec2f>(t, i, j)[0];
                    node.fy = flow.get<cv::Vec2f>(t, i, j)[1];
                    node.f = std::sqrt(node.fx*node.fx + node.fy*node.fy);
//                }
//                else {
//                    node.fx = 0;
//                    node.fy = 0;
//                    node.f = 0;
//                }
                
                // Initialize label.
                node.l = n;
                node.id = n;
                node.n = 1;
                
                assert(graph.getNode(n).id == n);
            }
        }
    }
}

void GraphSegmentation::buildEdges() {
    assert(graph.getNumNodes() > 0);
    
    // Count invalid flow:
    int invalid = 0;
    
    // Now add all edges and weights.
    for (int t = 0; t < T; t++) {
        for (int i = 0; i < H; i++) {
            for (int j = 0; j < W; j++) {
                int n = H*W*t + W*i + j;
                const VideoNode & node = graph.getNode(n);
                
                if (t < T - 1) {
                    int k = i + node.fy;
                    int l = j + node.fx;
                    
                    if (k >= 0 && k < H && l >= 0 && l < W) {
                        int m = H*W*(t + 1) + W*k + l;
                        VideoNode & other = graph.getNode(m);

                        assert(m == other.id);
                        
                        VideoEdge edge;
                        edge.n = n;
                        edge.m = m;
                        edge.w = (*distance)(node, other);
                        edge.r = false;
                        
                        graph.addEdge(edge);
                    }
                    else {
                        invalid++;
                    }
                }
                
                if (i < H - 1) {
                    int m = H*W*t + W*(i + 1) + j;
                    VideoNode & other = graph.getNode(m);
                    
                    assert(m == other.id);
                    
                    VideoEdge edge;
                    edge.n = n;
                    edge.m = m;
                    edge.w = (*distance)(node, other);
                    edge.r = false;
                    
                    graph.addEdge(edge);
                }
                
                if (j < W - 1) {
                    int m = H*W*t + W*i + (j + 1);
                    VideoNode & other = graph.getNode(m);
                    
                    assert(m == other.id);
                    
                    VideoEdge edge;
                    edge.n = n;
                    edge.m = m;
                    edge.w = (*distance)(node, other);
                    edge.r = false;
                    
                    graph.addEdge(edge);
                }
            }
        }
    }
}

void GraphSegmentation::oversegmentGraph() {
    assert(graph.getNumNodes() > 0);
    assert(graph.getNumEdges() > 0);
    
    int N = graph.getNumNodes();
    int E = graph.getNumEdges();
    
    // Sort edges.
    graph.sortEdges();
    
    for (int e = 0; e < E; e++) {
        VideoEdge edge = graph.getEdge(e);
        
        if ((*random)(edge)) {
            VideoNode & n = graph.getNode(edge.n);
            VideoNode & m = graph.getNode(edge.m);

            VideoNode & S_n = graph.findNodeComponent(n);
            VideoNode & S_m = graph.findNodeComponent(m);
            
            // Are the nodes in different components?
            if (S_m.id != S_n.id) {

                // Here comes the magic!
                if ((*magic)(S_n, S_m, edge)) {
                    graph.merge(S_n, S_m, edge);
                }
            }
            else {
                // Nodes already are in same component!
                edge.r = true;
            }
        }
    }
}

void GraphSegmentation::enforceMinimumSegmentSize(int M) {
    assert(graph.getNumNodes() > 0);
    // assert(graph.getNumEdges() > 0);
    
    int N = graph.getNumNodes();
    int E = graph.getNumEdges();
    
    for (int e = 0; e < E; e++) {
        VideoEdge edge = graph.getEdge(e);
        
        if (!edge.r) {
            VideoNode & n = graph.getNode(edge.n);
            VideoNode & m = graph.getNode(edge.m);

            VideoNode & S_n = graph.findNodeComponent(n);
            VideoNode & S_m = graph.findNodeComponent(m);

            if (S_n.l != S_m.l) {
                if (S_n.n < M || S_m.n < M) {
                    graph.merge(S_n, S_m, edge);
                }
            }
            else {
                // Already merged nodes.
                edge.r = true;
            }
        }
    }
}

void GraphSegmentation::buildRegionGraph() {
    assert(graph.getNumNodes() > 0);
    assert(graph.getNumEdges() > 0);
    
    assert(hdistance != 0);
    assert(hmagic != 0);
    
    int N = graph.getNumNodes();
    int E = graph.getNumEdges();
    
    // Someminor checks and resets.
    for (int n = 0; n < N; n++) {
        VideoNode & node = graph.getNode(n);
        node.max_w = 0;
    }
    
    for (int e = 0; e < E; e++) {
        VideoEdge edge = graph.getEdge(e);
        
        if (!edge.r) {
            VideoNode & n = graph.getNode(edge.n);
            VideoNode & m = graph.getNode(edge.m);
            
            VideoNode & S_n = graph.findNodeComponent(n);
            VideoNode & S_m = graph.findNodeComponent(m);
            
            if (S_n.l != S_m.l) {
                edge.w = (*hdistance)(S_n, S_m);
            }
            else {
                // This edge has already been merged.
                edge.r = true;
            }
        }
    }
}

void GraphSegmentation::addHierarchyLevel() {
    assert(graph.getNumNodes() > 0);
    // assert(graph.getNumEdges() > 0);
    
    assert(hdistance != 0);
    assert(hmagic != 0);
    
    hmagic->raise();
    
    int N = graph.getNumNodes();
    int E = graph.getNumEdges();
    
    for (int e = 0; e < E; e++) {
        VideoEdge edge = graph.getEdge(e);
        
        if ((*random)(edge)) {
            if (!edge.r) {
                VideoNode & n = graph.getNode(edge.n);
                VideoNode & m = graph.getNode(edge.m);

                VideoNode & S_n = graph.findNodeComponent(n);
                VideoNode & S_m = graph.findNodeComponent(m);

                // Are the nodes in different components?
                if (S_m.id != S_n.id) {

                    // Here comes the magic!
                    if ((*hmagic)(S_n, S_m, edge)) {
                        graph.merge(S_n, S_m, edge);
                    }
                }
                else {
                    // This edge has already beenprocessed and merged.
                    edge.r = true;
                }
            }
        }
    }
}

SegmentationVideo GraphSegmentation::deriveLabels() {
    assert(graph.getNumNodes() > 0);
    // assert(graph.getNumEdges() > 0);
    
    SegmentationVideo video;
    for (int t = 0; t < T; t++) {
        cv::Mat frame(H, W, CV_32SC1, cv::Scalar(0));
        
        for (int i = 0; i < H; i++) {
            for (int j = 0; j < W; j++) {
                int n = H*W*t + W*i + j;
                
                VideoNode & node = graph.getNode(n);
                VideoNode & S_node = graph.findNodeComponent(node);
                
                const int max = std::numeric_limits<int>::max();
                assert(S_node.id <= max);
                
                frame.at<int>(i, j) = S_node.id;
            }
        }
        
        video.addFrame(frame);
    }
    
    assert(video.getFrameNumber() > 0);
    assert(video.getFrameHeight() > 0);
    assert(video.getFrameWidth() > 0);
    
    video.relabel();
    return video;
}