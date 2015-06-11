# Hierarchical Graph-Based Video Segmentation

This is an implementation of the hierarchical graph-based video segmentation algorithm proposed by Grundmann et al. [1] based on the graph-based image segmentation algorithm by Felzenswalb and Huttenlocher [2].

    [1] M. Grundmann, V. Kwatra, M. Han, and I. A. Essa.
        Efficient hierarchical graph-based video segmentation.
        In Conference on Computer Vision and Pattern Recognition, pages 2141–2148,
        San Francisco, 2010, June 2010.
    [2] P. F. Felzenszwalb and D. P. Huttenlocher.
        Efficient graph-based image segmentation.
        International Journal of Computer Vision, 59(2):167–181, September 2004.

Further, evaluation metrics based on the Precision-Recall Framework for videos [3,4], Undersegmentation Error [4,5] and Achievable Segmentation Accuracy [3] are provided.

    [3] C. Xu and J. J. Corso.
        Evaluation of super-voxel methods for early video processing.
        In Computer Vision and Pattern Recognition, Conference on,
        pages 1202–1209, Providence, RI, June 2012.
    [4] F. Galasso, N. S. Nagaraja, T. J. Cardenas, T. Brox and B.Schiele.
        A Unified Video Segmentation Benchmark: Annotation, Metrics and Analysis.
        In International Conference on Computer Vision, 
        pages 3527-3534, Sydney, Australia, December 2013.
    [5] P. Neubert and P. Protzel.
        Superpixel benchmark and comparison.
        In Forum Bildverarbeitung, Regensburg, Germany, November 2012.

The algorithm is based on optical flow, therefore a command line tool to pre-compute optical flow using OpenCV is provided. For further optical flow algorithms, we refer to the [Sintel dataset](http://sintel.is.tue.mpg.de/results). Note that `alley_1` includes ten frames of a sample sequence taken from the Sintel dataset and the `alley_1_flow` folder contains the ground truth flow in the correct format (the original format is not compatible with this implementation; code provided by the Sintel dataset has been used for conversion).

![Example: original sequence, initial oversegmentation and a selected hierarchy level.](results/alley_1_full_spaced_690.png?raw=true "Example: original sequence, initial oversegmentation and a selected hierarchy level.")

## Compile

The implementation is based on [OpenCV](http://opencv.org/), see [here](http://docs.opencv.org/doc/tutorials/introduction/linux_install/linux_install.html#linux-installation) for installation instructions, [CMake](http://www.cmake.org/) and [Boost](http://www.boost.org/). The implementation has been tested on Ubuntu 14.04 and 14.10:

    $ sudo apt-get install build-essential cmake libboost-all-dev
    $ git clone https://github.com/davidstutz/hierarchical-graph-based-video-segmentation.git
    $ cd hierarchical-graph-based-video-segmentation
    $ mkdir build
    $ cd build
    $ cmake ..
    $ make
    Scanning dependencies of target segment
    [ 16%] Building CXX object lib_segment/CMakeFiles/segment.dir/graph_segmentation.cpp.o
    [ 33%] Building CXX object lib_segment/CMakeFiles/segment.dir/evaluation.cpp.o
    [ 50%] Building CXX object lib_segment/CMakeFiles/segment.dir/io.cpp.o
    [ 66%] Building CXX object lib_segment/CMakeFiles/segment.dir/io_util.cpp.o
    Linking CXX static library libsegment.a
    [ 66%] Built target segment
    Scanning dependencies of target segment_cli
    [ 83%] Building CXX object segment_cli/CMakeFiles/segment_cli.dir/main.cpp.o
    Linking CXX executable segment_cli
    [ 83%] Built target segment_cli
    Scanning dependencies of target optical_flow_cli
    [100%] Building CXX object optical_flow_cli/CMakeFiles/optical_flow_cli.dir/main.cpp.o
    Linking CXX executable optical_flow_cli
    [100%] Built target optical_flow_cli
    # Run video segmentation on the alley_1 sequence and save visualization to build/output_vis:
    $ ./segment_cli/segment_cli ../alley_1 ../alley_1_flow/ --vis-dir output_vis
    Built graph (0) (2.09012).
    Oversegmented graph (0) (3.23881).
    Enforced minimum region size (0) (2.45697).
    Built region graph (1) (1.22925).
    Segmented region graph (1) (0.865099).
    Enforce minimum segment size (1) (0.808102).
    # ...

For 10 frames, the algorithm needs approximately 8 seconds for the initial oversegmentation and approximately 3 seconds for every additional level.

The two command line tools provide the following options:

    $ ./segment_cli/segment_cli --help
    Allowed options:
      --help                           produce help message
      --input-video arg                directory of input video (provided as 
                                       sequence of individual images)
      --input-flow arg                 input flow directory (text files in 
                                       cv::Storage format, see io.h)
      --length arg (=10)               length of video to oversegment (may be lower
                                       than the actual sequence length)
      --flow-weight arg (=0.200000003) weight on flow angle for edge weight 
                                       computation
      --threshold arg (=0.0199999996)  threshold (is multiplied by 1.3 for each 
                                       additional hierarchy level)
      --hierarchies arg (=40)          number of hierarchies
      --vis-dir arg                    visualization directory (default will not 
                                       visualize the result!)
      --output-dir arg (=output)       output directory

    # Compute OpenCV optical flow and save in the correct format.
    $ ./optical_flow_cli/optical_flow_cli --help
    Allowed options:
      --help                                produce help message
      --input-dir arg                       input directory
      --pyramid-scale arg (=0.5)            pyramid scale
      --pyramid-levels arg (=1)             pyramid levels
      --window-size arg (=7)                window size
      --num-iterations arg (=10)            number of iterations at each pyramid 
                                            level
      --polynomial-neighborhood-size arg (=5)
                                            size of pixel neighborhood to find 
                                            polynomial expansion
      --polynomial-sigma arg (=1.10000002)  standard deviation of Gaussian used to 
                                            smooth derivatives
      --gaussian-filter                     use a Gaussian filter isntead of a box 
                                            filter
      --output-dir arg (=output)            output directory


## Usage

An usage example can be found in `segment_cli/main.cpp`. See also `io.h` and `io_util.h` for basic input and output and the format of the labels computed by the algorithm. Example usage:

    // The video is expected to be provided as sequence of images, see alley_1 as example.
    // The same holds for the optical flow, which is provided using text files formatted
    // according to cv::FileStorage, see io.h.
    boost::filesystem::path in_dir("path/to/video");
    boost::filesystem::path flow_dir("path/to/flow");
    
    // Length of the seuqnce to read (may be smaller than the actual length).
    int length = 10;
    Video video = IO::readVideo(in_dir, length);
    FlowVideo flowVideo = IO::readFlowVideo(flow_dir, length);
    
    assert(video.getFrameNumber() > 0);
    assert(video.getFrameNumber() == flowVideo.getFrameNumber());
    
    // Parameters:
    int M = 300; // Minimum segment size, enforced after segmentation.
    int L = 40; // Number of hierarchy levels.
    float c = 0.02; // Threshold used for the initial oversegmentation.
    float beta = 0.25; // Importance of flow information for edge weight computation.
    float alpha = 1 - beta; // ... importance of color.
    
    GraphSegmentationMagic* magic = new GraphSegmentationMagicThreshold(c);
    GraphSegmentationDistance* distance = new GraphSegmentationEuclideanRGBFlowAngle(alpha, beta);
    
    GraphSegmentation segmenter(distance, magic);
    
    boost::timer timer;
    // Setup the video graph.
    segmenter.buildGraph(video, flowVideo);
    segmenter.buildEdges();
    std::cout << "Built graph (0) (" << timer.elapsed() << ")." << std::endl;
    
    timer.restart();
    // Oversegment the video.
    segmenter.oversegmentGraph();
    std::cout << "Oversegmented graph (0) (" << timer.elapsed() << ")." 
            << std::endl;
    
    timer.restart();
    // Enforce minimum segment size.
    segmenter.enforceMinimumSegmentSize(M);
    std::cout << "Enforced minimum region size (0) (" << timer.elapsed() 
            << ")." << std::endl;
    
    // Get the corresponding segmentation video (that is containing the labels.
    // The video contains length three channel images where the labels are encoded 
    // as 24 bit numbers, see io_util.h
    SegmentationVideo sv_video = segmenter.deriveLabels();
    IO::writeSegmentationVideo(out_dir / boost::filesystem::path("0"), 
                sv_video);
    
    // Visualize if requested.
    if (visualize) {
        IO::writeColoredSegmentationVideo(vis_dir / boost::filesystem::path("0"), 
                sv_video);
    }
    
    // Each new hierarchy level, the threshold is raised by the factor 1.3.
    GraphSegmentationHierarchyMagic* hmagic = new GraphSegmentationHierarchyMagicThreshold(c, 1.3);
    GraphSegmentationHierarchyDistance* hdistance = 
            new GraphSegmentationHierarchyRGBChiSquareFlowAngle(alpha, beta);
    
    segmenter.setHierarchyMagic(hmagic);
    segmenter.setHierarchyDistance(hdistance);
    
    // Building the hierarchy proceeds similar to the initial oversegmentation.
    for (int l = 0; l < L; l++) {
        timer.restart();
        segmenter.buildRegionGraph();
        std::cout << "Built region graph (" << (l + 1) << ") ("
                << timer.elapsed() << ")." << std::endl;
        
        timer.restart();
        segmenter.addHierarchyLevel();
        std::cout << "Segmented region graph (" << (l + 1) << ") ("
                << timer.elapsed() << ")." << std::endl;
        
        timer.restart();
        segmenter.enforceMinimumSegmentSize(l/2 * M);
        std::cout << "Enforce minimum segment size (" << (l + 1) << ") ("
                << timer.elapsed() << ")." << std::endl;
        
        SegmentationVideo sv_video = segmenter.deriveLabels();
        IO::writeSegmentationVideo(out_dir / boost::filesystem::path(std::to_string(l + 1)), 
                sv_video);
        
        if (visualize) {
            IO::writeColoredSegmentationVideo(vis_dir / boost::filesystem::path(std::to_string(l + 1)), 
                    sv_video);
        }
    }



## License

Copyright (c) 2014 - 2015, David Stutz
All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice,this list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
* Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.