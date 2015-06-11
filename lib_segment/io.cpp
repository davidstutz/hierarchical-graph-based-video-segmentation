#include "io.h"
#include "io_util.h"
#include <assert.h>
#include <iomanip>
#include <stdio.h>
#include <limits>
#include "csv_iterator.h"

Video::Video() {
    
}

Video::~Video() {
    
}

int Video::addFrame(const cv::Mat& frame) {
    assert(frame.rows > 0 && frame.cols > 0);
    
    if (this->frames.empty()) {
        // This is the first frame, save meta data.
        this->frameHeight = frame.rows;
        this->frameWidth = frame.cols;
        this->frameType = frame.type();
        this->frameChannels = frame.channels();
    }
    
    assert(frame.rows == this->frameHeight && frame.cols == this->frameWidth);
    assert(frame.type() == this->frameType && this->frameChannels == frame.channels());
    
    this->frames.push_back(frame.clone());
    return this->frames.size();
}

int Video::getFrameWidth() {
    return this->frameWidth;
}

int Video::getFrameHeight() {
    return this->frameHeight;
}

int Video::getFrameType() {
    return this->frameType;
}

int Video::getFrameChannels() {
    return this->frameChannels;
}

int Video::getFrameNumber() {
    return this->frames.size();
}

cv::Mat & Video::getFrame(int t) {
    assert(t >= 0 && t < frames.size());
    
    return frames[t];
}

template<typename T>
T & Video::get(int t, int i, int j) {
    assert(t >= 0 && t < this->frames.size());
    assert(i >= 0 && i < this->frameHeight);
    assert(j >= 0 && j < this->frameWidth);
    
    return this->frames[t].at<T>(i, j);
}

template unsigned char & Video::get<unsigned char>(int, int, int);
template int & Video::get<int>(int, int, int);
template float & Video::get<float>(int, int, int);
template cv::Vec3b & Video::get<cv::Vec3b>(int, int, int);
template cv::Vec2f & Video::get<cv::Vec2f>(int, int, int);

SegmentationVideo::SegmentationVideo() {
    
}

SegmentationVideo::~SegmentationVideo() {
    
}

int SegmentationVideo::getSegmentNumber() {
    if (this->segments.empty()) {
        this->initializeSegments();
    }
    
    return this->segments.size();
}

int SegmentationVideo::getSegmentSize(int label) {
    assert(label >= 0 && label < segments.size());
    
    return this->segments[label];
}

void SegmentationVideo::initializeSegments() {
    int segmentCount = 0;
    for (int t = 0; t < this->frames.size(); ++t) {
        for (int i = 0; i < this->frameHeight; ++i) {
            for (int j = 0; j < this->frameWidth; ++j) {
                if (this->frames[t].at<int>(i, j) > segmentCount) {
                    segmentCount = this->frames[t].at<int>(i, j);
                }
            }
        }
    }

    this->segments = std::vector<int>(segmentCount + 1, 0);
    for (int t = 0; t < this->frames.size(); ++t) {
        for (int i = 0; i < this->frameHeight; ++i) {
            for (int j = 0; j < this->frameWidth; ++j) {
                ++this->segments[this->frames[t].at<int>(i, j)];
            }
        }
    }
}

void SegmentationVideo::relabel() {
    
    int segmentNumber = this->getSegmentNumber();
    std::vector<int> mapping(segmentNumber, -1);
    
    int label = 0;
    for (int t = 0; t < this->frames.size(); ++t) {
        for (int i = 0; i < this->frameHeight; ++i) {
            for (int j = 0; j < this->frameWidth; ++j) {
                if (mapping[this->frames[t].at<int>(i, j)] < 0) {
                    mapping[this->frames[t].at<int>(i, j)] = label;
                    label++;
                }
                
                this->frames[t].at<int>(i, j) = mapping[this->frames[t].at<int>(i, j)];
            }
        }
    }
    
    this->segments.clear();
}

void SegmentationVideo::relabelSemantic(std::multimap<int, int>& mapping) {
    int segmentNumber = this->getSegmentNumber();
    
    int label = 0;
    if (mapping.empty()) {
        mapping = std::multimap<int, int>();
    }
    else {
        // Find maximum of existing label.
        for (std::multimap<int, int>::iterator it = mapping.begin(); it != mapping.end(); ++it) {
            if (it->second > label) {
                label = it->second;
            }
        }

        label = label + 1;
    }
    
    for (int t = 0; t < this->frames.size(); ++t) {
        for (int i = 0; i < this->frameHeight; ++i) {
            for (int j = 0; j < this->frameWidth; ++j) {
                if (mapping.find(this->frames[t].at<int>(i, j)) == mapping.end()) {
                    mapping.insert(std::pair<int, int>(this->frames[t].at<int>(i, j), label));
                    label++;
                }
                
                this->frames[t].at<int>(i, j) = mapping.find(this->frames[t].at<int>(i, j))->second;
            }
        }
    }
    
    this->segments.clear();
}

SegmentationVideo SegmentationVideo::add(SegmentationVideo & videoA, 
        SegmentationVideo & videoB, bool relabel) {
    
    assert(videoA.getFrameNumber() == videoB.getFrameNumber());
    assert(videoA.getFrameHeight() == videoB.getFrameHeight());
    assert(videoA.getFrameWidth() == videoB.getFrameWidth());
    
    int T = videoA.getFrameNumber();
    int rows = videoA.getFrameHeight();
    int cols = videoA.getFrameWidth();
    
    int videoBSegments = videoB.getSegmentNumber();
    
    SegmentationVideo result;
    for (int t = 0; t < T; ++t) {
        
        cv::Mat frame(rows, cols, CV_32SC1, cv::Scalar(0));
        for (int i = 0; i < rows; ++i) {
            for (int j = 0; j < cols; ++j) {
                frame.at<int>(i, j) = videoBSegments*videoA.get<int>(t, i, j)
                        + videoB.get<int>(t, i, j);
            }
        }
        
        result.addFrame(frame);
    }
    
    // Relabel the result if requested.
    if (relabel) {
        result.relabel();
    }
    
    return result;
}

FlowVideo::FlowVideo() {
    
}

FlowVideo::~FlowVideo() {
    
}

Video IO::readVideo(boost::filesystem::path directory, int T) {
    std::multimap<std::string, boost::filesystem::path> frames = IOUtil::readDirectory(directory, 
            IOUtil::getImageExtensions());
    
    assert(frames.size() > 0);
    Video video;
    
    int t = 0;
    for (std::multimap<std::string, boost::filesystem::path>::iterator it = frames.begin(); 
            it != frames.end(); ++it) {
        
        if (T > 0 && t > T) {
            break;
        }
        
        cv::Mat frame = cv::imread(it->second.string());
        video.addFrame(frame);
        
        ++t;
    }
    
    return video;
}

Video IO::readVideoMat(boost::filesystem::path directory, int T) {
    std::multimap<std::string, boost::filesystem::path> frames = IOUtil::readDirectory(directory, 
            IOUtil::getTxtExtensions());
    
    assert(frames.size() > 0);
    Video video;
    
    int t = 0;
    for (std::multimap<std::string, boost::filesystem::path>::iterator it = frames.begin(); 
            it != frames.end(); ++it) {
        
        if (T > 0 && t > T) {
            break;
        }
        
        cv::Mat frame = IO::readMat(it->second);
        video.addFrame(frame);
        
        ++t;
    }
    
    return video;
}

int IO::writeVideo(boost::filesystem::path directory, Video & video) {
    if (!boost::filesystem::is_directory(directory)) {
        boost::filesystem::create_directories(directory);
    }
    
    int t = 0;
    for (t = 0; t < video.getFrameNumber(); ++t) {
        cv::Mat frame = video.getFrame(t);
        
        char name[6];
        sprintf(name, "%05d", t);
        std::string strName = std::string(name);
        
        cv::imwrite(directory.string() + DIRECTORY_SEPARATOR + strName + ".png", 
                frame);
    }
    
    return t;
}

int IO::writeVideoMat(boost::filesystem::path directory, Video & video) {
    if (!boost::filesystem::is_directory(directory)) {
        boost::filesystem::create_directories(directory);
    }
    
    int t = 0;
    for (t = 0; t < video.getFrameNumber(); ++t) {
        cv::Mat frame = video.getFrame(t);
        
        char name[6];
        sprintf(name, "%05d", t);
        std::string strName = std::string(name);
        
        boost::filesystem::path path(directory.string() + DIRECTORY_SEPARATOR 
                + strName + ".txt");
        
        IO::writeMat(path, frame);
    }
    
    return t;
}

SegmentationVideo IO::readSegmentationVideo(boost::filesystem::path directory, int T) {
    std::multimap<std::string, boost::filesystem::path> frames = IOUtil::readDirectory(directory, 
            IOUtil::getImageExtensions());
    
    assert(frames.size() > 0);  
    SegmentationVideo video;
    
    int t = 0;
    for (std::multimap<std::string, boost::filesystem::path>::iterator it = frames.begin(); 
            it != frames.end(); ++it) {
        
        if (T > 0 && t > T) {
            break;
        }
        
        cv::Mat frame = cv::imread(it->second.string());
        cv::Mat labelFrame = IOUtil::segmentationImageToLabelMap(frame);
        video.addFrame(labelFrame);
        
        ++t;
    }
    
    return video;
}

FlowVideo IO::readFlowVideo(boost::filesystem::path directory, int T) {
    std::multimap<std::string, boost::filesystem::path> frames = IOUtil::readDirectory(directory, 
            IOUtil::getTxtExtensions());
    
    assert(frames.size() > 0);  
    FlowVideo video;
    
    int t = 0;
    for (std::multimap<std::string, boost::filesystem::path>::iterator it = frames.begin(); 
            it != frames.end(); ++it) {
        
        if (T > 0 && t > T) {
            break;
        }
        
        cv::Mat frame = IO::readMat(it->second.string());
        video.addFrame(frame);
        
        ++t;
    }
    
    return video;
}

int IO::writeSegmentationVideo(boost::filesystem::path directory, 
        SegmentationVideo & video) {
    
    if (!boost::filesystem::is_directory(directory)) {
        boost::filesystem::create_directories(directory);
    }
    
    int t = 0;
    for (t = 0; t < video.getFrameNumber(); ++t) {
        cv::Mat labels = video.getFrame(t);
        cv::Mat imageLabels = IOUtil::labelMapToSegmentationImage(labels);
        
        char name[6];
        sprintf(name, "%05d", t);
        std::string strName = std::string(name);
        
        cv::imwrite(directory.string() + DIRECTORY_SEPARATOR + strName + ".png", 
                imageLabels);
    }
    
    return t;
}

int IO::writeColoredSegmentationVideo(boost::filesystem::path directory, 
        SegmentationVideo & video) {
    
    if (!boost::filesystem::is_directory(directory)) {
        boost::filesystem::create_directories(directory);
    }
    
    std::vector<cv::Vec3b> colors(video.getSegmentNumber(), cv::Vec3b(0, 0, 0));
    
    int t = 0;
    for (t = 0; t < video.getFrameNumber(); ++t) {
        cv::Mat labels = video.getFrame(t);
        cv::Mat coloredLabels(labels.rows, labels.cols, CV_8UC3, cv::Scalar(0, 0, 0));
        
        for (int i = 0; i < labels.rows; ++i) {
            for (int j = 0; j < labels.cols; ++j) {
                int label = labels.at<int>(i, j);
                
                while (colors[label][0] == 0) {
                    colors[label][0] = std::rand()%256;
                    colors[label][1] = std::rand()%256;
                    colors[label][2] = std::rand()%256;
                }
                
                coloredLabels.at<cv::Vec3b>(i, j) = colors[label];
            }
        }
        
        char name[6];
        sprintf(name, "%05d", t);
        std::string strName = std::string(name);
        
        cv::imwrite(directory.string() + DIRECTORY_SEPARATOR + strName + ".png", 
                coloredLabels);
    }
    
    return t;
}

cv::Mat IO::readMat(boost::filesystem::path file, std::string field) {
    assert(boost::filesystem::is_regular_file(file));
    assert(!field.empty());
    
    cv::FileStorage fs(file.string(), cv::FileStorage::READ);
    
    cv::Mat mat;
    fs[field] >> mat;
    
    fs.release();
    return mat;
}

int IO::writeMat(boost::filesystem::path file, const cv::Mat& mat, std::string field) {
    assert(mat.rows > 0 && mat.cols > 0);
    assert(!field.empty());
    
    cv::FileStorage fs(file.string(), cv::FileStorage::WRITE);
    fs << field << mat;
    
    fs.release();
    return mat.rows;
}

template<typename T>
int IO::writeMatCSV(boost::filesystem::path file, const cv::Mat& mat, 
        std::string separator, int precision) {
    
    assert(mat.channels() == 1);
    assert(precision >= 0);
    assert(!separator.empty());
    
    std::ofstream file_stream(file.c_str());
    for (int i = 0; i < mat.rows; i++) {
        for (int j = 0; j < mat.cols; j++) {
            file_stream << std::setprecision(precision) << mat.at<T>(i, j);
            
            if (j < mat.cols - 1) {
                file_stream << separator;
            }
        }
        
        if (i < mat.rows  - 1) {
            file_stream << "\n";
        }
    }
    
    return mat.rows;
}

template int IO::writeMatCSV<int>(boost::filesystem::path, 
        const cv::Mat&, std::string, int);
template int IO::writeMatCSV<float>(boost::filesystem::path, 
        const cv::Mat&, std::string, int);
template int IO::writeMatCSV<unsigned char>(boost::filesystem::path, 
        const cv::Mat&, std::string, int);