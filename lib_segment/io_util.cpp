#include "io_util.h"
#include <assert.h>

std::multimap<std::string, boost::filesystem::path> IOUtil::listSubdirectories(boost::filesystem::path directory) {
    assert(boost::filesystem::is_directory(directory));
    
    std::multimap<std::string, boost::filesystem::path> directories;
    boost::filesystem::directory_iterator end;
    
    for (boost::filesystem::directory_iterator it(directory); it != end; ++it) {
        if (boost::filesystem::is_directory(it->path())) {
            directories.insert(std::multimap<std::string, boost::filesystem::path>::value_type(it->path().string(), it->path()));
        }
    }
    
    return directories;
}

std::multimap<std::string, boost::filesystem::path> IOUtil::readDirectory(boost::filesystem::path directory) {
    assert(boost::filesystem::is_directory(directory));
    
    std::multimap<std::string, boost::filesystem::path> files;
    boost::filesystem::directory_iterator end;
    
    for (boost::filesystem::directory_iterator it(directory); it != end; ++it) {
        files.insert(std::multimap<std::string, boost::filesystem::path>::value_type(it->path().string(), it->path()));
    }
    
    return files;
}

std::multimap<std::string, boost::filesystem::path> IOUtil::readDirectory(boost::filesystem::path directory, 
        std::vector<std::string> extensions) {
    
    assert(boost::filesystem::is_directory(directory));
    assert(!extensions.empty());
    
    std::multimap<std::string, boost::filesystem::path> files;
    boost::filesystem::directory_iterator end;
    
    for (boost::filesystem::directory_iterator it(directory); it != end; ++it) {
        
        // Check extensions.
        bool correct = false;
        for (std::vector<std::string>::iterator ex = extensions.begin(); 
                ex != extensions.end(); ++ex) {
            
            if (*ex == it->path().extension().string()) {
                correct = true;
            }
        }
        
        if (correct) {
            files.insert(std::multimap<std::string, boost::filesystem::path>::value_type(it->path().string(), it->path()));
        }
    }
    
    return files;
}

int IOUtil::computeSegmentNumber(const cv::Mat & sp_image) {
    assert(sp_image.rows > 0);
    assert(sp_image.cols > 0);
    assert(sp_image.channels() == 1);
    assert(sp_image.type() == CV_32S);
    
    int segmentNumber = 0;
    for (int i = 0; i < sp_image.rows; ++i) {
        for (int j = 0; j < sp_image.cols; ++j) {
            if (sp_image.at<int>(i, j) > segmentNumber) {
                segmentNumber = sp_image.at<int>(i, j);
            }
        }
    }
    return segmentNumber + 1;
}

cv::Mat IOUtil::segmentationImageToLabelMap(const cv::Mat& image) {
    assert(image.rows > 0 && image.cols > 0);
    assert(image.channels() == 3);
    
    cv::Mat labels(image.rows, image.cols, CV_32SC1, cv::Scalar(0));
    for (int i = 0; i < image.rows; ++i) {
        for (int j = 0; j < image.cols; ++j) {
            labels.at<int>(i, j) = image.at<cv::Vec3b>(i, j)[0]
                    + 256*image.at<cv::Vec3b>(i, j)[1]
                    + 256*256*image.at<cv::Vec3b>(i, j)[2];
        }
    }
    
    return labels;
}

cv::Mat IOUtil::labelMapToSegmentationImage(const cv::Mat& labels) {
    assert(labels.rows > 0 && labels.cols > 0 && labels.channels() == 1);
    
    cv::Mat image(labels.rows, labels.cols, CV_8UC3, cv::Scalar(0, 0, 0));
    for (int i = 0; i < image.rows; ++i) {
        for (int j = 0; j < image.cols; ++j) {
            image.at<cv::Vec3b>(i, j)[0] = (labels.at<int>(i, j)) % 256;
            image.at<cv::Vec3b>(i, j)[1] = ((labels.at<int>(i, j)) % 65536)/256;
            image.at<cv::Vec3b>(i, j)[2] = (labels.at<int>(i, j)) / 65536;
        }
    }
    
    return image;
}

std::vector<std::string> IOUtil::getImageExtensions() {
    std::vector<std::string> extensions;
    extensions.push_back(".png");
    extensions.push_back(".jpg");
    extensions.push_back(".jpeg");
    extensions.push_back(".bmp");
    
    return extensions;
}

std::vector<std::string> IOUtil::getTxtExtensions() {
    std::vector<std::string> extensions;
    extensions.push_back(".txt");
    
    return extensions;
}