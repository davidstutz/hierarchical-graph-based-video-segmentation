/**
 * Taken and adapted from: http://stackoverflow.com/questions/1120140/how-can-i-read-and-parse-csv-files-in-c
 *
 * Usage:
 *
 *  std::ifstream file_stream(file.c_str());
 *  for(CSVIterator it(file_stream, ','); it != CSVIterator(); ++it) {
 *      if (it->size() > 0) {
 *          std::cout << "First element: " << (*loop)[0] << "\n";
 *      }
 *  }
 */

#ifndef CSVITERATOR_H
#define	CSVITERATOR_H

#include <iterator>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>

class CSVRow {
public:
    
    CSVRow(char delimiter) : delimiter_(delimiter) {
        
    }
    
    CSVRow() : delimiter_(',') {
        
    }
    
    std::string const& operator[](std::size_t index) const {
        return data_[index];
    }
    
    std::size_t size() const {
        return data_.size();
    }
    
    void read(std::istream& str) {
        std::string line;
        std::getline(str,line);

        std::stringstream lineStream(line);
        std::string cell;

        data_.clear();
        while(std::getline(lineStream, cell, ',')) {
            data_.push_back(cell);
        }
    }
    
protected:
    
    std::vector<std::string> data_;
    char delimiter_;
};

inline std::istream& operator>>(std::istream& str, CSVRow& data) {
    data.read(str);
    return str;
} 

class CSVIterator {   
public:
    
    CSVIterator(std::istream& str, char delimiter) : str_(str.good() ? &str : NULL), row_(delimiter) {
        ++(*this);
    }
    
    CSVIterator(std::istream& str) : str_(str.good() ? &str : NULL) {
        
    }
    
    CSVIterator() : str_(NULL) {
        
    }

    CSVIterator& operator++() {
        if (str_) {
            (*str_) >> row_;
            str_ = str_->good() ? str_ : NULL;
        }

        return *this;
    }
    
    CSVIterator operator++(int) {
        CSVIterator tmp(*this);
        ++(*this);

        return tmp;
    }
    
    CSVRow const& operator*() const {
        return row_;
    }
    
    CSVRow const* operator->() const {
        return &row_;
    }

    bool operator==(CSVIterator const& rhs) {
        return ((this == &rhs) || ((this->str_ == NULL) && (rhs.str_ == NULL)));
    }
    
    bool operator!=(CSVIterator const& rhs) {
        return !((*this) == rhs);
    }
        
protected:
    
    std::istream* str_;
    CSVRow row_;
};

#endif	/* CSVITERATOR_H */

