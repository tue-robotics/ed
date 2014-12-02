#ifndef ED_MASK_H_
#define ED_MASK_H_

#include <rgbd/types.h>
#include <opencv2/core/core.hpp>

#include <iostream>
#include <vector>
#include <algorithm>
#include <iterator>
#include <cassert>

namespace ed
{

class ImageMask {

public:

    ImageMask() {}

    ImageMask(int width, int height) : width_(width), height_(height)
    {
    }

    inline void setSize(int width, int height)
    {
        width_ = width;
        height_ = height;
    }

    inline void clear()
    {
        points_.clear();
    }

    inline int getSize() const { return points_.size(); }

    inline int width() const { return width_; }

    inline int height() const { return height_; }

//    inline void boundingRect(cv::Point2i& min, cv::Point2i& max)
//    {
//        min = cv::Point2i(rows_[y_min_].x_min, y_min_);
//        max = cv::Point2i(rows_[y_max_].x_max, y_max_);
//    }

    inline void addPoint(const cv::Point2i& p)
    {
         points_.push_back(p);
    }

    inline void addPoint(int x, int y)
    {
        addPoint(cv::Point2i(x, y));
    }

    inline void addPoint(int idx)
    {
        addPoint(idx % width_, idx / width_);
    }

    inline void addPoints(const std::vector<cv::Point2i>& ps)
    {
        for(std::vector<cv::Point2i>::const_iterator it = ps.begin(); it != ps.end(); ++it)
        {
            addPoint(*it);
        }
    }

    class const_iterator
    {
    public:

        const_iterator(const cv::Point2i* ptr, int factor)
            : ptr_(ptr), p_(ptr_->x * factor, ptr_->y * factor), dx_(0), dy_(0), factor_(factor)
        {
        }

        // post increment operator
        inline const_iterator operator++(int)
        {
            const_iterator i(*this);
            ++*this;
            return i;
        }

        // pre increment operator
        inline const_iterator& operator++()
        {
            ++dx_;
            if (dx_ == factor_)
            {
                dy_++;
                dx_ = 0;
            }

            if (dy_ == factor_)
            {
                dx_ = 0;
                dy_ = 0;
                ++ptr_;
            }

            p_ = cv::Point2i(ptr_->x * factor_ + dx_, ptr_->y * factor_ + dy_);

            return *this;
        }

        inline const cv::Point2i& operator*() { return p_; }
        inline const cv::Point2i* operator->() { return &p_; }

        inline bool operator==(const const_iterator& rhs) { return ptr_ == rhs.ptr_; }
        inline bool operator!=(const const_iterator& rhs) { return ptr_ != rhs.ptr_; }
    private:

        const cv::Point2i* ptr_;
        cv::Point2i p_;
        int dx_, dy_;
        int factor_;
    };

    const_iterator begin(int width = 0) const
    {
        if (width <= 0)
            return const_iterator(&points_.front(), 1);
        else if (points_.empty())
            return end();
        else
            return const_iterator(&points_.front(), width / width_);
    }

    const_iterator end() const
    {
        return const_iterator(&points_.back() + 1, 0);
    }

private:

    int width_;
    int height_;
    std::vector<cv::Point2i> points_;

};

}

#endif
