#ifndef BOBI_MASK_FACTORY_HPP
#define BOBI_MASK_FACTORY_HPP

#include <opencv2/opencv.hpp>

#include <string>
#include <algorithm>

namespace bobi {
    class Mask {
    public:
        virtual void roi(cv::Mat& frame) {}
        virtual void draw_roi(cv::Mat& frame, const cv::Scalar& colour) const {}
        virtual const std::string type() { return "Base"; }
        virtual cv::Point2f center() { return cv::Point2f(0, 0); }
    };

    class RectangleMask : public Mask {
    public:
        RectangleMask(int sx, int sy, int ex, int ey)
            : _roi(cv::Rect(sx, sy, ex - sx, ey - sy)),
              _center((ex + sx) / 2., (ey + sy) / 2.)
        {
        }

        void roi(cv::Mat& frame)
        {
            cv::Mat mask(frame.size(), frame.type(), cv::Scalar(0));
            mask(_roi) = cv::Scalar(255, 255, 255, 0);
            cv::bitwise_and(frame, mask, frame);
        }

        void draw_roi(cv::Mat& frame, const cv::Scalar& colour) const
        {
            cv::rectangle(frame, _roi, colour);
        }

        const std::string type() override
        {
            return "RectangleMask";
        }

        virtual cv::Point2f center() override
        {
            return _center;
        }

    protected:
        cv::Rect _roi;
        cv::Point2f _center;
    };

    class CircleMask : public Mask {
    public:
        CircleMask(int x, int y, int r)
            : _x(x), _y(y), _r(r) {}

        void roi(cv::Mat& frame)
        {
            cv::Mat mask(frame.size(), frame.type(), cv::Scalar(0));
            cv::circle(mask, cv::Point(_x, _y), _r, cv::Scalar(255, 255, 255, 0), -1);
            cv::bitwise_and(frame, mask, frame);
        }

        void draw_roi(cv::Mat& frame, const cv::Scalar& colour) const
        {
            cv::circle(frame, cv::Point(_x, _y), _r, colour, 2);
        }

        const std::string type() override
        {
            return "CircleMask";
        }

        virtual cv::Point2f center() override
        {
            return cv::Point2f(_x, _y);
        }

    protected:
        int _x;
        int _y;
        int _r;
    };

    typedef std::shared_ptr<Mask> MaskPtr;

    class MaskFactory {
    public:
        MaskPtr operator()(std::string mask_shape, const std::vector<int>& mask_specs, const cv::Size& size, const int type)
        {
            std::transform(mask_shape.begin(), mask_shape.end(), mask_shape.begin(),
                [](unsigned char c) { return std::tolower(c); });

            if (mask_shape == "rectangle") {
                return std::make_shared<RectangleMask>(RectangleMask(mask_specs[0], mask_specs[1], mask_specs[2], mask_specs[3]));
            }
            else if (mask_shape == "circle") {
                return std::make_shared<CircleMask>(CircleMask(mask_specs[0], mask_specs[1], mask_specs[2]));
            }
            else {
                return std::make_shared<Mask>(Mask());
            }
        }
    };
} // namespace bobi

#endif