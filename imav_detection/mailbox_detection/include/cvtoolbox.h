#ifndef CVTOOLBOX_H
#define CVTOOLBOX_H

#include <opencv2/opencv.hpp>
using namespace cv;

std::vector<std::vector<cv::Point>> FindContours(const cv::Mat &binary_img) {
  std::vector<std::vector<cv::Point>> contours;
  //const auto mode = CV_RETR_TREE;
  const auto mode = CV_RETR_EXTERNAL;
  const auto method = CV_CHAIN_APPROX_SIMPLE;
  cv::findContours(binary_img, contours, mode, method);
  return contours;
}

void DrawRotatedRect(const cv::Mat &img, const cv::RotatedRect &rect, const cv::Scalar &color, int thickness, double area) {
  cv::Point2f vertex[4];

  cv::Point2f center = rect.center;
  float angle = rect.angle;
  std::ostringstream ss;
  std::ostringstream aa;
  ss << angle;
  aa << area;
  std::string text_angle(ss.str());
  std::string text_area(aa.str());
  std::string text = "angle is :" + text_angle + ", area is :" + text_area;
  int font_face = cv::FONT_HERSHEY_COMPLEX;
  double font_scale = 0.5;
  cv::putText(img, text, center, font_face, font_scale, cv::Scalar(0, 255, 255), thickness, 8, 0);

  rect.points(vertex);
  for (int i = 0; i < 4; i++)
    cv::line(img, vertex[i], vertex[(i + 1) % 4], color, thickness);
}

void DrawRect(const cv::Mat &img, cv::Rect &rect, const cv::Scalar &color, int thickness)
{
    cv::rectangle(img,rect.tl(),rect.br(),color,thickness);
}

void fillHole(const cv::Mat srcimage, cv::Mat &dstimage)
{
    Size m_Size = srcimage.size();
    Mat temimage = Mat::zeros(m_Size.height + 2, m_Size.width + 2, srcimage.type());//延展图像
    srcimage.copyTo(temimage(Range(1, m_Size.height + 1), Range(1, m_Size.width + 1)));
    floodFill(temimage, Point(0, 0), Scalar(255));
    Mat cutImg;//裁剪延展的图像
    temimage(Range(1, m_Size.height + 1), Range(1, m_Size.width + 1)).copyTo(cutImg);
    dstimage = srcimage | (~cutImg);
}

int rectArea(RotatedRect &rect)
{
    Point2f corner_point[4];
    rect.points(corner_point);
    int rect_width = floor(sqrt((corner_point[0].x-corner_point[1].x)*(corner_point[0].x-corner_point[1].x)+(corner_point[0].y-corner_point[1].y)*(corner_point[0].y-corner_point[1].y)));
    int rect_height = floor(sqrt((corner_point[1].x-corner_point[2].x)*(corner_point[1].x-corner_point[2].x)+(corner_point[1].y-corner_point[2].y)*(corner_point[1].y-corner_point[2].y)));
    int rect_area = rect_width * rect_height;
    return rect_area;
}

#endif // CVTOOLBOX_H
