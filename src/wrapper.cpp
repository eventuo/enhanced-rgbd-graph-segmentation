/*
  Wrap the basic rgb library
*/

#include <cstdio>
#include <cstdlib>
#include <rgbd_graph_segmentation/rgbd_graph_segmentation.h>
#include <cv_bridge/cv_bridge.h>
#include <boost/foreach.hpp>
#include <map>
#include <ros/assert.h>
#include "stl_tools.h"
#include "felz.h"

using std::vector;
using std::pair;
using std::map;
using std::set;
using std::cerr;



namespace rgbd_graph_segmentation
{

namespace sm=sensor_msgs;
namespace f=felzenszwalb;

typedef boost::shared_ptr<f::Img> ImgPtr;
typedef cv::Mat_<cv::Vec3b> RgbImage;
typedef cv::Mat_<float> DepthImage;
typedef std::map<Pixel, uint32_t> PixelMap;
typedef vector<Pixel> Segment;
typedef map<uint32_t, Segment> SegmentMap;

// Constructor for segmentation object just sets up the reverse mapping from
// segment id to pixel list
Segmentation::Segmentation (const RgbImage& image, const PixelMap& segs) :
  segments_(boost::extents[image.rows][image.cols]), image_(image)
{
  for (const PixelMap::value_type& e : segs)
  {
    // Add mapping from pixel to segment
    segments_[e.first.first][e.first.second] = e.second;
    
    // Add reverse mapping from segment id to pixel
    pixels_[e.second].push_back(e.first); // Creates map entry if doesn't exist
    segment_ids_.insert(e.second);
  }
}

// Constructor for segmentation object just sets up the reverse mapping from
// segment id to pixel list
Segmentation::Segmentation (const RgbImage& image, const array_type& segs) :
  segments_(segs), image_(image)
{
  for (int r=0; r<image_.rows; r++)
  {
    for (int c=0; c<image_.cols; c++)
    {
      const uint32_t seg = segs[r][c];
      pixels_[seg].push_back(Pixel(r, c));
      segment_ids_.insert(seg);
    }
  }
}


// Random color
cv::Vec3b randomColor ()
{
  cv::Vec3b v;
  v[0] = rand() % 256;
  v[1] = rand() % 256;
  v[2] = rand() % 256;
  return v;
}

// An assignment of colors to segment ids used for drawing borders
// Attempts to make the colors visually distinct
cv::Vec3b colorOfSegment (const uint32_t t)
{
  static map<uint32_t, cv::Vec3b> colors;
  static unsigned r=0, g=100, b=200;
  if (!contains(colors, t))
  {
    cv::Vec3b color;
    color[0] = b; 
    color[1] = g;
    color[2] = r;
    colors[t] = color;

    r = (r+91)%256;
    g = (g+51)%256;
    b = (b+71)%256;
  }
  return colors[t];
}

// Average rgb value in a given segment
cv::Vec3b
Segmentation::segmentAverageColor (const size_t i) const
{
  int r=0, g=0, b=0;
  const Segment& seg = pixels(i);
  const size_t n = seg.size();
  BOOST_FOREACH (const Pixel& p, seg)
  {
    const cv::Vec3b& color = image_(p.first, p.second);
    r += color[2];
    g += color[1];
    b += color[0];
  }
  cv::Vec3b avg;
  avg[2] = static_cast<float>(r)/n;
  avg[1] = static_cast<float>(g)/n;
  avg[0] = static_cast<float>(b)/n;
  return avg;
}

// Central pixel of a segment
Pixel Segmentation::center (const uint32_t i) const
{
  double r=0, c=0;
  const vector<Pixel>& pix = pixels(i);
  ROS_ASSERT(pix.size()>0);
  for (const Pixel& p : pix)
  {
    r += p.first;
    c += p.second;
  }
  return Pixel(r/pix.size(), c/pix.size());
}


// Return new image in which the border of each segment is highlighted a
// different color
RgbImage Segmentation::segmentationImage () const
{
  const uint32_t dummy_segment = -1;
  map<uint32_t, cv::Vec3b> colors;
  for (const uint32_t i : segment_ids_) 
  {
    if (i != dummy_segment)
      colors[i] = segmentAverageColor(i);
  }
  
  RgbImage img(image_.rows, image_.cols);
  cv::Vec3b orange;
  orange[2] = 255;
  orange[1] = 165;
  
  for (int r=0; r<image_.rows; r++)
  {
    for (int c=0; c<image_.cols; c++)
    {
      co