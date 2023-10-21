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
// Attem