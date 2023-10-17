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

namespace 