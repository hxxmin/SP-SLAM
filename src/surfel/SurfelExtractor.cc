#include <chrono>
#include <thread>
#include <cmath>
#include "surfel/SurfelExtractor.h"
#include "surfel/SurfelMatcher.h"

namespace SP_SLAM
{
const float SurfelExtractor::DEPTH_THRES_MAX = 10.0;
const float SurfelExtractor::DEPTH_THRES_MIN = 1.0;
const float SurfelExtractor::DEPTH_COV_THRES= 0.0003;
const int SurfelExtractor::SP_SIZE= 20; 

const int SurfelExtractor::ITERATION_NUM= 2;
const int SurfelExtractor::THREAD_NUM= 3;
const float SurfelExtractor::HUBER_RANGE= 0.05;

const float SurfelExtractor::RADIUS_RATIO= 0.8;

/*
 * Constructor. Nothing is done here.
 */
bool sort_surfel(Surfel a, Surfel b)
{
    return a.radius < b.radius;
}
SurfelExtractor::SurfelExtractor()
    :mState(NOT_INITIALIZED)
{

}

SurfelExtractor::SurfelExtractor(float _fx, float _fy, float _cx, float _cy)
    :mState(NOT_INITIALIZED)
{
    fx = _fx;
    fy = _fy;
    cx = _cx;
    cy = _cy;
}

SurfelExtractor::~SurfelExtractor()
{

}

void SurfelExtractor::initialize(int _width, int _height)
{
    image_width = _width;
    image_height = _height;

    // need to change superpixel size depending on image depth..
    sp_width = image_width / SP_SIZE;
    sp_height = image_height / SP_SIZE;

    superpixel_seeds.resize(sp_width * sp_height);
    superpixel_index.resize(image_width * image_height);
    space_map.resize(image_width * image_height * 3);
    norm_map.resize(image_width * image_height * 3);

    mState = OK;
}


void SurfelExtractor::initialize(int _width, int _height, float avg_depth)
{
    image_width = _width;
    image_height = _height;

    // need to change superpixel size depending on image depth..
    sp_width = image_width / SP_SIZE;
    sp_height = image_height / SP_SIZE;

    superpixel_seeds.resize(sp_width * sp_height);
    superpixel_index.resize(image_width * image_height);
    space_map.resize(image_width * image_height * 3);
    norm_map.resize(image_width * image_height * 3);

    mState = OK;
}

void SurfelExtractor::project(float &x, float &y, float &z, float &u, float &v)
{
    u = x * fx / z + cx;
    v = y * fy / z + cy;
}

void SurfelExtractor::back_project(
    const float &u, const float &v, const float &depth, double &x, double &y, double &z)
{
    x = (u - cx) / fx * depth;
    y = (v - cy) / fy * depth;
    z = depth;
}
