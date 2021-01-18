#ifndef SURFEL_EXTRACT_H
#define SURFEL_EXTRACT_H

#include <eigen3/Eigen/Eigen>
#include <opencv2/opencv.hpp>

#include "Surfels.h"

//#define ITERATION_NUM 2
//#define THREAD_NUM 10 // for test, 나중에 10으로 하기
//#define SP_SIZE 30
//#define MAX_ANGLE_COS 0.1

//#define HUBER_RANGE 0.4

using namespace cv;

namespace SP_SLAM
{

class Surfel;

/** \brief superpixel representation. **/
// used to extract surfel
struct Superpixel
{
    float x, y;
    float size;
    float norm_x, norm_y, norm_z;
    float posi_x, posi_y, posi_z;
    float view_cos;
    float mean_depth;
    float mean_intensity;
    bool stable;
    bool side;
    int mean_r, mean_g, mean_b;

    float cov_depth;
    float cov_color;

    // for debug
    float min_eigen_value;
    float max_eigen_value;
};


class SurfelExtractor
{
public:
    const static float DEPTH_THRES_MAX;
    const static float DEPTH_THRES_MIN;
    const static float DEPTH_COV_THRES;

    const static int ITERATION_NUM;
    const static int THREAD_NUM; // for test, 나중에 10으로 하기
    const static int SP_SIZE;
    const static float HUBER_RANGE;

    const static float RADIUS_RATIO;

    SurfelExtractor();
    SurfelExtractor(float _fx, float _fy, float _cx, float _cy);
    ~SurfelExtractor();

    void initialize(int _width, int _height);
    void initialize(int _width, int _height, float avg_depth);

    void extractSurfel(const Mat &input_image, const Mat &input_depth,
                        std::vector<Surfel>& surfels);

    // Tracking states
    enum eSurfelState{
        NOT_INITIALIZED=1,
        OK=2
    };

    eSurfelState mState;

private:

    double fx, fy, cx, cy;
    int image_width, image_height;
    int sp_width, sp_height;
    cv::Mat image, depth;

    std::vector<double> space_map; // xyz 좌표 저장
    std::vector<float> norm_map;
    std::vector<int> superpixel_index;
    std::vector<Superpixel> superpixel_seeds;
    std::vector<int> superpixel_index_merged;
    std::vector<Superpixel> superpixel_seeds_merged;

    std::vector<Surfel> *new_surfels_ptr;

    void project(float &x, float &y, float &z, float &u, float &v);
    void back_project(const float &u, const float &v, const float &depth, double&x, double&y, double&z);

    void generate_super_pixels();
    void initialize_seeds();
    void initialize_seeds_kernel(int thread_i, int thread_num);
    void update_pixels();
    void update_pixels_kernel(int thread_i, int thread_num);
    void update_seeds();
    void update_seeds_kernel(int thread_i, int thread_num);
    void calculate_norms();

    void get_huber_norm(float &nx, float &ny, float &nz, float &nb, std::vector<float> &points);

    bool calculate_cost(
        float &nodepth_cost, float &depth_cost,
        const float &pixel_intensity, const float &pixel_inverse_depth,
        const int &x, const int &y,
        const int &sp_x, const int &sp_y);

    void calculate_spaces_kernel(int thread_i, int thread_num);
    void calculate_sp_norms_kernel(int thread_i, int thread_num);
    void calculate_sp_depth_norms_kernel(int thread_i, int thread_num);
    void calculate_pixels_norms_kernel(int thread_i, int thread_num);

    // for surfel
    void initialize_surfels();
     float get_weight(float &depth);

    // for debug
    void debug_save();

    int getSurfelNum();

    int merge_surfel();

};



}//namespace SP_SLAM

#endif /* SURFEL_EXTRACT_H */
