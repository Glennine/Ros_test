#include "v_test/odometry_calculation.h"
#include "iostream"
#include "chrono"
using namespace std;
using namespace cv;
typedef vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> Vector2dVector;
typedef vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> Vector3dVector;

void odometry_calculation::init(const cv::Mat &K,
                          std::vector<KeyPoint>& keypoints_1,
                          std::vector<KeyPoint>& keypoints_2,
                          std::vector<cv::DMatch>& matches,
                          Mat &R_pre, Mat &t_pre,
                          Mat &R, Mat &t,
                          vector<Point3d> &points_3d_new){
    // init Epipolar Geometry
    estimate_pose.pose_estimation_2d2d(K,keypoints_1, keypoints_2, matches, R, t);
    t = (Mat_<double>(3, 1) << 0, 1, 0);
    // init 3d points use triangulation
    estimate_pose.triangulation(keypoints_1, keypoints_2, matches, R_pre,t_pre,R, t,points_3d_new);
}





void odometry_calculation::calculate_match_pose(const cv::Mat &K,
                          std::vector<KeyPoint>& keypoints_1,
                          std::vector<KeyPoint>& keypoints_2,
                          std::vector<DMatch>& matches,
                          Mat &R_pre, Mat &t_pre,
                          Mat& R, Mat& t,
                          vector<Point3d>& points_3d,
                          vector<Point3d>& points_3d_new){
    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    vector<Point3f> pts_3d;
    vector<Point2f> pts_2d;
        for(DMatch m:matches){
            ushort d = points_3d[m.queryIdx].z;
            //find keypoint1 match the depth info on d1 image
            if (d > 0)   // bad depth
            {
            float dd = d / 5000.0; // depth in meter
            Point2d pt1 = estimate_pose.pixel2camera(keypoints_1[m.queryIdx].pt, K);
            pts_3d.push_back(Point3f(pt1.x*dd, pt1.y*dd, dd));
            pts_2d.push_back(keypoints_2[m.trainIdx].pt);}
        }
    Mat r;
    solvePnP(pts_3d, pts_2d, K, Mat(), r, t,false,cv::SOLVEPNP_EPNP);
    Rodrigues(r, R);//旋转向量转化为旋转矩阵
    //Mat pt2_trans = R*( Mat_<double>(3,1) << points_3d[i].x, points_3d[i].y, points_3d[i].z ) + t;
    estimate_pose.triangulation(keypoints_1, keypoints_2, matches, R_pre,t_pre,R, t,points_3d_new);
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();

                          }

void odometry_calculation::calculate_optial_pose(const cv::Mat &K,
                           std::vector<Point2f>& pt1,
                           std::vector<Point2f>& pt2,
                           std::vector<uchar> &status,
                           Mat &R_pre, Mat &t_pre,
                           Mat& R, Mat& t,
                           vector<Point3d>& points_3d,
                           vector<Point3d>& points_3d_new){
    //use PnP
    std::vector<Point3f> pts_3d;
    std::vector<Point2f> pts_2d;
    for (int i = 0; i < pt2.size(); i++)
    {   ushort d = points_3d[i].z;
        //if ((status[i])&&(d>0)) 
        if (d>0)
        {   float dd = d/100.0;// inital 5000
            Point2d new_pt1 = estimate_pose.pixel2camera(pt1[i], K);
            pts_3d.push_back(Point3f(new_pt1.x*dd,new_pt1.y*dd,dd));
            pts_2d.push_back(pt2[i]);
        }
    }
    Mat r;
    solvePnP(pts_3d, pts_2d, K, Mat(), r, t,false,cv::SOLVEPNP_EPNP);
    Rodrigues(r, R);//旋转向量转化为旋转矩阵
    estimate_pose.flow_triangulation(pt1,pt2,status,R_pre,t_pre,R,t,points_3d_new);

                           }

void odometry_calculation::calculate_3d_2d_pose(const cv::Mat &K,
                          std::vector<KeyPoint> keypoints_1,
                          std::vector<KeyPoint> keypoints_2,
                          std::vector<DMatch> matches,
                          Mat &R_pre, Mat &t_pre,
                          Mat& R, Mat& t){
        Vector2dVector pts_2d_eigen;
        Vector3dVector pts_3d_eigen;
        Sophus::SE3d pose_g2o;
        estimate_pose.BA_g2o(pts_3d_eigen, pts_2d_eigen, K, pose_g2o);
 
                          }