//
// Created by g on 11/4/2022.
//
#include "iostream"
#include "chrono"
#include "v_test/estimate_pose.h"
using namespace std;
using namespace cv;
Mat K = (Mat_<double>(3, 3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1); //相机内参
// string file_1 = "/home/g/CLionProjects/slambook2/ch7/1.png";  // first image
// string file_2 = "/home/g/CLionProjects/slambook2/ch7/2.png";  // second image
// string file_3;
int Flag = 0;
typedef vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> Vector2dVector;
typedef vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> Vector3dVector;
// int main(){
//     Mat img1 = imread(file_1,0);
//     Mat img2 = imread(file_2,0);
//     if(img1.empty() || img2.empty()){
//         cout << "can not open image" << endl;
//         return -1;
//     }
//     vector<KeyPoint> keypoints_1, keypoints_2;
//     vector<DMatch> matches;
//     chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
//     estimate_pose.find_feature_matches(img1, img2, keypoints_1, keypoints_2, matches);
//     cout << "find " << matches.size() << " matches" << endl;
//     chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
//     chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
//     cout << "time used: " << time_used.count() << " seconds." << endl;
//     if (file_3.empty()&&Flag==0){
//         cout << "pose estimate by 2d-2d" << endl;
//         //triangulate to achieve 3d-2d
//          //estimate the pose
//         Mat R, t;
//         t1 = chrono::steady_clock::now();
//         estimate_pose.pose_estimation_2d2d(K,keypoints_1, keypoints_2, matches, R, t);
//         t2 = chrono::steady_clock::now();
//         cout<<"time used: "<<chrono::duration_cast<chrono::duration<double>>(t2-t1).count()<<" seconds."<<endl;

//         //evaluate the estimation and real data
//         Mat t_x =
//         (Mat_<double>(3, 3) << 0, -t.at<double>(2, 0), t.at<double>(1, 0),
//         t.at<double>(2, 0), 0, -t.at<double>(0, 0),
//         -t.at<double>(1, 0), t.at<double>(0, 0), 0);
//         cout<<"t^R= "<<endl<<t_x*R<<endl;

//         // verify the estimation
//         for(DMatch m:matches){
//             Point2d pt1 = estimate_pose.pixel2camera(keypoints_1[m.queryIdx].pt, K);
//             Mat y1 = (Mat_<double>(3, 1) << pt1.x, pt1.y, 1); //像素坐标转相机坐标 (x/z,y/z,1)
//             Point2d pt2 = estimate_pose.pixel2camera(keypoints_2[m.trainIdx].pt, K);
//             Mat y2 = (Mat_<double>(3, 1) << pt2.x, pt2.y, 1);
//             Mat d = y2.t()*t_x*R*y1; //if d=0, the two points are in the same line
//             cout<<"epipolar constraint= "<<endl<<d<<endl;}
//             cout<<"2d-2d:\nR= "<<endl<<R<<endl<<"t="<<endl<<t<<endl;
//         //triangulate to achieve depth information
//         vector<Point3d> points_3d;
//         estimate_pose.triangulation(keypoints_1, keypoints_2, matches, R, t, points_3d);
//         //verdify the triangulation
//          Mat img1_plot = img1.clone();
//          Mat img2_plot = img2.clone();
// //         for (int i = 0; i < matches.size(); i++) {
// //             // 第一个图
// //             float depth1 = points_3d[i].z;
// //             cout << "depth: " << depth1 << endl;
// //             Point2d pt1_cam = pixel2camera(keypoints_1[matches[i].queryIdx].pt, K);
//         //     cv::circle(img1_plot, keypoints_1[matches[i].queryIdx].pt, 2, get_color(depth1), 2);

//         //     // 第二个图
//         //     Mat pt2_trans = R * (Mat_<double>(3, 1) << points_3d[i].x, points_3d[i].y, points_3d[i].z) + t;
//         //     float depth2 = pt2_trans.at<double>(2, 0);
//         //     cv::circle(img2_plot, keypoints_2[matches[i].trainIdx].pt, 2, get_color(depth2), 2);
//         //}
//         // cv::imshow("img 1", img1_plot);
//         // cv::imshow("img 2", img2_plot);
//         // cv::waitKey();

//     }else{
//         cout<<"pose estimate by 3d-2d"<<endl;
//         Mat d1 = imread(file_3,IMREAD_UNCHANGED);
//         vector<Point3f> pts_3d;
//         vector<Point2f> pts_2d;
//         for(DMatch m:matches){
//             ushort d = d1.ptr<unsigned short>(int(keypoints_1[m.queryIdx].pt.y))[int(keypoints_1[m.queryIdx].pt.x)];
//             //find keypoint1 match the depth info on d1 image
//             if (d == 0)   // bad depth
//             continue;
//             float dd = d / 5000.0; // depth in meter
//             Point2d pt1 = estimate_pose.pixel2camera(keypoints_1[m.queryIdx].pt, K);
//             pts_3d.push_back(Point3f(pt1.x*dd, pt1.y*dd, dd));
//             pts_2d.push_back(keypoints_2[m.trainIdx].pt);
//         }
//         cout << "3d-2d pairs: " << pts_3d.size() << endl;
//         Mat r,t;
//         t1 = chrono::steady_clock::now();
//         solvePnP(pts_3d, pts_2d, K, Mat(), r, t);//重投影误差
//         Mat R;
//         Rodrigues(r, R); // r为旋转向量形式，用Rodrigues公式转换为矩阵
//         t2 = chrono::steady_clock::now();
//         cout<<"solvePnP time used: "<<chrono::duration_cast<chrono::duration<double>>(t2-t1).count()<<" seconds."<<endl;
//         cout << "3d-2d:\nR = " << endl << R << endl;
//         cout << "t = " << endl << t << endl;
//         //3d-2d pose estimate BA method
//         Vector2dVector pts_2d_eigen;
//         Vector3dVector pts_3d_eigen;
//         cout << "calling bundle adjustment by g2o" << endl;
//         Sophus::SE3d pose_g2o;
//         t1 = chrono::steady_clock::now();
//         estimate_pose.BA_g2o(pts_3d_eigen, pts_2d_eigen, K, pose_g2o);
//         t2 = chrono::steady_clock::now();
//         cout << "solve pnp by g2o cost time: " << chrono::duration_cast<chrono::duration<double>>(t2 - t1).count()<< " seconds." << endl;

//     }
    

//     return 0;
       
// }

//use feature matching(ORB/flow/direct) to find the essential matrix
void estimate_pose::find_feature_matches(const cv::Mat& img_1, const cv::Mat& img_2,
                          std::vector<cv::KeyPoint>& RR_keypoints_1,
                          std::vector<cv::KeyPoint>& RR_keypoints_2,
                          std::vector<cv::DMatch>& matches){
    //-- init
    vector<KeyPoint> keypoints_1,keypoints_2;
    Mat descriptors_1, descriptors_2;
    //use orb to find the keypoints and descriptors
    Ptr<FeatureDetector> detector = ORB::create();
    Ptr<DescriptorExtractor> descriptor = ORB::create();
    Ptr<DescriptorMatcher> matcher  = DescriptorMatcher::create("BruteForce-Hamming");
    //FAST
    detector->detect(img_1, keypoints_1);
    detector->detect(img_2, keypoints_2);
    //compute the descriptors
    descriptor->compute(img_1, keypoints_1, descriptors_1);
    descriptor->compute(img_2, keypoints_2, descriptors_2);
    //match the descriptors
    vector<DMatch> match;
    matcher->match(descriptors_1, descriptors_2, match);
    //find the good matches (define the max and min distance)
    // double min_dist=10000, max_dist=0;
    // for(int i = 0; i < descriptors_1.rows; i++){
    //     double dist = match[i].distance;
    //     if(dist < min_dist) min_dist = dist;
    //     if(dist > max_dist) max_dist = dist;
    // }
    // cout<<"-- Max dist : "<< max_dist<<endl;
    // cout<<"-- Min dist : "<< min_dist<<endl;
    // //matches that are not too far away
    // for(int i = 0; i < descriptors_1.rows; i++){
    //     if(match[i].distance <= max(2*min_dist, 30.0)){
    //         matches.push_back(match[i]);
    //     }
    // }
    //use ransac to erase the outliers
    //convert keypoints to point2f type
    vector<KeyPoint> R_keypoints_1,R_keypoints_2;
    for (size_t i=0;i<match.size();i++)   
    {
        R_keypoints_1.push_back(keypoints_1[match[i].queryIdx]);
        R_keypoints_2.push_back(keypoints_2[match[i].trainIdx]);
    }
    vector<Point2f> points_1, points_2;
    for (size_t i=0;i<match.size();i++)
    {
        points_1.push_back(R_keypoints_1[i].pt);
        points_2.push_back(R_keypoints_2[i].pt);
    }
    vector<uchar> RansacStatus;
    Mat Fundamental= cv::findFundamentalMat(points_1,points_2,RansacStatus,FM_RANSAC);
    //erase the outliers
    int index=0;
    for (size_t i=0;i<match.size();i++)
    {
        if (RansacStatus[i]!=0)
        {
            RR_keypoints_1.push_back(KeyPoint(points_1[i], 1.f));
            RR_keypoints_2.push_back(KeyPoint(points_2[i], 1.f));
            match[i].queryIdx=index;
            match[i].trainIdx=index;
            matches.push_back(match[i]);
            index++;
        }
    }

}

void estimate_pose::find_feature_flow(const cv::Mat& img_1, const cv::Mat& img_2,
                            std::vector<cv::Point2f>& pt1,
                            std::vector<cv::Point2f>& pt2,
                            std::vector<uchar> status){
    //init
    vector<KeyPoint> keypoint_1;
    // Ptr<GFTTDetector> detector_gftt = GFTTDetector::create(500, 0.01, 1, 3, false, 0.04);
    // detector_gftt->detect(img_1, keypoint_1);
    Ptr<FeatureDetector> detector = ORB::create();
    detector->detect(img_1, keypoint_1);
    //compute the optical flow
    for(int i = 0; i < keypoint_1.size(); i++){
        pt1.push_back(keypoint_1[i].pt);}
    vector<float> error;
    //cv::calcOpticalFlowPyrLK(img_1, img_2, pt1, pt2, status, error);
    std::vector<uchar> status_tmp;
    cv::calcOpticalFlowPyrLK(img_1, img_2, pt1, pt2, status_tmp, error);
    reduceVector(pt1,status_tmp);
    reduceVector(pt2,status_tmp);
    cv::findFundamentalMat(pt1,pt2, cv::FM_RANSAC, 1, 0.99, status);
    reduceVector(pt1, status);
    reduceVector(pt2, status);
}

void estimate_pose::reduceVector(vector<cv::Point2f> &v, vector<uchar> status) {
  int j = 0;
  for (int i = 0; i < int(v.size()); i++)
    if (status[i])
      v[j++] = v[i];
  v.resize(j);
}

Point2d estimate_pose::pixel2camera ( const Point2d& p, const Mat& K )
{
    return Point2d
            (
                    ( p.x - K.at<double> ( 0,2 ) ) / K.at<double> ( 0,0 ),
                    ( p.y - K.at<double> ( 1,2 ) ) / K.at<double> ( 1,1 )
            );
}

void estimate_pose::pose_estimation_2d2d(const cv::Mat &K,
                          std::vector<cv::KeyPoint> keypoints_1,
                          std::vector<cv::KeyPoint> keypoints_2,
                          std::vector<cv::DMatch> matches, cv::Mat& R, cv::Mat& t){
    //find the corresponding points
    vector<cv::Point2f> points1, points2;
    for(int i = 0; i < (int)matches.size(); i++){
        points1.push_back(keypoints_1[matches[i].queryIdx].pt);
        points2.push_back(keypoints_2[matches[i].trainIdx].pt);
    }
    //calculate the fundamental matrix
    Mat fundamental_matrix;
    fundamental_matrix = findFundamentalMat(points1, points2, 2);
    cout<<"fundamental matrix= "<<endl<<fundamental_matrix<<endl;
    //calculate the essential matrix
    Mat essential_matrix = findEssentialMat(points1, points2, K, RANSAC, 0.999, 1.0, noArray());
    //LMeds本质上还是使用了全部点，如果离群点太多，是会影响到结果的，相对来说，点集足够的情况下，RANSAC更好
    // Point2d principal_point(325.1, 249.7);  //相机光心, TUM dataset标定值
    // double focal_length = 521;      //相机焦距, TUM dataset标定值
    // Mat essential_matrix = findEssentialMat(points1, points2, focal_length, principal_point);
    cout<<"essential matrix= "<<endl<<essential_matrix<<endl;
    //calculate homography matrix
    Mat homography_matrix = findHomography(points1, points2, RANSAC, 3);
    //find the rotation and translation
    recoverPose(essential_matrix, points1, points2,K, R, t );//只是恢复移动的“结构性”，并不是真实值。
}

void estimate_pose::triangulation(
  const vector<KeyPoint> &keypoint_1,
  const vector<KeyPoint> &keypoint_2,
  const std::vector<DMatch> &matches,
  const Mat &R, const Mat &t,
  vector<Point3d> &points){
  Mat T1 = (Mat_<double>(3,4) <<
    1,0,0,0,
    0,1,0,0,
    0,0,1,0);
    Mat T2 = (Mat_<double>(3,4) <<
    R.at<double>(0,0),R.at<double>(0,1),R.at<double>(0,2),t.at<double>(0,0),
    R.at<double>(1,0),R.at<double>(1,1),R.at<double>(1,2),t.at<double>(1,0),
    R.at<double>(2,0),R.at<double>(2,1),R.at<double>(2,2),t.at<double>(2,0));
    vector<Point2f> pts_1, pts_2;
    for (DMatch m:matches) {
    // save the camera coordinates
    pts_1.push_back(pixel2camera(keypoint_1[m.queryIdx].pt, K));
    pts_2.push_back(pixel2camera(keypoint_2[m.trainIdx].pt, K));}

    Mat pts_4d;
    cv::triangulatePoints(T1, T2, pts_1, pts_2, pts_4d);//pts_4d=(x,y,z,w)
    // convert to (x/w,y/w,z/w)
    for(int i=0;i<pts_4d.cols;i++){
        Mat x = pts_4d.col(i);
        x /= x.at<float>(3,0);
        points.push_back(Point3d(x.at<float>(0,0),x.at<float>(1,0),x.at<float>(2,0)));
    }
    //The point at the centre of the image cannot be triangulated (no visual error) 
    //although there is displacement when the camera advances
  }

//g2o ba 优化 3d-2d result
class VertexPose : public g2o::BaseVertex<6, Sophus::SE3d>{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    virtual void setToOriginImpl() override{
        _estimate = Sophus::SE3d();
    }
    virtual void oplusImpl(const double* update) override{
        Eigen::Matrix<double,6,1> update_eigen;
        update_eigen << update[0], update[1], update[2], update[3], update[4], update[5];
        _estimate = Sophus::SE3d::exp(update_eigen) * _estimate;
}
    virtual bool read(istream& in) override{}
   
    virtual bool write(ostream& out) const override{}   
};


class EdgeProjection : public g2o::BaseUnaryEdge<2, Eigen::Vector2d, VertexPose> {
  public:
   EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

   EdgeProjection(const Eigen::Vector3d& point, const Eigen::Matrix3d &K) : _pos3d(point) ,_K(K){}  
   virtual void computeError() override {
    const VertexPose *v = static_cast<VertexPose *> (_vertices[0]);
    Sophus::SE3d T = v->estimate();
    Eigen::Vector3d pos_pixel = _K * (T * _pos3d);
    pos_pixel /= pos_pixel[2];
    _error = _measurement - pos_pixel.head<2>();
  }


virtual void linearizeOplus() override {
    const VertexPose *v = static_cast<VertexPose *> (_vertices[0]);
    Sophus::SE3d T = v->estimate();
    Eigen::Vector3d pos_cam = T * _pos3d;
    double fx = _K(0, 0);
    double fy = _K(1, 1);
    double cx = _K(0, 2);
    double cy = _K(1, 2);
    double X = pos_cam[0];
    double Y = pos_cam[1];
    double Z = pos_cam[2];
    double Z2 = Z * Z;
    _jacobianOplusXi
      << -fx / Z, 0, fx * X / Z2, fx * X * Y / Z2, -fx - fx * X * X / Z2, fx * Y / Z,
      0, -fy / Z, fy * Y / (Z * Z), fy + fy * Y * Y / Z2, -fy * X * Y / Z2, -fy * X / Z;
  }

  virtual bool read(istream &in) override {}

  virtual bool write(ostream &out) const override {}

private:
  Eigen::Vector3d _pos3d;
  Eigen::Matrix3d _K;
};

//一般在特征点投影完成一次的时候就要进行一次优化，优化是为了将匹配的误差最小化，进而剔除一些outlier点。
void estimate_pose::BA_g2o(const Vector3dVector &points_3d, const Vector2dVector &points_2d,const Mat &K,Sophus::SE3d &pose){
    //g2o
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<6, 3>> BlockSolverType;
    typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType> LinearSolverType;
    auto solver = new g2o::OptimizationAlgorithmGaussNewton(//can also choose other algorithm
        g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>()));
    g2o::SparseOptimizer optimizer;     // graph optimizer model
    optimizer.setAlgorithm(solver);   // set solver
    optimizer.setVerbose(true);
    Eigen::Matrix3d K_eigen;
    K_eigen <<
          K.at<double>(0, 0), K.at<double>(0, 1), K.at<double>(0, 2),
    K.at<double>(1, 0), K.at<double>(1, 1), K.at<double>(1, 2),
    K.at<double>(2, 0), K.at<double>(2, 1), K.at<double>(2, 2);
    //vertex (pose)
    VertexPose *v_pose = new VertexPose();
    v_pose->setId(0);
    v_pose->setEstimate(pose);
    optimizer.addVertex(v_pose);
    //edge
    int index = 1;
    for(size_t i=0;i<points_3d.size();i++){
        EdgeProjection *edge = new EdgeProjection(points_3d[i],K_eigen);
        edge->setId(index);
        edge->setVertex(0, v_pose);
        edge->setMeasurement(points_2d[i]);
        edge->setInformation(Eigen::Matrix2d::Identity());
        optimizer.addEdge(edge);
        index++;
    }
    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    optimizer.setVerbose(true);
    optimizer.initializeOptimization();
    optimizer.optimize(10);
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    cout<<"optimization costs time: "<<chrono::duration_cast<chrono::duration<double>>(t2-t1).count()<<endl;
    cout << "pose estimated by g2o =\n" << v_pose->estimate().matrix() << endl;
    pose = v_pose->estimate();
}

cv::Mat estimate_pose::visualizeMatches(const Mat &img_1, const Mat &img_2, 
                          std::vector<cv::KeyPoint> keypoints_1,
                          std::vector<cv::KeyPoint> keypoints_2,
                          std::vector<cv::DMatch> matches){
    Mat img_match;
    //drawKeypoints(img_1,keypoints_1,outimg1,Scalar::all(-1),DrawMatchesFlags::DEFAULT);
    drawMatches(img_1,keypoints_1,img_2,keypoints_2,matches,img_match);
    return img_match;
}

cv::Mat estimate_pose::visualizeOpticalFlow(const Mat &img_2,
                           std::vector<cv::Point2f>& pt1,
                           std::vector<cv::Point2f>& pt2,
                           vector<uchar> status){
    Mat img_flow = img_2.clone();
    //cv::cvtColor(img_flow, img_flow, cv::COLOR_GRAY2BGR);
    //we convert the color to gray, because we only need the flow
    for (int i = 0; i < pt2.size(); ++i) {
    //     if (status[i]==1) {
    //         cout<<pt1[i]<<" "<<pt2[i]<<endl;
            cv::circle(img_flow, pt2[i], 3, cv::Scalar(0, 250, 0), -1);
            cv::line(img_flow, pt1[i], pt2[i], cv::Scalar(0, 250, 0));
    //     }
    }
    return img_flow;
}