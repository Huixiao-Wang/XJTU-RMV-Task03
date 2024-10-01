#include "windmill.hpp"
#include "ceres/ceres.h"
#include "glog/logging.h"

using namespace std;
using namespace cv;

// CostFunctor y-cos(alpha)
struct CostFunctor
{
    CostFunctor(double x, double y) : x_(x), y_(y) {}

    template <typename T>
    bool operator()(const T *const A0, const T *const A, const T *const w, const T *const phi, T *residual) const
    {
        residual[0] = y_ - cos(A0[0] * x_ + A[0] / w[0] * (cos(phi[0] + 1.5707963) - cos(w[0] * x_ + phi[0] + 1.5707963)));
        return true;
    }

private:
    const double x_;
    const double y_;
};

bool checkcomb(double nowA0, double nowA, double noww, double nowphi)
{
    if (nowphi < 0.25 && 1.78 < noww && 1.23 < nowA0 && nowA < 0.83 && nowA0 < 1.37 && 0.74 < nowA && noww < 1.98 && 0.22 < nowphi)
    {
        return true;
    }
    return false;
}

int main()
{
    double t_sum = 0;
    const int N = 10;
    for (int num = 0; num < N; num++)
    {
        std::chrono::milliseconds t = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch());
        double t_start = (double)t.count();
        WINDMILL::WindMill wm(t_start);
        Mat src;

        ceres::Problem problem;
        double A0 = 0.305, A = 1.785, w = 0.884, phi = 1.24;

        // starttime
        int64 start_time = getTickCount();

        while (1)
        {
            t = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch());
            double t_now = (double)t.count();
            src = wm.getMat(t_now); // Is here wrong? (why to divide 1000?) Still, it can run well. But it is unreasonal.

            /*code*/

            // 1. draw circles
            // gray
            Mat highlightGray;
            cvtColor(src, highlightGray, COLOR_BGR2GRAY);

            // binary
            Mat binary;
            threshold(highlightGray, binary, 50, 255, THRESH_BINARY);

            /* // findcontours way 1
            vector<vector<Point>> contours;
            findContours(binary, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

            // find hammer & R
            int R_id, hammer_id;
            for (size_t i = 0; i < contours.size(); i++)
            {
                double area = contourArea(contours[i]);
                if (area < 5000)
                {
                    if (area < 200)
                    {
                        R_id = i;
                        continue;
                    }
                    hammer_id = i;
                    continue;
                }
            } */

            // findcontours way 2
            vector<vector<Point>> contours;
            vector<Vec4i> hierarchy;
            findContours(binary, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);
            int R_id = -1, hammer_id = -1;
            for (size_t i = 0; i < contours.size(); i++)
            {
                if (R_id != -1 && hammer_id != -1)
                {
                    break;
                }
                if (hierarchy[i][3] == -1)
                {
                    double area = contourArea(contours[i]);
                    if (area < 5000)
                    {
                        if (area < 200)
                        {
                            R_id = i;
                            continue;
                        }
                        hammer_id = hierarchy[i][2];
                        continue;
                    }
                }
            }

            // locate & draw
            Moments M_R = moments(contours[R_id]);
            Point R_xy(int(M_R.m10 / M_R.m00), int(M_R.m01 / M_R.m00));
            // circle(src, R_xy, 4, Scalar(255, 0, 0), -1); // draw

            Moments M_ham = moments(contours[hammer_id]);
            Point ham_xy(int(M_ham.m10 / M_ham.m00), int(M_ham.m01 / M_ham.m00));
            // circle(src, ham_xy, 4, Scalar(255, 0, 0), -1); // draw

            // calculate vector RH & cos(angle_now)
            Point RH = ham_xy - R_xy;
            Point2d RHi = Point2d(RH) / norm(RH);

            // calculate xt,yt;
            double xt = (t_now - t_start) / 1000;
            double yt = RHi.x;

            // ceres
            problem.AddResidualBlock(new ceres::AutoDiffCostFunction<CostFunctor, 1, 1, 1, 1, 1>(new CostFunctor(xt, yt)), NULL, &A0, &A, &w, &phi);

            ceres::Solver::Options options;
            options.max_num_iterations = 25;
            options.linear_solver_type = ceres::DENSE_QR;

            problem.SetParameterLowerBound(&A0, 0, 0.5);
            problem.SetParameterUpperBound(&A0, 0, 5.0);
            problem.SetParameterLowerBound(&A, 0, 0.5);
            problem.SetParameterUpperBound(&A, 0, 5.0);
            problem.SetParameterLowerBound(&w, 0, 0.5);
            problem.SetParameterUpperBound(&w, 0, 5.0);
            problem.SetParameterLowerBound(&phi, 0, 0.1);
            problem.SetParameterUpperBound(&phi, 0, 3.14);

            ceres::Solver::Summary summary;
            Solve(options, &problem, &summary);

            if (checkcomb(A0, A, w, phi))
            {
                // endtime
                int64 end_time = getTickCount();
                t_sum += (end_time - start_time) / getTickFrequency();
                break;
            }

            /*code*/
            imshow("windmill", src);
            waitKey(1);
        }
    }
    cout << t_sum / N << endl;
}