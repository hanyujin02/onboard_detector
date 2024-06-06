#ifndef REGRESSION_H
#define REGRESSION_H
#include<Eigen/Eigen>
#include<iostream>


namespace onboardDetector{
    class LLS{
        public:
            ~LLS(){}
            Eigen::MatrixXd pred(const Eigen::MatrixXd &x, const Eigen::MatrixXd &y, const Eigen::MatrixXd &px, const int &order){
                int numIter = 10;
                Eigen::MatrixXd vx = vandermonde(x,order);
                Eigen::MatrixXd Im;
                Im.resize(order+1, order+1);
                Im.setIdentity();
                Im(order, order) = 0;
                double lambda = 0.01;
                Eigen::MatrixXd Xt = vx.transpose();
                Eigen::MatrixXd XtX = Xt*vx + lambda*Im;
                Eigen::MatrixXd XtXinverse = XtX.inverse();
                Eigen::MatrixXd Xty = Xt*y;
                // Eigen::MatrixXd w(order+1,1);
                // w.setRandom();
                // // for (int i=0; i<order+1; i++){
                // //     w(i,0) = rand();
                // // }
                // for (int i=0; i<numIter; i++){
                //     w -= 0.01*(2*Xt*(vx*w-y));
                // }
                Eigen::MatrixXd w = XtXinverse*Xty;
                Eigen::MatrixXd X = vandermonde(px,order);
                
                Eigen::MatrixXd predy = X*w;
                return predy;
            }
            Eigen::MatrixXd vandermonde(const Eigen::MatrixXd &x, const int &order){
                Eigen::MatrixXd X;
                X.resize(x.rows(),order+1);
                for (int j=0; j<x.rows(); j++){
                    for (int k=0; k<order+1; k++){
                        // py += ob.Vy*dt;
                        X(j,k) = pow(x(j,0),k);
                    }
                }
                return X;
            }
        };
}

#endif