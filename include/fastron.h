#include <iostream>
#include <iomanip>
#include <stdlib.h>
#include <chrono>

#include <Eigen/Core>

// Helper functions
template <class T>
Eigen::ArrayXi find(T vec);

template <class T>
void keepSelectRows(T *matPtr, Eigen::ArrayXi rowsToRetain);

template <class T>
void keepSelectCols(T *matPtr, Eigen::ArrayXi colsToRetain);

template <class T>
void keepSelectRowsCols(T *matPtr, Eigen::ArrayXi rowsToRetain, Eigen::ArrayXi colsToRetain, bool shiftOnly = false);

Eigen::MatrixXd randn(int m, int n);

// Fastron class. When using this class, use the following pipeline:
//              +----------------+    +---------------+
//     data --->+ updateLabels() +--->+ updateModel() +---> model (eval())
//              +----------------+    +---------------+
//                ^     +------------------+       |
//                +------+ activeLearning() +<-----+
//                      +------------------+
class Fastron {
    public:
        // constructor. initialize using dataset
        Fastron(Eigen::MatrixXd data);

        // destructor
        ~Fastron();

        // model update parameters: gamma (kernel width), beta (conditional bias)
        double g = 10, beta = 1;

        // max update iterations, max number of support points
        int maxUpdates = 1000, maxSupportPoints = 0;

        // Gram matrix and dataset of configurations
        Eigen::MatrixXd G, data;

        // number of datapoints and dimensionality
        int N, d;

        // weights, hypothesis, and true labels
        Eigen::ArrayXd alpha, F, y;

        // functions and variables for model update
        void updateModel();
        double calculateMarginRemoved(int *idx);
        Eigen::ArrayXi gramComputed;
        void computeGramMatrixCol(int idx, int startIdx);
        void sparsify();

        // perform proxy check
        Eigen::ArrayXd eval(Eigen::MatrixXd* query_points);

        // active learning parameters: allowance (number of new samples), kNS (number of points near supports), sigma (Gaussian sampling std), exploitP (proportion of exploitation samples)
        int allowance = 800, kNS = 4;
        double sigma, exploitP = 0.5;
        void activeLearning();

        // kinematics-based collision detector. Not defined, as its implementation is depends on robot/environment
        virtual double kcd(Eigen::RowVectorXd query_point, int colDetector);

        // update all labels in 
        void updateLabels();
        void updateLabels(int colDetector);
    private:
        // count of points with nonzero weights
        int numberSupportPoints = 0;
};