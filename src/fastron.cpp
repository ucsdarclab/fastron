#include "fastron.h"

// return the indices of nonzero elements
template <class T>
Eigen::ArrayXi find(T vec)
{
    Eigen::ArrayXi idx = Eigen::ArrayXi::Zero(vec.count());
    int ii = 0;
    for (int i = 0; i < vec.size(); ++i)
        if (vec(i))
            idx(ii++) = i;
    return idx;
}

// create m x n matrix of 0-mean normal distribution samples
Eigen::MatrixXd randn(int m, int n)
{
    Eigen::MatrixXd mat = Eigen::MatrixXd::Zero(m, n);

    static std::default_random_engine generator(std::chrono::system_clock::now().time_since_epoch().count());
    std::normal_distribution<double> distribution(0, 1);

    for (int j = 0; j < n; ++j)
        for (int i = 0; i < m; ++i)
            mat(i, j) = distribution(generator);
    return mat;
}

// remove specific rows
template <class T>
void keepSelectRows(T *matPtr, Eigen::ArrayXi rowsToRetain)
{
    // Use pointer to prevent having to return a copy
    int ii = 0;
    for (int i = 0; i < rowsToRetain.size(); ++i)
        (*matPtr).row(ii++) = (*matPtr).row(rowsToRetain(i));
    (*matPtr).conservativeResize(ii, Eigen::NoChange);
}

// remove specific columns
template <class T>
void keepSelectCols(T *matPtr, Eigen::ArrayXi colsToRetain)
{
    // Use pointer to prevent having to return a copy
    int ii = 0;
    for (int i = 0; i < colsToRetain.size(); ++i)
        (*matPtr).col(ii++) = (*matPtr).col(colsToRetain(i));
    (*matPtr).conservativeResize(Eigen::NoChange, ii);
}

// remove specific rows and columns; if shiftOnly, do not resize matrix
template <class T>
void keepSelectRowsCols(T *matPtr, Eigen::ArrayXi rowsToRetain, Eigen::ArrayXi colsToRetain, bool shiftOnly)
{
    for (int j = 0; j < colsToRetain.size(); ++j)
        for (int i = 0; i < rowsToRetain.size(); ++i)
            (*matPtr)(i, j) = (*matPtr)(rowsToRetain(i), colsToRetain(j));
    if (!shiftOnly)
        (*matPtr).conservativeResize(rowsToRetain.size(), colsToRetain.size());
}

Fastron::Fastron(Eigen::MatrixXd input_data)
{
    data = input_data;

    N = data.rows();
    d = data.cols();

    // Initialization
    y = Eigen::ArrayXd::Zero(N);
    alpha = F = Eigen::VectorXd::Zero(N);
    gramComputed.setConstant(N, 0);
    G.resize(N, N);
};

Fastron::~Fastron()
{
    alpha.resize(0);
    F.resize(0);
    y.resize(0);
    gramComputed.resize(0);
    data.resize(0, 0);
    G.resize(0, 0);
}

void Fastron::computeGramMatrixCol(int idx, int startIdx = 0)
{
    Eigen::ArrayXd r2;
    r2.setOnes(N - startIdx);
    for (int j = 0; j < d; ++j)
        r2 += g / 2 * (data.block(startIdx, j, N - startIdx, 1).array() - data(idx, j)).square();
    G.block(startIdx, idx, N - startIdx, 1) = 1 / (r2 * r2);
    gramComputed(idx) = 1;
}

double Fastron::calculateMarginRemoved(int *idx)
{
    double max = 0, removed;
    for (int i = 0; i < N; ++i)
        if (alpha(i))
        {
            removed = y(i) * (F(i) - alpha(i));
            if (removed > max)
            {
                max = removed;
                *idx = i;
            }
        }
    return max;
}

void Fastron::updateModel()
{
    Eigen::ArrayXd margin = y * F;

    int idx;

    double delta, maxG;
    int NN;
    for (int i = 0; i < maxUpdates; ++i)
    {
        margin = y * F;
        if (margin.minCoeff(&idx) <= 0)
        {
            if (!gramComputed(idx))
                computeGramMatrixCol(idx);
            delta = (y(idx) < 0 ? -1.0 : beta) - F(idx);
            if (alpha(idx)) // already a support point, doesn't hurt to modify it
            {
                alpha(idx) += delta;
                F += G.block(0, idx, N, 1).array() * delta;
                continue;
            }
            else if (numberSupportPoints < maxSupportPoints) // adding new support point?
            {
                alpha(idx) = delta;
                F += G.block(0, idx, N, 1).array() * delta;
                ++numberSupportPoints;
                continue;
            }
            // else // If you reach this point, there is a need to correct a point but you can't
        }

        // Remove redundant points
        if (calculateMarginRemoved(&idx) > 0)
        {
            F -= G.block(0, idx, N, 1).array() * alpha(idx);
            alpha(idx) = 0;
            margin = y * F;
            --numberSupportPoints;
            continue;
        }

        if (numberSupportPoints == maxSupportPoints)
        {
            std::cout << "Fail: Hit support point limit in " << std::setw(4) << i << " iterations!" << std::endl;
            sparsify();
            return;
        }
        else
        {
            std::cout << "Success: Model update complete in " << std::setw(4) << i << " iterations!" << std::endl;
            sparsify();
            return;
        }
    }

    std::cout << "Failed to converge after " << maxUpdates << " iterations!" << std::endl;
    sparsify();
    return;
}

void Fastron::sparsify()
{
    Eigen::ArrayXi retainIdx = find(alpha);

    N = retainIdx.size();
    numberSupportPoints = N;

    // sparsify model
    keepSelectRows(&data, retainIdx);

    keepSelectRows(&alpha, retainIdx);
    keepSelectRows(&gramComputed, retainIdx);

    keepSelectRowsCols(&G, retainIdx, retainIdx, true);

    // sparsify arrays needed for updating
    keepSelectRows(&F, retainIdx);
    keepSelectRows(&y, retainIdx);
}

Eigen::ArrayXd Fastron::eval(Eigen::MatrixXd *query_points)
{
    // returns -1.0 for collision-free, otherwise +1.0
    Eigen::ArrayXd acc(query_points->rows());
    Eigen::ArrayXd temp(N);

    for (int i = 0; i < query_points->rows(); ++i)
    {
        // rat quad. loop unrolling is faster.
        switch (d)
        {
        case 7:
            temp = 2.0 / g + (data.col(0).array() - (*query_points)(i, 0)).square() + (data.col(1).array() - (*query_points)(i, 1)).square() + (data.col(2).array() - (*query_points)(i, 2)).square() + (data.col(3).array() - (*query_points)(i, 3)).square() + (data.col(4).array() - (*query_points)(i, 4)).square() + (data.col(5).array() - (*query_points)(i, 5)).square() + (data.col(6).array() - (*query_points)(i, 6)).square();
            break;
        case 6:
            temp = 2.0 / g + (data.col(0).array() - (*query_points)(i, 0)).square() + (data.col(1).array() - (*query_points)(i, 1)).square() + (data.col(2).array() - (*query_points)(i, 2)).square() + (data.col(3).array() - (*query_points)(i, 3)).square() + (data.col(4).array() - (*query_points)(i, 4)).square() + (data.col(5).array() - (*query_points)(i, 5)).square();
            break;
        case 5:
            temp = 2.0 / g + (data.col(0).array() - (*query_points)(i, 0)).square() + (data.col(1).array() - (*query_points)(i, 1)).square() + (data.col(2).array() - (*query_points)(i, 2)).square() + (data.col(3).array() - (*query_points)(i, 3)).square() + (data.col(4).array() - (*query_points)(i, 4)).square();
            break;
        case 4:
            temp = 2.0 / g + (data.col(0).array() - (*query_points)(i, 0)).square() + (data.col(1).array() - (*query_points)(i, 1)).square() + (data.col(2).array() - (*query_points)(i, 2)).square() + (data.col(3).array() - (*query_points)(i, 3)).square();
            break;
        default:
            temp = 2.0 / g + (data.col(0).array() - (*query_points)(i, 0)).square();
            for (int j = 1; j < d; ++j)
                temp += (data.col(j).array() - (*query_points)(i, j)).square();
        }
        acc(i) = (alpha / (temp * temp)).sum();
    }

    temp.resize(0);
    return acc.sign();
}

void Fastron::activeLearning()
{
    int N_prev = N;
    N += allowance;

    y.conservativeResize(N);
    y.tail(allowance) = 0;

    // make room for new data
    data.conservativeResize(N, Eigen::NoChange);

    // START ACTIVE LEARNING
    // copy support points as many times as possible
    int k;
    if (allowance / N_prev)
    {
        // Exploitation
        for (k = 0; k < std::min(kNS, allowance / N_prev); ++k)
            data.block((k + 1) * N_prev, 0, N_prev, d) = (data.block(0, 0, N_prev, d) + sigma * randn(N_prev, d)).cwiseMin(1.0).cwiseMax(-1.0);

        // Exploration
        data.bottomRows(allowance - k * N_prev) = Eigen::MatrixXd::Random(allowance - k * N_prev, d);
    }
    else
    {
        std::vector<int> idx;
        for (int i = 0; i < N_prev; ++i)
            idx.push_back(i);
        std::random_shuffle(idx.begin(), idx.end());

        for (int i = 0; i < allowance; i++)
            data.row(i + N_prev) = (data.row(idx[i]) + sigma * randn(1, d)).cwiseMin(1.0).cwiseMax(-1.0);
    }
    // END ACTIVE LEARNING

    // Update Gram matrix
    if (G.cols() < N)
        G.conservativeResize(N, N);
    gramComputed.conservativeResize(N);
    gramComputed.tail(allowance) = 0;

    // Update hypothesis vector and Gram matrix
    F.conservativeResize(N);
    Eigen::ArrayXi idx = find(alpha);
    for (int i = 0; i < idx.size(); ++i)
        computeGramMatrixCol(idx(i), N_prev); // this is the slowest part of this function

    F.tail(allowance) = (G.block(N_prev, 0, allowance, N_prev) * alpha.matrix()).array();

    alpha.conservativeResize(N);
    alpha.tail(allowance) = 0;
}

double Fastron::kcd(Eigen::RowVectorXd query_point, int colDetector = 0)
{
    std::cerr << "Error: No kinematics collision detector defined! Labeling as in-collision by default!" << std::endl;
    return 1;
}

void Fastron::updateLabels(int colDetector = 0)
{
    for (int i = 0; i < N; ++i)
        y(i) = kcd(data.row(i), colDetector);
}

void Fastron::updateLabels()
{
    for (int i = 0; i < N; ++i)
        y(i) = kcd(data.row(i));
}