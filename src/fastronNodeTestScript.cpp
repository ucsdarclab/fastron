#include <ros/ros.h>

#include "fastron.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "fastron");
    ros::NodeHandle nh;

    double radius = 1.5;

    ros::Rate rate(2);

    // training set, y
    int N = 10000;
    int d = 7;
    Eigen::MatrixXd data = 2.0*Eigen::MatrixXd::Random(N, d) - Eigen::MatrixXd::Ones(N, d);
    // Eigen::ArrayXd y = Eigen::ArrayXd::Zero(N,1);

    // for (int i = 0 ; i < y.rows() ; ++i)
    //     y(i) = 2.0*(data.row(i).squaredNorm() < radius*radius) - 1.0;

    // test set
    int N_q = 30000;
    Eigen::MatrixXd q;
    q = 2.0*Eigen::MatrixXd::Random(N_q, d) - Eigen::MatrixXd::Ones(N_q, d);
    Eigen::ArrayXd y_q = Eigen::ArrayXd::Zero(N_q,1);
    Eigen::ArrayXd y_q_true = Eigen::ArrayXd::Zero(N_q,1);

    // instantiate Fastron object
    // Fastron fastron(data, y, 1000);

    // derive a fastron class with a redefined KCD function
    class FastronKCD : public Fastron {
        public:
            FastronKCD(Eigen::MatrixXd input_data) : Fastron(input_data) {}
            // use a hypercube as the configuration space obstacle
            double kcd(Eigen::RowVectorXd query_point)
            {
                return 2.0*(query_point.cwiseAbs().maxCoeff() < 1) - 1.0;
            }
    };

    // variables for timing testing
    double iterations = 0;
    double trial_t, total_t;

    // Create Fastron object
    FastronKCD fastron(data);

    // Fastron parameters
    fastron.g = 10;
    fastron.beta = 100;
    fastron.kNS = 3*d;

    fastron.updateLabels();
    std::cout << "Collision-free points (training set): " << (fastron.y >= 0).count() << std::endl;
    std::cout << "In-collision points (training set): " << (fastron.y < 0).count() << std::endl;
    fastron.updateModel();
    
    while (ros::ok())
    {
        /* Fastron algorithm
                 +--------------+    +-------------+
        data --->+ updateLabels +--->+ updateModel +---> model
                 +--------------+    +-------------+
                   ^     +----------------+      |
                   +-----+ activeLearning +<-----+
                         +----------------+
        */

        clock_t t;
        clock_t t_loop;
        t_loop = clock();

        // execute Fastron loop
        fastron.activeLearning();
        fastron.updateLabels();
        t = clock();
        fastron.updateModel();
        std::cout << "Update loop Time: " << (clock() - (double)t_loop)/CLOCKS_PER_SEC*1e3 << " ms" << std::endl;

        // "dynamic" env:
        // radius = 0.4*std::rand()/(double)RAND_MAX - 0.2 + 1.5;

        // Test collision check performance
        q = 1.5*Eigen::MatrixXd::Random(N_q, d) - 0.75*Eigen::MatrixXd::Ones(N_q, d);
        t = clock();
        y_q = fastron.eval(&q);
        trial_t = (clock() - (double)t)/(CLOCKS_PER_SEC*N_q)*1e6;
        std::cout << "Time per collision check: " << trial_t << " us" << std::endl;
        
        clock_t trial_t_kcd;
        trial_t_kcd = clock();
        for (int i = 0 ; i < N_q ; ++i)
            y_q_true(i) = fastron.kcd(q.row(i));
        std::cout << "Time per KCD collision check: " << (clock() - (double)trial_t_kcd)/(CLOCKS_PER_SEC*N_q)*1e6 << " us" << std::endl;

        total_t += trial_t;
        std::cout << "Avg time per collision check: " << total_t/(++iterations) << " us" << std::endl;
        std::cout << "Support points: " << fastron.alpha.count() << std::endl;
        std::cout << "Collision-free points (training set): " << (fastron.y >= 0).count() << std::endl;
        std::cout << "In-collision points (training set): " << (fastron.y < 0).count() << std::endl;
        
        double tp = (y_q == 1 && y_q_true == 1).count();
        double fp = (y_q == 1 && y_q_true == -1).count();
        double tn = (y_q == -1 && y_q_true == -1).count();
        double fn = (y_q == -1 && y_q_true == 1).count();

        std::cout << "Accuracy: " << (tp+tn)/(tp+tn+fp+fn) << std::endl;

        std::cout << "Precision: " << tp/(tp+fp) << std::endl;
        std::cout << "Recall/Sensitivity: " << tp/(tp+fn) << std::endl;
        // std::cout << tn/(fp+tn) << std::endl;
        std::cout << "Specificity/TNR: " << tn/(tn + fp) << std::endl;
        // std::cout << "FNR: " << fn/(tp + fn) << std::endl;
        std::cout << "Collision-free points (test set): " << (y_q < 0).count() << std::endl;
        std::cout << "In-collision points (test set): " << (y_q >= 0).count() << std::endl;
        std::cout << "True collision-free points (test set): " << (y_q_true < 0).count() << std::endl;
        std::cout << "True in-collision points (test set): " << (y_q_true >= 0).count() << std::endl;

        std::cout << std::endl;

        // loop
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}