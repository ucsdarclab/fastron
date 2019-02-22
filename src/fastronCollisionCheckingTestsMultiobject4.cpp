#include <ros/ros.h>

#include "fastron.h"
// #include "fcl/narrowphase/gjk_libccd.h"
#include "fcl/narrowphase/gjk.h"
#include "fcl/narrowphase/narrowphase.h"
#include <sys/stat.h>
#include <moveit/robot_state/conversions.h>

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/collision_detection_fcl/collision_world_fcl.h>
#include <moveit/collision_detection/collision_tools.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include "interactive_robot.h"

// #include <ccd/ccd.h>
// #include <ccd/quat.h>

// #include <bullet/BulletCollision/NarrowPhaseCollision/btGjkEpa3.h>
// #include "bullet/BulletCollision/NarrowPhaseCollision/btVoronoiSimplexSolver.h"
// #include "bullet/BulletCollision/CollisionShapes/btBoxShape.h"
// #include "bullet/BulletCollision/NarrowPhaseCollision/btGjkPairDetector.h"
// #include <bullet/BulletCollision/NarrowPhaseCollision/btDiscreteCollisionDetectorInterface.h>
// #include "bullet/BulletCollision/NarrowPhaseCollision/btPointCollector.h"
// #include "bullet/BulletCollision/NarrowPhaseCollision/btConvexPenetrationDepthSolver.h"
// #include "bullet/BulletCollision/NarrowPhaseCollision/btGjkEpaPenetrationDepthSolver.h"
// #include "bullet/LinearMath/btTransformUtil.h"

// planning_scene::PlanningScene *g_planning_scene = 0;
// shapes::ShapePtr g_world_cube_shape;

fcl::Transform3f eigen2fcltf(Eigen::Affine3d tf)
{
    fcl::Matrix3f R;
    fcl::Vec3f T;
    R.setValue(tf(0, 0), tf(0, 1), tf(0, 2), tf(1, 0), tf(1, 1), tf(1, 2), tf(2, 0), tf(2, 1), tf(2, 2));
    T.setValue(tf(0, 3), tf(1, 3), tf(2, 3));
    fcl::Transform3f tf2(R, T);
    return tf2;
}

int collisionDetection(InteractiveRobot &robot, std::vector<std::string> joint_names, Eigen::RowVectorXd query_point, planning_scene::PlanningScene *PlanningScenePtr, collision_detection::AllowedCollisionMatrix *acmPtr, int colDetector = 0)
{
    double joint_value = 0.0;
    for (int i = 0; i < joint_names.size(); ++i)
        if (i < query_point.cols())
            robot.robotState()->setJointPositions(joint_names[i], query_point.data() + i);
        else
            robot.robotState()->setJointPositions(joint_names[i], &joint_value);

    // GJK 1
    if (colDetector)
    {
        fcl::Vec3f *contact_points;
        fcl::FCL_REAL *penetration_depth;
        fcl::Vec3f *normal;

        fcl::Box S1(0.2, 0.2, 0.2);
        robot.robotState()->update(true);
        for (int j = 0; j < joint_names.size(); ++j)
        {
            std::string childName = robot.robotState()->getJointModel(joint_names[j])->getChildLinkModel()->getName();
            Eigen::Affine3d tf = robot.robotState()->getFrameTransform(childName) * robot.robotState()->getLinkModel(childName)->getCollisionOriginTransforms().front();

            std::shared_ptr<const shapes::Cylinder> cyl = std::dynamic_pointer_cast<const shapes::Cylinder>(robot.robotState()->getLinkModel(childName)->getShapes().front());

            // pad cylinders by multiplying by a constant
            fcl::Cylinder S2(1.2 * cyl->radius, cyl->length);
            fcl::Transform3f tf2(eigen2fcltf(tf));
            void *o2 = fcl::details::GJKInitializer<fcl::Cylinder>::createGJKObject(S2, tf2);

            for (int obs = 0; obs < PlanningScenePtr->getCollisionWorld()->getWorld()->getObject("obs")->shape_poses_.size(); ++obs)
            {
                fcl::Transform3f tf1(eigen2fcltf(PlanningScenePtr->getCollisionWorld()->getWorld()->getObject("obs")->shape_poses_[obs]));
                void *o1 = fcl::details::GJKInitializer<fcl::Box>::createGJKObject(S1, tf1);
                if (fcl::details::GJKCollide(o1, fcl::details::GJKInitializer<fcl::Box>::getSupportFunction(), fcl::details::GJKInitializer<fcl::Box>::getCenterFunction(),
                                             o2, fcl::details::GJKInitializer<fcl::Cylinder>::getSupportFunction(), fcl::details::GJKInitializer<fcl::Cylinder>::getCenterFunction(),
                                             1000, 1E-6, contact_points, penetration_depth, normal))
                {
                    fcl::details::GJKInitializer<fcl::Cylinder>::deleteGJKObject(o1);
                    fcl::details::GJKInitializer<fcl::Cylinder>::deleteGJKObject(o2);
                    delete contact_points;
                    delete penetration_depth;
                    delete normal;
                    return 1.0;
                }
                else
                {
                    fcl::details::GJKInitializer<fcl::Cylinder>::deleteGJKObject(o1);
                }
            }
            fcl::details::GJKInitializer<fcl::Cylinder>::deleteGJKObject(o2);
        }
        delete contact_points;
        delete penetration_depth;
        delete normal;
        return -1.0;

        // for (int obs = 0; obs < PlanningScenePtr->getCollisionWorld()->getWorld()->find("obs")->second->shape_poses_.size(); ++obs)
        // {
        //     fcl::Box S1(0.1, 0.1, 0.1);
        //     fcl::Transform3f tf1(eigen2fcltf(PlanningScenePtr->getCollisionWorld()->getWorld()->getObject("obs")->shape_poses_[obs]));
        //     void *o1 = fcl::details::GJKInitializer<fcl::Box>::createGJKObject(S1, tf1);
        //     robot.robotState()->update(true);
        //     for (int j = 0; j < joint_names.size(); ++j)
        //     {
        //         std::string childName = robot.robotState()->getJointModel(joint_names[j])->getChildLinkModel()->getName();
        //         Eigen::Affine3d tf = robot.robotState()->getFrameTransform(childName) * robot.robotState()->getLinkModel(childName)->getCollisionOriginTransforms().front();

        //         std::shared_ptr<const shapes::Cylinder> cyl = std::dynamic_pointer_cast<const shapes::Cylinder>(robot.robotState()->getLinkModel(childName)->getShapes().front());

        //         // pad cylinders by multiplying by a constant
        //         fcl::Cylinder S2(1.2 * cyl->radius, cyl->length);
        //         fcl::Transform3f tf2(eigen2fcltf(tf));
        //         void *o2 = fcl::details::GJKInitializer<fcl::Cylinder>::createGJKObject(S2, tf2);
        //         if (fcl::details::GJKCollide(o1, fcl::details::GJKInitializer<fcl::Box>::getSupportFunction(), fcl::details::GJKInitializer<fcl::Box>::getCenterFunction(),
        //                                      o2, fcl::details::GJKInitializer<fcl::Cylinder>::getSupportFunction(), fcl::details::GJKInitializer<fcl::Cylinder>::getCenterFunction(),
        //                                      1000, 1E-6, contact_points, penetration_depth, normal))
        //         {
        //             return 1.0;
        //         }
        //     }
        // }
        // return -1.0;
    }
    // FCL 0
    else
    {
        collision_detection::CollisionRequest c_req;
        collision_detection::CollisionResult c_res;
        c_req.group_name = robot.getGroupName();

        c_res.clear();
        robot.robotState()->update(true);

        PlanningScenePtr->getCollisionWorld()->checkRobotCollision(c_req, c_res, *PlanningScenePtr->getCollisionRobot(), *robot.robotState(), *acmPtr);

        return c_res.collision ? 1.0 : -1.0;
    }

    // return query_point.cwiseAbs().maxCoeff() < 0.5 ? 1.0 : -1.0;
}

Eigen::ArrayXd collisionDetection(InteractiveRobot &robot, std::vector<std::string> joint_names, Eigen::MatrixXd query_point, planning_scene::PlanningScene *PlanningScenePtr, collision_detection::AllowedCollisionMatrix *acmPtr)
{
    Eigen::ArrayXd y;
    // y.resize(query_point.rows());
    // if (1)
    // {
    //     collision_detection::CollisionRequest c_req;
    //     collision_detection::CollisionResult c_res;
    //     c_req.group_name = robot.getGroupName();
    //     c_req.distance = false;
    //     c_req.cost = false;
    //     c_req.contacts = false;
    //     c_req.verbose = false;
    //     c_req.max_contacts = 1;
    //     c_req.max_contacts_per_pair = 1;

    //     for (int j = 0; j < query_point.rows(); ++j)
    //     {
    //         for (int i = 0; i < joint_names.size(); ++i)
    //             robot.robotState()->setJointPositions(joint_names[i], &query_point(j, i));

    //         c_res.clear();
    //         robot.robotState()->update(true);
    //         PlanningScenePtr->getCollisionWorld()->checkRobotCollision(c_req, c_res, *PlanningScenePtr->getCollisionRobot(), *robot.robotState(), *acmPtr);
    //         y(j) = c_res.collision ? 1.0 : -1.0;
    //     }
    // }
    // else
    // {
    //     robot.robotModel()->getLinkModel("test")->getShapes().front();
    // }
    return y;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "fastron");

    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::Publisher planning_scene_publisher = nh.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
    moveit_msgs::PlanningScene planning_scene;

    // derive a fastron class with a redefined KCD function
    class FastronKCD : public Fastron
    {
      public:
        InteractiveRobot *robotPtr;
        std::vector<std::string> joint_names;
        planning_scene::PlanningScene *psPtr;
        collision_detection::AllowedCollisionMatrix *acmPtr;
        FastronKCD(Eigen::MatrixXd input_data, InteractiveRobot *robotPtrInput, std::vector<std::string> joint_names_input, planning_scene::PlanningScene *psPtrInput, collision_detection::AllowedCollisionMatrix *acmPtrInput) : Fastron(input_data)
        {
            robotPtr = robotPtrInput;
            joint_names = joint_names_input;
            // std::cout << "Robot group name: " << robotPtr->getGroupName() << std::endl;
            psPtr = psPtrInput;
            acmPtr = acmPtrInput;
        }
        Eigen::RowVectorXd scaleConfiguration(Eigen::RowVectorXd query_point)
        {
            Eigen::RowVectorXd scaledQueryPoint;
            scaledQueryPoint.resize(query_point.size());
            double max, min;
            for (int i = 0; i < query_point.size(); ++i)
            {
                max = robotPtr->robotState()->getRobotModel()->getVariableBounds(joint_names[i]).max_position_;
                min = robotPtr->robotState()->getRobotModel()->getVariableBounds(joint_names[i]).min_position_;
                scaledQueryPoint(i) = (query_point(i) + 1.0) / 2.0 * (max - min) + min;
            }
            return scaledQueryPoint;
        }
        Eigen::MatrixXd scaleConfiguration(Eigen::MatrixXd query_point)
        {
            Eigen::MatrixXd scaledQueryPoint;
            scaledQueryPoint.resize(query_point.rows(), query_point.cols());
            double max, min;
            for (int i = 0; i < query_point.cols(); ++i)
            {
                max = robotPtr->robotState()->getRobotModel()->getVariableBounds(joint_names[i]).max_position_;
                min = robotPtr->robotState()->getRobotModel()->getVariableBounds(joint_names[i]).min_position_;
                scaledQueryPoint.col(i) = ((query_point.col(i).array() + 1.0) / 2.0 * (max - min) + min).matrix();
            }
            return scaledQueryPoint;
        }
        double kcd(Eigen::RowVectorXd query_point, int colDetector = 0)
        {
            return collisionDetection(*robotPtr, joint_names, scaleConfiguration(query_point), psPtr, acmPtr, colDetector);
        }
        Eigen::ArrayXd kcdMult(Eigen::MatrixXd *query_point)
        {
            return collisionDetection(*robotPtr, joint_names, scaleConfiguration(*query_point), psPtr, acmPtr);
        }
    };

    // Create object
    shape_msgs::SolidPrimitive primitive_msg;
    primitive_msg.type = primitive_msg.BOX;
    primitive_msg.dimensions.resize(3);
    primitive_msg.dimensions[0] = 0.2;
    primitive_msg.dimensions[1] = 0.2;
    primitive_msg.dimensions[2] = 0.2;

    moveit_msgs::CollisionObject obs;
    obs.id = "obs";
    obs.header.frame_id = "/world";
    obs.operation = obs.ADD;
    // obs.primitives.push_back(primitive_msg);

    geometry_msgs::Pose obs_pose_msg;
    obs_pose_msg.position.x = 0.5;
    obs_pose_msg.position.y = -0.7;
    obs_pose_msg.position.z = 0.4;
    obs_pose_msg.orientation.x = 0;
    obs_pose_msg.orientation.y = 0;
    obs_pose_msg.orientation.z = 0;
    obs_pose_msg.orientation.w = 1;
    // obs.primitive_poses.push_back(obs_pose_msg);

    moveit_msgs::ObjectColor obsColor;
    obsColor.id = "obs";
    obsColor.color.r = 0.5;
    obsColor.color.g = 0.5;
    obsColor.color.b = 1.0;
    obsColor.color.a = 1.0;
    planning_scene.object_colors.push_back(obsColor);

    // additional objects
    // int numObs = 3;

    // Set up robot
    InteractiveRobot robot;
    std::vector<std::string> r_joint_names;
    r_joint_names.push_back("right_s0");
    r_joint_names.push_back("right_s1");
    r_joint_names.push_back("right_e0");
    r_joint_names.push_back("right_e1");
    r_joint_names.push_back("right_w0");
    r_joint_names.push_back("right_w1");
    r_joint_names.push_back("right_w2");

    // Add obstacles to scene
    planning_scene.name = "scene";
    planning_scene.world.collision_objects.push_back(obs);
    planning_scene::PlanningScene ps(robot.robotModel());
    planning_scene.is_diff = true;

    // Set up collision detector
    std::vector<std::string> linkModel = robot.robotModel()->getLinkModelNamesWithCollisionGeometry();
    collision_detection::AllowedCollisionMatrix acm;
    for (int i = 0; i < linkModel.size() - 1; ++i)
    {
        bool ignoreLink = std::strncmp(linkModel[i].c_str(), "right_", strlen("right_")) && std::strncmp(linkModel[i].c_str(), "r_", strlen("r_"));
        acm.setEntry(linkModel[i], "obs", ignoreLink);
    }

    sensor_msgs::JointState js;

    for (int i = 0; i < r_joint_names.size(); ++i)
    {
        js.name.push_back(r_joint_names[i]);
        js.position.push_back(*robot.robotState()->getJointPositions(r_joint_names[i]));
    }
    planning_scene.robot_state.joint_state = js;

    // training set
    int N = 10000;
    int d = std::atof(argv[6]);
    Eigen::MatrixXd data = Eigen::MatrixXd::Random(N, d);
    int useGJKforFastron = std::atof(argv[4]);

    // test set
    int N_q = 20000;
    // Eigen::MatrixXd q;
    // q = 2.0 * Eigen::MatrixXd::Random(N_q, d) - Eigen::MatrixXd::Ones(N_q, d);
    Eigen::ArrayXd y_q = Eigen::ArrayXd::Zero(N_q, 1);
    Eigen::ArrayXd y_q_true = Eigen::ArrayXd::Zero(N_q, 1);
    Eigen::ArrayXd y_q_gjk = Eigen::ArrayXd::Zero(N_q, 1);

    planning_scene_publisher.publish(planning_scene);
    ps.setPlanningSceneMsg(planning_scene);

    usleep(1000000);

    // std::cout << "In collision samples: " << (fastron.y == 1).count() << std::endl;
    // std::cout << "Collision-free samples: " << (fastron.y == -1).count() << std::endl;

    // ros::shutdown();

    Eigen::MatrixXd query_points;
    query_points = Eigen::MatrixXd::Random(N_q, d);
    ros::Rate rate(100);

    int iter = 0;
    // int dir = 1;

    Eigen::ArrayXd obsLowLim(3), obsUpLim(3);
    obsLowLim << -0.4, -1.4, -0.9;
    obsUpLim << 1.4, 0.6, 1.3;
    std::string fileLoc = "~/fastronResults/" + std::to_string(d) + "DOF" + std::to_string(static_cast<long int>(std::time(0))) + "colType" + std::to_string(useGJKforFastron) + "sweepNumObs/";
    const int dir_err = mkdir(fileLoc.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);

    const double val = 0.0;

    moveit_msgs::DisplayRobotState displayRobotState;
    moveit_msgs::RobotState robotStateMsg;
    ros::Publisher r_arm_pub;
    r_arm_pub = nh.advertise<moveit_msgs::DisplayRobotState>("displayRobotState", 1);
    Eigen::RowVectorXd start_config_display_right(7), start_config_display_left(7);
    // start_config_display_right << std::atof(argv[10]), std::atof(argv[11]), 0, 0, 0, 0, 0;
    start_config_display_right << 0, -0.4, 0, 1.3, 0, -0.5, 0;

    while (ros::ok())
    {
        for (int numObs = std::atof(argv[8]); numObs <= std::atof(argv[9]); ++numObs)
        {
            // obs.primitives.push_back(primitive_msg);

            // obs_pose_msg.position.x = 0.5 + 0.02*(double)std::rand()/RAND_MAX - 0.01;
            // obs_pose_msg.position.x = -0.7 + 0.02*(double)std::rand()/RAND_MAX - 0.01;
            // obs_pose_msg.position.x = 0.4 + 0.02*(double)std::rand()/RAND_MAX - 0.01;
            // obs_pose_msg.position.x = (double)std::rand() / RAND_MAX;
            // obs_pose_msg.position.y = 2 * (double)std::rand() / RAND_MAX - 1;
            // obs_pose_msg.position.z = 2 * (double)std::rand() / RAND_MAX - 1;

            // obs.primitive_poses.push_back(obs_pose_msg);

            while (obs.primitive_poses.size() < numObs)
            {
                obs.primitives.push_back(primitive_msg);
                obs.primitive_poses.push_back(obs_pose_msg);
            }
            planning_scene.world.collision_objects.pop_back();
            planning_scene.world.collision_objects.push_back(obs);

            for (int trial = 0; trial < std::atof(argv[5]); ++trial)
            {
                std::cout << "Trial: " << trial << std::endl;
                Eigen::MatrixXd vel = Eigen::MatrixXd::Random(numObs, 3);
                // for (int obs = 0; obs < numObs; ++obs)
                // {
                //     // double speed = (0.01 * std::rand() / RAND_MAX + 0.01);
                //     vel.row(obs) = (0.01 * std::rand() / RAND_MAX + 0.01) * vel.row(obs) / vel.row(obs).norm();
                // }

                for (int o = 0; o < numObs; ++o)
                {
                    Eigen::ArrayXd startPos = 0.5 * (Eigen::ArrayXd::Random(3) + 1) * (obsUpLim - obsLowLim) + obsLowLim;

                    planning_scene.world.collision_objects.front().primitive_poses[o].position.x = startPos(0);
                    planning_scene.world.collision_objects.front().primitive_poses[o].position.y = startPos(1);
                    planning_scene.world.collision_objects.front().primitive_poses[o].position.z = startPos(2);

                    // double speed = (0.01 * std::rand() / RAND_MAX + 0.01);
                    vel.row(o) = (0.01 * std::rand() / RAND_MAX + 0.01) * vel.row(o) / vel.row(o).norm();
                }

                planning_scene_publisher.publish(planning_scene);
                ps.setPlanningSceneMsg(planning_scene);

                // Create Fastron object
                std::cout << "init fastron" << std::endl;
                FastronKCD fastron(data, &robot, r_joint_names, &ps, &acm);
                fastron.g = std::atof(argv[1]);
                fastron.beta = std::atof(argv[2]);
                fastron.maxSupportPoints = 3000;
                fastron.maxUpdates = 5000;
                fastron.kNS = 2 * d;
                fastron.sigma = std::atof(argv[3]);
                fastron.allowance = floor(std::atof(argv[7]) * N);

                std::cout << "updating labels (first)" << std::endl;
                fastron.updateLabels(useGJKforFastron);
                std::cout << "updating model (first)" << std::endl;
                fastron.updateModel();

                while (1)
                {
                    bool outOfBounds = false;
                    // Update env
                    std::cout << "test3" << std::endl;
                    for (int o = 0; o < numObs; ++o)
                    {
                        if (planning_scene.world.collision_objects.front().primitive_poses[o].position.x < obsLowLim(0) ||
                            planning_scene.world.collision_objects.front().primitive_poses[o].position.x > obsUpLim(0) ||
                            planning_scene.world.collision_objects.front().primitive_poses[o].position.y < obsLowLim(1) ||
                            planning_scene.world.collision_objects.front().primitive_poses[o].position.y > obsUpLim(1) ||
                            planning_scene.world.collision_objects.front().primitive_poses[o].position.z < obsLowLim(2) ||
                            planning_scene.world.collision_objects.front().primitive_poses[o].position.z > obsUpLim(2))
                        {
                            outOfBounds = true;
                            std::cout << "OOB" << std::endl;
                        }

                        planning_scene.world.collision_objects.front().primitive_poses[o].position.x += vel(o, 0);
                        planning_scene.world.collision_objects.front().primitive_poses[o].position.y += vel(o, 1);
                        planning_scene.world.collision_objects.front().primitive_poses[o].position.z += vel(o, 2);
                    }
                    if (outOfBounds)
                        break;

                    planning_scene_publisher.publish(planning_scene);
                    ps.setPlanningSceneMsg(planning_scene);

                    // perform fastron update
                    clock_t t_al;
                    std::cout << "active learning" << std::endl;
                    fastron.activeLearning();
                    double al_time = (clock() - (double)t_al) / CLOCKS_PER_SEC * 1e3;

                    std::cout << "updating labels" << std::endl;
                    fastron.updateLabels(useGJKforFastron);
                    std::cout << "updating model" << std::endl;
                    fastron.updateModel();

                    // perform testing
                    query_points = Eigen::MatrixXd::Random(N_q, d);

                    clock_t t_pcc;
                    t_pcc = clock();
                    std::cout << "fastron eval" << std::endl;
                    y_q = fastron.eval(&query_points);
                    double pcc_time = (clock() - (double)t_pcc) / CLOCKS_PER_SEC * 1e6 / N_q;

                    clock_t t_kcc;
                    t_kcc = clock();
                    // y_q_true = fastron.kcdMult(&query_points);
                    std::cout << "gjk eval" << std::endl;
                    for (int i = 0; i < N_q; ++i)
                        y_q_gjk(i) = fastron.kcd(query_points.row(i), 1);
                    double kcc_time_gjk = (clock() - (double)t_kcc) / CLOCKS_PER_SEC * 1e6 / N_q;

                    t_kcc = clock();
                    // y_q_true = fastron.kcdMult(&query_points);
                    std::cout << "fcl eval" << std::endl;
                    for (int i = 0; i < N_q; ++i)
                        y_q_true(i) = fastron.kcd(query_points.row(i), 0);
                    double kcc_time_fcl = (clock() - (double)t_kcc) / CLOCKS_PER_SEC * 1e6 / N_q;

                    // get results
                    double tp = (y_q == 1 && y_q_true == 1).count();
                    double fp = (y_q == 1 && y_q_true == -1).count();
                    double tn = (y_q == -1 && y_q_true == -1).count();
                    double fn = (y_q == -1 && y_q_true == 1).count();

                    double tp_gjk = (y_q_gjk == 1 && y_q_true == 1).count();
                    double fp_gjk = (y_q_gjk == 1 && y_q_true == -1).count();
                    double tn_gjk = (y_q_gjk == -1 && y_q_true == -1).count();
                    double fn_gjk = (y_q_gjk == -1 && y_q_true == 1).count();

                    for (int i = 0; i < r_joint_names.size(); ++i)
                        if (i < start_config_display_right.cols())
                            robot.robotState()->setJointPositions(r_joint_names[i], start_config_display_right.data() + i);

                    moveit::core::robotStateToRobotStateMsg(*robot.robotState(), displayRobotState.state);
                    r_arm_pub.publish(displayRobotState);

                    // write to file
                    if (1)
                    {
                        std::ofstream file;
                        file.open(fileLoc + "results_numObs" + std::to_string(numObs) + "_trial_" + std::to_string(trial) + ".txt", std::ios_base::app);
                        file << numObs << ","
                             << pcc_time << "," << kcc_time_gjk << "," << kcc_time_fcl << ","
                             << (y_q_true == 1).count() / (double)N_q << "," << (y_q_true == -1).count() / (double)N_q
                             << std::endl;
                    }
                    std::cout << "Iteration " << iter << std::endl;
                    ++iter;
                    ros::spinOnce();
                    rate.sleep();
                }
            }
        }
        std::cout << "All trials done" << std::endl;
        ros::shutdown();
    }

    return 0;
}