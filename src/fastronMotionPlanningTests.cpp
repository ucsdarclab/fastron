#include <ros/ros.h>

#include "fastron.h"

#include "fcl/narrowphase/gjk.h"
#include "fcl/narrowphase/narrowphase.h"
#include <sys/stat.h>

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/collision_detection_fcl/collision_world_fcl.h>
#include <moveit/collision_detection/collision_tools.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include "interactive_robot.h"

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/bitstar/BITstar.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/util/Console.h>
#include <moveit/robot_state/conversions.h>
// planning_scene::PlanningScene *g_planning_scene = 0;
// shapes::ShapePtr g_world_cube_shape;

enum plannerTypes
{
    RRT,
    RRTCONNECT,
    RRTSTAR,
    BITSTAR
};

namespace ob = ompl::base;
namespace og = ompl::geometric;

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
        fcl::Transform3f tf1(eigen2fcltf(PlanningScenePtr->getCollisionWorld()->getWorld()->getObject("obs")->shape_poses_.front()));
        void *o1 = fcl::details::GJKInitializer<fcl::Box>::createGJKObject(S1, tf1);
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
                fcl::details::GJKInitializer<fcl::Cylinder>::deleteGJKObject(o2);
        }
        fcl::details::GJKInitializer<fcl::Cylinder>::deleteGJKObject(o1);
        delete contact_points;
        delete penetration_depth;
        delete normal;
        return -1.0;
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
}

// derive a fastron class with a redefined KCD function
class FastronKCD : public Fastron
{
  public:
    InteractiveRobot *robotPtr;
    std::vector<std::string> joint_names;
    planning_scene::PlanningScene *psPtr;
    collision_detection::AllowedCollisionMatrix *acmPtr;

    int validityMethod;
    FastronKCD(Eigen::MatrixXd input_data, InteractiveRobot *robotPtrInput, std::vector<std::string> joint_names_input, planning_scene::PlanningScene *psPtrInput, collision_detection::AllowedCollisionMatrix *acmPtrInput, int validityMethodInput) : Fastron(input_data)
    {
        robotPtr = robotPtrInput;
        joint_names = joint_names_input;
        // std::cout << "Robot group name: " << robotPtr->getGroupName() << std::endl;
        psPtr = psPtrInput;
        acmPtr = acmPtrInput;
        validityMethod = validityMethodInput;
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
    // Eigen::RowVectorXd unscaleConfiguration(Eigen::RowVectorXd query_point)
    // {
    //     Eigen::RowVectorXd unscaledQueryPoint(query_point.size());
    //     double max, min;
    //     for (int i = 0; i < query_point.size(); ++i)
    //     {
    //         max = robotPtr->robotState()->getRobotModel()->getVariableBounds(joint_names[i]).max_position_;
    //         min = robotPtr->robotState()->getRobotModel()->getVariableBounds(joint_names[i]).min_position_;
    //         unscaledQueryPoint(i) = (query_point(i) - min) / (max - min) * 2.0 - 1.0;
    //     }
    //     return unscaledQueryPoint;
    // }
    double kcd(Eigen::RowVectorXd query_point, int colDetector = 0)
    {
        return collisionDetection(*robotPtr, joint_names, scaleConfiguration(query_point), psPtr, acmPtr, colDetector);
    }
    bool isStateValid(const ob::State *state)
    {
        const auto *jointState = state->as<ob::RealVectorStateSpace::StateType>();
        Eigen::MatrixXd stateEigen(1, d);
        for (int i = 0; i < d; ++i)
            stateEigen(i) = jointState->values[i];

        switch (validityMethod)
        {
        case (0):
            return kcd(stateEigen.row(0), 0) < 0 ? true : false;
            break;
        case (1):
            return kcd(stateEigen.row(0), 1) < 0 ? true : false;
            break;
        case (2):
        default:
            return eval(&stateEigen)(0) < 0 ? true : false;
        }
    }
};

class myStateValidityCheckerClass : public ob::StateValidityChecker
{
  public:
    FastronKCD *fastron;
    myStateValidityCheckerClass(const ob::SpaceInformationPtr &si, FastronKCD *fastronInput) : ob::StateValidityChecker(si)
    {
        fastron = fastronInput;
    }
    virtual bool isValid(const ob::State *state) const
    {
        return fastron->isStateValid(state);
    }
};

struct planResults
{
    double timeTaken, timeTakenVerify, repairTime;
    bool isValid;
    bool solutionFound;
    int pathLength;
    double invalidProportion;
};

planResults plan(int d, plannerTypes plannerType, FastronKCD *fastronPtr, Eigen::ArrayXd startVector, Eigen::ArrayXd goalVector)
{
    planResults planResult;
    planResult.timeTaken = 0;
    planResult.timeTakenVerify = 0;
    planResult.repairTime = 0;
    planResult.isValid = true;
    planResult.solutionFound = true;
    planResult.pathLength = 0;

    // auto space(std::make_shared<ob::SE3StateSpace>());
    auto space(std::make_shared<ob::RealVectorStateSpace>(d));

    ob::RealVectorBounds bounds(space->getDimension());
    bounds.setLow(-1);
    bounds.setHigh(1);
    // for (int i = 0; i < d; ++i)
    // {
    //     double max = fastronPtr->robotPtr->robotState()->getRobotModel()->getVariableBounds(fastronPtr->joint_names[i]).max_position_;
    //     double min = fastronPtr->robotPtr->robotState()->getRobotModel()->getVariableBounds(fastronPtr->joint_names[i]).min_position_;
    //     std::cout << i << "," << max << "," << min << std::endl;
    // bounds.setLow(i, min);
    // bounds.setHigh(i, max);
    // }

    space->setBounds(bounds);

    // space->setLongestValidSegmentFraction(.001);

    auto si(std::make_shared<ob::SpaceInformation>(space));
    // si->setStateValidityChecker(isStateValid);
    si->setStateValidityChecker(std::make_shared<myStateValidityCheckerClass>(si, fastronPtr));
    // std::cout << "RESOLUTION " << si->getStateValidityCheckingResolution() << std::endl;
    // std::cout << si->getMaximumExtent() << std::endl;
    si->setStateValidityCheckingResolution(.001);

    ob::ScopedState<> start(space);
    for (int i = 0; i < d; ++i)
        start[i] = startVector(i);
    ob::ScopedState<> goal(space);
    for (int i = 0; i < d; ++i)
        goal[i] = goalVector(i);

    auto pdef(std::make_shared<ob::ProblemDefinition>(si));
    pdef->setStartAndGoalStates(start, goal);

    ob::Planner *planner = NULL;
    ob::OptimizationObjectivePtr obj(new ob::PathLengthOptimizationObjective(si));
    double maxSolveTime = 1.0;
    switch (plannerType)
    {
    default:
    case RRT:
        planner = new og::RRT(si);
        planner->setProblemDefinition(pdef);
        planner->as<og::RRT>()->setRange(0.006); // 1 degree max movement
        break;
    case RRTCONNECT:
        planner = new og::RRTConnect(si);
        planner->setProblemDefinition(pdef);
        planner->as<og::RRTConnect>()->setRange(0.006); // 1 degree max movement
        break;
    case RRTSTAR:
        planner = new og::RRTstar(si);
        obj->setCostThreshold(ob::Cost(5.0 * (startVector - goalVector).matrix().norm()));
        pdef->setOptimizationObjective(obj);
        planner->setProblemDefinition(pdef);
        planner->as<og::RRTstar>()->setRange(0.006); // 1 degree max movement
        maxSolveTime = 3.0;
        break;
    case BITSTAR:
        planner = new og::BITstar(si);
        planner->setProblemDefinition(pdef);
        planner->as<og::BITstar>()->setStopOnSolnImprovement(true);
        break;
    }
    planner->setup();

    clock_t t_mp;
    t_mp = clock();
    ob::PlannerStatus solved = planner->ob::Planner::solve(maxSolveTime);
    planResult.timeTaken += (clock() - (double)t_mp) / CLOCKS_PER_SEC * 1e3;

    // if (!std::strcmp(planner->getName().c_str(), "kBITstar"))
    // {
    //     std::cout << planner->as<og::BITstar>()->numIterations() << std::endl;
    //     std::cout << planner->as<og::BITstar>()->getStopOnSolnImprovement() << std::endl;
    // }
    // ob::PlannerData plannerData(si);
    // planner->getPlannerData(plannerData);
    // std::cout << fastronPtr->validityMethod << "NUM VERTICES " << plannerData.numVertices() << std::endl;

    og::PathGeometric *gpath;
    if (solved)
    {
        ob::PathPtr path = pdef->getSolutionPath();
        gpath = path->as<og::PathGeometric>();
        std::vector<ompl::base::State *> spath = gpath->getStates();
        ob::RealVectorStateSpace::StateType *jointState;

        Eigen::MatrixXd eigenPath(spath.size(), d);

        // double check validity of solution using FCL
        double invalidCount = 0;
        Eigen::ArrayXd invalidMark;
        invalidMark.setZero(spath.size());
        int tempPrevMethod = fastronPtr->validityMethod;
        fastronPtr->validityMethod = 0;
        for (int i = 0; i < spath.size(); ++i)
        {
            jointState = spath[i]->as<ob::RealVectorStateSpace::StateType>();
            for (int j = 0; j < d; ++j)
                eigenPath(i, j) = jointState->values[j];
            t_mp = clock();
            if (!fastronPtr->isStateValid(jointState))
            {
                planResult.isValid = false;
                ++invalidCount;
                invalidMark(i) = 1;
            }
            // check edge if it is longer than allowed edge length
            if (i > 0 && (eigenPath.row(i) - eigenPath.row(i - 1)).norm() > .006)
                if (!si->checkMotion(spath[i - 1], spath[i]))
                    invalidMark(i) = invalidMark(i - 1) = 1;
            planResult.timeTakenVerify += (clock() - (double)t_mp) / CLOCKS_PER_SEC * 1e3;
        }

        // std::cout << (eigenPath.row(1) - eigenPath.row(0)).norm() << std::endl;

        // if edges are not interpolated, check edges too
        // if (!std::strcmp(planner->getName().c_str(), "kBITstar"))
        // {
        //     t_mp = clock();
        //     for (int i = 0; i < spath.size() - 1; ++i)
        //         if (!si->checkMotion(spath[i], spath[i + 1]))
        //             invalidMark(i) = invalidMark(i + 1) = 1;
        //     planResult.timeTakenVerify += (clock() - (double)t_mp) / CLOCKS_PER_SEC * 1e3;
        // alternate method
        // for (int j = 0; j < spath.size() - 1; ++j)
        // {
        //     if (invalidMark(j))
        //         continue;
        //     int numSteps = (int)((eigenPath.row(j) - eigenPath.row(j + 1)).norm() / .006);
        //     t_mp = clock();
        //     for (int i = 1; i < numSteps; ++i)
        //         if (fastronPtr->kcd((1 + (double)i / numSteps) * eigenPath.row(j) - (double)i / numSteps * eigenPath.row(j + 1), 0) > 0)
        //         {
        //             invalidMark(j) = invalidMark(j + 1) = 1;
        //             break;
        //         }
        //     planResult.timeTakenVerify += (clock() - (double)t_mp) / CLOCKS_PER_SEC * 1e3;
        // }
        // }

        // begin path repair
        if (!planResult.isValid)
        {
            Eigen::ArrayXd startRepair, goalRepair;
            Eigen::ArrayXi invalidIdx = find(invalidMark);

            // std::cout << "SEGMENTS: " << (invalidMark.topRows(spath.size() - 1) - invalidMark.bottomRows(spath.size() - 1)).count() << std::endl;
            int startIdx = std::max(invalidIdx(0) - 1, 0), goalIdx = std::min(invalidIdx(invalidIdx.size() - 1) + 1, (int)(invalidIdx.size() - 1));
            startRepair = eigenPath.row(startIdx).array();
            goalRepair = eigenPath.row(goalIdx).array();
            planResults planResultRepair = plan(d, plannerType, fastronPtr, startRepair, goalRepair);
            planResult.repairTime += planResultRepair.timeTaken + planResultRepair.timeTakenVerify;

            planResult.isValid = planResultRepair.isValid;
            invalidCount = 0;
        }
        // end path repair
        fastronPtr->validityMethod = tempPrevMethod;
        planResult.invalidProportion = invalidCount / spath.size();
    }
    else
    {
        planResult.isValid = false;
        planResult.solutionFound = false;
        std::cout << "NO SOLUTION" << std::endl;
    }
    delete planner;
    return planResult;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "fastron");

    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::Publisher planning_scene_publisher = nh.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
    moveit_msgs::PlanningScene planning_scene;

    ompl::msg::noOutputHandler();

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
    obs.primitives.push_back(primitive_msg);

    geometry_msgs::Pose obs_pose_msg;
    obs_pose_msg.position.x = 0.5;
    obs_pose_msg.position.y = -0.7;
    obs_pose_msg.position.z = 0.4;
    obs_pose_msg.orientation.x = 0;
    obs_pose_msg.orientation.y = 0;
    obs_pose_msg.orientation.z = 0;
    obs_pose_msg.orientation.w = 1;
    obs.primitive_poses.push_back(obs_pose_msg);

    moveit_msgs::ObjectColor obsColor;
    obsColor.id = "obs";
    obsColor.color.r = 0.5;
    obsColor.color.g = 0.5;
    obsColor.color.b = 1.0;
    obsColor.color.a = 1.0;
    planning_scene.object_colors.push_back(obsColor);

    // additional objects
    for (int i = 1; i < 1; ++i)
    {
        obs.primitives.push_back(primitive_msg);

        // obs_pose_msg.position.x = 0.5 + 0.02*(double)std::rand()/RAND_MAX - 0.01;
        // obs_pose_msg.position.x = -0.7 + 0.02*(double)std::rand()/RAND_MAX - 0.01;
        // obs_pose_msg.position.x = 0.4 + 0.02*(double)std::rand()/RAND_MAX - 0.01;
        obs_pose_msg.position.x = (double)std::rand() / RAND_MAX;
        obs_pose_msg.position.y = 2 * (double)std::rand() / RAND_MAX - 1;
        obs_pose_msg.position.z = 2 * (double)std::rand() / RAND_MAX - 1;

        obs.primitive_poses.push_back(obs_pose_msg);
    }

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

    std::vector<std::string> l_joint_names;
    l_joint_names.push_back("left_s0");
    l_joint_names.push_back("left_s1");
    l_joint_names.push_back("left_e0");
    l_joint_names.push_back("left_e1");
    l_joint_names.push_back("left_w0");
    l_joint_names.push_back("left_w1");
    l_joint_names.push_back("left_w2");

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

    Eigen::MatrixXd query_points;
    query_points = Eigen::MatrixXd::Random(N_q, d);
    ros::Rate rate(100);

    int iter = 0;
    // int dir = 1;

    Eigen::ArrayXd obsLowLim(3), obsUpLim(3);
    obsLowLim << 0.4, -0.8, 0.3;
    obsUpLim << 0.7, -0.6, 0.5;
    std::string fileLoc = "~/fastronResults/" + std::to_string(d) + "DOF" + std::to_string(static_cast<long int>(std::time(0))) + "colType" + std::to_string(useGJKforFastron) + "/";
    const int dir_err = mkdir(fileLoc.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);

    const double val = 0.0;

    // for display purposes only:
    // moveit_msgs::DisplayRobotState displayRobotState;
    // moveit_msgs::RobotState robotStateMsg;
    // ros::Publisher r_arm_pub;
    // r_arm_pub = nh.advertise<moveit_msgs::DisplayRobotState>("displayRobotState", 1);
    // Eigen::RowVectorXd start_config_display_right(7), start_config_display_left(7);
    // start_config_display_right << 0, -0.4, 0, 1.3, 0, -0.5, 0;
    // start_config_display_left << 0, -0.3, 0, 1.3, 0, 0.5, 0;

    // for kernel comparison
    // Eigen::MatrixXd q1, q2;
    // double time_rq = 0;
    // double denom, output;

    while (ros::ok())
    {
        // for display purposes only:
        // planning_scene.world.collision_objects.front().primitive_poses.front().position.x = 0.55;
        // planning_scene.world.collision_objects.front().primitive_poses.front().position.y = -0.2;
        // planning_scene.world.collision_objects.front().primitive_poses.front().position.z = 0.4;

        // planning_scene_publisher.publish(planning_scene);
        // ps.setPlanningSceneMsg(planning_scene);

        // for (int i = 0; i < r_joint_names.size(); ++i)
        //     if (i < start_config_display_right.cols())
        //     {
        //         robot.robotState()->setJointPositions(r_joint_names[i], start_config_display_right.data() + i);
        //         robot.robotState()->setJointPositions(l_joint_names[i], start_config_display_left.data() + i);
        //     }

        // moveit::core::robotStateToRobotStateMsg(*robot.robotState(), displayRobotState.state);

        // r_arm_pub.publish(displayRobotState);
        // continue;

        // for kernel comparison
        // q1 = Eigen::MatrixXd::Random(5000, 10);
        // q2 = Eigen::MatrixXd::Random(5000, 10);
        // Eigen::MatrixXd K = Eigen::MatrixXd::Random(5000, 5000);
        // Eigen::ArrayXd r2;
        // clock_t t_rq;
        // t_rq = clock();

        // for (int i = 0; i < 5000; ++i)
        // {
        //     // r2.setOnes(5000);
        //     // for (int j = 0; j < 10; ++j)
        //     //     r2 += 10 / 8 * (q1.col(j).array() - q2(i, j)).square();
        //     // r2 = r2 * r2;
        //     // r2 = r2 * r2;
        //     // K.col(i) = 1 / (r2 * r2);

        //     r2.setZero(5000);
        //     for (int j = 0; j < 10; ++j)
        //         r2 -= 10 * (q1.col(j).array() - q2(i, j)).square();
        //     K.col(i) = r2.exp();
        // }
        // time_rq += (clock() - (double)t_rq) / CLOCKS_PER_SEC * 1e6;
        // std::cout << time_rq / (++iter) << " " << iter << std::endl;
        // continue;

        for (int trial = 0; trial < std::atof(argv[5]); ++trial)
        {
            std::cout << "Trial: " << trial << std::endl;

            // set up env
            Eigen::ArrayXd vel = Eigen::ArrayXd::Random(3);
            double speed = 0.01 * std::rand() / RAND_MAX + 0.01;
            vel = speed * vel / vel.matrix().norm();

            Eigen::ArrayXd startPos = 0.5 * (Eigen::ArrayXd::Random(3) + 1) * (obsUpLim - obsLowLim) + obsLowLim;

            planning_scene.world.collision_objects.front().primitive_poses.front().position.x = startPos(0);
            planning_scene.world.collision_objects.front().primitive_poses.front().position.y = startPos(1);
            planning_scene.world.collision_objects.front().primitive_poses.front().position.z = startPos(2);

            planning_scene_publisher.publish(planning_scene);
            ps.setPlanningSceneMsg(planning_scene);

            // Create Fastron object and motion planner
            FastronKCD fastron(data, &robot, r_joint_names, &ps, &acm, 0);
            fastron.g = std::atof(argv[1]);
            fastron.beta = std::atof(argv[2]);
            fastron.maxSupportPoints = 3000;
            fastron.maxUpdates = 5000;
            fastron.kNS = 2 * d;
            fastron.sigma = std::atof(argv[3]);
            fastron.allowance = floor(std::atof(argv[7]) * N);

            fastron.updateLabels(useGJKforFastron);
            fastron.updateModel();
            fastron.validityMethod = 2; // FCL: 0, GJK: 1, Fastron: 2

            // std::cout << plan(d, &fastron) << " ms" << std::endl;

            while (1)
            {
                // Update env
                if (planning_scene.world.collision_objects.front().primitive_poses.front().position.x < obsLowLim(0) ||
                    planning_scene.world.collision_objects.front().primitive_poses.front().position.x > obsUpLim(0) ||
                    planning_scene.world.collision_objects.front().primitive_poses.front().position.y < obsLowLim(1) ||
                    planning_scene.world.collision_objects.front().primitive_poses.front().position.y > obsUpLim(1) ||
                    planning_scene.world.collision_objects.front().primitive_poses.front().position.z < obsLowLim(2) ||
                    planning_scene.world.collision_objects.front().primitive_poses.front().position.z > obsUpLim(2))
                    break;

                planning_scene.world.collision_objects.front().primitive_poses.front().position.x += vel(0);
                planning_scene.world.collision_objects.front().primitive_poses.front().position.y += vel(1);
                planning_scene.world.collision_objects.front().primitive_poses.front().position.z += vel(2);

                planning_scene_publisher.publish(planning_scene);
                ps.setPlanningSceneMsg(planning_scene);

                // perform fastron update
                clock_t t_al;
                t_al = clock();
                fastron.activeLearning();
                double al_time = (clock() - (double)t_al) / CLOCKS_PER_SEC * 1e3;

                clock_t t_ul;
                t_ul = clock();
                fastron.updateLabels(useGJKforFastron);
                double ul_time = (clock() - (double)t_ul) / CLOCKS_PER_SEC * 1e3;

                clock_t t_mu;
                t_mu = clock();
                fastron.updateModel();
                double mu_time = (clock() - (double)t_mu) / CLOCKS_PER_SEC * 1e3;

                // perform motion planning
                Eigen::ArrayXd totalPlanTime = Eigen::ArrayXd::Zero(3);
                Eigen::ArrayXd totalPlanTimeVerify = Eigen::ArrayXd::Zero(3);
                Eigen::ArrayXd repairTime = Eigen::ArrayXd::Zero(3);
                Eigen::ArrayXd successRate = Eigen::ArrayXd::Zero(3);

                // Eigen::ArrayXd start = Eigen::ArrayXd::Random(d), goal = Eigen::ArrayXd::Random(d);
                Eigen::ArrayXd start, goal;
                Eigen::MatrixXd startMat, goalMat;
                do
                {
                    start = 0.5 * (Eigen::ArrayXd::Random(d) + 1.0);
                    startMat = start.matrix().transpose();
                } while (fastron.kcd(start, 0) > 0 || fastron.kcd(start, 1) > 0 || fastron.eval(&startMat)(0) > 0);
                do
                {
                    goal = 0.5 * (Eigen::ArrayXd::Random(d) - 1.0);
                    goalMat = goal.matrix().transpose();
                } while (fastron.kcd(goal, 0) > 0 || fastron.kcd(goal, 1) > 0 || fastron.eval(&goalMat)(0) > 0);

                int innerTrials = 10;
                for (int m = 0; m < 3; ++m)
                {
                    fastron.validityMethod = m; // FCL: 0, GJK: 1, Fastron: 2
                    for (int i = 0; i < innerTrials; ++i)
                    {
                        planResults planResult = plan(d, BITSTAR, &fastron, start, goal);
                        totalPlanTime(m) += planResult.timeTaken;
                        totalPlanTimeVerify(m) += planResult.timeTakenVerify;
                        repairTime(m) += planResult.repairTime;
                        successRate(m) += planResult.isValid ? 1 : 0;
                        // successRate(m) += planResult.isValid || planResult.invalidProportion < 0.05 ? 1 : 0;
                        // std::cout << i << std::endl;
                    }
                }
                std::cout << totalPlanTime / innerTrials << " ms" << std::endl;

                // write to file
                if (1)
                {
                    std::ofstream file;
                    file.open(fileLoc + "results_trial" + std::to_string(trial) + ".txt", std::ios_base::app);

                    file << (totalPlanTime / innerTrials).matrix().transpose()
                         << " " << (totalPlanTimeVerify / innerTrials).matrix().transpose()
                         << " " << (repairTime / innerTrials).matrix().transpose()
                         << " " << (successRate / innerTrials).matrix().transpose()
                         << std::endl;
                }
                std::cout << "Iteration " << iter << std::endl;
                ++iter;
                ros::spinOnce();
                rate.sleep();
            }
        }
        std::cout << "All trials done" << std::endl;
        ros::shutdown();
    }

    return 0;
}