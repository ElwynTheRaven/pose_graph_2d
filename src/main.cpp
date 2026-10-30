#include <gtsam/geometry/Pose2.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/DoglegOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/dataset.h>
 
#include <cmath>
#include <iostream>
#include <fstream>

using namespace std;

/* 小例子
int main()
{
    gtsam::NonlinearFactorGraph graph;
    auto priorNoise = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(0.1, 0.1, 0.05));
    graph.addPrior(0,gtsam::Pose2(0.0,0.0,0.0),priorNoise); //先验因子

    gtsam::Pose2 odom_noisy(2.1, 0.05, M_PI / 2 + 0.05);

    //测量因子
    gtsam::Pose2 odom(2.0, 0.0, M_PI / 2);
    auto odomNoise = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(0.2, 0.2, 0.1));
    graph.add(gtsam::BetweenFactor<gtsam::Pose2>(0,1,odom_noisy,odomNoise));
    graph.add(gtsam::BetweenFactor<gtsam::Pose2>(1,2,odom_noisy,odomNoise));
    graph.add(gtsam::BetweenFactor<gtsam::Pose2>(2,3,odom_noisy,odomNoise));
    graph.add(gtsam::BetweenFactor<gtsam::Pose2>(3,0,odom,odomNoise)); //回环边

    //初始估计
    gtsam::Pose2 x0(0.0, 0.0, 0.0);
    gtsam::Pose2 x1 = x0.compose(odom_noisy);
    gtsam::Pose2 x2 = x1.compose(odom_noisy);
    gtsam::Pose2 x3 = x2.compose(odom_noisy);

    //存初始估计
    gtsam::Values initial;
    initial.insert(0, x0);
    initial.insert(1, x1);
    initial.insert(2, x2);
    initial.insert(3, x3);

    //LM优化
    gtsam::LevenbergMarquardtOptimizer optimizer(graph, initial);
    gtsam::Values result = optimizer.optimize();

    initial.print("初始估计:\n");
    result.print("优化结果:\n");
    std::cout << "优化前总误差: " << graph.error(initial) << std::endl;
    std::cout << "优化后总误差: " << graph.error(result) << std::endl;

    return 0;
}
*/

int main( int argc, char** argv )
{
    string path = "data/intel_uniform.g2o";
    if (argc >= 2)
    {
        path = argv[1];
    }
    
    auto [graph,initial] = gtsam::readG2o(path);
    
    auto priorNoise = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(0.1, 0.1, 0.05));
    graph->addPrior(0,initial->at <gtsam::Pose2>(0),priorNoise);

    gtsam::LevenbergMarquardtOptimizer optimizer(*graph, *initial); //创建LM优化器，给它地形 *graph 和起点 *initial
    gtsam::Values result = optimizer.optimize(); //开始优化，将结果存进 result

    cout << "优化前总误差: " << graph->error(*initial) << std::endl;
    cout << "迭代次数(LM): " << optimizer.iterations() << std::endl; //走了几步 iterations()
    cout << "优化后总误差(LM): " << graph->error(result) << std::endl;

    gtsam::GaussNewtonOptimizer optimizer_gn(*graph, *initial);
    gtsam::Values result_gn = optimizer_gn.optimize();

    cout << "迭代次数(GN): " << optimizer_gn.iterations() << std::endl;
    cout << "优化后总误差(GN): " << graph->error(result_gn) << std::endl;

    gtsam::DoglegOptimizer optimizer_dl(*graph, *initial);
    gtsam::Values result_dl = optimizer_dl.optimize();

    cout << "迭代次数(Dl): " << optimizer_dl.iterations() << std::endl;
    cout << "优化后总误差(Dl): " << graph->error(result_dl) << std::endl;

    ofstream file_init("result_init.csv"); //ofstream = output file stream（输出文件流），创建变量，括号里是文件名没有就新建（反之清空重写）
    file_init << "id,x,y,theta" << endl; //表头
    ofstream file_opt("result_opt.csv");
    file_opt << "id,x,y,theta" << endl; 

    for (gtsam::Key key : result.keys()) //遍历位姿
    {
        gtsam::Pose2 p = result.at<gtsam::Pose2>(key);
        file_opt << key << "," << p.x() << "," << p.y() << "," << p.theta() << endl; 
    }

    for (gtsam::Key key : initial->keys()) 
    {
        gtsam::Pose2 p = initial->at<gtsam::Pose2>(key);
        file_init << key << "," << p.x() << "," << p.y() << "," << p.theta() << endl; 
    }
 
    return 0;
}