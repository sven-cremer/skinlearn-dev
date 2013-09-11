/*
 * rbdlModelTest.cpp
 *
 *  Created on: Sep 11, 2013
 *      Author: isura
 */

#include <ros/ros.h>

#include "rbdl.h"
#include "rbdl_utils.h"
#include "rbdl_urdfreader.cc"

#include <urdf/model.h>

using namespace std;

bool verbose = false;

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

int main (int argc, char *argv[])
{

        ros::init(argc, argv, "rbdl_model_test");
        ros::NodeHandle node;

        std::string urdf_param_ = "/robot_description";
		std::string urdf_string;

		urdf::Model urdf_model;

		if (!node.getParam(urdf_param_, urdf_string))
		{
		ROS_ERROR("URDF not loaded from parameter: %s)", urdf_param_.c_str());
		return false;
		}

		if (!urdf_model.initString(urdf_string))
		{
		ROS_ERROR("Failed to parse URDF file");
		return -1;
		}
		ROS_INFO("Successfully parsed URDF file");

        RigidBodyDynamics::Model model;
        model.Init();

//        urdf_string = "atlas.urdf";

        if (!RigidBodyDynamics::Addons::construct_model ( &model, &urdf_model, verbose, "torso_lift_link"))
//        if (!RigidBodyDynamics::Addons::read_urdf_model(urdf_string.c_str(), &model, verbose))
        {
                cerr << "Loading of urdf model failed!" << endl;
                return -1;
        }

        cout << "Model loading successful!" << endl;

        cout << "Degree of freedom overview:" << endl;
        cout << RigidBodyDynamics::Utils::GetModelDOFOverview(model);

        MatrixNd H = MatrixNd::Zero (model.dof_count, model.dof_count);
        VectorNd Q = VectorNd::Zero (model.dof_count);
        VectorNd QDot = VectorNd::Zero (model.dof_count);
        VectorNd Tau = VectorNd::Ones (model.dof_count);
        VectorNd QDDot = VectorNd::Zero (model.dof_count);

        // Computes the joint space inertia matrix by using the Composite Rigid Body Algorithm
        //RigidBodyDynamics::CompositeRigidBodyAlgorithm( model, Q, H, false );

        // Computes forward dynamics with the Articulated Body Algorithm
        RigidBodyDynamics::ForwardDynamics ( model, Q, QDot, Tau, QDDot );

        std::cout << "Q: "<< QDDot.transpose() << std::endl << std::endl;

        RigidBodyDynamics::InverseDynamics ( model, Q, QDot, QDDot, Tau );

        std::cout << "Tau: "<< Tau.transpose() << std::endl << std::endl;

        return 0;
}


