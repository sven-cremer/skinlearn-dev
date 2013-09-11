#ifndef _RBDL_URDFREADER_H
#define _RBDL_URDFREADER_H

#include <urdf/model.h>

namespace RigidBodyDynamics {

class Model;

namespace Addons {

	bool read_urdf_float_model (const char* filename, Model* model, bool verbose = false, std::string baseLink = "pelvis");
	bool read_urdf_model (const char* filename, Model* model, bool verbose = false, std::string baseLink = "pelvis");
	bool construct_float_model (Model* rbdl_model, urdf::Model *urdf_model, bool verbose = false, std::string baseLink = "pelvis");
	bool construct_model (Model* rbdl_model, urdf::Model *urdf_model, bool verbose = false, std::string baseLink = "pelvis");
}

}

/* _RBDL_URDFREADER_H */
#endif
