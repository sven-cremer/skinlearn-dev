#ifndef _RBDL_URDFREADER_H
#define _RBDL_URDFREADER_H

#include <rbdl/rbdl_config.h>

namespace RigidBodyDynamics {

class Model;

typedef boost::shared_ptr<urdf::ModelInterface> ModelPtr;

namespace Addons {
	RBDL_DLLAPI bool read_urdf_model (const char* filename, Model* model, bool verbose = false);
	RBDL_DLLAPI bool construct_model (Model* rbdl_model, ModelPtr urdf_model, bool verbose);
}

}

/* _RBDL_URDFREADER_H */
#endif
