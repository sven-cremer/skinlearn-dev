/*
 * RBDL - Rigid Body Dynamics Library
 * Copyright (c) 2011-2012 Martin Felis <martin.felis@iwr.uni-heidelberg.de>
 * Edited by Isura Ranatunga
 * 04/24/2013
 *
 * Licensed under the zlib license. See LICENSE for more details.
 */

#include <iostream>
#include <limits>
#include <cstring>
#include <assert.h>

#include "rbdl_mathutils.h"
#include "Logging.h"

#include "Model.h"
#include "Jacobian.h"

namespace RigidBodyDynamics {

using namespace Math;

void CalcJacobian (
                Model &model,
                const VectorNd &Q,
                unsigned int body_id,
                const Vector3d &point_position,
                MatrixNd &G,
                bool update_kinematics
        ) {
        LOG << "-------- " << __func__ << " --------" << std::endl;

        // update the Kinematics if necessary
        if (update_kinematics) {
                UpdateKinematicsCustom (model, &Q, NULL, NULL);
        }

        Vector3d point_base_pos = CalcBodyToBaseCoordinates (model, Q, body_id, point_position, false);
        SpatialMatrix point_trans = Xtrans_mat (point_base_pos);

        assert (G.rows() == 6 && G.cols() == model.dof_count );

        G.setZero();

        // we have to make sure that only the joints that contribute to the
        // bodies motion also get non-zero columns in the jacobian.
        // VectorNd e = VectorNd::Zero(Q.size() + 1);
        char *e = new char[Q.size() + 1];
        if (e == NULL) {
                std::cerr << "Error: allocating memory." << std::endl;
                abort();
        }
        memset (&e[0], 0, Q.size() + 1);

        unsigned int reference_body_id = body_id;

        if (model.IsFixedBodyId(body_id)) {
                unsigned int fbody_id = body_id - model.fixed_body_discriminator;
                reference_body_id = model.mFixedBodies[fbody_id].mMovableParent;
        }

        unsigned int j = reference_body_id;

        // e[j] is set to 1 if joint j contributes to the jacobian that we are
        // computing. For all other joints the column will be zero.
        while (j != 0) {
                e[j] = 1;
                j = model.lambda[j];
        }

        for (j = 1; j < model.mBodies.size(); j++) {
                if (e[j] == 1) {
                        SpatialVector S_base;
                        S_base = point_trans * spatial_inverse(model.X_base[j].toMatrix()) * model.S[j];

                        // Orientation
                        G(3, j - 1) = S_base[0];
                        G(4, j - 1) = S_base[1];
                        G(5, j - 1) = S_base[2];

                        // Position
                        G(0, j - 1) = S_base[3];
                        G(1, j - 1) = S_base[4];
                        G(2, j - 1) = S_base[5];
                }
        }

        delete[] e;
}


}
