/*
 * RBDL - Rigid Body Dynamics Library
 * Copyright (c) 2011-2012 Martin Felis <martin.felis@iwr.uni-heidelberg.de>
 * Edited by Isura Ranatunga
 * 04/24/2013
 *
 * Licensed under the zlib license. See LICENSE for more details.
 */

#ifndef _JACOBIAN_H
#define _JACOBIAN_H

#include <rbdl_math.h>
#include <assert.h>
#include <iostream>
#include "Logging.h"

namespace RigidBodyDynamics {

/** \defgroup kinematics_group Kinematics
 * @{
 *
 * \note Please note that in the Rigid %Body Dynamics Library all angles
 * are specified in radians.
 *
 */

/** \brief Computes the jacobian for a point on a body
 *
 * If a position of a point is computed by a function \f$g(q(t))\f$ for which its
 * time derivative is \f$\frac{d}{dt} g(q(t)) = G(q)\dot{q}\f$ then this
 * function computes the jacobian matrix \f$G(q)\f$.
 *
 * \param model   rigid body model
 * \param Q       state vector of the internal joints
 * \param body_id the id of the body
 * \param point_position the position of the point in body-local data
 * \param G       a matrix where the result will be stored in
 * \param update_kinematics whether UpdateKinematics() should be called or not (default: true)
 *
 * \returns A 3 x \#dof_count matrix of the point jacobian
 */
void CalcJacobian (Model &model,
                const Math::VectorNd &Q,
                unsigned int body_id,
                const Math::Vector3d &point_position,
                Math::MatrixNd &G,
                bool update_kinematics = true
                );


/** @} */

}

#endif /* _JACOBIAN_H */
