/** BSD 3-Clause License

* Copyright (c) 2022, National University of Singapore
* All rights reserved.

* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:

* 1. Redistributions of source code must retain the above copyright notice, this
*    list of conditions and the following disclaimer.

* 2. Redistributions in binary form must reproduce the above copyright notice,
*    this list of conditions and the following disclaimer in the documentation
*    and/or other materials provided with the distribution.

* 3. Neither the name of the copyright holder nor the names of its
*    contributors may be used to endorse or promote products derived from
*    this software without specific prior written permission.

* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/** \author Sipu Ruan */

#pragma once

#include "DistanceImplicit.h"

namespace collision_implicit {

/** \class Variables
 * \brief Variables inherited from ifopt::VariableSet, defines variables */
template <typename G1, typename G2>
class Variables : public ifopt::VariableSet {
  public:
    /** \brief Constructor
     * \param name Variable name
     * \param s1 First object
     * \param s2 Second object */
    Variables(const std::string& name, const G1* s1, const G2* s2)
        : ifopt::VariableSet(6, name) {
        // Initial guess
        pointVar_ = Eigen::VectorXd::Zero(6);
        pointVar_(0) = s1->pos()[0];
        pointVar_(1) = s1->pos()[1];
        pointVar_(2) = s1->pos()[2];
        pointVar_(3) = s2->pos()[0];
        pointVar_(4) = s2->pos()[1];
        pointVar_(5) = s2->pos()[2];
    }

    /** \brief Transform the Eigen::Vector into whatever internal representation
     * \param var Optimization variables */
    void SetVariables(const Eigen::VectorXd& var) override { pointVar_ = var; }

    /** \brief Reverse transformation from the internal representation to
     * Eigen::VectorXd
     * \return Get variable values in the form of Eigen::VectorXd */
    Eigen::VectorXd GetValues() const override { return pointVar_; }

    /** \brief Set variables upper and lower bound
     * \return Variable bound as VecBound */
    VecBound GetBounds() const override {
        VecBound bounds(GetRows());
        for (size_t i = 0; i < bounds.size(); ++i) {
            bounds.at(i) = ifopt::NoBound;
        }
        return bounds;
    }

  private:
    Eigen::VectorXd pointVar_;
};

/** \class DistanceCost
 * \brief Cost function */
class DistanceCost : public ifopt::CostTerm {
  public:
    /** \brief Constructor
     * \param name Name of the cost function */
    DistanceCost(const std::string& name) : ifopt::CostTerm(name) {}

    /** \brief Compute cost value
     * \return Cost value */
    double GetCost() const override {
        Eigen::VectorXd var =
            GetVariables()->GetComponent("cartesian")->GetValues();

        return getSquareDistance(var);
    }

    /** \brief Compute squared distance
     * \param var Optimization variables
     * \return Squared distance between two points */
    double getSquareDistance(const Eigen::VectorXd& var) const {
        double squaredDistance = 0.5 * (std::pow(var(0) - var(3), 2.0) +
                                        std::pow(var(1) - var(4), 2.0) +
                                        std::pow(var(2) - var(5), 2.0));

        return squaredDistance;
    }

  private:
    void FillJacobianBlock(std::string var_set, Jacobian& jac) const override {
        Eigen::VectorXd var =
            GetVariables()->GetComponent(var_set)->GetValues();

        jac.coeffRef(0, 0) = var(0) - var(3);
        jac.coeffRef(0, 1) = var(1) - var(4);
        jac.coeffRef(0, 2) = var(2) - var(5);
        jac.coeffRef(0, 3) = -(var(0) - var(3));
        jac.coeffRef(0, 4) = -(var(1) - var(4));
        jac.coeffRef(0, 5) = -(var(2) - var(5));
    }
};

/** \class ImplicitConstraints
 * \brief Constraint functions */
template <typename G1, typename G2>
class ImplicitConstraints : public ifopt::ConstraintSet {
  public:
    /** \brief Constructor
     * \param name Constraint function name
     * \param s1 First object
     * \param s2 Second object */
    ImplicitConstraints(const std::string& name, const G1* s1, const G2* s2)
        : ifopt::ConstraintSet(2, name), s1_(s1), s2_(s2) {}

    /** \brief Compute constraint values
     * \return Constraint values as vector */
    VectorXd GetValues() const override {
        VectorXd g(GetRows());
        VectorXd var = GetVariables()->GetComponent("cartesian")->GetValues();

        g(0) = getImplicitFromGlobalPoint<G1>(var.head(3), s1_);
        g(1) = getImplicitFromGlobalPoint<G2>(var.tail(3), s2_);

        return g;
    }

    /** \brief Get bounding conditions
     * \return Bound values */
    VecBound GetBounds() const override {
        VecBound bounds(GetRows());
        bounds.at(0) = ifopt::Bounds(-Inf, 0.0);
        bounds.at(1) = ifopt::Bounds(-Inf, 0.0);

        return bounds;
    }

  private:
    template <typename G>
    double getImplicitFromGlobalPoint(const Eigen::Vector3d& point,
                                      const G* s) const {
        Eigen::Vector3d sLocal =
            s->quaternion().toRotationMatrix().transpose() *
            (point - Eigen::Vector3d(s->pos()[0], s->pos()[1], s->pos()[2]));

        double g = s->getImplicitFunction(sLocal);

        return g;
    }

    void FillJacobianBlock(std::string var_set, Jacobian& jac) const override {
        Eigen::VectorXd var =
            GetVariables()->GetComponent(var_set)->GetValues();

        jac.coeffRef(0, 0) =
            (getImplicitFromGlobalPoint<G1>(
                 Eigen::Vector3d({var(0) + EPSILON, var(1), var(2)}), s1_) -
             getImplicitFromGlobalPoint<G1>(
                 Eigen::Vector3d({var(0) - EPSILON, var(1), var(2)}), s1_)) /
            (2 * EPSILON);
        jac.coeffRef(0, 1) =
            (getImplicitFromGlobalPoint<G1>(
                 Eigen::Vector3d({var(0), var(1) + EPSILON, var(2)}), s1_) -
             getImplicitFromGlobalPoint<G1>(
                 Eigen::Vector3d({var(0), var(1) - EPSILON, var(2)}), s1_)) /
            (2 * EPSILON);
        jac.coeffRef(0, 2) =
            (getImplicitFromGlobalPoint<G1>(
                 Eigen::Vector3d({var(0), var(1), var(2) + EPSILON}), s1_) -
             getImplicitFromGlobalPoint<G1>(
                 Eigen::Vector3d({var(0), var(1), var(2) - EPSILON}), s1_)) /
            (2 * EPSILON);
        jac.coeffRef(1, 3) =
            (getImplicitFromGlobalPoint<G2>(
                 Eigen::Vector3d({var(3) + EPSILON, var(4), var(5)}), s2_) -
             getImplicitFromGlobalPoint<G2>(
                 Eigen::Vector3d({var(3) - EPSILON, var(4), var(5)}), s2_)) /
            (2 * EPSILON);
        jac.coeffRef(1, 4) =
            (getImplicitFromGlobalPoint<G2>(
                 Eigen::Vector3d({var(3), var(4) + EPSILON, var(5)}), s2_) -
             getImplicitFromGlobalPoint<G2>(
                 Eigen::Vector3d({var(3), var(4) - EPSILON, var(5)}), s2_)) /
            (2 * EPSILON);
        jac.coeffRef(1, 5) =
            (getImplicitFromGlobalPoint<G2>(
                 Eigen::Vector3d({var(3), var(4), var(5) + EPSILON}), s2_) -
             getImplicitFromGlobalPoint<G2>(
                 Eigen::Vector3d({var(3), var(4), var(5) - EPSILON}), s2_)) /
            (2 * EPSILON);
    }

    const G1* s1_;
    const G2* s2_;
};

template <typename G1, typename G2>
DistanceImplicit<G1, G2>::DistanceImplicit(const cfc::Shape3D& s1_shape,
                                           const cfc::Shape3D& s2_shape)
    : cfc::DistanceSolver<G1, G2>::DistanceSolver(s1_shape, s2_shape) {
    setupSolver();
}

template <typename G1, typename G2>
DistanceImplicit<G1, G2>::DistanceImplicit(const G1* s1, const G2* s2)
    : cfc::DistanceSolver<G1, G2>::DistanceSolver(s1, s2) {
    setupSolver();
}

template <typename G1, typename G2>
DistanceImplicit<G1, G2>::~DistanceImplicit() {}

template <typename G1, typename G2>
void DistanceImplicit<G1, G2>::query() {
    pointDistance();

    // Store results
    this->res_.closest_point_s1 = var_.head(3);
    this->res_.closest_point_s2 = var_.tail(3);

    // Line connecting two closest points
    Eigen::Vector3d line_s2_to_s1 =
        this->res_.closest_point_s2 - this->res_.closest_point_s1;

    this->res_.distance = line_s2_to_s1.norm();

    if (this->res_.distance > 1e-3) {
        this->res_.is_collision = false;
    } else {
        this->res_.is_collision = true;
    }

    // Optimal gradient/normal vector
    Eigen::Vector3d p_s1_local =
        this->M1_.transpose() *
        Eigen::Vector3d(this->res_.closest_point_s1[0] - this->s1_->pos()[0],
                        this->res_.closest_point_s1[1] - this->s1_->pos()[1],
                        this->res_.closest_point_s1[2] - this->s1_->pos()[2]);
    this->gradient_ = this->M1_.transpose().inverse() *
                      this->s1_->getGradientFromCartesian(p_s1_local);
    this->normal_ = this->gradient_.normalized();
    this->res_.optimal_normal = this->normal_;

    // Necessary condition for minimum distance query: normal at closest
    // point on s1 should be colinear with the line connecting two closest
    // points Gradient at closest point
    this->res_.necessary_condition =
        (this->res_.optimal_normal.cross(line_s2_to_s1)).norm();

    // Number of iterations
    this->res_.num_iteration = nlp_.GetIterationCount();
}

template <typename G1, typename G2>
void DistanceImplicit<G1, G2>::setupSolver() {
    // Choose solver and options
    ipopt_.SetOption("tol", 1e-5);
    ipopt_.SetOption("max_iter", 100);
    ipopt_.SetOption("linear_solver", "mumps");
    ipopt_.SetOption("jacobian_approximation", "exact");
    ipopt_.SetOption("print_level", 0);
}

template <typename G1, typename G2>
void DistanceImplicit<G1, G2>::pointDistance() {
    // Define the problem
    nlp_.AddVariableSet(
        std::make_shared<Variables<G1, G2>>("cartesian", this->s1_, this->s2_));
    nlp_.AddConstraintSet(std::make_shared<ImplicitConstraints<G1, G2>>(
        "cartesian_constraints", this->s1_, this->s2_));
    nlp_.AddCostSet(std::make_shared<DistanceCost>("cartesian_cost"));

    // Solve
    ipopt_.Solve(nlp_);
    Eigen::VectorXd var_result = nlp_.GetOptVariables()->GetValues();

    setVariables(var_result);
}

}  // namespace collision_implicit
