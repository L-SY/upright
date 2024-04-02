//
// Created by lsy on 24-4-1.
//

#pragma once

#include <vector>
#include <array>
#include <cmath>
#include <algorithm>
#include <cassert>
#include <Eigen/Dense>

namespace upright {

    template<typename Scalar>
    extern const Scalar DEFAULT_TOLERANCE;

    template<typename Scalar>
    Scalar VectorNorm(const std::vector<Scalar> &v);

    template<typename Scalar>
    std::vector<Scalar> VectorScale(const std::vector<Scalar> &v, Scalar scale);

    template<typename Scalar>
    std::vector<Scalar> VectorSub(const std::vector<Scalar> &v1, const std::vector<Scalar> &v2);

    template<typename Scalar>
    std::vector<Scalar> Orth2d(const std::vector<Scalar> &a);

    template<typename Scalar>
    std::vector<Scalar> lineSegmentHalfSpaceIntersection(const std::vector<Scalar> &v1, const std::vector<Scalar> &v2,
                                                         const std::vector<Scalar> &point,
                                                         const std::vector<Scalar> &normal, Scalar tol);

    template<typename Scalar>
    std::vector<std::vector<Scalar>>
    ClipLineSegmentWithHalfSpace(const std::vector<Scalar> &v1, const std::vector<Scalar> &v2,
                                 const std::vector<Scalar> &point, const std::vector<Scalar> &normal, Scalar tol);

    template<typename Scalar>
    std::vector<std::vector<Scalar>> ClipPolygonWithHalfSpace(
            const std::vector<std::vector<Scalar>> &V,
            const std::vector<Scalar> &point,
            const std::vector<Scalar> &normal,
            Scalar tol);

    template<typename Scalar>
    std::vector<std::vector<Scalar>> ClipPolygonWithPolygon(
            const std::vector<std::vector<Scalar>> &V1,
            const std::vector<std::vector<Scalar>> &V2,
            Scalar tol);

    Eigen::MatrixXd
    ProjectVerticesOnAxes(const Eigen::MatrixXd &vertices, const Eigen::VectorXd &point, const Eigen::MatrixXd &axes);

    std::pair<Eigen::MatrixXd, std::vector<int>> WindPolygonVertices(const Eigen::MatrixXd &V);


    class ConvexPolyhedron {
    public:
        ConvexPolyhedron(const Eigen::MatrixXd &vertices, const Eigen::MatrixXd &normals,
                         const Eigen::Vector3d &position = Eigen::Vector3d::Zero(),
                         const Eigen::Matrix3d &rotation = Eigen::Matrix3d::Identity(),
                         const Eigen::MatrixXi &incidence = Eigen::MatrixXi());

        ConvexPolyhedron();

        static ConvexPolyhedron Box(const Eigen::Vector3d &half_extents,
                                    const Eigen::Vector3d &position = Eigen::Vector3d::Zero(),
                                    const Eigen::Matrix3d &rotation = Eigen::Matrix3d::Identity());

        static ConvexPolyhedron Wedge(const Eigen::Vector3d &half_extents,
                                      const Eigen::Vector3d &position = Eigen::Vector3d::Zero(),
                                      const Eigen::Matrix3d &rotation = Eigen::Matrix3d::Identity());

        ConvexPolyhedron Transform(const Eigen::Vector3d &translation = Eigen::Vector3d::Zero(),
                                   const Eigen::Matrix3d &rotation = Eigen::Matrix3d::Identity()) const;

        Eigen::Vector2d LimitsAlongAxis(const Eigen::MatrixXd &vertices, const Eigen::Vector3d &axis) const;

        double LengthAlongAxis(const Eigen::MatrixXd &vertices, const Eigen::Vector3d &axis) const;

        double Height(const Eigen::MatrixXd &vertices) const;

        Eigen::Vector3d MaxVertexAlongAxis(const Eigen::MatrixXd &vertices, const Eigen::Vector3d &axis) const;

        Eigen::MatrixXd
        GetVerticesInPlane(const Eigen::MatrixXd &vertices, const Eigen::Vector3d &point, const Eigen::Vector3d &normal,
                           double tol) const;

        Eigen::MatrixXd
        GetPolygonInPlane(const Eigen::MatrixXd &vertices, const Eigen::Vector3d &point,
                          const Eigen::Vector3d &plane_normal,
                          const Eigen::Matrix<double, 3, 2> &plane_span,
                          double tol) const;

        double DistanceFromCentroidToBoundary(const Eigen::MatrixXd &vertices,
                                              const Eigen::Vector3d &axis,
                                              const Eigen::Vector3d &offset = Eigen::Vector3d::Zero(),
                                              double tol = 1e-6) const;

        Eigen::MatrixXd clipWithHalfSpace(const Eigen::MatrixXd &V, const Eigen::Vector3d &point,
                                          const Eigen::Vector3d &normal, double tol);

    private:
        int nv;  // number of vertices
        int nf;  // number of faces
        int ne;  // number of edges
        Eigen::MatrixXd vertices;
        Eigen::MatrixXd normals;
        Eigen::Vector3d position;
        Eigen::Matrix3d rotation;
        Eigen::MatrixXi incidence;

        Eigen::MatrixXi ComputeIncidenceMatrix(double tol);

        void ComputeFaces();
    };

}  // namespace upright

#include "impl/Polyhedron.tpp"