//
// Created by lsy on 24-4-1.
//

#pragma once

#include "upright_core/Polyhedron.h"
#include "upright_core/Util.h"
#include <iostream>
#include <vector>
#include <cmath>    // For std::sqrt
#include <cassert>
#include <algorithm> // For std::transform
#include <Eigen/Dense>
#include <limits>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/SparseCore>
#include <Eigen/SparseCholesky>
#include <Eigen/SparseQR>

namespace upright {

    template<typename Scalar>
    const Scalar DEFAULT_TOLERANCE = static_cast<Scalar>(1e-8);

    template<typename Scalar>
    Scalar VectorNorm(const std::vector<Scalar> &v) {
        Scalar sum = 0;
        for (const auto &x: v) {
            sum += x * x;
        }
        return std::sqrt(sum);
    }

    template<typename Scalar>
    std::vector<Scalar> VectorScale(const std::vector<Scalar> &v, Scalar scale) {
        std::vector<Scalar> result(v.size());
        for (size_t i = 0; i < v.size(); ++i) {
            result[i] = v[i] * scale;
        }
        return result;
    }

    template<typename Scalar>
    std::vector<Scalar> VectorSub(const std::vector<Scalar> &v1, const std::vector<Scalar> &v2) {
        assert(v1.size() == v2.size());
        std::vector<Scalar> result(v1.size());
        for (size_t i = 0; i < v1.size(); ++i) {
            result[i] = v1[i] - v2[i];
        }
        return result;
    }

    template<typename Scalar>
    std::vector<Scalar> Orth2d(const std::vector<Scalar> &a) {
// Return vector `a` rotated by 90 degrees counter-clockwise.
// For a polygon with counter-clockwise winding, this gives the inward-facing normal for a given edge.
// equivalent to np.array([[0, -1], [1, 0]]) @ a
        return {-a[1], a[0]};
    }

    template<typename Scalar>
    std::vector<Scalar> lineSegmentHalfSpaceIntersection(const std::vector<Scalar> &v1, const std::vector<Scalar> &v2,
                                                         const std::vector<Scalar> &point,
                                                         const std::vector<Scalar> &normal, Scalar tol) {
// Intersection of a line segment and a half space.
// The line segment is defined by vertices `v1` and `v2`.
// The half-space passes through `point` and is defined by `normal`.
// Returns the intersection point or an empty vector if there is no intersection.
        assert(v1.size() == v2.size() && v1.size() == point.size() && v1.size() == normal.size());

        Scalar norm = VectorNorm(normal);

        std::vector<Scalar> normalized_normal(normal.size());
        for (size_t i = 0; i < normal.size(); ++i) {
            normalized_normal[i] = normal[i] / norm;
        }

        Scalar d1 = 0, d2 = 0;
        for (size_t i = 0; i < v1.size(); ++i) {
            d1 += normalized_normal[i] * (v1[i] - point[i]);
            d2 += normalized_normal[i] * (v2[i] - point[i]);
        }

// if either vertex is on the plane defining the halfspace, just return that
// vertex. If both are, then the line lies on the plane and we can return
// any point on the segment.
        if (std::abs(d1) < tol) {
            return v1;
        }
        if (std::abs(d2) < tol) {
            return v2;
        }

// if both vertices are on one side of the half space, there is no
// intersection
        if ((d1 < tol && d2 < tol) || (d1 > -tol && d2 > -tol)) {
            return {};
        }

        Scalar t = 0;
        for (size_t i = 0; i < v1.size(); ++i) {
            t += normalized_normal[i] * (point[i] - v1[i]);
        }
        t /= d2 - d1;
        assert(t >= 0 && t <= 1);

        std::vector<Scalar> intersection(v1.size());
        for (size_t i = 0; i < v1.size(); ++i) {
            intersection[i] = v1[i] + t * (v2[i] - v1[i]);
        }
        return intersection;
    }

    template<typename Scalar>
    std::vector<std::vector<Scalar>> ClipLineSegmentWithHalfSpace(
            const std::vector<Scalar> &v1,
            const std::vector<Scalar> &v2,
            const std::vector<Scalar> &point,
            const std::vector<Scalar> &normal,
            Scalar tol) {

        assert(v1.size() == v2.size() && v1.size() == point.size() && v1.size() == normal.size());

        // Normalizing normal vector
        Scalar norm = VectorNorm(normal);
        std::vector<Scalar> normalized_normal(normal.size());
        for (size_t i = 0; i < normal.size(); ++i) {
            normalized_normal[i] = normal[i] / norm;
        }
        // Computing d1 and d2
        Scalar d1 = 0, d2 = 0;
        for (size_t i = 0; i < v1.size(); ++i) {
            d1 += normalized_normal[i] * (v1[i] - point[i]);
            d2 += normalized_normal[i] * (v2[i] - point[i]);
        }
        // Checking the position of v1 and v2 relative to the half-space
        std::vector<std::vector<Scalar>> new_vertices;
        if (d1 >= -tol && d2 >= -tol) {
            new_vertices.push_back(v1);
            new_vertices.push_back(v2);
            return new_vertices;
        }
        if (d1 <= tol && d2 <= tol) {
            return {};
        }

        std::vector<Scalar> intersection = lineSegmentHalfSpaceIntersection(v1, v2, point, normal, tol);
        if (d1 > 0) {
            new_vertices.push_back(v1);
            new_vertices.push_back(intersection);
            return new_vertices;;
        } else {
            new_vertices.push_back(intersection);
            new_vertices.push_back(v2);
            return new_vertices;
        }
    }

    template<typename Scalar>
    std::vector<std::vector<Scalar>> ClipPolygonWithHalfSpace(
            const std::vector<std::vector<Scalar>> &V,
            const std::vector<Scalar> &point,
            const std::vector<Scalar> &normal,
            Scalar tol) {
        assert(V.size() > 0 && V[0].size() == 2);

        std::vector<std::vector<Scalar>> clipped_vertices;

        // Clip each edge of the polygon with the half space
        for (size_t i = 0; i < V.size(); ++i) {
            size_t j = (i + 1) % V.size();
            const std::vector<Scalar> &v1 = V[i];
            const std::vector<Scalar> &v2 = V[j];
            std::vector<std::vector<Scalar>> new_vertices =
                    ClipLineSegmentWithHalfSpace(v1, v2, point, normal, tol);
            for (const auto &vertex: new_vertices) {
                clipped_vertices.push_back(vertex);
            }
        }

        // Early return if the whole polygon is removed
        if (clipped_vertices.empty()) {
            return {};
        }

        // Filter out duplicate vertices
        std::vector<std::vector<Scalar>> new_vertices;
        for (const auto &candidate_vertex: clipped_vertices) {
            bool exists = false;
            for (const auto &existing_vertex: new_vertices) {
                Scalar distance = 0;
                for (size_t i = 0; i < candidate_vertex.size(); ++i) {
                    distance += std::pow(candidate_vertex[i] - existing_vertex[i], 2);
                }
                distance = std::sqrt(distance);
                if (distance < tol) {
                    exists = true;
                    break;
                }
            }
            if (!exists) {
                new_vertices.push_back(candidate_vertex);
            }
        }

        // NOTE: no need to re-wind vertices because ordering is preserved by above routines
        return new_vertices;
    }

    template<typename Scalar>
    std::vector<std::vector<Scalar>> ClipPolygonWithPolygon(
            const std::vector<std::vector<Scalar>> &V1,
            const std::vector<std::vector<Scalar>> &V2,
            Scalar tol) {
        assert(V1.size() > 0 && V1[0].size() == 2);
        assert(V2.size() > 0 && V2[0].size() == 2);

        std::vector<std::vector<Scalar>> V = V1;

        for (size_t i = 0; i < V2.size(); ++i) {
            size_t j = (i + 1) % V2.size();
            const std::vector<Scalar> &point = V2[i];
            std::vector<Scalar> a = VectorSub(V2[j], point);
            Scalar norm = VectorNorm(a);
            if (norm < tol) {
                throw std::runtime_error("Clipping polygon has repeated vertices.");
            }
            a = VectorScale(a, static_cast<Scalar>(1) / norm);
            std::vector<Scalar> normal = Orth2d(a); // inward-facing normal

            V = ClipPolygonWithHalfSpace(V, point, normal, tol);

            // If the clip ever excluded the whole polygon (i.e. there is no overlap), then we are done
            if (V.empty()) {
                return {};
            }
        }
        return V;
    }

    Eigen::MatrixXd
    ProjectVerticesOnAxes(const Eigen::MatrixXd &vertices, const Eigen::VectorXd &point, const Eigen::MatrixXd &axes) {
        return (axes.transpose() * (vertices.rowwise() - point.transpose()).transpose()).transpose();
    }

    std::pair<Eigen::MatrixXd, std::vector<int>> WindPolygonVertices(const Eigen::MatrixXd &V) {
        /*
         * Order vertices counter-clockwise.
         *
         * Parameters:
         * V: shape (n, 2) array
         *
         * Returns:
         * An array of shape (n, 2) containing the sorted vertices.
         */
        assert(V.cols() == 2);

        Eigen::Vector2d c = V.colwise().mean();
        std::vector<std::pair<double, int>> angles;
        angles.reserve(V.rows());
        for (int i = 0; i < V.rows(); i++) {
            double angle = std::atan2(V(i, 1) - c(1), V(i, 0) - c(0));
            angles.emplace_back(angle, i);
        }

        std::sort(angles.begin(), angles.end());
        Eigen::MatrixXd sortedV(V.rows(), 2);
        std::vector<int> indices(V.rows());

        for (int i = 0; i < angles.size(); i++) {
            sortedV.row(i) = V.row(angles[i].second);
            indices[i] = angles[i].second;
        }

        return std::make_pair(sortedV, indices);
    }

    Eigen::MatrixXi ConvexPolyhedron::ComputeIncidenceMatrix(double tol) {
        Eigen::MatrixXi fake;
        return fake;
    }

    Eigen::Vector2d
    ConvexPolyhedron::LimitsAlongAxis(const Eigen::MatrixXd &vertices, const Eigen::Vector3d &axis) const {
        Eigen::Vector3d unitAxis = axis.normalized();

        Eigen::VectorXd projection = vertices * unitAxis;

        double minProj = std::numeric_limits<double>::max();
        double maxProj = std::numeric_limits<double>::lowest();
        for (int i = 0; i < projection.size(); ++i) {
            minProj = std::min(minProj, projection[i]);
            maxProj = std::max(maxProj, projection[i]);
        }

        return Eigen::Vector2d(minProj, maxProj);
    }

    double ConvexPolyhedron::LengthAlongAxis(const Eigen::MatrixXd &vertices, const Eigen::Vector3d &axis) const {
        Eigen::Vector3d unitAxis = axis.normalized();
        Eigen::Vector2d limits = LimitsAlongAxis(vertices, axis);
        return limits[1] - limits[0];
    }

    double ConvexPolyhedron::Height(const Eigen::MatrixXd &vertices) const {
        return LengthAlongAxis(vertices, Eigen::Vector3d(0, 0, 1));
    }

    Eigen::Vector3d
    ConvexPolyhedron::MaxVertexAlongAxis(const Eigen::MatrixXd &vertices, const Eigen::Vector3d &axis) const {
        Eigen::Vector3d unitAxis = axis.normalized();
        Eigen::VectorXd projection = vertices * unitAxis;
        Eigen::Index maxIndex;
        projection.maxCoeff(&maxIndex);
        return vertices.row(maxIndex);
    }

    Eigen::MatrixXd ConvexPolyhedron::GetVerticesInPlane(const Eigen::MatrixXd &vertices, const Eigen::Vector3d &point,
                                                         const Eigen::Vector3d &normal,
                                                         double tol) const {
        Eigen::VectorXd projection = ProjectVerticesOnAxes(vertices, point, normal);
        std::vector<Eigen::Index> indices;
        for (Eigen::Index i = 0; i < projection.size(); ++i) {
            if (std::abs(projection[i]) < tol) {
                indices.push_back(i);
            }
        }
        Eigen::MatrixXd verticesInPlane(indices.size(), 3);
        for (Eigen::Index i = 0; i < indices.size(); ++i) {
            verticesInPlane.row(i) = vertices.row(indices[i]);
        }
        return verticesInPlane;
    }

    Eigen::MatrixXd
    ConvexPolyhedron::GetPolygonInPlane(const Eigen::MatrixXd &vertices, const Eigen::Vector3d &point,
                                        const Eigen::Vector3d &planeNormal,
                                        const Eigen::Matrix<double, 3, 2> &planeSpan,
                                        double tol) const {
        Eigen::MatrixXd V_3d = GetVerticesInPlane(vertices, point, planeNormal, tol);
        Eigen::MatrixXd V_2d = ProjectVerticesOnAxes(V_3d, point, planeSpan);
        Eigen::MatrixXd polygon = WindPolygonVertices(V_2d).first;
        return polygon;
    }

    double
    ConvexPolyhedron::DistanceFromCentroidToBoundary(const Eigen::MatrixXd &vertices, const Eigen::Vector3d &axis,
                                                     const Eigen::Vector3d &offset, double tol) const {
        Eigen::Vector3d normalizedAxis = axis.normalized();

        int nv = vertices.rows();
        Eigen::VectorXd c(nv + 1);
        c.head(1).setConstant(-1.0);
        c.tail(nv).setZero();

        Eigen::MatrixXd Aeq(4, nv + 1);
        Aeq.topRows<3>().col(0) = normalizedAxis;
        Aeq.topRows<3>().bottomLeftCorner(3, nv) = -vertices.transpose();
        Aeq.row(3).tail(nv) = Eigen::VectorXd::Ones(nv);

        Eigen::VectorXd beq(4);
        beq.head<3>() = -(this->position + offset);
        beq(3) = 1.0;

        Eigen::LeastSquaresConjugateGradient<Eigen::MatrixXd> solver;
        solver.compute(Aeq);
        Eigen::VectorXd x = solver.solve(c);

        double distance = x(0);
        assert(distance >= -tol);
        return distance;
    }

//    Eigen::MatrixXd ConvexPolyhedron::clipWithHalfSpace(const Eigen::MatrixXd &V, const Eigen::Vector3d &point,
//                                                        const Eigen::Vector3d &normal, double tol) {
//        std::vector<Eigen::Vector3d> clippedVertices;
//        std::vector<int> clippedIndices;
//
//        int nv = V.rows();
//        for (int i = 0; i < nv; ++i) {
//            int j = (i + 1) % nv;
//            const Eigen::Vector3d &v1 = V.row(i);
//            const Eigen::Vector3d &v2 = V.row(j);
//            std::vector<Eigen::Vector3d> newVertices = ClipLineSegmentWithHalfSpace(v1, v2, point, normal, tol);
//            for (const auto &vertex: newVertices) {
//                clippedVertices.push_back(vertex);
//                clippedIndices.push_back(i);
//            }
//        }
//
//        std::vector<Eigen::Vector3d> uniqueVertices;
//        std::vector<int> uniqueIndices;
//        for (size_t i = 0; i < clippedVertices.size(); ++i) {
//            bool exists = false;
//            for (const auto &existingVertex: uniqueVertices) {
//                if ((clippedVertices[i] - existingVertex).norm() < tol) {
//                    exists = true;
//                    break;
//                }
//            }
//            if (!exists) {
//                uniqueVertices.push_back(clippedVertices[i]);
//                uniqueIndices.push_back(clippedIndices[i]);
//            }
//        }
//
//        return Eigen::MatrixXd::Map(uniqueVertices.data(), uniqueVertices.size(), 3);
//    }

    ConvexPolyhedron::ConvexPolyhedron() {
        std::cout << "Just for test!" << std::endl;
    }

    ConvexPolyhedron::ConvexPolyhedron(const Eigen::MatrixXd &vertices, const Eigen::MatrixXd &normals,
                                       const Eigen::Vector3d &position, const Eigen::Matrix3d &rotation,
                                       const Eigen::MatrixXi &incidence)
            : vertices(vertices), normals(normals), position(position), rotation(rotation) {
        nv = vertices.rows();  // number of vertices
        nf = normals.rows();   // number of faces
        // TODO: possibly more convenient to compute wound indices of vertices
        // for each face: then clipping can be done face-wise
        if (incidence.size() == 0) {
            this->incidence = ComputeIncidenceMatrix(static_cast<double >(1e-8));
        } else {
            static_assert(std::is_same<typename Eigen::MatrixXi::Scalar, int>::value,
                          "Incidence matrix must have integer type");
            if (incidence.rows() != nv || incidence.cols() != nv)
                throw std::invalid_argument("Incidence matrix of wrong size");
            if (!incidence.isApprox(incidence.transpose()))
                throw std::invalid_argument("Incidence matrix is not symmetric");
            this->incidence = incidence;
        }
        ne = 0;
        for (int i = 0; i < nv; ++i) {
            for (int j = 0; j <= i; ++j) {
                if (incidence(i, j) != 0) {
                    ++ne;
                }
            }
        }
    }
    // TODO : Add this most important last
//    def axis_aligned_contact(poly1, poly2, tol=DEFAULT_TOLERANCE):

}//namespace upright
