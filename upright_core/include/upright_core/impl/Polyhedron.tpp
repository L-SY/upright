//
// Created by lsy on 24-4-1.
//

#pragma once

#include "upright_core/Polyhedron.h"
#include <iostream>
#include <vector>
#include <tuple>
#include <numeric> // For std::inner_product
#include <cmath>    // For std::sqrt
#include <cassert>
#include <algorithm> // For std::transform

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

}//namespace upright
