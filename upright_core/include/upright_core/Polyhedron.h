//
// Created by lsy on 24-4-1.
//

#pragma once

#include <vector>
#include <array>
#include <cmath>
#include <algorithm>
#include <cassert>

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

    template<typename Scalar>
    std::vector<Scalar> PlaneSpan(const std::vector<Scalar> &normal);

    template<typename Scalar>
    std::vector<std::vector<Scalar>>
    ProjectVerticesOnAxes(const std::vector<std::vector<Scalar>> &vertices, const std::vector<Scalar> &point,
                          const std::vector<Scalar> &axes);

    template<typename Scalar>
    std::vector<std::vector<Scalar>> WindPolygonVertices(const std::vector<std::vector<Scalar>> &V);

    template<typename Scalar>
    class ConvexPolyhedron {
    public:
        ConvexPolyhedron(const std::vector<std::vector<Scalar>> &vertices,
                         const std::vector<std::vector<Scalar>> &normals,
                         const std::vector<Scalar> &position = {0, 0, 0},
                         const std::vector<Scalar> &rotation = {1, 0, 0, 0},
                         const std::vector<std::vector<bool>> &incidence = {});

        static ConvexPolyhedron
        Box(const std::vector<Scalar> &half_extents, const std::vector<Scalar> &position = {0, 0, 0},
            const std::vector<Scalar> &rotation = {1, 0, 0, 0});

        static ConvexPolyhedron
        Wedge(const std::vector<Scalar> &half_extents, const std::vector<Scalar> &position = {0, 0, 0},
              const std::vector<Scalar> &rotation = {1, 0, 0, 0});

        ConvexPolyhedron Transform(const std::vector<Scalar> &translation = {0, 0, 0},
                                   const std::vector<Scalar> &rotation = {1, 0, 0, 0}) const;

        std::array<Scalar, 2> LimitsAlongAxis(const std::vector<Scalar> &axis) const;

        Scalar LengthAlongAxis(const std::vector<Scalar> &axis) const;

        Scalar Height() const;

        std::vector<Scalar> MaxVertexAlongAxis(const std::vector<Scalar> &axis) const;

        std::vector<std::vector<Scalar>>
        GetVerticesInPlane(const std::vector<Scalar> &point, const std::vector<Scalar> &normal,
                           Scalar tol = DEFAULT_TOLERANCE<Scalar>) const;

        std::vector<std::vector<Scalar>>
        GetPolygonInPlane(const std::vector<Scalar> &point, const std::vector<Scalar> &plane_normal,
                          const std::vector<Scalar> &plane_span, Scalar tol = DEFAULT_TOLERANCE<Scalar>) const;

        Scalar
        DistanceFromCentroidToBoundary(const std::vector<Scalar> &axis, const std::vector<Scalar> &offset = {0, 0, 0},
                                       Scalar tol = DEFAULT_TOLERANCE<Scalar>) const;
        // std::vector<std::vector<Scalar>> ClipWithHalfSpace(const std::vector<std::vector<Scalar>>& V, const std::vector<Scalar>& point, const std::vector<Scalar>& normal, Scalar tol = DEFAULT_TOLERANCE<Scalar>) const;

    private:
        std::vector<std::vector<Scalar>> vertices_;
        std::vector<std::vector<Scalar>> normals_;
        std::vector<Scalar> position_;
        std::vector<Scalar> rotation_;
        std::vector<std::vector<bool>> incidence_;
        size_t nv_; // number of vertices
        size_t nf_; // number of faces
        size_t ne_; // number of edges

        std::vector<std::vector<bool>> ComputeIncidenceMatrix(Scalar tol = DEFAULT_TOLERANCE<Scalar>) const;
    };

}  // namespace upright

#include "impl/Polyhedron.tpp"