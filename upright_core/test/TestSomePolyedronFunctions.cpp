//
// Created by lsy on 24-4-2.
//

#include "upright_core/Polyhedron.h"

void TestProjectedVertices() {
    Eigen::MatrixXd vertices(3, 3); // 假设有3个顶点，每个顶点在三维空间中
    vertices << 1, 2, 3,
            4, 5, 6,
            7, 8, 9;
    Eigen::VectorXd point(3); // 投影平面通过的点
    point << 0, 0, 0;
    Eigen::MatrixXd axes(3, 2); // 定义投影平面的轴，假设我们将顶点投影到由前两个基向量定义的子空间
    axes << 1, 0,
            0, 1,
            0, 0;
    auto projectedVertices = upright::ProjectVerticesOnAxes(vertices, point, axes);
    std::cout << "Projected vertices:\n" << projectedVertices << std::endl;
}

void TestWindPolygonVertice() {
    Eigen::MatrixXd V(5, 2);
    V << 1, 2,
            4, 5,
            7, 8,
            3, 6,
            2, 4;

    // 调用 windPolygonVertices 函数并输出结果
    Eigen::MatrixXd sortedV = upright::WindPolygonVertices(V).first;
    std::vector<int> sortedIndices = upright::WindPolygonVertices(V).second;
    std::cout << "Sorted vertices:\n" << sortedV << std::endl;
    std::cout << "sortedIndices :\n";
    for (int index: sortedIndices) {
        std::cout << index << " ";
    }
    std::cout << std::endl;
}

void TestNull() {
    Eigen::Matrix<double, 3, 1> v(1, 2, 3);
    Eigen::Matrix<double, 2, 3> S = upright::null(v);

    std::cout << "Input vector: " << std::endl << v << std::endl;
    std::cout << "Output matrix: " << std::endl << S << std::endl;
}

void TestIncidenceMatrix() {
//    Eigen::MatrixXd vertices(3, 3);
//    vertices << 0, 0, 0,
//            1, 0, 0,
//            0, 1, 0;
//
//    Eigen::MatrixXd normals(1, 3);
//    normals << 0, 0, 1;
//
//    // 计算顶点之间的连接关系
//    Eigen::MatrixXd C = upright::computeIncidenceMatrix(vertices, normals);
//
//    // 打印结果
//    std::cout << "Vertices:" << std::endl;
//    std::cout << vertices << std::endl;
//
//    std::cout << "Normals:" << std::endl;
//    std::cout << normals << std::endl;
//
//    std::cout << "Incidence Matrix:" << std::endl;
//    std::cout << C << std::endl;
}

void TestLimitsAlongAxis() {
    Eigen::MatrixXd vertices(4, 3);
    vertices << 1, 2, 3,
            4, 5, 6,
            7, 8, 9,
            10, 11, 12;
    Eigen::Vector3d axis(1, 0, 0);
    upright::ConvexPolyhedron polyhedron;
    Eigen::Vector2d result = polyhedron.LimitsAlongAxis(vertices, axis);

    std::cout << "Minimum projection: " << result[0] << std::endl;
    std::cout << "Maximum projection: " << result[1] << std::endl;
}

void TestGetVerticesInPlane() {
    Eigen::MatrixXd vertices(8, 3);
    vertices << 0, 0, 0,
            1, 0, 0,
            1, 1, 0,
            0, 1, 0,
            0, 0, 1,
            1, 0, 1,
            1, 1, 1,
            0, 1, 1;

    upright::ConvexPolyhedron polyhedron;
    Eigen::Vector3d point(0.5, 0.5, 0);
    Eigen::Vector3d normal(0, 0, 1);
    double tol = 1e-6;
    Eigen::MatrixXd verticesInPlane = polyhedron.GetVerticesInPlane(vertices, point, normal, tol);
    std::cout << "Vertices in plane:" << std::endl;
    std::cout << verticesInPlane << std::endl;
}

void TestGetPolygonInPlane() {
    Eigen::MatrixXd vertices(8, 3);
    vertices << 0, 0, 0,
            1, 0, 0,
            1, 1, 0,
            0, 1, 0,
            0, 0, 1,
            1, 0, 1,
            1, 1, 1,
            0, 1, 1;

    upright::ConvexPolyhedron polyhedron;
    Eigen::Vector3d point(0.5, 0.5, 0.0);
    Eigen::Vector3d planeNormal(0, 0, 1);
    Eigen::Matrix<double, 3, 2> planeSpan;
    planeSpan << 1, 0,
            0, 1,
            0, 0;

    Eigen::MatrixXd polygon = polyhedron.GetPolygonInPlane(vertices, point, planeNormal, planeSpan, 1e-6);

    std::cout << "Polygon vertices:\n" << polygon << std::endl;
}

int main() {
//    TestProjectedVertices();
//    TestWindPolygonVertice();
//    TestNull();
//    TestIncidenceMatrix();
//    TestLimitsAlongAxis();
//    TestGetVerticesInPlane();
    TestGetPolygonInPlane();
    return 0;

}