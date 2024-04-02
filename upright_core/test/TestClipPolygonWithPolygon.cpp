//
// Created by lsy on 24-4-1.
//

#include "upright_core/Polyhedron.h"

int main() {
    std::vector<std::vector<double>> V1 = {{1, 1},
                                           {4, 1},
                                           {4, 4},
                                           {1, 4}};
    std::vector<std::vector<double>> V2 = {{2, 0},
                                           {5, 0},
                                           {5, 5},
                                           {2, 5}};

    double tol = 1e-8;
    std::vector<std::vector<double>> clippedPolygon = upright::ClipPolygonWithPolygon<double>(V1, V2, tol);
    std::cout << "testIntersecting passed." << std::endl;
    // 打印结果
    std::cout << "Clipped Polygon vertices:" << std::endl;
    for (const auto &vertex: clippedPolygon) {
        std::cout << "(" << vertex[0] << ", " << vertex[1] << ")" << std::endl;
    }
}