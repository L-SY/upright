////
//// Created by lsy on 24-4-1.
////
//
//#include <yaml-cpp/yaml.h>
//#include "upright_core/Nominal.h"
//#include "upright_core/Contact.h"
//#include "upright_core/"
//#pragma once
//
//namespace upright{
//
//template <typename Scalar>
//struct BalancedObjectWrapper {
//    RigidBody<Scalar> body;
//    ConvexPolyhedron<Scalar> box;
//    std::string parent_name;
//    bool fixture;
//
//    BalancedObjectWrapper(const RigidBody<Scalar>& b, const ConvexPolyhedron<Scalar>& bx, const std::string& pn, bool f)
//            : body(b), box(bx), parent_name(pn), fixture(f) {}
//};
//
//template <typename Scalar>
//Scalar parseNumber(const std::string& str) {
//    if (str.back() == 'i' && str.substr(str.length() - 2, 2) == "pi") {
//        std::string factor_str = str.substr(0, str.length() - 2);
//        Scalar factor = static_cast<Scalar>(std::stod(factor_str));
//        return factor * static_cast<Scalar>(M_PI);
//    }
//    return static_cast<Scalar>(std::stod(str));
//}
//
//template <typename Scalar>
//std::vector<Scalar> parseSupportOffset(const YAML::Node& node) {
//    Scalar x = node["x"].as<Scalar>(0.0);
//    Scalar y = node["y"].as<Scalar>(0.0);
//    if (node["r"].IsDefined() && node["θ"].IsDefined()) {
//        Scalar r = node["r"].as<Scalar>();
//        Scalar theta = parseNumber<Scalar>(node["θ"].as<std::string>());
//        x += r * std::cos(theta);
//        y += r * std::sin(theta);
//    }
//
//    return {x, y};
//}
//
//
//
//}