load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "latency_component_lib",
    srcs = ["latency.cc"],
    hdrs = ["latency.h"],
    copts = [
        "-DMODULE_NAME=\\\"latency\\\"",
    ],
    deps = [
        "//cyber",
        "//cyber/time:clock",
        "//modules/control/common",
        "//modules/common/adapters:adapter_gflags",
        "@com_github_gflags_gflags//:gflags",
        "//modules/canbus/proto:chassis_cc_proto",
        "//modules/common/latency_recorder",
        "//modules/localization/proto:localization_cc_proto",
        "//modules/planning/proto:planning_cc_proto",
        "//modules/planning/proto:pad_msg_cc_proto",
        "//modules/prediction/proto:prediction_obstacle_cc_proto",
        "//modules/latency/proto:latency_cc_proto",
        
    ],
)
cc_binary(
    name = "latency_component.so",
    linkshared = True,
    linkstatic = False,
    deps = [":latency_component_lib"],
)
