load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library", "cc_test")
load("//tools/install:install.bzl", "install", "install_files", "install_src_files")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

NETWORK_COPTS = ['-DMODULE_NAME=\\"network\\"']

cc_library(
    name = "network_lib",
    copts = NETWORK_COPTS,
    deps = [
        "//cyber",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/latency_recorder",
        "//modules/common/monitor_log",
        "//modules/common/status",
        "//modules/common/util",
        "//modules/network/proto:network_conf_cc_proto",
        "//modules/common_msgs/control_msgs:control_cmd_cc_proto",
        "//modules/common_msgs/localization_msgs:localization_cc_proto",
        "//modules/common_msgs/planning_msgs:planning_cc_proto",
        "//modules/common_msgs/chassis_msgs:chassis_cc_proto",
        "@com_github_gflags_gflags//:gflags",
    ],
)

cc_library(
    name = "network_component_lib",
    srcs = ["network_component.cc"],
    hdrs = ["network_component.h"],
    copts = NETWORK_COPTS,
    deps = [
        ":network_lib",
    ],
    alwayslink = True
)

cc_binary(
    name = "libnetwork_component.so",
    linkshared = True,
    linkstatic = True,
    deps = [":network_component_lib"],
)



install(
    name = "install",
    library_dest = "network/lib",
    data_dest = "network",
    data = [
        ":runtime_data",
        ":cyberfile.xml",
        "network.BUILD",
    ],
    targets = [
        ":libnetwork_component.so",
    ],
    deps = [
        ":pb_hdrs",
        "//modules/network/proto:py_pb_network",
    ],
)

install(
    name = "pb_hdrs",
    data_dest = "network/include",
    data = [
        "//modules/network/proto:network_conf_cc_proto",
 
    ],
)


install_src_files(
    name = "install_src",
    deps = [
        ":install_network_src",
        ":install_network_hdrs"
    ],
)

install_src_files(
    name = "install_network_src",
    src_dir = ["."],
    dest = "network/src",
    filter = "*",
)

install_src_files(
    name = "install_network_hdrs",
    src_dir = ["."],
    dest = "network/include",
    filter = "*.h",
)

filegroup(
    name = "runtime_data",
    srcs = glob([
        "conf/*.txt",
        "conf/*.conf",
        "dag/*.dag",
        "launch/*.launch",
    ]),
)



cpplint()
