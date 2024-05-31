load("@rules_cc//cc:defs.bzl", "cc_binary")

cc_binary(
    name = "grid",
    srcs = ["src/main.cpp", "src/gif.h"],
    copts = ["-I/usr/include/opencv4"],
    linkopts = ["-lopencv_core", "-lopencv_highgui", "-lopencv_imgcodecs", "-lopencv_imgproc"],
)
