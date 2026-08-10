# OpenCV iOS framework

This directory contains the `arm64` iPhone slice of the official OpenCV 4.14.0
iOS framework. It is linked statically by the Sphera application and is not
embedded as a dynamic framework.

- Upstream release: https://github.com/opencv/opencv/releases/tag/4.14.0
- Asset: `opencv-4.14.0-ios-framework.zip`
- Upstream asset SHA-256: `f4f428fa9270adf95b3ed4705eb18ba3028ed2c2777a4f1a002739b869986a6a`
- License: Apache-2.0 (OpenCV 4.x)

The framework is kept outside the Swift/C++ source boundary; only files in
`Sphera/EngineBridge` include its C++ headers.
