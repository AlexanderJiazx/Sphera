# Sphera

Sphera is a native iOS app that captures and stitches 360° spherical panoramas
entirely on-device. The UI is SwiftUI (`Sphera/`), and the panorama stitching
engine is C++/OpenCV with an Objective-C++ bridge (`Sphera/EngineBridge/`).

## Cursor Cloud specific instructions

The Cursor Cloud VM is **Linux x86_64**, but the shipping product is a
macOS/Xcode + iOS project. Keep the following in mind:

- **The iOS app (`Sphera.xcodeproj`) cannot be built or run on the Linux VM.**
  It requires macOS, Xcode (~26.x), an Apple signing team, and a physical iPhone
  (camera + CoreMotion) for real end-to-end capture. Simulator builds also
  require macOS/Xcode.
- **The `SpheraCore` SwiftPM package (`Package.swift`) does not build on Linux.**
  `Sphera/OrientationMath.swift` uses Apple's `simd` module (`simd_quatd`,
  `simd_double3x3`, `simd_act`, `simd_normalize`, ...), which is not part of the
  open-source Swift toolchain on Linux. `swift build` / `swift test` for
  `SpheraCore` / `SpheraCoreTests` therefore only work on macOS.
- **What DOES run on Linux: the C++/OpenCV stitching engine under `NativeTests/`.**
  This is the core of the on-device pipeline (pose-overlap graph, sensor-anchored
  ray-bundle solver, adaptive ring-seam blending). Desktop OpenCV is provided by
  the system (`libopencv-dev`, currently 4.6.0). Note the app itself links a
  vendored iOS-only `Vendor/OpenCV/opencv2.framework` (4.14.0, arm64) that is
  **not** usable for desktop builds — desktop builds use system OpenCV.

### Building / testing the native engine (Linux)

Preferred (CMake) — finds system OpenCV automatically:

```
cmake -S NativeTests -B NativeTests/build
cmake --build NativeTests/build -j
ctest --test-dir NativeTests/build --output-on-failure   # or: ./NativeTests/build/test_sensor_first
```

Fallback (Makefile) — the Makefile probes Homebrew (`brew --prefix opencv`),
which is absent on Linux, so you MUST pass `OPENCV_PREFIX` to reach the
pkg-config branch (any prefix that exposes `opencv4.pc` works; system OpenCV is
already on the default pkg-config path):

```
cd NativeTests && make test OPENCV_PREFIX=/usr
```

- `test_sensor_first` is a self-contained unit suite (no image data needed).
- `run_native_stitch <capture-dir> <output-dir>` runs a full stitch but needs a
  real capture package (`sphera-engine-request.json` + `images/`), none of which
  is checked into the repo.

### Other notes

- No linter is configured (no SwiftLint / CI workflows). Xcode warnings-as-errors
  act as the main static check for the app.
- No secrets, `.env` files, network services, or databases — the product is fully
  offline/on-device.
