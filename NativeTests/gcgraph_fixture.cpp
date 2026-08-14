#include <opencv2/core.hpp>
#include <opencv2/imgproc/detail/gcgraph.hpp>
#include <cstdint>
#include <iostream>

int main() {
  constexpr int width = 17;
  constexpr int height = 13;
  cv::detail::GCGraph<float> graph(width * height,
                                   (width - 1) * height + width * (height - 1));
  std::uint32_t state = 0x12345678u;
  auto random = [&]() {
    state = state * 1664525u + 1013904223u;
    return static_cast<float>((state >> 8) & 0xffffu) / 257.0f;
  };
  for (int i = 0; i < width * height; ++i) {
    graph.addVtx();
    graph.addTermWeights(i, random(), random());
  }
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      const int i = y * width + x;
      if (x + 1 < width) {
        const float w = 0.1f + random();
        graph.addEdges(i, i + 1, w, w);
      }
      if (y + 1 < height) {
        const float w = 0.1f + random();
        graph.addEdges(i, i + width, w, w);
      }
    }
  }
  std::cout << graph.maxFlow() << "\n";
  for (int i = 0; i < width * height; ++i) {
    std::cout << (graph.inSourceSegment(i) ? '1' : '0');
  }
  std::cout << "\n";
}
