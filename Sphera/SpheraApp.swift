import SwiftUI

@main
struct SpheraApp: App {
  var body: some Scene {
    WindowGroup {
      ContentView()
        .task {
          #if DEBUG
            if ProcessInfo.processInfo.arguments.contains("--loftr-smoke-test") {
              await LoFTRSmokeTest.run()
            }
          #endif
        }
    }
  }
}
