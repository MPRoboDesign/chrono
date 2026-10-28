# module_vehicle

Chrono::Vehicle has **no mandatory third-party dependency** beyond Chrono itself — nothing to install here by default.

Optional, only if a specific feature is turned on later:
- **OpenCRG** (`CH_ENABLE_OPENCRG`) — road/terrain description library, off by default.
- **VSG + vsgImGui + vsgXchange** — only required if Chrono::Vehicle's VSG-based visualization is enabled.

Neither is needed for the current chrono_ws build (`find_package(Chrono COMPONENTS Irrlicht Vehicle ROS Parsers Sensor ...)` doesn't turn either on).
