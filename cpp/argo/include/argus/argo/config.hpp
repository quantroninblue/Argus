#pragma once

#include <string>

#include "argus/argo/runtime.hpp"

namespace argus::argo {

// Everything Argo needs to start on a robot, read from one YAML file:
//
//   node_id: robot-01
//   profile: /etc/argo/profile.yaml
//   profile_hash: sha256:...        # written by argus setup
//   hub:                             # leave out to run with no hub
//     target: 192.168.1.20:7443
//     server_name: argus-hub         # optional
//     ca: /etc/argo/ca.pem
//     cert: /etc/argo/node.pem
//     key: /etc/argo/node.key
//
// Relative paths are taken from the file's own directory.
struct ArgoFiles {
  capabilities::Profile profile;
  RuntimeConfig runtime;
};

// Throws std::runtime_error naming the field at fault.
ArgoFiles load_argo_config(const std::string& path);

}  // namespace argus::argo
