#include "argus/argo/config.hpp"

#include <filesystem>
#include <fstream>
#include <sstream>
#include <stdexcept>

#include <yaml-cpp/yaml.h>

namespace argus::argo {

namespace fs = std::filesystem;

namespace {

std::string read_file(const fs::path& path, const std::string& field) {
  std::ifstream in(path);
  if (!in) {
    throw std::runtime_error(field + ": cannot read " + path.string());
  }
  std::ostringstream text;
  text << in.rdbuf();
  return text.str();
}

std::string required(const YAML::Node& node, const std::string& field, const std::string& parent = "") {
  const YAML::Node value = node[field];
  if (!value || !value.IsScalar() || value.as<std::string>().empty()) {
    throw std::runtime_error(parent + field + ": required");
  }
  return value.as<std::string>();
}

}  // namespace

ArgoFiles load_argo_config(const std::string& path) {
  YAML::Node root;
  try {
    root = YAML::LoadFile(path);
  } catch (const YAML::Exception& e) {
    throw std::runtime_error(path + ": " + e.what());
  }
  const fs::path dir = fs::path(path).parent_path();
  auto resolve = [&](const std::string& p) { return fs::path(p).is_absolute() ? fs::path(p) : dir / p; };

  ArgoFiles out;
  out.profile = capabilities::load_profile_file(resolve(required(root, "profile")).string());
  out.runtime.session.node_id = required(root, "node_id");
  out.runtime.session.profile_hash = required(root, "profile_hash");
  if (root["calibration_version"]) {
    out.runtime.session.calibration_version = root["calibration_version"].as<std::string>();
  }
  if (const YAML::Node hub = root["hub"]) {
    auto& link = out.runtime.link;
    link.target = required(hub, "target", "hub.");
    if (hub["server_name"]) {
      link.server_name = hub["server_name"].as<std::string>();
    }
    // Mutual TLS or nothing: a robot never talks to a hub in the clear by accident.
    link.ca_pem = read_file(resolve(required(hub, "ca", "hub.")), "hub.ca");
    link.cert_pem = read_file(resolve(required(hub, "cert", "hub.")), "hub.cert");
    link.key_pem = read_file(resolve(required(hub, "key", "hub.")), "hub.key");
  }
  return out;
}

}  // namespace argus::argo
