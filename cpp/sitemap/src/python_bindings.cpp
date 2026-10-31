#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <limits>

#include "argus/capabilities/capabilities.hpp"
#include "argus/capabilities/profile.hpp"
#include "argus/sitemap/route.hpp"
#include "argus/sitemap/site_map.hpp"

namespace py = pybind11;
using namespace argus::sitemap;

namespace {

Index3 index3(const py::sequence& s) {
  if (py::len(s) != 3) {
    throw py::value_error("expected three values");
  }
  return Index3(s[0].cast<int>(), s[1].cast<int>(), s[2].cast<int>());
}

Vec3 vec3(const py::sequence& s) {
  if (py::len(s) != 3) {
    throw py::value_error("expected three values");
  }
  return Vec3(s[0].cast<double>(), s[1].cast<double>(), s[2].cast<double>());
}

py::tuple xyz(const Vec3& v) { return py::make_tuple(v.x(), v.y(), v.z()); }

py::tuple tile_tuple(const Index3& t) { return py::make_tuple(t.x(), t.y(), t.z()); }

py::dict view_dict(const PlanarView& v) {
  py::array_t<int8_t> cells({v.nx, v.ny});
  auto out = cells.mutable_unchecked<2>();
  for (int x = 0; x < v.nx; ++x) {
    for (int y = 0; y < v.ny; ++y) {
      out(x, y) = static_cast<int8_t>(v.at(x, y));
    }
  }
  py::dict d;
  d["cells"] = cells;
  d["origin"] = py::make_tuple(v.origin.x(), v.origin.y());
  d["resolution"] = v.resolution;
  return d;
}

}  // namespace

PYBIND11_MODULE(sitemap, m) {
  m.doc() = "Persistent site map (sparse voxel tiles) used by the Argus hub.";
  m.attr("UNKNOWN") = static_cast<int>(Cell::Unknown);
  m.attr("FREE") = static_cast<int>(Cell::Free);
  m.attr("OCCUPIED") = static_cast<int>(Cell::Occupied);
  m.attr("TILE_SIZE") = argus::grid::kTileSize;

  m.def(
      "encode_evidence",
      [](const std::vector<std::pair<std::vector<int>, int>>& deltas) {
        std::vector<CellDelta> in;
        for (const auto& [i, d] : deltas) {
          if (i.size() != 3) throw py::value_error("cell index needs three values");
          in.push_back({Index3(i[0], i[1], i[2]), d});
        }
        return py::bytes(argus::grid::encode_evidence(in));
      },
      py::arg("deltas"), "Encode [((x, y, z), delta), ...] as a LocalEvidence payload.");

  py::class_<SiteMap>(m, "SiteMap")
      .def(py::init<std::string, double, uint64_t>(), py::arg("site_id"),
           py::arg("resolution_m") = 0.1, py::arg("epoch") = 1)
      .def_property_readonly("site_id", &SiteMap::site_id)
      .def_property_readonly("epoch", &SiteMap::epoch)
      .def_property_readonly("resolution", &SiteMap::resolution)
      .def_property_readonly("version", &SiteMap::version)
      .def_property_readonly("tile_count", &SiteMap::tile_count)
      .def("reset", &SiteMap::reset)
      .def(
          "apply_evidence",
          [](SiteMap& map, uint64_t epoch, const py::bytes& payload, double stamp) {
            const auto deltas = argus::grid::decode_evidence(std::string(payload));
            return map.apply(epoch, deltas, stamp);
          },
          py::arg("epoch"), py::arg("payload"), py::arg("stamp"),
          "Apply a LocalEvidence payload. False if the epoch is stale; ValueError if malformed.")
      .def(
          "integrate",
          [](SiteMap& map, const std::vector<double>& origin,
             const py::array_t<double, py::array::c_style | py::array::forcecast>& ends,
             const std::vector<bool>& hits, double stamp) {
            if (origin.size() != 3) throw py::value_error("origin needs three values");
            if (ends.ndim() != 2 || ends.shape(1) != 3 ||
                static_cast<size_t>(ends.shape(0)) != hits.size()) {
              throw py::value_error("ends must be N x 3 with one hit flag per row");
            }
            auto e = ends.unchecked<2>();
            std::vector<Vec3> points(hits.size());
            for (py::ssize_t k = 0; k < e.shape(0); ++k) {
              points[k] = Vec3(e(k, 0), e(k, 1), e(k, 2));
            }
            map.integrate(Vec3(origin[0], origin[1], origin[2]), points, hits, stamp);
          },
          py::arg("origin"), py::arg("ends"), py::arg("hits"), py::arg("stamp"))
      .def("cell", [](const SiteMap& map, const py::sequence& i) {
        return static_cast<int>(map.cell(index3(i)));
      })
      .def("at", [](const SiteMap& map, double x, double y, double z) {
        return static_cast<int>(map.at(Vec3(x, y, z)));
      })
      .def(
          "planar_view",
          [](const SiteMap& map, double z_low, double z_high) {
            return view_dict(map.planar_view(z_low, z_high));
          },
          py::arg("z_low_m"), py::arg("z_high_m"),
          "Dict with 'cells' (int8 [nx, ny]), 'origin' (cell index of [0, 0]) and 'resolution'.")
      .def("changed_since",
           [](const SiteMap& map, uint64_t version) {
             py::list out;
             for (const Index3& t : map.changed_since(version)) out.append(tile_tuple(t));
             return out;
           })
      .def("encode_tiles",
           [](const SiteMap& map, const std::vector<std::vector<int>>& keys) {
             std::vector<Index3> tiles;
             for (const auto& k : keys) {
               if (k.size() != 3) throw py::value_error("tile index needs three values");
               tiles.emplace_back(k[0], k[1], k[2]);
             }
             return py::bytes(map.encode_tiles(tiles));
           })
      .def("encode_all", [](const SiteMap& map) { return py::bytes(map.encode_all()); })
      .def("load_tiles", [](SiteMap& map, const py::bytes& payload) {
        map.load_tiles(std::string(payload));
      });

  py::class_<RoutePlanner>(m, "RoutePlanner")
      .def(py::init([](const std::string& profile_text, double min_obstacle_height_m,
                       double resolution_m) {
             const auto profile = argus::capabilities::load_profile_text(profile_text);
             const auto caps = argus::capabilities::derive_capabilities(profile);
             return std::make_unique<RoutePlanner>(
                 RouteBody::from(profile, caps, min_obstacle_height_m, resolution_m));
           }),
           py::arg("profile_text"), py::arg("min_obstacle_height_m") = 0.05,
           py::arg("resolution_m") = 0.1)
      .def_property_readonly("planar", [](const RoutePlanner& p) { return p.body().planar; })
      .def_property_readonly("min_clearance_m",
                             [](const RoutePlanner& p) { return p.body().min_clearance_m; })
      .def_property_readonly("turn_clearance_m",
                             [](const RoutePlanner& p) { return p.body().turn_clearance_m; })
      .def(
          "plan",
          [](RoutePlanner& planner, const SiteMap& map, const py::sequence& start,
             const py::sequence& goal, double margin_m, double unknown_penalty,
             double aerial_resolution_m, double max_leg_m, int max_expansions,
             const std::vector<std::vector<double>>& blocked, double z_min_m, double z_max_m) {
            RouteRequest req;
            req.z_min_m = z_min_m;
            req.z_max_m = z_max_m;
            for (const auto& b : blocked) {
              if (b.size() != 4) throw py::value_error("blocked zones are (x, y, z, radius)");
              req.blocked.emplace_back(b[0], b[1], b[2], b[3]);
            }
            req.start = vec3(start);
            req.goal = vec3(goal);
            req.margin_m = margin_m;
            req.unknown_penalty = unknown_penalty;
            req.aerial_resolution_m = aerial_resolution_m;
            req.max_leg_m = max_leg_m;
            req.max_expansions = max_expansions;
            Route route;
            {
              py::gil_scoped_release release;
              route = planner.plan(map, req);
            }
            py::list waypoints;
            for (const RouteWaypoint& w : route.waypoints) {
              py::dict d;
              d["position"] = xyz(w.position);
              d["speculative"] = w.speculative;
              waypoints.append(d);
            }
            py::array_t<double> path({static_cast<py::ssize_t>(route.path.size()), py::ssize_t(3)});
            auto out = path.mutable_unchecked<2>();
            for (size_t k = 0; k < route.path.size(); ++k) {
              for (int a = 0; a < 3; ++a) out(k, a) = route.path[k][a];
            }
            py::dict d;
            d["reached"] = route.reached;
            d["exhausted"] = route.exhausted;
            d["goal_blocked"] = route.goal_blocked;
            d["waypoints"] = waypoints;
            d["path"] = path;
            d["length_m"] = route.length_m;
            d["unknown_m"] = route.unknown_m;
            return d;
          },
          py::arg("site_map"), py::arg("start"), py::arg("goal"), py::arg("margin_m") = 3.0,
          py::arg("unknown_penalty") = 2.0, py::arg("aerial_resolution_m") = 0.2,
          py::arg("max_leg_m") = 3.0, py::arg("max_expansions") = 2000000,
          py::arg("blocked") = std::vector<std::vector<double>>{},
          py::arg("z_min_m") = std::numeric_limits<double>::quiet_NaN(),
          py::arg("z_max_m") = std::numeric_limits<double>::quiet_NaN());
}
