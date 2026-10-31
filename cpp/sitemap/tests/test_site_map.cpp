#include <cmath>
#include <random>
#include <stdexcept>

#include <gtest/gtest.h>

#include "argus/node/local_map.hpp"
#include "argus/sitemap/site_map.hpp"

using namespace argus::sitemap;

namespace {

// A fan of horizontal rays from `origin` hitting a wall at x = wall_x.
void wall_frame(const Vec3& origin, double wall_x, std::vector<Vec3>& ends, std::vector<bool>& hits) {
  ends.clear();
  hits.clear();
  for (int k = -20; k <= 20; ++k) {
    const double a = k * 0.03;
    const double r = (wall_x - origin.x()) / std::cos(a);
    ends.push_back(origin + Vec3(std::cos(a), std::sin(a), 0.0) * r);
    hits.push_back(true);
  }
}

}  // namespace

TEST(SiteMap, IntegratesLikeTheLocalMap) {
  SiteMap map("site-a");
  std::vector<Vec3> ends;
  std::vector<bool> hits;
  wall_frame(Vec3(0.05, 0.05, 0.25), 3.05, ends, hits);
  map.integrate(Vec3(0.05, 0.05, 0.25), ends, hits, 1.0);
  EXPECT_EQ(map.at(Vec3(1.5, 0.05, 0.25)), Cell::Free);
  EXPECT_EQ(map.at(Vec3(3.05, 0.05, 0.25)), Cell::Occupied);
  EXPECT_EQ(map.at(Vec3(4.0, 0.05, 0.25)), Cell::Unknown);
  EXPECT_EQ(map.at(Vec3(-40.0, 3.0, -2.0)), Cell::Unknown);
}

TEST(SiteMap, EvidenceFromArgoReproducesItsCells) {
  argus::node::LocalMapConfig config;
  config.half_width_m = 4.0;
  config.z_below_m = 0.5;
  config.z_above_m = 1.0;
  config.record_evidence = true;
  argus::node::LocalMap local(config);
  SiteMap site("site-a");

  std::vector<Vec3> ends;
  std::vector<bool> hits;
  // Drive along x so the window moves and recorded slots get reused.
  for (int step = 0; step < 30; ++step) {
    const Vec3 origin(-2.0 + 0.13 * step, 0.31, 0.25);
    local.recenter(origin);
    wall_frame(origin, 2.95, ends, hits);
    local.integrate(origin, ends, hits);
    if (step % 4 == 0) {
      ASSERT_TRUE(site.apply(site.epoch(), argus::grid::decode_evidence(
                                               argus::grid::encode_evidence(local.take_evidence())),
                             step));
    }
  }
  ASSERT_TRUE(site.apply(site.epoch(), local.take_evidence(), 30.0));

  int compared = 0;
  const auto& lo = local.window_min();
  const auto& dims = local.dims();
  for (int x = 0; x < dims.x(); ++x) {
    for (int y = 0; y < dims.y(); ++y) {
      for (int z = 0; z < dims.z(); ++z) {
        const Index3 i = lo + Index3(x, y, z);
        const Cell want = local.cell(i);
        if (want != Cell::Unknown) {
          ++compared;
        }
        ASSERT_EQ(site.cell(i), want) << i.transpose();
      }
    }
  }
  EXPECT_GT(compared, 100);
  // Cells that left Argo's window are still in the site map.
  EXPECT_EQ(site.at(Vec3(-2.0, 0.31, 0.25)), Cell::Free);
}

TEST(SiteMap, SettledCellsAreNotSentAgain) {
  argus::node::LocalMapConfig config;
  config.half_width_m = 4.0;
  config.z_below_m = 0.5;
  config.z_above_m = 1.0;
  config.record_evidence = true;
  argus::node::LocalMap local(config);
  SiteMap site("site-a");

  std::vector<Vec3> ends;
  std::vector<bool> hits;
  const Vec3 origin(0.0, 0.31, 0.25);
  local.recenter(origin);
  wall_frame(origin, 2.95, ends, hits);
  size_t first = 0;
  size_t last = 0;
  for (int frame = 0; frame < 20; ++frame) {
    local.integrate(origin, ends, hits);
    const auto deltas = local.take_evidence();
    if (frame == 0) first = deltas.size();
    last = deltas.size();
    ASSERT_TRUE(site.apply(site.epoch(), deltas, frame));
  }
  // Free cells saturate after ten misses and hits after two; then a static view costs nothing.
  EXPECT_GT(first, 50u);
  EXPECT_EQ(last, 0u);
  const auto& lo = local.window_min();
  const auto& dims = local.dims();
  for (int x = 0; x < dims.x(); ++x) {
    for (int y = 0; y < dims.y(); ++y) {
      for (int z = 0; z < dims.z(); ++z) {
        const Index3 i = lo + Index3(x, y, z);
        ASSERT_EQ(site.cell(i), local.cell(i)) << i.transpose();
      }
    }
  }
}

TEST(SiteMap, StaleEpochIsRefused) {
  SiteMap map("site-a");
  const uint64_t old = map.epoch();
  ASSERT_TRUE(map.apply(old, {{Index3(1, 1, 1), argus::grid::kHit}}, 0.0));
  map.reset();
  EXPECT_EQ(map.cell(Index3(1, 1, 1)), Cell::Unknown);
  EXPECT_FALSE(map.apply(old, {{Index3(2, 2, 2), argus::grid::kHit}}, 0.0));
  EXPECT_EQ(map.tile_count(), 0u);
  EXPECT_TRUE(map.apply(map.epoch(), {{Index3(2, 2, 2), argus::grid::kHit}}, 0.0));
  EXPECT_EQ(map.cell(Index3(2, 2, 2)), Cell::Occupied);
}

TEST(SiteMap, PlanarViewUsesTheBodyBand) {
  SiteMap map("site-a");
  using argus::grid::kHit;
  using argus::grid::kMiss;
  // Column A: free at 0.25 m, occupied at 1.55 m (a shelf above a low body).
  // Column B: occupied at 0.05 m (below the band). Column C: seen nowhere.
  map.apply(map.epoch(),
            {{Index3(10, 10, 2), kMiss}, {Index3(10, 10, 15), kHit}, {Index3(20, 10, 0), kHit},
             {Index3(20, 10, 3), kMiss}},
            0.0);
  const PlanarView low = map.planar_view(0.1, 0.6);
  auto at = [&](const PlanarView& v, int x, int y) { return v.at(x - v.origin.x(), y - v.origin.y()); };
  EXPECT_EQ(at(low, 10, 10), Cell::Free);
  EXPECT_EQ(at(low, 20, 10), Cell::Free);
  EXPECT_EQ(at(low, 15, 10), Cell::Unknown);
  const PlanarView tall = map.planar_view(0.1, 1.8);
  EXPECT_EQ(at(tall, 10, 10), Cell::Occupied);
  const PlanarView floor = map.planar_view(0.0, 0.6);
  EXPECT_EQ(at(floor, 20, 10), Cell::Occupied);
}

TEST(SiteMap, PlanarViewMatchesCellByCell) {
  SiteMap map("site-a");
  std::mt19937 rng(3);
  std::vector<CellDelta> deltas;
  for (int k = 0; k < 4000; ++k) {
    deltas.push_back({Index3(static_cast<int>(rng() % 70) - 35, static_cast<int>(rng() % 70) - 35,
                             static_cast<int>(rng() % 30) - 5),
                      (rng() % 3 == 0) ? argus::grid::kHit : argus::grid::kMiss});
  }
  map.apply(map.epoch(), deltas, 0.0);
  const double z_low = 0.05;
  const double z_high = 1.25;
  const PlanarView v = map.planar_view(z_low, z_high);
  const int z0 = argus::grid::index_of(Vec3(0, 0, z_low), 0.1).z();
  const int z1 = argus::grid::index_of(Vec3(0, 0, z_high), 0.1).z();
  for (int x = 0; x < v.nx; ++x) {
    for (int y = 0; y < v.ny; ++y) {
      Cell want = Cell::Unknown;
      for (int z = z0; z <= z1; ++z) {
        want = std::max(want, map.cell(Index3(v.origin.x() + x, v.origin.y() + y, z)));
      }
      ASSERT_EQ(v.at(x, y), want);
    }
  }
}

TEST(SiteMap, DeltasCarryOnlyChangedTiles) {
  SiteMap map("site-a");
  map.apply(map.epoch(), {{Index3(1, 1, 1), 6}, {Index3(40, 1, 1), -2}}, 1.0);
  const uint64_t seen = map.version();
  map.apply(map.epoch(), {{Index3(41, 2, 1), 6}}, 2.0);
  const auto changed = map.changed_since(seen);
  ASSERT_EQ(changed.size(), 1u);
  EXPECT_EQ(changed[0], argus::grid::tile_of(Index3(41, 2, 1)));

  SiteMap mirror("site-a");
  mirror.load_tiles(map.encode_all());
  EXPECT_EQ(mirror.tile_count(), map.tile_count());
  EXPECT_EQ(mirror.cell(Index3(1, 1, 1)), Cell::Occupied);
  EXPECT_EQ(mirror.cell(Index3(40, 1, 1)), Cell::Free);
  EXPECT_EQ(mirror.cell(Index3(41, 2, 1)), Cell::Occupied);
  EXPECT_EQ(mirror.cell(Index3(2, 2, 2)), Cell::Unknown);
  EXPECT_EQ(mirror.version(), map.version());
}

TEST(SiteMap, MalformedTilesChangeNothing) {
  SiteMap map("site-a");
  map.apply(map.epoch(), {{Index3(1, 1, 1), 6}}, 1.0);
  const std::string good = map.encode_all();
  SiteMap target("site-a");
  target.apply(target.epoch(), {{Index3(100, 0, 0), 6}}, 1.0);
  for (size_t n : {size_t(0), size_t(3), size_t(20), good.size() - 1}) {
    EXPECT_THROW(target.load_tiles(good.substr(0, n)), std::invalid_argument);
  }
  std::string bad = good;
  bad[4 + 12 + 16] = static_cast<char>(100);  // first log-odds value, out of range
  EXPECT_THROW(target.load_tiles(bad), std::invalid_argument);
  EXPECT_EQ(target.tile_count(), 1u);
  EXPECT_EQ(target.cell(Index3(1, 1, 1)), Cell::Unknown);
}
