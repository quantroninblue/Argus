#include <cmath>
#include <random>
#include <stdexcept>

#include <gtest/gtest.h>

#include "argus/grid/evidence.hpp"
#include "argus/grid/trace.hpp"

using namespace argus::grid;

TEST(Grid, TileIndexingRoundTrips) {
  for (const Index3& c : {Index3(0, 0, 0), Index3(15, 15, 15), Index3(16, 0, -1),
                          Index3(-1, -16, -17), Index3(-300, 47, 5)}) {
    const Index3 t = tile_of(c);
    const int k = offset_in_tile(c);
    ASSERT_GE(k, 0);
    ASSERT_LT(k, kTileCells);
    EXPECT_EQ(cell_at(t, k), c);
  }
  EXPECT_EQ(tile_of(Index3(-1, -16, -17)), Index3(-1, -1, -2));
}

TEST(Grid, EvidenceRoundTripSumsAndClips) {
  const std::vector<CellDelta> in = {
      {Index3(1, 2, 3), kMiss}, {Index3(1, 2, 3), kMiss}, {Index3(-20, 5, 0), kHit},
      {Index3(40, 40, 1), 0},   {Index3(7, 7, 7), 100}};
  std::vector<CellDelta> out = decode_evidence(encode_evidence(in));
  ASSERT_EQ(out.size(), 4u);
  auto find = [&](const Index3& i) {
    for (const CellDelta& d : out) {
      if (d.index == i) return d.delta;
    }
    ADD_FAILURE() << "missing cell";
    return -999;
  };
  EXPECT_EQ(find(Index3(1, 2, 3)), 2 * kMiss);
  EXPECT_EQ(find(Index3(-20, 5, 0)), kHit);
  EXPECT_EQ(find(Index3(40, 40, 1)), 0);
  EXPECT_EQ(find(Index3(7, 7, 7)), kDeltaLimit);
}

TEST(Grid, EvidenceEncodingIsDeterministic) {
  std::vector<CellDelta> a = {{Index3(5, 0, 0), 1}, {Index3(-5, 0, 0), 2}, {Index3(30, 1, 0), 3}};
  std::vector<CellDelta> b = {a[2], a[0], a[1]};
  EXPECT_EQ(encode_evidence(a), encode_evidence(b));
}

TEST(Grid, MalformedEvidenceIsRefused) {
  const std::string good = encode_evidence({{Index3(1, 1, 1), kHit}, {Index3(99, 0, 0), kMiss}});
  for (size_t n = 0; n < good.size(); ++n) {
    EXPECT_THROW(decode_evidence(good.substr(0, n)), std::invalid_argument) << "length " << n;
  }
  EXPECT_THROW(decode_evidence(good + "x"), std::invalid_argument);

  std::string bad_offset = good;
  bad_offset[18] = static_cast<char>(0xff);  // first cell offset, high byte
  bad_offset[17] = static_cast<char>(0xff);
  EXPECT_THROW(decode_evidence(bad_offset), std::invalid_argument);

  std::string huge_count = good;
  huge_count[0] = huge_count[1] = huge_count[2] = huge_count[3] = static_cast<char>(0xff);
  EXPECT_THROW(decode_evidence(huge_count), std::invalid_argument);
}

TEST(Grid, RandomBytesNeverCrashTheDecoder) {
  std::mt19937 rng(7);
  for (int trial = 0; trial < 2000; ++trial) {
    std::string bytes(rng() % 64, '\0');
    for (char& c : bytes) c = static_cast<char>(rng());
    try {
      decode_evidence(bytes);
    } catch (const std::invalid_argument&) {
    }
  }
}

TEST(Grid, RaysEndingOnCellBoundariesKeepTheirHit) {
  // Wall faces on grid lines put ray ends exactly on cell boundaries, where rounding in the
  // walk used to stop it one cell short and drop the hit.
  std::mt19937 rng(11);
  std::uniform_real_distribution<double> u(-3.0, 3.0);
  FrameTracer tracer;
  const double res = 0.1;
  int checked = 0;
  for (int trial = 0; trial < 3000; ++trial) {
    const Vec3 origin(u(rng), u(rng), 0.3 + 0.1 * u(rng));
    Vec3 end(std::round(u(rng) * 10) / 10, u(rng), origin.z());
    if (trial % 2) std::swap(end.x(), end.y());
    if ((end - origin).norm() < 0.2) continue;
    std::vector<Index3> hit_cells;
    std::vector<Index3> walked;
    tracer.trace(
        origin, {end}, {true}, res, [](const Index3&) { return true; },
        [&](const Index3& i, int delta) { (delta > 0 ? hit_cells : walked).push_back(i); });
    ASSERT_EQ(hit_cells.size(), 1u) << "origin " << origin.transpose() << " end " << end.transpose();
    EXPECT_EQ(hit_cells[0], index_of(end, res));
    // The walk is connected: each cell is a face neighbour of the one before.
    walked.push_back(hit_cells[0]);
    EXPECT_EQ(walked.front(), index_of(origin, res));
    for (size_t k = 1; k < walked.size(); ++k) {
      ASSERT_EQ((walked[k] - walked[k - 1]).cwiseAbs().sum(), 1);
    }
    ++checked;
  }
  EXPECT_GT(checked, 2500);
}
