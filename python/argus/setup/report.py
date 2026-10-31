"""A plain-text capability report for a body profile: what the body can do and see, where it
is blind, and what is still unverified."""

from __future__ import annotations

from argus import profile as profiles


def _ranges(values: list[float], step: float) -> str:
    """Bearings as compact ranges: 0-30, 90, 300-355."""
    if not values:
        return "none"
    values = sorted(values)
    out, start, prev = [], values[0], values[0]
    for v in values[1:]:
        if v - prev > step + 1e-6:
            out.append(f"{start:g}" if start == prev else f"{start:g}-{prev:g}")
            start = v
        prev = v
    out.append(f"{start:g}" if start == prev else f"{start:g}-{prev:g}")
    return ", ".join(out)


def capability_report(doc: dict) -> str:
    caps = profiles.derive(doc)
    m, c, s = caps.motion, caps.collision, caps.sensing
    lines = [f"Profile {doc.get('id')} ({doc.get('name', '')})", ""]
    lines.append("Motion")
    lines.append(f"  space {m.space}, reverse {'yes' if m.can_reverse else 'no'}, strafe "
                 f"{'yes' if m.can_strafe else 'no'}, turn in place {'yes' if m.can_turn_in_place else 'no'}"
                 + (f", min turn radius {m.min_turn_radius_m:g} m" if m.min_turn_radius_m > 0 else ""))
    for name, axis in (("x", m.x), ("y", m.y), ("z", m.z)):
        if axis.max_mps > 0 or axis.min_mps < 0:
            top = max(axis.max_mps, -axis.min_mps)
            stop = top * top / (2 * axis.decel_mps2) if axis.decel_mps2 > 0 else float("inf")
            lines.append(f"  {name}: {axis.min_mps:g} to {axis.max_mps:g} m/s, braking {axis.decel_mps2:g} m/s2, "
                         f"stops from top speed in {stop:.2f} m (plus {m.latency_s:g} s latency)")
    lines.append(f"  yaw: {m.yaw_rate_max_radps:g} rad/s")
    lines.append("")
    lines.append("Body")
    lo, hi = c.aabb_min, c.aabb_max
    lines.append(f"  size {hi[0] - lo[0]:.2f} x {hi[1] - lo[1]:.2f} x {hi[2] - lo[2]:.2f} m, "
                 f"radius {c.body_radius_m:.2f} m, padding {c.padding_m:g} m")
    lines.append("")
    lines.append("Sensing")
    level = [b for b in s.bins if b.elevation_deg == 0.0] or list(s.bins)
    step = 360.0 / len(level) if level else 0.0
    seen = [b.azimuth_deg for b in level if b.full_height_reach_m > 0.0]
    partial = [b.azimuth_deg for b in level if b.full_height_reach_m <= 0.0 and b.max_seen_m > 0.0]
    blind = [b.azimuth_deg for b in level if b.max_seen_m <= 0.0]
    lines.append(f"  seen at every height (deg): {_ranges(seen, step)}")
    lines.append(f"  seen at some heights only:  {_ranges(partial, step)}")
    lines.append(f"  blind:                      {_ranges(blind, step)}")
    bands = sorted({(round(u.low_m, 2), round(u.high_m, 2)) for b in level for u in b.unseen_bands})
    if bands:
        lines.append("  height bands never seen: " + ", ".join(f"{a:g}-{b:g} m" for a, b in bands[:6])
                     + (" ..." if len(bands) > 6 else ""))
    loc = caps.localization
    sources = [n for n, v in (("wheel odometry", loc.wheel_odometry), ("external odometry", loc.external_odometry),
                              ("inertial", loc.inertial), ("visual", loc.visual), ("range", loc.range)) if v]
    lines.append(f"  localization from: {', '.join(sources) or 'nothing'}")
    lines.append(f"  obstacles lower than {caps.min_obstacle_height_m * 100:.0f} cm are treated as ground")
    if caps.assumed_obstacle_height_m > 0:
        lines.append(f"  assumed (declared): obstacles stand on the ground and are at least "
                     f"{caps.assumed_obstacle_height_m * 100:.0f} cm tall")
    lines.append("")
    prov = doc.get("provenance", {})
    unverified = sorted(k for k, v in prov.items() if v in ("declared", "cad", "llm_draft"))
    lines.append("Still to verify: " + (", ".join(unverified) if unverified else "nothing"))
    lines.append("Ready to drive: " + ("yes" if caps.safety_ready else
                                       "no, unreviewed: " + ", ".join(caps.unreviewed_fields)))
    return "\n".join(lines)
