"""Render the virtual room as a self-contained SVG string.

Pure function of controller state, so it can be unit-tested without
Streamlit. Shapes use patterns, line styles and text labels as well as
colour, so the picture still reads in greyscale.
"""

from __future__ import annotations

import math
from html import escape

from src.controller import RobotController
from src.models import STATE_INFO, RobotState, Verdict
from src.navigation import OccupancyGrid, RouteStatus

SCALE = 140  # pixels per metre
MARGIN = 36

BEAM_STYLE = {
    Verdict.CLEAR: ("#2563eb", "0.18", "none"),
    Verdict.OBSTACLE: ("#d97706", "0.30", "none"),
    Verdict.INVALID: ("#6b7280", "0.22", "6 4"),
}


def render_room_svg(controller: RobotController, show_grid: bool = True) -> str:
    sim = controller.sim
    world = sim.world
    width_px = world.width_m * SCALE + 2 * MARGIN
    height_px = world.height_m * SCALE + 2 * MARGIN

    def sx(x: float) -> float:
        return MARGIN + x * SCALE

    def sy(y: float) -> float:
        return MARGIN + (world.height_m - y) * SCALE

    pose = sim.pose
    info = STATE_INFO[controller.state]
    assessment = controller.last_assessment
    verdict = assessment.verdict if assessment else Verdict.INVALID
    parts: list[str] = []
    add = parts.append

    summary = (
        f"Simulated room {world.width_m:g} by {world.height_m:g} metres. Robot at x {pose.x:.2f}, "
        f"y {pose.y:.2f}, heading {pose.heading_deg:.0f} degrees. State {controller.state.value}. "
        f"Sensor verdict {verdict.value}."
    )
    add(
        f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {width_px:.0f} {height_px:.0f}" '
        f'width="100%" role="img" aria-label="{escape(summary)}" '
        f'style="max-width:{width_px:.0f}px;font-family:sans-serif;display:block">'
    )
    add(f"<title>Simulated room map</title><desc>{escape(summary)}</desc>")
    add(
        "<defs>"
        '<pattern id="hatch" patternUnits="userSpaceOnUse" width="8" height="8" patternTransform="rotate(45)">'
        '<rect width="8" height="8" fill="#e5e7eb"/><line x1="0" y1="0" x2="0" y2="8" stroke="#374151" stroke-width="3"/>'
        "</pattern>"
        '<marker id="arrow" viewBox="0 0 10 10" refX="5" refY="5" markerWidth="5" markerHeight="5" orient="auto-start-reverse">'
        '<path d="M 0 0 L 10 5 L 0 10 z" fill="#111827"/></marker>'
        "</defs>"
    )
    add(f'<rect x="0" y="0" width="{width_px:.0f}" height="{height_px:.0f}" fill="#ffffff"/>')
    add(
        f'<rect x="{sx(0):.1f}" y="{sy(world.height_m):.1f}" width="{world.width_m * SCALE:.1f}" '
        f'height="{world.height_m * SCALE:.1f}" fill="#f8fafc"/>'
    )

    # Grid every 0.5 m with labels every 1 m.
    steps_x = int(round(world.width_m / 0.5))
    steps_y = int(round(world.height_m / 0.5))
    for i in range(steps_x + 1):
        x = i * 0.5
        add(f'<line x1="{sx(x):.1f}" y1="{sy(0):.1f}" x2="{sx(x):.1f}" y2="{sy(world.height_m):.1f}" stroke="#e2e8f0" stroke-width="1"/>')
        if i % 2 == 0:
            add(f'<text x="{sx(x):.1f}" y="{sy(0) + 18:.1f}" font-size="11" text-anchor="middle" fill="#475569">{x:g} m</text>')
    for j in range(steps_y + 1):
        y = j * 0.5
        add(f'<line x1="{sx(0):.1f}" y1="{sy(y):.1f}" x2="{sx(world.width_m):.1f}" y2="{sy(y):.1f}" stroke="#e2e8f0" stroke-width="1"/>')
        if j % 2 == 0:
            add(f'<text x="{sx(0) - 6:.1f}" y="{sy(y) + 4:.1f}" font-size="11" text-anchor="end" fill="#475569">{y:g}</text>')

    # Walls.
    add(
        f'<rect x="{sx(0):.1f}" y="{sy(world.height_m):.1f}" width="{world.width_m * SCALE:.1f}" '
        f'height="{world.height_m * SCALE:.1f}" fill="none" stroke="#111827" stroke-width="4"/>'
    )

    # Obstacles.
    for o in world.obstacles:
        add(
            f'<rect x="{sx(o.x):.1f}" y="{sy(o.y_max):.1f}" width="{o.width * SCALE:.1f}" height="{o.height * SCALE:.1f}" '
            f'fill="url(#hatch)" stroke="#111827" stroke-width="2"><title>{escape(o.name)}</title></rect>'
        )
        add(
            f'<text x="{sx(o.x + o.width / 2):.1f}" y="{sy(o.y_max) - 5:.1f}" font-size="12" font-weight="bold" '
            f'text-anchor="middle" fill="#111827">{escape(o.name)}</text>'
        )

    # Planning grid: cells where the robot centre may not go (inflated walls/obstacles).
    if show_grid:
        add(_grid_svg(controller.occupancy_grid(), sx, sy))

    # Planned route with start and goal markers.
    nav = controller.navigator
    if nav is not None:
        blocked = nav.status is RouteStatus.BLOCKED
        points = (nav.plan.start, *nav.plan.waypoints)
        route = " ".join(f"{sx(x):.1f},{sy(y):.1f}" for x, y in points)
        dash = ' stroke-dasharray="10 6"' if blocked else ""
        colour = "#6b7280" if blocked else "#7c3aed"
        add(f'<polyline points="{route}" fill="none" stroke="{colour}" stroke-width="3.5" '
            f'stroke-linejoin="round"{dash}><title>Planned route</title></polyline>')
        for x, y in nav.plan.waypoints[:-1]:
            add(f'<circle cx="{sx(x):.1f}" cy="{sy(y):.1f}" r="3" fill="{colour}"/>')
        (stx, sty), (gx, gy) = nav.plan.start, nav.plan.goal
        add(f'<rect x="{sx(stx) - 8:.1f}" y="{sy(sty) - 8:.1f}" width="16" height="16" fill="#ffffff" '
            f'stroke="{colour}" stroke-width="2.5"><title>Route start</title></rect>')
        add(f'<text x="{sx(stx):.1f}" y="{sy(sty) + 4:.1f}" font-size="11" font-weight="bold" '
            f'text-anchor="middle" fill="{colour}">S</text>')
        add(f'<circle cx="{sx(gx):.1f}" cy="{sy(gy):.1f}" r="11" fill="#ffffff" stroke="{colour}" stroke-width="3">'
            f'<title>Destination: {escape(nav.label)}</title></circle>')
        add(f'<text x="{sx(gx):.1f}" y="{sy(gy) + 4:.1f}" font-size="12" font-weight="bold" '
            f'text-anchor="middle" fill="{colour}">G</text>')
        tag = f"GOAL: {nav.label}" + (" (ROUTE BLOCKED)" if blocked else "")
        add(f'<text x="{sx(gx):.1f}" y="{sy(gy) - 16:.1f}" font-size="12" font-weight="bold" text-anchor="middle" '
            f'fill="{colour}" stroke="#ffffff" stroke-width="3" paint-order="stroke">{escape(tag)}</text>')

    # Trail.
    if len(sim.trail) > 1:
        points = " ".join(f"{sx(x):.1f},{sy(y):.1f}" for x, y in sim.trail)
        add(f'<polyline points="{points}" fill="none" stroke="#64748b" stroke-width="2" stroke-dasharray="3 4"/>')

    # Sensor beam: drawn to the measured distance when valid, else to max range (dashed grey).
    ox, oy = sim.sensor_origin()
    sensor_cfg = controller.config.sensor
    beam_len = assessment.distance_m if assessment and assessment.distance_m is not None else sensor_cfg.max_range_m
    half = math.radians(sensor_cfg.beam_width_deg) / 2
    colour, opacity, dash = BEAM_STYLE[verdict]
    a1, a2 = pose.heading_rad - half, pose.heading_rad + half
    p1 = (ox + beam_len * math.cos(a1), oy + beam_len * math.sin(a1))
    p2 = (ox + beam_len * math.cos(a2), oy + beam_len * math.sin(a2))
    r_px = beam_len * SCALE
    add(
        f'<path d="M {sx(ox):.1f} {sy(oy):.1f} L {sx(p1[0]):.1f} {sy(p1[1]):.1f} '
        f'A {r_px:.1f} {r_px:.1f} 0 0 1 {sx(p2[0]):.1f} {sy(p2[1]):.1f} Z" '
        f'fill="{colour}" fill-opacity="{opacity}" stroke="{colour}" stroke-width="1.5" stroke-dasharray="{dash}"/>'
    )

    # Stop-threshold arc.
    t = controller.stop_threshold_m
    t1 = (ox + t * math.cos(a1 - 0.15), oy + t * math.sin(a1 - 0.15))
    t2 = (ox + t * math.cos(a2 + 0.15), oy + t * math.sin(a2 + 0.15))
    add(
        f'<path d="M {sx(t1[0]):.1f} {sy(t1[1]):.1f} A {t * SCALE:.1f} {t * SCALE:.1f} 0 0 1 {sx(t2[0]):.1f} {sy(t2[1]):.1f}" '
        f'fill="none" stroke="#b91c1c" stroke-width="2.5" stroke-dasharray="5 3"><title>Stop threshold {t:.2f} m</title></path>'
    )

    # Robot body and heading arrow.
    r = controller.config.robot.radius_m
    hx = pose.x + r * 0.9 * math.cos(pose.heading_rad)
    hy = pose.y + r * 0.9 * math.sin(pose.heading_rad)
    add(
        f'<circle cx="{sx(pose.x):.1f}" cy="{sy(pose.y):.1f}" r="{r * SCALE:.1f}" fill="#fde68a" '
        f'stroke="#111827" stroke-width="2.5"><title>Robot</title></circle>'
    )
    add(
        f'<line x1="{sx(pose.x):.1f}" y1="{sy(pose.y):.1f}" x2="{sx(hx):.1f}" y2="{sy(hy):.1f}" '
        f'stroke="#111827" stroke-width="3" marker-end="url(#arrow)"/>'
    )
    add(f'<circle cx="{sx(ox):.1f}" cy="{sy(oy):.1f}" r="4" fill="#111827"><title>Ultrasonic sensor</title></circle>')

    # Labels.
    add(f'<text x="{MARGIN}" y="22" font-size="13" font-weight="bold" fill="#1d4ed8">SIMULATED - no hardware</text>')
    add(
        f'<text x="{width_px - MARGIN:.0f}" y="22" font-size="13" font-weight="bold" text-anchor="end" '
        f'fill="#111827">{escape(info.symbol)} {escape(info.label)}</text>'
    )
    if controller.state in (RobotState.EMERGENCY_STOP, RobotState.FAULT):
        add(
            f'<rect x="{sx(0) + 3:.1f}" y="{sy(world.height_m) + 3:.1f}" width="{world.width_m * SCALE - 6:.1f}" '
            f'height="{world.height_m * SCALE - 6:.1f}" fill="none" stroke="#b91c1c" stroke-width="6" stroke-dasharray="14 8"/>'
        )
        banner_y = sy(0) - 44
        add(
            f'<rect x="{width_px / 2 - 150:.0f}" y="{banner_y:.1f}" width="300" height="34" rx="4" '
            f'fill="#ffffff" stroke="#b91c1c" stroke-width="2"/>'
        )
        add(
            f'<text x="{width_px / 2:.0f}" y="{banner_y + 24:.1f}" font-size="20" font-weight="bold" '
            f'text-anchor="middle" fill="#b91c1c">{escape(info.symbol)} {escape(info.label)}</text>'
        )
    add("</svg>")
    return "".join(parts)


def _grid_svg(grid: OccupancyGrid, sx, sy) -> str:
    """Blocked-but-not-occupied cells as merged horizontal runs (keeps the SVG small)."""
    res = grid.resolution_m
    rects = []
    for row in range(grid.rows):
        col = 0
        while col < grid.cols:
            if (col, row) in grid.blocked and (col, row) not in grid.occupied:
                start = col
                while col < grid.cols and (col, row) in grid.blocked and (col, row) not in grid.occupied:
                    col += 1
                x0, x1, y1 = start * res, col * res, (row + 1) * res
                rects.append(f'<rect x="{sx(x0):.1f}" y="{sy(y1):.1f}" width="{(x1 - x0) * SCALE:.1f}" '
                             f'height="{res * SCALE:.1f}"/>')
            else:
                col += 1
    return ('<g fill="#94a3b8" fill-opacity="0.28"><title>No-go zone for the robot centre '
            f'(robot radius + safety margin = {grid.inflation_radius_m:.2f} m)</title>' + "".join(rects) + "</g>")
