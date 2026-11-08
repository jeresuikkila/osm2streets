use anyhow::Result;
use geojson::Feature;
use geom::{Distance, Line, PolyLine, Polygon, Pt2D, Ring};

use super::lane_markings::{LinePattern, LineStyle, MarkingStyle};
use super::{serialize_features, Filter};
use crate::road::RoadEdge;
use crate::{CrossingKind, DrivingSide, Intersection, LaneType, Road, StreetNetwork};

impl StreetNetwork {
    pub fn to_intersection_markings_geojson(&self, filter: &Filter) -> Result<String> {
        serialize_features(self.intersection_marking_features(filter)?)
    }

    /// The features of [`Self::to_intersection_markings_geojson`].
    pub fn intersection_marking_features(&self, filter: &Filter) -> Result<Vec<Feature>> {
        self.intersection_marking_features_styled(filter, &MarkingStyle::default(), false)
    }

    /// [`Self::intersection_marking_features`] in a given style; with `connect_seams`, the solid lane lines of two
    /// roads that only continue each other (exactly two roads with traffic lanes meet, no crossing: where a way's
    /// lanes, layer, name or bridge tag change) run on through the intersection between them instead of stopping at
    /// its edges.
    pub fn intersection_marking_features_styled(
        &self,
        filter: &Filter,
        style: &MarkingStyle,
        connect_seams: bool,
    ) -> Result<Vec<Feature>> {
        let mut features = Vec::new();
        for intersection in filter.intersections(self) {
            let first = features.len();
            for polygon in make_sidewalk_corners(self, intersection) {
                let mut f = Feature::from(polygon.to_geojson(Some(&self.gps_bounds)));
                f.set_property("type", "sidewalk corner");
                features.push(f);
            }

            if let Some(ref crossing) = intersection.crossing {
                match crossing.kind {
                    CrossingKind::Signalized | CrossingKind::Marked => {
                        for polygon in draw_zebra_crossing(self, intersection) {
                            let mut f = Feature::from(polygon.to_geojson(Some(&self.gps_bounds)));
                            f.set_property("type", "marked crossing line");
                            features.push(f);
                        }
                    }
                    CrossingKind::Unmarked => {
                        for polygon in draw_unmarked_crossing(self, intersection) {
                            let mut f = Feature::from(polygon.to_geojson(Some(&self.gps_bounds)));
                            f.set_property("type", "unmarked crossing outline");
                            features.push(f);
                        }
                    }
                }
            }
            if connect_seams && intersection.crossing.is_none() {
                for (polygon, kind) in seam_lines(self, intersection, style) {
                    let mut f = Feature::from(polygon.to_geojson(Some(&self.gps_bounds)));
                    f.set_property("type", kind);
                    features.push(f);
                }
            }
            for f in &mut features[first..] {
                f.set_property("intersection", intersection.id.0);
            }
        }
        Ok(features)
    }
}

/// For an intersection, show all corners where sidewalks meet.
fn make_sidewalk_corners(streets: &StreetNetwork, intersection: &Intersection) -> Vec<Polygon> {
    // Look at every adjacent pair of edges
    let mut edges = match RoadEdge::try_calculate(
        streets.roads_per_intersection(intersection.id),
        intersection.id,
    ) {
        Ok(edges) => edges,
        Err(err) => {
            warn!(
                "Skipping sidewalk corners of intersection {:?}: {}",
                intersection.osm_ids, err
            );
            return Vec::new();
        }
    };
    if edges.is_empty() {
        error!("Intersection {:?} has no edges", intersection.osm_ids);
        return Vec::new();
    }
    edges.push(edges[0].clone());
    let mut results = Vec::new();
    for pair in edges.windows(2) {
        let one = &pair[0];
        let two = &pair[1];

        // Only want corners between two roads
        if one.road == two.road {
            continue;
        }

        // Only want two sidewalks or shoulders
        if !(one.lane.lt == LaneType::Sidewalk || one.lane.lt == LaneType::Shoulder) {
            continue;
        }
        if !(two.lane.lt == LaneType::Sidewalk || two.lane.lt == LaneType::Shoulder) {
            continue;
        }

        // Only want roads with more than just a sidewalk/shoulder lane
        if streets.roads[&one.road].lane_specs_ltr.len() == 1
            || streets.roads[&two.road].lane_specs_ltr.len() == 1
        {
            continue;
        }

        // These points should be right on the intersection polygon
        let outer_corner1 = one.pl.last_pt();
        let outer_corner2 = two.pl.last_pt();

        let mut pts_along_intersection = Vec::new();
        if outer_corner1 == outer_corner2 {
            pts_along_intersection.push(outer_corner1);
        } else {
            if let Some(pl) = intersection
                .polygon
                .get_outer_ring()
                .get_shorter_slice_btwn(outer_corner1, outer_corner2)
            {
                pts_along_intersection = pl.into_points();
            } else {
                // Something went wrong; bail out
                continue;
            }
        }

        // Now find the inner sides of each sidewalk.
        let inner_pl1 = one.pl.must_shift_right(one.lane.width);
        let inner_pl2 = two.pl.must_shift_left(two.lane.width);

        // Imagine the inner lines extended into the intersection. If the point where they meet is
        // still inside the intersection, let's use it.
        let mut meet_pt = None;
        if let Some(pt) = inner_pl1
            .last_line()
            .infinite()
            .intersection(&inner_pl2.last_line().infinite())
        {
            if intersection.polygon.contains_pt(pt) {
                meet_pt = Some(pt);
            }
        }

        // Assemble everything into a ring.

        // This points from one to two, tracing along the intersection.
        let mut pts = pts_along_intersection;
        // Now add two's inner corner
        pts.push(inner_pl2.last_pt());
        // If we have a point where the two infinite lines meet, use it
        pts.extend(meet_pt);
        // one's inner corner
        pts.push(inner_pl1.last_pt());
        // Close the ring
        pts.push(pts[0]);

        if let Ok(ring) = Ring::deduping_new(pts) {
            results.push(ring.into_polygon());
        }
    }
    results
}

// TODO The alternate approach would be to preserve the original OSM way representing the crossing,
// even as the intersection polygon gets built
fn get_crossing_line_and_min_width(
    streets: &StreetNetwork,
    intersection: &Intersection,
) -> Option<(PolyLine, Distance)> {
    // Find the pedestrian roads making up the crossing, and the endpoint on the side of the
    // intersection for each
    let mut roads = Vec::new();
    for r in &intersection.roads {
        let road = &streets.roads[r];
        if road.is_footway() {
            let endpt = center_line_pointed_at(road, intersection).last_pt();
            roads.push((road, endpt));
        }
    }

    // Normally there are exactly two to connect. But when a tiny road gets consolidated, there'll
    // be more. Pick the pair farthest away from each other as a heuristic.
    let ((r1, pt1), (r2, pt2)) = farthest_pair(roads)?;

    // Create the line connecting these two roads.
    // TODO Subset the reference_lines by trim_start/end to get more detail
    let pl = PolyLine::new(vec![pt1, pt2]).ok()?;
    let width = r1.total_width().min(r2.total_width());
    Some((pl, width))
}

fn farthest_pair(candidates: Vec<(&Road, Pt2D)>) -> Option<((&Road, Pt2D), (&Road, Pt2D))> {
    let mut max_dist = Distance::ZERO;
    let mut max_pair = None;

    for idx1 in 0..candidates.len() {
        for idx2 in (idx1 + 1)..candidates.len() {
            let dist = candidates[idx1].1.dist_to(candidates[idx2].1);
            if dist > max_dist {
                max_dist = dist;
                max_pair = Some((candidates[idx1], candidates[idx2]));
            }
        }
    }

    max_pair
}

fn draw_zebra_crossing(streets: &StreetNetwork, intersection: &Intersection) -> Vec<Polygon> {
    let mut results = Vec::new();
    let Some((line, total_width)) = get_crossing_line_and_min_width(streets, intersection) else {
        return results;
    };

    // Pretty arbitrary parameters
    let width = 0.8 * total_width;
    let thickness = Distance::meters(0.15);
    let step_size = 3.0 * thickness;
    let buffer_ends = step_size;
    for (pt1, angle) in line.step_along(step_size, buffer_ends) {
        // Project away an arbitrary amount
        let pt2 = pt1.project_away(Distance::meters(1.0), angle);
        results.push(perp_line(Line::must_new(pt1, pt2), width).make_polygons(thickness));
    }

    results
}

fn draw_unmarked_crossing(streets: &StreetNetwork, intersection: &Intersection) -> Vec<Polygon> {
    let mut results = Vec::new();
    let Some((line, total_width)) = get_crossing_line_and_min_width(streets, intersection) else {
        return results;
    };

    let width = 0.8 * total_width;
    let thickness = Distance::meters(0.15);

    for shift in [width / 2.0, -width / 2.0] {
        if let Ok(pl) = line.shift_either_direction(shift) {
            results.push(pl.make_polygons(thickness));
        }
    }

    results
}

fn center_line_pointed_at(road: &Road, intersection: &Intersection) -> PolyLine {
    if road.dst_i == intersection.id {
        road.center_line.clone()
    } else {
        road.center_line.reversed()
    }
}

// this always does it at pt1
fn perp_line(l: Line, length: Distance) -> Line {
    let pt1 = l.shift_right(length / 2.0).pt1();
    let pt2 = l.shift_left(length / 2.0).pt1();
    Line::must_new(pt1, pt2)
}

impl StreetNetwork {
    /// The intersections that are only seams between two roads (see [`Self::intersection_marking_features_styled`]).
    pub fn seams(&self) -> Vec<crate::IntersectionID> {
        self.intersections
            .values()
            .filter(|i| i.crossing.is_none() && is_seam(&self.roads_per_intersection(i.id)))
            .map(|i| i.id)
            .collect()
    }

    /// Redraws every seam's polygon as the two road ends joined with mitred edges (osm2streets' polygon for two
    /// roads meeting at an angle sticks out past the outside of the bend in a spike). Where both roads have a
    /// sidewalk on the same side, the polygon stops at its inner edge and the sidewalk goes on around the bend: those
    /// wedges are returned, each with the road coming in, to be drawn as sidewalk.
    pub fn tidy_seam_polygons(&mut self) -> Vec<(crate::RoadID, Polygon)> {
        let mut sidewalks = Vec::new();
        for id in self.seams() {
            let roads = self.roads_per_intersection(id);
            let (Some(a), Some(b)) = (
                SeamEnd::new(roads[0], id, true),
                SeamEnd::new(roads[1], id, false),
            ) else {
                continue;
            };
            let limit = roads[0]
                .total_width()
                .inner_meters()
                .max(roads[1].total_width().inner_meters());
            // a band across the way through, from `da` / `db` metres off the left edge to `ea` / `eb`
            let band = |da: f64, db: f64, ea: f64, eb: f64| -> Option<Polygon> {
                let (al, ar, bl, br) = (a.at(da)?, a.at(ea)?, b.at(db)?, b.at(eb)?);
                let mut ring = vec![al];
                ring.extend(miter(&a, &b, al, bl, limit));
                ring.extend([bl, br]);
                ring.extend(miter(&a, &b, ar, br, limit));
                ring.push(ar);
                ring.dedup_by(|p, q| p.approx_eq(*q, Distance::meters(0.01)));
                if ring.len() < 3 {
                    return None;
                }
                ring.push(ring[0]);
                Ring::new(ring).ok().map(|r| r.into_polygon())
            };
            // a sidewalk carries on round the bend only where both roads have one on that side
            let (wl, wr) = (
                if a.walk.0 > 0.0 && b.walk.0 > 0.0 {
                    (a.walk.0, b.walk.0)
                } else {
                    (0.0, 0.0)
                },
                if a.walk.1 > 0.0 && b.walk.1 > 0.0 {
                    (a.walk.1, b.walk.1)
                } else {
                    (0.0, 0.0)
                },
            );
            let Some(carriageway) = band(wl.0, wl.1, a.width - wr.0, b.width - wr.1) else {
                continue;
            };
            if wl.0 > 0.0 {
                sidewalks.extend(band(0.0, 0.0, wl.0, wl.1).map(|p| (roads[0].id, p)));
            }
            if wr.0 > 0.0 {
                sidewalks.extend(
                    band(a.width - wr.0, b.width - wr.1, a.width, b.width)
                        .map(|p| (roads[0].id, p)),
                );
            }
            self.intersections.get_mut(&id).unwrap().polygon = carriageway;
        }
        sidewalks
    }
}

/// One road's end at a seam, seen along the way through it (in along the first road, out along the second).
struct SeamEnd<'a> {
    road: &'a Road,
    at_end: bool,
    /// The way through runs the same way as the road.
    with_road: bool,
    /// The way through's direction at this end.
    dir: (f64, f64),
    width: f64,
    /// Sidewalk widths on the left and right of the way through (0: none).
    walk: (f64, f64),
}

impl<'a> SeamEnd<'a> {
    fn new(road: &'a Road, i: crate::IntersectionID, incoming: bool) -> Option<Self> {
        let at_end = road.dst_i == i;
        let line = if at_end {
            road.center_line.last_line()
        } else {
            road.center_line.first_line().reversed()
        };
        // towards the intersection
        let (dx, dy) = (
            line.pt2().x() - line.pt1().x(),
            line.pt2().y() - line.pt1().y(),
        );
        let n = (dx * dx + dy * dy).sqrt();
        if n < 1e-9 {
            return None;
        }
        let sign = if incoming { 1.0 } else { -1.0 };
        let with_road = at_end == incoming;
        let walk = |l: Option<&crate::LaneSpec>| {
            l.filter(|l| {
                matches!(
                    l.lt,
                    LaneType::Sidewalk | LaneType::Footway | LaneType::SharedUse
                )
            })
            .map_or(0.0, |l| l.width.inner_meters())
        };
        let (first, last) = (
            walk(road.lane_specs_ltr.first()),
            walk(road.lane_specs_ltr.last()),
        );
        Some(SeamEnd {
            road,
            at_end,
            with_road,
            dir: (sign * dx / n, sign * dy / n),
            width: road.total_width().inner_meters(),
            walk: if with_road {
                (first, last)
            } else {
                (last, first)
            },
        })
    }

    /// The point `d` metres in from the way through's left edge, on this end.
    fn at(&self, d: f64) -> Option<Pt2D> {
        // metres right of the road's centre line
        let off = if self.with_road {
            d - self.width / 2.0
        } else {
            self.width / 2.0 - d
        };
        let pl = if off.abs() < 1e-6 {
            self.road.center_line.clone()
        } else if off > 0.0 {
            self.road
                .center_line
                .shift_right(Distance::meters(off))
                .ok()?
        } else {
            self.road
                .center_line
                .shift_left(Distance::meters(-off))
                .ok()?
        };
        Some(if self.at_end {
            pl.last_pt()
        } else {
            pl.first_pt()
        })
    }
}

/// Where the edge through `p` (on the road coming in) meets the edge through `q` (on the road going out), when that
/// is ahead of `p` and before `q`, within `limit` metres of both.
fn miter(a: &SeamEnd, b: &SeamEnd, p: Pt2D, q: Pt2D, limit: f64) -> Option<Pt2D> {
    let ((adx, ady), (bdx, bdy)) = (a.dir, b.dir);
    // p + t * a = q - u * b
    let det = adx * bdy - ady * bdx;
    if det.abs() < 1e-6 {
        return None;
    }
    let (wx, wy) = (q.x() - p.x(), q.y() - p.y());
    let t = (wx * bdy - wy * bdx) / det;
    let u = (adx * wy - ady * wx) / det;
    if t <= 0.0 || u <= 0.0 || t > limit || u > limit {
        return None;
    }
    Some(Pt2D::new(p.x() + t * adx, p.y() + t * ady))
}

fn is_seam(roads: &[&Road]) -> bool {
    roads.len() == 2
        && roads.iter().all(|r| {
            r.lane_specs_ltr
                .iter()
                .any(|l| matches!(l.lt, LaneType::Driving | LaneType::Bus))
        })
}

/// Where a lane line sits across a road end, for matching it with the line it continues on the other road.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum LineKey {
    /// Between the two directions.
    Centre,
    /// The n-th separator from the centre line, on the left (false) or right (true) of the way through.
    Side(bool, usize),
    /// A one-way road's n-th separator, counted from the side lanes are not added on.
    OneWay(usize),
}

/// The solid lane lines through a seam between two roads: each centre line or separator of the road coming in, joined to the
/// one at the same place on the road going out (counted from the centre line, or on a one-way road from the side
/// lanes are not added on), with the style's line. Dashed lines leave a gap anyway, so only solid patterns are drawn.
fn seam_lines(
    streets: &StreetNetwork,
    intersection: &Intersection,
    style: &MarkingStyle,
) -> Vec<(Polygon, &'static str)> {
    let roads = streets.roads_per_intersection(intersection.id);
    if !is_seam(&roads) {
        return Vec::new();
    }
    let i = intersection.id;
    // Way through: into the intersection along roads[0], out along roads[1].
    let a = boundary_ends(
        roads[0],
        roads[0].dst_i == i,
        true,
        streets.config.driving_side,
    );
    let b = boundary_ends(
        roads[1],
        roads[1].dst_i == i,
        false,
        streets.config.driving_side,
    );
    let mut out = Vec::new();
    for (key, kind, pa) in &a {
        let Some((_, _, pb)) = b.iter().find(|(k, kd, _)| k == key && kd == kind) else {
            continue;
        };
        let d = pa.dist_to(*pb);
        if d < Distance::meters(0.1) || d > Distance::meters(25.0) {
            continue;
        }
        let line_style = if *kind == "center line" {
            &style.centre_line
        } else {
            &style.lane_separator
        };
        if let Ok(line) = PolyLine::new(vec![*pa, *pb]) {
            out.extend(
                solid_parts(line_style, &line)
                    .into_iter()
                    .map(|p| (p, *kind)),
            );
        }
    }
    out
}

/// The solid lines of a pattern along a short connector (its dashed lines left out).
fn solid_parts(s: &LineStyle, line: &PolyLine) -> Vec<Polygon> {
    let (w, off) = (
        Distance::meters(s.width),
        Distance::meters((s.spacing + s.width) / 2.0),
    );
    let one = |l: Result<PolyLine>| l.ok().and_then(|l| l.try_make_polygons(w).ok());
    match s.pattern {
        LinePattern::None | LinePattern::Dashed => Vec::new(),
        LinePattern::Solid => one(Ok(line.clone())).into_iter().collect(),
        LinePattern::DoubleSolid => [one(line.shift_left(off)), one(line.shift_right(off))]
            .into_iter()
            .flatten()
            .collect(),
        LinePattern::SolidDashed => one(line.shift_left(off)).into_iter().collect(),
        LinePattern::DashedSolid => one(line.shift_right(off)).into_iter().collect(),
    }
}

/// A road's lane lines (centre line, separators between driving lanes) at its end at the intersection, keyed by
/// their place across the way through. `at_end`: the intersection is at the road's end (else its start); `incoming`:
/// the way through comes in along this road (else it leaves along it).
fn boundary_ends(
    road: &Road,
    at_end: bool,
    incoming: bool,
    driving_side: DrivingSide,
) -> Vec<(LineKey, &'static str, geom::Pt2D)> {
    let centers = road.get_lane_center_lines();
    let lanes = &road.lane_specs_ltr;
    // Each boundary between lane k and k + 1, in the road's left-to-right order.
    let mut bounds = Vec::new();
    for (k, pair) in lanes.windows(2).enumerate() {
        let kind = if pair[0].dir != pair[1].dir {
            "center line"
        } else if pair[0].lt == LaneType::Driving && pair[1].lt == LaneType::Driving {
            "lane separator"
        } else {
            continue;
        };
        let Ok(line) = centers[k].shift_right(pair[0].width / 2.0) else {
            continue;
        };
        let pt = if at_end {
            line.last_pt()
        } else {
            line.first_pt()
        };
        bounds.push((k, kind, pt));
    }
    // Left to right as seen along the way through: the road's own order if the way through runs with the road.
    let with_road = at_end == incoming;
    if !with_road {
        bounds.reverse();
    }
    let n = lanes.len().saturating_sub(1);
    let pos = |k: usize| if with_road { k } else { n - 1 - k };
    let centre = bounds
        .iter()
        .find(|(_, kind, _)| *kind == "center line")
        .map(|(k, _, _)| pos(*k));
    let mut out = Vec::new();
    match centre {
        Some(c) => {
            let (mut left, mut right): (Vec<_>, Vec<_>) = (Vec::new(), Vec::new());
            for &(k, kind, pt) in &bounds {
                let p = pos(k);
                if kind == "center line" {
                    if p == c {
                        out.push((LineKey::Centre, kind, pt));
                    }
                } else if p < c {
                    left.push((p, kind, pt));
                } else {
                    right.push((p, kind, pt));
                }
            }
            // counted outward from the centre line
            left.sort_by_key(|(p, _, _)| std::cmp::Reverse(*p));
            right.sort_by_key(|(p, _, _)| *p);
            for (n, (_, kind, pt)) in left.into_iter().enumerate() {
                out.push((LineKey::Side(false, n), kind, pt));
            }
            for (n, (_, kind, pt)) in right.into_iter().enumerate() {
                out.push((LineKey::Side(true, n), kind, pt));
            }
        }
        None => {
            // one-way: lanes are added on the kerb side, so count from the other one
            let mut seps: Vec<_> = bounds
                .iter()
                .map(|&(k, kind, pt)| (pos(k), kind, pt))
                .collect();
            seps.sort_by_key(|(p, _, _)| *p);
            if driving_side == DrivingSide::Left {
                seps.reverse();
            }
            for (n, (_, kind, pt)) in seps.into_iter().enumerate() {
                out.push((LineKey::OneWay(n), kind, pt));
            }
        }
    }
    out
}
