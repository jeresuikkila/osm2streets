use anyhow::Result;
use geojson::Feature;
use geom::{ArrowCap, Distance, Line, PolyLine, Polygon};

use super::{serialize_features, Filter};
use crate::{
    BufferType, Direction, DrivingSide, LaneSpec, LaneType, ParkingType, Road, StreetNetwork,
};
use osm2lanes::TurnDirection;

/// How lane markings look. `Default` is osm2streets' classic look; every length is in metres.
#[derive(Clone, Debug, PartialEq)]
pub struct MarkingStyle {
    /// Between lanes of opposite directions.
    pub centre_line: LineStyle,
    /// Between driving lanes of the same direction.
    pub lane_separator: LineStyle,
    pub arrows: ArrowStyle,
    pub stop_line_width: f64,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LineStyle {
    pub pattern: LinePattern,
    pub width: f64,
    pub dash: f64,
    pub gap: f64,
    /// The gap between the two lines of a double line.
    pub spacing: f64,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LinePattern {
    None,
    Solid,
    Dashed,
    DoubleSolid,
    /// Solid on the left of the road's direction, dashed on its right.
    SolidDashed,
    /// Dashed on the left of the road's direction, solid on its right.
    DashedSolid,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ArrowStyle {
    pub placement: ArrowPlacement,
    /// `Interval`: an arrow every `interval`, none closer than `ends` to a lane's ends.
    pub interval: f64,
    pub ends: f64,
    pub length: f64,
    /// A filled arrow, or (classic) its outline.
    pub solid: bool,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ArrowPlacement {
    None,
    /// Through arrows along every travel lane.
    Interval,
    /// One arrow per lane before its end, showing the lane's `turn:lanes` (lanes without turn tags get none).
    StopLine,
}

impl Default for MarkingStyle {
    fn default() -> Self {
        MarkingStyle {
            centre_line: LineStyle {
                pattern: LinePattern::Dashed,
                width: 0.25,
                dash: 2.0,
                gap: 1.0,
                spacing: 0.2,
            },
            lane_separator: LineStyle {
                pattern: LinePattern::Dashed,
                width: 0.25,
                dash: 1.0,
                gap: 1.5,
                spacing: 0.2,
            },
            arrows: ArrowStyle {
                placement: ArrowPlacement::Interval,
                interval: 20.0,
                ends: 5.0,
                length: 1.75,
                solid: false,
            },
            stop_line_width: 0.5,
        }
    }
}

impl LineStyle {
    /// The paint along `line` (the boundary between two lanes, in the road's direction).
    fn draw(&self, line: &PolyLine) -> Result<Vec<Polygon>> {
        let m = Distance::meters;
        let (w, off) = (m(self.width), m((self.spacing + self.width) / 2.0));
        let solid = |l: &PolyLine| vec![l.make_polygons(w)];
        let dashed = |l: &PolyLine| l.dashed_lines(w, m(self.dash), m(self.gap));
        Ok(match self.pattern {
            LinePattern::None => Vec::new(),
            LinePattern::Solid => solid(line),
            LinePattern::Dashed => dashed(line),
            LinePattern::DoubleSolid => {
                let mut v = solid(&line.shift_left(off)?);
                v.extend(solid(&line.shift_right(off)?));
                v
            }
            LinePattern::SolidDashed => {
                let mut v = solid(&line.shift_left(off)?);
                v.extend(dashed(&line.shift_right(off)?));
                v
            }
            LinePattern::DashedSolid => {
                let mut v = dashed(&line.shift_left(off)?);
                v.extend(solid(&line.shift_right(off)?));
                v
            }
        })
    }
}

impl StreetNetwork {
    /// Generate polygons representing lane markings, with a property indicating type.
    pub fn to_lane_markings_geojson(&self, filter: &Filter) -> Result<String> {
        serialize_features(self.lane_marking_features(filter)?)
    }

    /// The features of [`Self::to_lane_markings_geojson`].
    pub fn lane_marking_features(&self, filter: &Filter) -> Result<Vec<Feature>> {
        self.lane_marking_features_styled(filter, &MarkingStyle::default())
    }

    /// [`Self::lane_marking_features`] in a given style.
    pub fn lane_marking_features_styled(
        &self,
        filter: &Filter,
        style: &MarkingStyle,
    ) -> Result<Vec<Feature>> {
        // TODO Split this up!
        let gps_bounds = Some(&self.gps_bounds);
        let mut features = Vec::new();

        for road in filter.roads(self) {
            let first = features.len();
            // Always oriented in the direction of the road
            let mut lane_centers = road.get_lane_center_lines();

            for (idx, pair) in road.lane_specs_ltr.windows(2).enumerate() {
                // Generate a "center line" between lanes of different directions
                if pair[0].dir != pair[1].dir {
                    let between = lane_centers[idx].shift_right(pair[0].width / 2.0)?;
                    // TODO Ideally we would return a full LineString, and the caller would choose
                    // how to style these as thickened dashed lines.
                    // TODO We could also at least return a MultiPolygon here
                    for poly in style.centre_line.draw(&between)? {
                        let mut f = Feature::from(poly.to_geojson(gps_bounds));
                        f.set_property("type", "center line");
                        f.set_property("layer", road.layer);
                        features.push(f);
                    }
                    continue;
                }

                // Generate a "lane separator" between driving lanes only
                if pair[0].lt == LaneType::Driving && pair[1].lt == LaneType::Driving {
                    let between = lane_centers[idx].shift_right(pair[0].width / 2.0)?;
                    for poly in style.lane_separator.draw(&between)? {
                        let mut f = Feature::from(poly.to_geojson(gps_bounds));
                        f.set_property("type", "lane separator");
                        f.set_property("layer", road.layer);
                        features.push(f);
                    }
                }
            }

            // Stop line distances are relative to the direction of the road, not the lane!
            for (lane, center) in road.lane_specs_ltr.iter().zip(lane_centers.iter()) {
                for (polygon, kind) in draw_stop_lines(lane, center, road, style.stop_line_width) {
                    let mut f = Feature::from(polygon.to_geojson(gps_bounds));
                    f.set_property("type", kind);
                    features.push(f);
                }
            }

            // Below renderings need lane centers to point in the direction of the lane
            for (lane, center) in road.lane_specs_ltr.iter().zip(lane_centers.iter_mut()) {
                if lane.dir == Direction::Backward {
                    *center = center.reversed();
                }
            }

            // Draw arrows along any travel lane
            for (lane, center) in road.lane_specs_ltr.iter().zip(lane_centers.iter()) {
                if !lane.lt.is_for_moving_vehicles() {
                    continue;
                }

                for arrow in draw_arrows(lane, center, &style.arrows, gps_bounds) {
                    let mut f = Feature::from(arrow);
                    f.set_property("type", "lane arrow");
                    f.set_property("layer", road.layer);
                    features.push(f);
                }
            }

            // Add stripes to show most buffers.
            for (lane, center) in road.lane_specs_ltr.iter().zip(lane_centers.iter()) {
                // TODO Revisit rendering for different buffer types
                if !matches!(lane.lt, LaneType::Buffer(_)) {
                    continue;
                }
                if lane.lt == LaneType::Buffer(BufferType::Curb) {
                    continue;
                }

                // Mark the sides of the lane clearly
                let thickness = Distance::meters(0.25);
                for dir in [-1.0, 1.0] {
                    let mut f = Feature::from(
                        center
                            .shift_either_direction(dir * (lane.width - thickness) / 2.0)
                            .unwrap()
                            .make_polygons(thickness)
                            .to_geojson(gps_bounds),
                    );
                    f.set_property("type", "buffer edge");
                    f.set_property("layer", road.layer);
                    features.push(f);
                }

                // Diagonal stripes along the lane
                let step_size = Distance::meters(3.0);
                let buffer_ends = Distance::meters(5.0);
                for (center, angle) in center.step_along(step_size, buffer_ends) {
                    // Extend the stripes into the side lines
                    let left =
                        center.project_away(lane.width / 2.0 + thickness, angle.rotate_degs(45.0));
                    let right = center.project_away(
                        lane.width / 2.0 + thickness,
                        angle.rotate_degs(45.0).opposite(),
                    );
                    let mut f = Feature::from(
                        Line::must_new(left, right)
                            .make_polygons(thickness)
                            .to_geojson(gps_bounds),
                    );
                    f.set_property("type", "buffer stripe");
                    f.set_property("layer", road.layer);
                    features.push(f);
                }
            }

            for (lane, center) in road.lane_specs_ltr.iter().zip(lane_centers.iter()) {
                let polygons = match lane.lt {
                    LaneType::Parking(ParkingType::Parallel) => {
                        draw_parallel_parking_lines(lane, center, self)
                    }
                    LaneType::Parking(ParkingType::Diagonal) => {
                        draw_diagonal_parking_lines(lane, center, self)
                    }
                    LaneType::Parking(ParkingType::Perpendicular) => {
                        draw_perpendicular_parking_lines(lane, center, self)
                    }
                    _ => continue,
                };
                for polygon in polygons {
                    let mut f = Feature::from(polygon.to_geojson(gps_bounds));
                    f.set_property("type", "parking hatch");
                    features.push(f);
                }
            }

            for (lane, center) in road.lane_specs_ltr.iter().zip(lane_centers.iter()) {
                if lane.lt != LaneType::Sidewalk {
                    continue;
                }
                for polygon in draw_sidewalk_lines(lane, center) {
                    let mut f = Feature::from(polygon.to_geojson(gps_bounds));
                    f.set_property("type", "sidewalk line");
                    features.push(f);
                }
            }

            for (lane, center) in road.lane_specs_ltr.iter().zip(lane_centers.iter()) {
                if lane.lt != LaneType::SharedUse && lane.lt != LaneType::Footway {
                    continue;
                }
                for polygon in draw_path_outlines(lane, center) {
                    let mut f = Feature::from(polygon.to_geojson(gps_bounds));
                    f.set_property("type", "path outline");
                    features.push(f);
                }
            }

            // Every marking knows its road and that road's layer, so a consumer can put it on the road's level
            // (bridges, overpasses) without matching geometry.
            for f in &mut features[first..] {
                f.set_property("road", road.id.0);
                f.set_property("layer", road.layer);
            }
        }

        Ok(features)
    }
}

/// The arrows painted in a travel lane (`center` points the way the lane goes).
fn draw_arrows(
    lane: &LaneSpec,
    center: &PolyLine,
    style: &ArrowStyle,
    gps: Option<&geom::GPSBounds>,
) -> Vec<geojson::Geometry> {
    let len = Distance::meters(style.length);
    let mut out = Vec::new();
    match style.placement {
        ArrowPlacement::None => {}
        ArrowPlacement::Interval => {
            let thickness = Distance::meters(0.25);
            for (pt, angle) in center.step_along(
                Distance::meters(style.interval),
                Distance::meters(style.ends),
            ) {
                let line = PolyLine::must_new(vec![
                    pt.project_away(len / 2.0, angle.opposite()),
                    pt.project_away(len / 2.0, angle),
                ]);
                out.push(arrow_shape(&line, thickness, style.solid, gps));
            }
        }
        ArrowPlacement::StopLine => {
            // Only lanes tagged with their turns: one arrow (one head per turn) ending 8 m before the lane does,
            // behind the crosswalk and the stop line.
            let back = Distance::meters(8.0);
            if lane.allowed_turns.is_empty() || center.length() < len + back + Distance::meters(2.0)
            {
                return out;
            }
            let end = center.length() - back;
            let Ok((tip, angle)) = center.dist_along(end) else {
                return out;
            };
            let base = tip.project_away(len, angle.opposite());
            let thickness = Distance::meters((lane.width.inner_meters() * 0.08).clamp(0.2, 0.35));
            let knee = tip.project_away(len * 0.45, angle.opposite());
            for turn in lane.allowed_turns.iter() {
                let (degs, reach) = match turn {
                    TurnDirection::Through => (0.0, 0.0),
                    TurnDirection::Left | TurnDirection::SharpLeft => (-90.0, 0.4),
                    TurnDirection::Right | TurnDirection::SharpRight => (90.0, 0.4),
                    TurnDirection::SlightLeft | TurnDirection::MergeLeft => (-45.0, 0.45),
                    TurnDirection::SlightRight | TurnDirection::MergeRight => (45.0, 0.45),
                    TurnDirection::Reverse => (-180.0, 0.3),
                };
                let pts = if degs == 0.0 {
                    vec![base, tip]
                } else {
                    vec![
                        base,
                        knee,
                        knee.project_away(len * reach, angle.rotate_degs(degs)),
                    ]
                };
                if let Ok(line) = PolyLine::new(pts) {
                    out.push(arrow_shape(&line, thickness, style.solid, gps));
                }
            }
        }
    }
    out
}

fn arrow_shape(
    line: &PolyLine,
    thickness: Distance,
    solid: bool,
    gps: Option<&geom::GPSBounds>,
) -> geojson::Geometry {
    let arrow = line.make_arrow(thickness * 2.0, ArrowCap::Triangle);
    if solid {
        arrow.to_geojson(gps)
    } else {
        arrow
            .get_outer_ring()
            .to_outline(thickness / 2.0)
            .to_geojson(gps)
    }
}

fn draw_stop_lines(
    lane: &LaneSpec,
    center: &PolyLine,
    road: &Road,
    width: f64,
) -> Vec<(Polygon, &'static str)> {
    let mut results = Vec::new();

    if !matches!(
        lane.lt,
        LaneType::Driving | LaneType::Bus | LaneType::Biking
    ) {
        return results;
    }
    let thickness = Distance::meters(width);

    let stop_line = if lane.dir == Direction::Forward {
        &road.stop_line_end
    } else {
        &road.stop_line_start
    };

    // The vehicle line
    if let Some(dist) = stop_line.vehicle_distance {
        if let Ok((pt, angle)) = center.dist_along(dist) {
            results.push((
                Line::must_new(
                    pt.project_away(lane.width / 2.0, angle.rotate_degs(90.0)),
                    pt.project_away(lane.width / 2.0, angle.rotate_degs(-90.0)),
                )
                .make_polygons(thickness),
                "vehicle stop line",
            ));
        }
    }

    if let Some(dist) = stop_line.bike_distance {
        if let Ok((pt, angle)) = center.dist_along(dist) {
            results.push((
                Line::must_new(
                    pt.project_away(lane.width / 2.0, angle.rotate_degs(90.0)),
                    pt.project_away(lane.width / 2.0, angle.rotate_degs(-90.0)),
                )
                .make_polygons(thickness),
                "bike stop line",
            ));
        }
    }

    // TODO Change the rendering based on interruption too

    results
}

fn draw_parallel_parking_lines(
    lane: &LaneSpec,
    center: &PolyLine,
    streets: &StreetNetwork,
) -> Vec<Polygon> {
    let mut result = Vec::new();

    // No spots next to intersections
    let spots =
        (center.length() / streets.config.parallel_street_parking_spot_length).floor() - 2.0;
    let num_spots = if spots >= 1.0 {
        spots as usize
    } else {
        return result;
    };

    let leg_length = Distance::meters(1.0);
    for idx in 0..=num_spots {
        let (pt, lane_angle) = center.must_dist_along(
            streets.config.parallel_street_parking_spot_length * (1.0 + idx as f64),
        );
        let perp_angle = if streets.config.driving_side == DrivingSide::Right {
            lane_angle.rotate_degs(270.0)
        } else {
            lane_angle.rotate_degs(90.0)
        };
        // Find the outside of the lane. Actually, shift inside a little bit, since the line
        // will have thickness, but shouldn't really intersect the adjacent line
        // when drawn.
        let t_pt = pt.project_away(lane.width * 0.4, perp_angle);
        // The perp leg
        let p1 = t_pt.project_away(leg_length, perp_angle.opposite());
        result.push(Line::must_new(t_pt, p1).make_polygons(Distance::meters(0.25)));
        // Upper leg
        let p2 = t_pt.project_away(leg_length, lane_angle);
        result.push(Line::must_new(t_pt, p2).make_polygons(Distance::meters(0.25)));
        // Lower leg
        let p3 = t_pt.project_away(leg_length, lane_angle.opposite());
        result.push(Line::must_new(t_pt, p3).make_polygons(Distance::meters(0.25)));
    }

    result
}

fn draw_diagonal_parking_lines(
    lane: &LaneSpec,
    center: &PolyLine,
    streets: &StreetNetwork,
) -> Vec<Polygon> {
    let mut result = Vec::new();

    // No spots next to intersections
    // TODO This needs to account for the 45 degree angle too
    let spots = (center.length() / streets.config.vehicle_width_for_parking_spots).floor() - 2.0;
    let num_spots = if spots >= 1.0 {
        spots as usize
    } else {
        return result;
    };

    // TODO Would PolyLine::step_along be simpler?
    for idx in 0..=num_spots {
        let (pt, lane_angle) = center
            .must_dist_along(streets.config.vehicle_width_for_parking_spots * (1.0 + idx as f64));
        let offset = if lane.dir == Direction::Forward {
            -45.0
        } else {
            45.0
        };
        let diag_angle = if streets.config.driving_side == DrivingSide::Right {
            lane_angle.rotate_degs(270.0 + offset)
        } else {
            lane_angle.rotate_degs(90.0 + offset)
        };
        // Find the inside and outside of the lane
        // TODO Do trig, the length changes
        let outside_pt = pt.project_away(lane.width * 0.4, diag_angle);
        let inside_pt = pt.project_away(lane.width * 0.5, diag_angle.opposite());
        result.push(Line::must_new(outside_pt, inside_pt).make_polygons(Distance::meters(0.25)));
    }

    result
}

fn draw_perpendicular_parking_lines(
    lane: &LaneSpec,
    center: &PolyLine,
    streets: &StreetNetwork,
) -> Vec<Polygon> {
    let mut result = Vec::new();

    // No spots next to intersections
    let spots = (center.length() / streets.config.vehicle_width_for_parking_spots).floor() - 2.0;
    let num_spots = if spots >= 1.0 {
        spots as usize
    } else {
        return result;
    };

    for idx in 0..=num_spots {
        let (pt, lane_angle) = center
            .must_dist_along(streets.config.vehicle_width_for_parking_spots * (1.0 + idx as f64));
        let perp_angle = if streets.config.driving_side == DrivingSide::Right {
            lane_angle.rotate_degs(270.0)
        } else {
            lane_angle.rotate_degs(90.0)
        };
        // Find the inside and outside of the lane
        let outside_pt = pt.project_away(lane.width * 0.4, perp_angle);
        let inside_pt = pt.project_away(lane.width * 0.5, perp_angle.opposite());
        result.push(Line::must_new(outside_pt, inside_pt).make_polygons(Distance::meters(0.25)));
    }

    result
}

fn draw_sidewalk_lines(lane: &LaneSpec, center: &PolyLine) -> Vec<Polygon> {
    center
        .step_along(lane.width, lane.width)
        .into_iter()
        .map(|(pt, angle)| {
            // Project away an arbitrary amount
            let pt2 = pt.project_away(Distance::meters(1.0), angle);
            perp_line(Line::must_new(pt, pt2), lane.width).make_polygons(Distance::meters(0.25))
        })
        .collect()
}

fn draw_path_outlines(lane: &LaneSpec, center: &PolyLine) -> Vec<Polygon> {
    let mut result = Vec::new();
    // Dashed lines on both sides
    for dir in [-1.0, 1.0] {
        // A bend tighter than half the path's width can't be shifted; leave that side undashed
        let Ok(pl) = center.shift_either_direction(dir * lane.width / 2.0) else {
            continue;
        };
        result.extend(pl.exact_dashed_polygons(
            Distance::meters(0.25),
            Distance::meters(1.0),
            Distance::meters(1.5),
        ));
    }
    result
}

// this always does it at pt1
fn perp_line(l: Line, length: Distance) -> Line {
    let pt1 = l.shift_right(length / 2.0).pt1();
    let pt2 = l.shift_left(length / 2.0).pt1();
    Line::must_new(pt1, pt2)
}
