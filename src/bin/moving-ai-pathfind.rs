use graf::{AdjacencyList, Edge, Node, NodeMap, Weight};

use clap::Parser;
use movingai::{Coords2D, Map2D as _};
use movingai::{MovingAiMap, SceneRecord};

use std::collections::HashMap;
use std::path::{Path, PathBuf};

#[derive(Debug, Parser)]
#[command(author, version)]
struct Cli {
    /// The path to a .scen file from moving AI, or a directory of .scen files
    scenario: PathBuf,
    /// Maps directory
    #[arg(long)]
    maps: PathBuf,
    #[arg(long)]
    output_graph: Option<PathBuf>,
    #[arg(long)]
    output_map: Option<PathBuf>,
    #[arg(long)]
    run_ref_impl: bool,
}

mod dbg {
    use super::*;

    pub fn dump_map_format(
        graph: &AdjacencyList,
        node2coord: &NodeMap<Coords2D>,
        width: usize,
        height: usize,
        path: &Path,
    ) {
        let cap = width * height;
        let mut data = Vec::new();
        data.resize(cap, 'T');

        for n in graph.nodes() {
            for n2 in graph.edges(n).map(|x| x.node) {
                let (x, y) = node2coord[n2];
                data[x + width * y] = '.';
            }
        }

        let mut contents = String::with_capacity(cap + height);
        for y in 0..height {
            for x in 0..width {
                let idx = x + width * y;
                contents.push(data[idx]);
            }
            contents.push('\n');
        }

        std::fs::write(path, contents).unwrap_or_else(|_| panic!("Bad path: {}", path.display()));
    }

    pub fn dump_graph(graph: &AdjacencyList, node2coord: &NodeMap<Coords2D>, path: &Path) {
        use std::fmt::Write as _;
        let mut contents = String::new();
        for node in graph.nodes() {
            write!(contents, "{} ({:?}):", node, node2coord[&node]).unwrap();
            let mut first = true;
            for child in graph.edges(node).map(|x| x.node) {
                if !first {
                    write!(contents, ", ").unwrap();
                }
                first = false;
                write!(contents, "{} ({:?}), ", child, node2coord[child]).unwrap();
            }
            contents.push('\n');
        }
        std::fs::write(path, contents).unwrap_or_else(|_| panic!("Bad path: {}", path.display()));
    }

    #[allow(dead_code)]
    pub fn dump_path(gpath: &graf::Path, node2coord: &NodeMap<Coords2D>) {
        let mut total = 0.0;
        for Edge { node, weight: cost } in gpath {
            let c = node2coord[node];
            total += cost;
            println!("({:?}, {})", c, cost);
        }

        println!("total: {}", total);
    }
}
const DIAG_COST: Weight = std::f32::consts::SQRT_2;
const STRAIGHT_COST: Weight = 1.0;
fn neighbors(map: &MovingAiMap, tile: Coords2D) -> Vec<(Coords2D, Weight)> {
    let (x, y) = (tile.0 as isize, tile.1 as isize);
    let all: Vec<((isize, isize), Weight)> = vec![
        ((x + 1, y), STRAIGHT_COST),
        ((x + 1, y + 1), DIAG_COST),
        ((x + 1, y - 1), DIAG_COST),
        ((x, y + 1), STRAIGHT_COST),
        ((x, y - 1), STRAIGHT_COST),
        ((x - 1, y), STRAIGHT_COST),
        ((x - 1, y - 1), DIAG_COST),
        ((x - 1, y + 1), DIAG_COST),
    ];
    all.into_iter()
        .filter(|&((x, y), _)| {
            x >= 0 && x < map.width() as isize && y >= 0 && y <= map.height() as isize
        })
        .map(|((x, y), c)| ((x as usize, y as usize), c))
        .filter(|(n, _)| map.is_traversable_from(tile, *n))
        .collect()
}

struct HeuristicDistance<'a> {
    node2coord: &'a NodeMap<Coords2D>,
    goal_pos: Coords2D,
}

impl<'a> graf::HeuristicDistance for HeuristicDistance<'a> {
    fn cost(&self, node: &Node) -> Weight {
        let (n_x, n_y) = self
            .node2coord
            .get(node)
            .expect("Unrecognized node, no matching coords");
        let x = self.goal_pos.0 as Weight - *n_x as Weight;
        let y = self.goal_pos.1 as Weight - *n_y as Weight;
        (x.powi(2) + y.powi(2)).sqrt()
    }
}

struct ScenarioResult {
    path: graf::Path,
}

impl ScenarioResult {
    fn cost(&self) -> Weight {
        self.path.iter().fold(
            0.0,
            |acc,
             Edge {
                 node: _,
                 weight: cost,
             }| acc + cost,
        )
    }
}

fn run_single_scenario(
    scenario: &movingai::SceneRecord,
    astar_acc: &mut graf::AStarAcceleration,
    coord2node: &HashMap<Coords2D, Node>,
    node2coord: &NodeMap<Coords2D>,
) -> ScenarioResult {
    let start = *coord2node
        .get(&scenario.start_pos)
        .expect("This should have a node assigned");
    let end = *coord2node
        .get(&scenario.goal_pos)
        .expect("This should have a node assigned");

    let heuristic = HeuristicDistance {
        node2coord,
        goal_pos: scenario.goal_pos,
    };

    let path = graf::a_star(astar_acc, start, end, heuristic).expect("Failed to find path");

    ScenarioResult { path }
}

fn parse_scenario_file(file: &Path) -> (Vec<SceneRecord>, String) {
    let scenarios = movingai::parser::parse_scen_file(file).unwrap();
    let first_map = scenarios[0].map_file.clone();
    let same = scenarios.iter().all(|s| s.map_file == first_map);
    assert!(
        same,
        "All maps are not the same as {first_map} in {scenario}",
        scenario = file.display()
    );
    (scenarios, first_map)
}

mod refimpl {
    use std::cmp::Ordering;
    use std::collections::BinaryHeap;
    use std::path::Path;
    use std::time::Instant;

    use movingai::parser::parse_map_file;
    use movingai::parser::parse_scen_file;
    use movingai::Coords2D;
    use movingai::Map2D;
    use movingai::MovingAiMap;

    #[derive(Debug)]
    struct SearchNode {
        pub f: f64,
        pub h: f64,
        pub g: f64,
        pub current: Coords2D,
    }

    impl PartialEq for SearchNode {
        fn eq(&self, other: &SearchNode) -> bool {
            self.current == other.current
        }
    }

    impl Eq for SearchNode {
        // add code here
    }

    impl PartialOrd for SearchNode {
        fn partial_cmp(&self, other: &SearchNode) -> Option<Ordering> {
            Some(self.cmp(other))
        }
    }

    impl Ord for SearchNode {
        fn cmp(&self, other: &SearchNode) -> Ordering {
            // This is reversed on purpose to make the max-heap into min-heap.
            if self.f < other.f {
                Ordering::Greater
            } else if self.f > other.f {
                Ordering::Less
            } else {
                Ordering::Equal
            }
        }
    }

    fn distance(a: Coords2D, b: Coords2D) -> f64 {
        let (x, y) = (a.0 as f64, a.1 as f64);
        let (p, q) = (b.0 as f64, b.1 as f64);
        ((x - p) * (x - p) + (y - q) * (y - q)).sqrt()
    }

    fn shortest_path(map: &MovingAiMap, start: Coords2D, goal: Coords2D) -> Option<f64> {
        let mut heap = BinaryHeap::new();
        let mut visited = Vec::<Coords2D>::new();

        // We're at `start`, with a zero cost
        heap.push(SearchNode {
            f: 0.0,
            g: 0.0,
            h: distance(start, goal),
            current: start,
        });

        while let Some(SearchNode {
            f: _f,
            g,
            h: _h,
            current,
        }) = heap.pop()
        {
            if current == goal {
                return Some(g);
            }

            if visited.contains(&current) {
                continue;
            }

            visited.push(current);

            for neigh in map.neighbors(current) {
                let new_h = distance(neigh, goal);
                let i = distance(neigh, current);
                let next = SearchNode {
                    f: g + i + new_h,
                    g: g + i,
                    h: new_h,
                    current: neigh,
                };
                heap.push(next);
            }
        }

        // Goal not reachable
        None
    }

    pub fn run_for_scenario_file(map_path: &Path, scenario: &Path) {
        // TODO: Make this return the path and diff compared to my impl.
        let map = parse_map_file(map_path).unwrap();
        let scenes = parse_scen_file(scenario).unwrap();
        for scene in scenes {
            let start = scene.start_pos;
            let goal = scene.goal_pos;
            let optimal = scene.optimal_length;
            let t = Instant::now();
            match shortest_path(&map, start, goal) {
                Some(g) => {
                    let duration = t.elapsed();
                    let millis = duration.as_millis();
                    let delta = (optimal - g).abs();
                    if delta > 0.00001 {
                        println!(
                            "{:?} -> {:?} = {:.5} \t\tin {:.5} ms (Δ{:.5})",
                            start, goal, g, millis, delta
                        );
                    } else {
                        println!(
                            "{:?} -> {:?} = {:.5} \t\tin {:.5} ms",
                            start, goal, g, millis
                        );
                    }
                }
                None => println!("None"),
            }
        }
    }
}

fn run_for_scenario_file(
    scenario: &Path,
    maps: &Path,
    output_map: &Option<PathBuf>,
    output_graph: &Option<PathBuf>,
    run_ref_impl: bool,
) {
    let (scenarios, first_map) = parse_scenario_file(scenario);
    let mut path = std::path::PathBuf::from(maps);
    path.push(first_map);

    if run_ref_impl {
        refimpl::run_for_scenario_file(&path, scenario);
    }

    let raw_map = movingai::parser::parse_map_file(&path).unwrap();
    let size = raw_map.width() * raw_map.height();
    let mut graph = AdjacencyList::with_capacity(size);
    let mut coord2node = HashMap::<Coords2D, Node>::with_capacity(size);
    let mut node2coord = NodeMap::<Coords2D>::with_capacity(size);

    for y in 0..raw_map.height() {
        for x in 0..raw_map.width() {
            let n = graph.add_node();
            coord2node.insert((x, y), n);
            node2coord.insert(n, (x, y));
        }
    }

    for coord in raw_map.coords() {
        let n = coord2node[&coord];
        let neighbours = neighbors(&raw_map, coord);
        for (neighbour, cost) in neighbours {
            let n2 = coord2node[&neighbour];
            graph.add_edge(n, n2, cost);
        }
    }

    if let Some(o) = output_map {
        dbg::dump_map_format(&graph, &node2coord, raw_map.width(), raw_map.height(), o);
    }

    if let Some(o) = output_graph {
        dbg::dump_graph(&graph, &node2coord, o);
    }

    let mut astar_acc = graf::AStarAcceleration::new(&graph);

    println!("Scenario count: {}", scenarios.len());
    println!("Graph size: {}", graph.len());
    for (i, scenario) in scenarios.iter().enumerate() {
        let result = run_single_scenario(scenario, &mut astar_acc, &coord2node, &node2coord);
        let cost = result.cost() as f64;
        let expected = scenario.optimal_length;
        let diff = (expected - cost).abs();
        if diff > 0.001 {
            println!(
                "[{}/{}] shortest path mismatch: Expected {}, got {}, diff {}",
                i,
                scenarios.len(),
                expected,
                cost,
                diff
            );
            println!(
                "start: {:?}, end: {:?}",
                scenario.start_pos, scenario.goal_pos
            );
        }
    }
}

fn run(cli: Cli) {
    let start = std::time::Instant::now();
    let path = &cli.scenario;
    if path.is_dir() {
        let itr = std::fs::read_dir(path).expect("Failed to read directory contents");
        for path in itr {
            let path = path.expect("Failed to read path");
            if path.file_name().to_str().unwrap().ends_with(".scen") {
                println!(
                    "Running for scenarios in {p}. (cargo run --release -- {p} --maps {m})",
                    p = path.path().display(),
                    m = cli.maps.display(),
                );
                run_for_scenario_file(
                    &path.path(),
                    &cli.maps,
                    &cli.output_map,
                    &cli.output_graph,
                    cli.run_ref_impl,
                );
            }
        }
    } else {
        run_for_scenario_file(
            path,
            &cli.maps,
            &cli.output_map,
            &cli.output_graph,
            cli.run_ref_impl,
        );
    }
    println!("Took {} s to run", start.elapsed().as_secs_f32());
}

fn main() {
    let cli = Cli::parse();
    run(cli)
}
