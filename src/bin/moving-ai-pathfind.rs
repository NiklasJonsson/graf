use movingai::{
    parser::{parse_map_file, parse_scen_file},
    MovingAiMap,
};
use movingai::{Coords2D, Map2D as _};
use structopt::StructOpt;

use indicatif::ProgressIterator;

use graf::{AdjacencyList, Cost, Edge, Node, NodeMap};

use std::{collections::HashMap, io::Write};

use std::path::{Path, PathBuf};

#[derive(Debug, StructOpt)]
struct Args {
    #[structopt(parse(from_os_str))]
    input: PathBuf,
    #[structopt(long, parse(from_os_str))]
    output_graph: Option<PathBuf>,
    #[structopt(long, parse(from_os_str))]
    output_map: Option<PathBuf>,
}

fn dump_map_format(
    graph: &AdjacencyList,
    node2coord: &NodeMap<Coords2D>,
    width: usize,
    height: usize,
    path: &Path,
) {
    use std::fs::File;

    let mut file = File::create(&path).expect(&format!("Bad path: {}", path.display()));
    let mut data = Vec::with_capacity(width * height);
    data.resize(width * height, false);
    for n in graph.nodes() {
        for n2 in graph.edges(n).map(|x| x.node) {
            let (x, y) = node2coord[n2];
            data[x + width * y] = true;
        }
    }

    for i in 0..height {
        let start = i * width;
        let end = (i + 1) * width;
        for &reachable in &data[start..end] {
            if reachable {
                write!(file, ".").unwrap();
            } else {
                write!(file, "T").unwrap();
            }
        }
        writeln!(file, "").unwrap();
    }
}

fn dump_graph(graph: &AdjacencyList, node2coord: &NodeMap<Coords2D>, fpath: &Path) {
    use std::fs::File;
    let mut file = File::create(&fpath).expect(&format!("Bad path: {}", fpath.display()));
    for node in graph.nodes() {
        write!(file, "{:?} ({:?}):", node, node2coord[&node]).unwrap();
        let mut first = true;
        for child in graph.edges(node).map(|x| x.node) {
            if !first {
                write!(file, ", ").unwrap();
            }
            first = false;
            write!(file, "{:?} ({:?})", child, node2coord[child]).unwrap();
        }
        writeln!(file, "").unwrap();
    }
}

fn dump_path(gpath: &graf::Path, node2coord: &NodeMap<Coords2D>) {
    let mut total = 0.0;
    for Edge { node, cost } in gpath {
        let c = node2coord[node];
        total += cost;
        println!("({:?}, {})", c, cost);
    }

    println!("total: {}", total);
}

const DIAG_COST: Cost = std::f32::consts::SQRT_2;
const STRAIGHT_COST: Cost = 1.0;
fn neighbors(map: &MovingAiMap, tile: Coords2D) -> Vec<(Coords2D, Cost)> {
    let (x, y) = (tile.0 as isize, tile.1 as isize);
    let all: Vec<((isize, isize), Cost)> = vec![
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

fn run(args: Args) {
    let scenarios = parse_scen_file(&args.input).unwrap();
    let map = scenarios[0].map_file.clone();
    let mut path = std::path::PathBuf::from("data/dao-map");
    path.push(map);
    let raw_map = parse_map_file(&path).unwrap();
    let size = raw_map.width() * raw_map.height();
    let mut graph = AdjacencyList::new(graf::GraphType::Directed);
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
        for (neighbour, cost) in neighbors(&raw_map, coord) {
            let n2 = coord2node[&neighbour];
            graph.add_edge(n, n2, cost);
        }
    }

    if let Some(o) = args.output_map {
        dump_map_format(&graph, &node2coord, raw_map.width(), raw_map.height(), &o);
    }

    if let Some(o) = args.output_graph {
        dump_graph(&graph, &node2coord, &o);
    }

    for scenario in scenarios.iter() {
        let start = *coord2node
            .get(&scenario.start_pos)
            .expect("This should have a node assigned");
        let end = *coord2node
            .get(&scenario.goal_pos)
            .expect("This should have a node assigned");

        let heuristic = |n: &Node| -> Cost {
            let (n_x, n_y) = node2coord
                .get(n)
                .expect("Unrecognized node, no matching coords");
            let x = scenario.goal_pos.0 as Cost - *n_x as Cost;
            let y = scenario.goal_pos.1 as Cost - *n_y as Cost;
            (x.powi(2) + y.powi(2)).sqrt()
        };

        let path = graf::shortest_path(&graph, start, end, heuristic).expect("Failed to find path");
        let cost = path
            .iter()
            .fold(0.0, |acc, Edge { node: _, cost }| acc + cost) as f64;
        let diff = (scenario.optimal_length - cost).abs();
        if diff > 0.0001 {
            println!("shortest path mismatch: {}", diff);
            println!(
                "start: {:?}, end: {:?}",
                scenario.start_pos, scenario.goal_pos
            );
        }
    }
}

fn main() {
    let args = Args::from_args();
    run(args)
}
