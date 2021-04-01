use movingai::{
    parser::{parse_map_file, parse_scen_file},
    MovingAiMap,
};
use movingai::{Coords2D, Map2D as _};
use structopt::StructOpt;

use graf::{AdjacencyList, Cost, Node, NodeMap};

use std::collections::hash_map::Entry;
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
        for (n2, _) in graph.edges(n) {
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
        for (c, _) in graph.edges(node) {
            if !first {
                write!(file, ", ").unwrap();
            }
            first = false;
            write!(file, "{:?} ({:?})", c, node2coord[c]).unwrap();
        }
        writeln!(file, "").unwrap();
    }
}

fn dump_path(gpath: &graf::Path, node2coord: &NodeMap<Coords2D>) {
    let mut total = 0.0;
    for (node, cost) in gpath {
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
    let mut graph = AdjacencyList::new(graf::GraphType::Directed);
    let mut coord2node = HashMap::<Coords2D, Node>::new();
    let mut node2coord = NodeMap::<Coords2D>::new();

    let mut get_or_insert = |g: &mut AdjacencyList, c: Coords2D| -> Node {
        let n = match coord2node.entry(c) {
            Entry::Vacant(entry) => {
                let n = g.add_node();
                node2coord.insert(n, c);
                *entry.insert(n)
            }
            Entry::Occupied(entry) => *entry.get(),
        };

        n
    };

    for coord in raw_map.coords() {
        let n = get_or_insert(&mut graph, coord);

        for (neighbour, cost) in neighbors(&raw_map, coord) {
            let n2 = get_or_insert(&mut graph, neighbour);
            graph.add_edge(n, n2, cost);
        }
    }

    if let Some(o) = args.output_map {
        dump_map_format(&graph, &node2coord, raw_map.width(), raw_map.height(), &o);
    }

    if let Some(o) = args.output_graph {
        dump_graph(&graph, &node2coord, &o);
    }

    for scenario in &scenarios {
        let start = *coord2node
            .get(&scenario.start_pos)
            .expect("This should have a node assigned");
        let end = *coord2node
            .get(&scenario.goal_pos)
            .expect("This should have a node assigned");
        println!("{:?} ({:?}):", start, node2coord[&start]);
        println!("{:?} ({:?}):", end, node2coord[&end]);
        let path = graf::pathfind(&graph, start, end).expect("Failed to find path");
        dump_path(&path, &node2coord);
        assert_eq!(
            scenario.optimal_length as f32,
            path.iter().fold(0.0, |acc, e| acc + e.1)
        );
    }
}

fn main() {
    let args = Args::from_args();
    run(args)
}
