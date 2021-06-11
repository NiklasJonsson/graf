#![feature(trait_alias)]

use std::collections::VecDeque;
use std::{cmp::Ordering, collections::BinaryHeap};

mod map;
mod set;

pub use map::NodeMap;
pub use set::NodeSet;

#[derive(Debug, PartialEq, Eq)]
pub enum GraphType {
    Directed,
    Undirected,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct Node(usize);
pub type Cost = f32;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Edge {
    pub node: Node,
    pub cost: Cost,
}
impl Eq for Edge {}
impl Ord for Edge {
    fn cmp(&self, o: &Self) -> Ordering {
        o.cost
            .partial_cmp(&self.cost)
            .expect("Invalid float")
            .then_with(|| self.node.cmp(&o.node))
    }
}
impl PartialOrd for Edge {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

fn edge(n: Node, c: Cost) -> Edge {
    Edge { node: n, cost: c }
}

impl From<(Node, Cost)> for Edge {
    fn from((node, cost): (Node, Cost)) -> Self {
        Edge { node, cost }
    }
}

pub struct AdjacencyList {
    nodes: Vec<Vec<Edge>>,
    ty: GraphType,
}

impl AdjacencyList {
    pub fn new(ty: GraphType) -> Self {
        Self {
            nodes: Vec::new(),
            ty,
        }
    }

    pub fn add_node(&mut self) -> Node {
        let n = Node(self.nodes.len());
        self.nodes.push(Vec::new());
        n
    }

    fn is_valid(&self, n: Node) -> bool {
        n.0 < self.nodes.len()
    }

    pub fn add_edge(&mut self, a: Node, b: Node, c: Cost) {
        assert!(self.is_valid(a) && self.is_valid(b));
        if self.has_directed_edge_unchecked(a, b) {
            return;
        }
        self.nodes[a.0].push(edge(b, c));
        if self.ty == GraphType::Undirected {
            assert!(self.has_directed_edge_unchecked(b, a));
            self.nodes[b.0].push(edge(a, c));
        }
    }

    fn has_directed_edge_unchecked(&self, a: Node, b: Node) -> bool {
        return self.nodes[a.0]
            .iter()
            .find(|&&edge| edge.node == b)
            .is_some();
    }

    pub fn has_edge(&self, a: Node, b: Node) -> bool {
        if !self.is_valid(a) || !self.is_valid(b) {
            return false;
        }

        if self.has_directed_edge_unchecked(a, b) {
            return true;
        }

        if self.ty == GraphType::Undirected {
            return self.has_directed_edge_unchecked(b, a);
        }

        false
    }

    pub fn edges(&self, n: Node) -> impl Iterator<Item = &Edge> {
        self.nodes[n.0].iter()
    }

    pub fn nodes(&self) -> impl Iterator<Item = Node> {
        (0..self.nodes.len()).map(Node)
    }

    pub fn size(&self) -> usize {
        self.nodes.len()
    }
}

pub fn dfs(g: &AdjacencyList, mut visit: impl FnMut(Node)) {
    if g.nodes.is_empty() {
        return;
    }

    let mut queue = VecDeque::new();
    let mut visited = NodeSet::new();

    for n in g.nodes() {
        if visited.has(n) {
            continue;
        }

        queue.clear();
        queue.push_back(n);
        while let Some(n) = queue.pop_back() {
            if visited.has(n) {
                continue;
            }
            visited.add(n);
            visit(n);
            for &Edge { node: child, .. } in g.edges(n) {
                queue.push_back(child);
            }
        }
    }
}

pub fn bfs(g: &AdjacencyList, mut visit: impl FnMut(Node)) {
    if g.nodes.is_empty() {
        return;
    }

    let mut queue = VecDeque::new();
    let mut visited = NodeSet::with_capacity(g.size());

    for n in g.nodes() {
        if visited.has(n) {
            continue;
        }

        queue.clear();
        queue.push_back(n);
        while let Some(n) = queue.pop_front() {
            if visited.has(n) {
                continue;
            }
            visited.add(n);
            visit(n);
            for &Edge { node: child, .. } in g.edges(n) {
                queue.push_back(child);
            }
        }
    }
}

pub type Path = Vec<Edge>;

fn reconstruct(start: &Node, end: &Node, parents: &NodeMap<Edge>) -> Option<Path> {
    let mut child = *end;
    let mut path = Path::new();
    loop {
        let Edge { node: parent, cost } = *parents.get(&child)?;
        path.push(edge(child, cost));
        child = parent;
        if child == *start {
            path.push(edge(*start, 0.0));
            path.reverse();
            return Some(path);
        }
    }
}

/// Find a path between two nodes
pub fn path(g: &AdjacencyList, start: Node, end: Node) -> Option<Path> {
    if g.nodes.is_empty() || start == end {
        return None;
    }

    let mut queue = VecDeque::new();
    let mut visited = NodeSet::with_capacity(g.size());
    let mut parents = NodeMap::with_capacity(g.size());
    queue.push_back(start);
    visited.add(start);

    while let Some(n) = queue.pop_front() {
        for &Edge { node: child, cost } in g.edges(n) {
            if visited.has(child) {
                continue;
            }

            parents.insert(child, edge(n, cost));
            if child == end {
                return reconstruct(&start, &end, &parents);
            }

            visited.add(child);
            queue.push_back(child);
        }
    }

    None
}

pub trait HeuristicDistanceFn = Fn(&Node) -> Cost;

pub fn shortest_path(
    g: &AdjacencyList,
    start: Node,
    end: Node,
    heuristic: impl HeuristicDistanceFn,
) -> Option<Path> {
    a_star(g, start, end, heuristic)
}
/// Find the shortest path between two nodes
pub fn a_star(
    g: &AdjacencyList,
    start: Node,
    end: Node,
    heuristic: impl HeuristicDistanceFn,
) -> Option<Path> {
    if g.nodes.is_empty() || start == end {
        return None;
    }

    let mut node_cost: NodeMap<Cost> = NodeMap::with_capacity(g.size());
    let mut parents: NodeMap<Edge> = NodeMap::with_capacity(g.size());
    let mut queue: BinaryHeap<Edge> = std::collections::BinaryHeap::new();
    node_cost.insert(start, 0.0);
    queue.push(edge(start, 0.0));

    while let Some(Edge { node: cur, .. }) = queue.pop() {
        if cur == end {
            return reconstruct(&start, &end, &parents);
        }

        for &Edge { node: child, cost } in g.edges(cur) {
            let start_to_child_cost = node_cost[cur] + cost;
            if !node_cost.has(&child) || start_to_child_cost < node_cost[child] {
                node_cost.insert(child, start_to_child_cost);
                parents.insert(child, edge(cur, cost));

                if queue.iter().find(|e| e.node == child).is_none() {
                    let estimated_end_cost = start_to_child_cost + heuristic(&child);
                    queue.push(edge(child, estimated_end_cost));
                }
            }
        }
    }

    None
}

#[cfg(test)]
mod test {
    use crate::{AdjacencyList, GraphType, Node};

    #[test]
    fn add_simple() {
        let mut g = AdjacencyList::new(GraphType::Directed);

        let a = g.add_node();
        let b = g.add_node();
        g.add_edge(a, b, 1.0);
        assert!(g.has_edge(a, b));
    }

    fn example_edges() -> [(usize, usize); 6] {
        [(1, 4), (3, 6), (4, 1), (10, 19), (0, 19), (0, 4)]
    }

    fn init(edges: &[(usize, usize)]) -> AdjacencyList {
        let mut g = AdjacencyList::new(GraphType::Directed);

        let nodes: Vec<Node> = (0..20).map(|_| g.add_node()).collect();
        for e in edges.iter() {
            g.add_edge(nodes[e.0], nodes[e.1], 1.0);
        }

        g
    }

    #[test]
    fn add() {
        let edges = example_edges();
        let g = init(&edges);
        for e in &edges {
            assert!(g.has_edge(Node(e.0), Node(e.1)));
        }
    }

    #[test]
    fn dfs() {
        let edges = example_edges();
        let g = init(&edges);
        let mut visited = Vec::new();
        let visit = |n: Node| visited.push(n);
        crate::dfs(&g, visit);

        let expected: Vec<Node> = vec![
            0, 4, 1, 19, 2, 3, 6, 5, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18,
        ]
        .into_iter()
        .map(Node)
        .collect();
        assert_eq!(visited, expected);
    }

    #[test]
    fn bfs() {
        let edges = example_edges();
        let g = init(&edges);
        let mut visited = Vec::new();
        let visit = |n: Node| visited.push(n);
        crate::bfs(&g, visit);

        let expected: Vec<Node> = vec![
            0, 19, 4, 1, 2, 3, 6, 5, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18,
        ]
        .into_iter()
        .map(Node)
        .collect();
        assert_eq!(visited, expected);
    }
}
