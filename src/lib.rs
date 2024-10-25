use std::cmp::Ordering;
use std::collections::VecDeque;

mod astar;
mod fmt;
mod map;
mod set;

pub use astar::{a_star, AStarAcceleration, HeuristicDistance};
pub use map::NodeMap;
pub use set::NodeSet;

pub use fmt::to_dot;

pub type Path = Vec<Edge>;

// TODO: u32
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct Node(usize);

impl std::fmt::Display for Node {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_fmt(format_args!("N{}", self.0))
    }
}

pub type Weight = f32;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Edge {
    pub node: Node,
    pub weight: Weight,
}
impl Eq for Edge {}
impl Ord for Edge {
    fn cmp(&self, o: &Self) -> Ordering {
        o.weight
            .partial_cmp(&self.weight)
            .expect("Invalid float")
            .then_with(|| self.node.cmp(&o.node))
    }
}
impl PartialOrd for Edge {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

fn edge(n: Node, weight: Weight) -> Edge {
    Edge { node: n, weight }
}

impl From<(Node, Weight)> for Edge {
    fn from((node, weight): (Node, Weight)) -> Self {
        Edge { node, weight }
    }
}

#[derive(Clone)]
pub struct AdjacencyList {
    nodes: Vec<Vec<Edge>>,
}

impl AdjacencyList {
    pub fn new() -> Self {
        Self::with_capacity(0)
    }

    pub fn with_capacity(cap: usize) -> Self {
        Self {
            nodes: Vec::with_capacity(cap),
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

    pub fn add_edge(&mut self, a: Node, b: Node, c: Weight) {
        assert!(self.is_valid(a) && self.is_valid(b));
        if self.has_directed_edge_unchecked(a, b) {
            return;
        }
        self.nodes[a.0].push(edge(b, c));
    }

    fn has_directed_edge_unchecked(&self, a: Node, b: Node) -> bool {
        self.nodes[a.0].iter().any(|edge| edge.node == b)
    }

    pub fn has_edge(&self, a: Node, b: Node) -> bool {
        if !self.is_valid(a) || !self.is_valid(b) {
            return false;
        }

        if self.has_directed_edge_unchecked(a, b) {
            return true;
        }

        false
    }

    /// Return the outgoing edges from n
    pub fn edges(&self, n: Node) -> impl Iterator<Item = &Edge> {
        assert!(self.is_valid(n));
        self.nodes[n.0].iter()
    }

    pub fn remove_edge(&mut self, a: Node, b: Node) -> Option<Edge> {
        assert!(self.is_valid(a));
        let edges = &mut self.nodes[a.0];
        let pos = edges.iter().position(|e| e.node == b)?;
        Some(edges.swap_remove(pos))
    }

    pub fn clear_edges(&mut self, n: Node) -> Vec<Edge> {
        assert!(self.is_valid(n));
        std::mem::take(&mut self.nodes[n.0])
    }

    pub fn nodes(&self) -> impl Iterator<Item = Node> {
        (0..self.nodes.len()).map(Node)
    }

    pub fn len(&self) -> usize {
        self.nodes.len()
    }

    pub fn is_empty(&self) -> bool {
        self.nodes.is_empty()
    }

    pub fn inverted(&self) -> Self {
        let mut out = Self::new();
        out.nodes.reserve(self.nodes.len());
        for _ in self.nodes() {
            out.add_node();
        }
        for n in self.nodes() {
            for e in self.edges(n) {
                out.add_edge(e.node, n, 1.0);
            }
        }
        out
    }
}

pub fn compute_roots(g: &AdjacencyList) -> Vec<Node> {
    let mut roots: NodeSet = NodeSet::with_capacity(g.len());
    for n in g.nodes() {
        roots.add(n);
    }

    for n in g.nodes() {
        for e in g.edges(n) {
            roots.remove(e.node);
        }
    }

    roots.to_vec()
}

fn dfs_at_impl(
    g: &AdjacencyList,
    n: Node,
    mut visit: impl FnMut(Node),
    queue: &mut VecDeque<Node>,
    visited: &mut NodeSet,
) {
    queue.clear();
    queue.push_back(n);
    while let Some(n) = queue.pop_back() {
        let new = visited.add(n);
        if !new {
            continue;
        }
        visit(n);
        for &Edge { node: child, .. } in g.edges(n) {
            queue.push_back(child);
        }
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

        dfs_at_impl(g, n, &mut visit, &mut queue, &mut visited);
    }
}

pub fn bfs(g: &AdjacencyList, mut visit: impl FnMut(Node)) {
    if g.nodes.is_empty() {
        return;
    }

    let mut queue = VecDeque::new();
    let mut visited = NodeSet::with_capacity(g.len());

    for n in g.nodes() {
        if visited.has(n) {
            continue;
        }

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

fn walk_backwards(start: &Node, end: &Node, parents: &NodeMap<Edge>) -> Option<Path> {
    let mut child = *end;
    let mut path = Path::new();
    loop {
        let Edge {
            node: parent,
            weight,
        } = *parents.get(&child)?;
        path.push(edge(child, weight));
        child = parent;
        if child == *start {
            path.push(edge(*start, 0.0));
            path.reverse();
            return Some(path);
        }
    }
}

pub fn topsort(g: &AdjacencyList) -> Vec<Node> {
    let mut out = vec![];

    let mut roots = compute_roots(g);

    let mut outgoing = g.clone();
    let mut incoming: AdjacencyList = g.inverted();

    while let Some(n) = roots.pop() {
        out.push(n);

        for e in outgoing.clear_edges(n) {
            incoming
                .remove_edge(e.node, n)
                .expect("Missing edge, inverted graph is incorrect");

            if incoming.edges(e.node).count() == 0 {
                roots.push(e.node);
            }
        }
    }

    out
}

#[cfg(test)]
mod test {
    use crate::{AdjacencyList, Node};

    fn example_graph_trivial() -> AdjacencyList {
        let mut g = AdjacencyList::new();
        let ns: [Node; 3] = std::array::from_fn(|_| g.add_node());

        g.add_edge(ns[0], ns[1], 1.0);
        g.add_edge(ns[0], ns[2], 1.0);

        g
    }

    fn example_edges_cyclic() -> Vec<(usize, usize)> {
        vec![(1, 4), (3, 6), (4, 1), (10, 19), (0, 19), (0, 4)]
    }

    fn example_edges_dag() -> Vec<(usize, usize)> {
        vec![
            (0, 1),
            (1, 2),
            (1, 3),
            (1, 4),
            (7, 10),
            (10, 11),
            (11, 12),
            (12, 13),
            (8, 9),
            (14, 9),
            (9, 16),
            (9, 17),
        ]
    }

    fn init(edges: &[(usize, usize)]) -> AdjacencyList {
        let mut g = AdjacencyList::new();

        let max = edges
            .iter()
            .max_by(|x, y| x.0.max(x.1).cmp(&y.0.max(y.1)))
            .unwrap();
        let max = std::cmp::max(max.0, max.1);
        let nodes: Vec<Node> = (0..max + 1).map(|_| g.add_node()).collect();
        for e in edges.iter() {
            g.add_edge(nodes[e.0], nodes[e.1], 1.0);
        }

        g
    }

    fn graph_from(edges_fn: fn() -> Vec<(usize, usize)>) -> AdjacencyList {
        let e = edges_fn();
        init(&e)
    }

    fn check_ordering(
        name: &'static str,
        origin: &AdjacencyList,
        actual: &[Node],
        expected: &[usize],
    ) {
        fn fmt(nodes: &[Node]) -> String {
            let mut s = String::new();
            for n in nodes {
                s.push_str(&format!("{},", n.0));
            }
            if !s.is_empty() {
                s.truncate(s.len() - 1);
            }
            s
        }

        let expected_nodes: Vec<Node> = expected.iter().copied().map(Node).collect();
        if actual != expected_nodes {
            println!("Node order diff!");
            println!("Expected:");
            println!("{}", fmt(&expected_nodes));
            println!("Actual:");
            println!("{}", fmt(actual));
            let filename = format!("{name}.dot");
            println!("Writing {filename}");
            std::fs::write(filename, crate::fmt::to_dot(origin)).unwrap();
            panic!("The ordering check failed, aboorting test");
        }
    }

    #[test]
    fn add_edge() {
        let mut g = AdjacencyList::new();

        let a = g.add_node();
        let b = g.add_node();
        g.add_edge(a, b, 1.0);
        assert!(g.has_edge(a, b));
        assert!(!g.has_edge(b, a));
    }

    #[test]
    fn add_unidirectional() {
        let mut g = AdjacencyList::new();

        let a = g.add_node();
        let b = g.add_node();
        g.add_edge(a, b, 1.0);
        g.add_edge(b, a, 1.0);
        assert!(g.has_edge(a, b));
        assert!(g.has_edge(b, a));
    }

    #[test]
    fn add() {
        let edges = example_edges_cyclic();
        let g = init(&edges);
        for e in &edges {
            assert!(g.has_edge(Node(e.0), Node(e.1)));
        }
    }

    #[test]
    fn dfs() {
        let g = graph_from(example_edges_cyclic);
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
        let g = graph_from(example_edges_cyclic);
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

    #[test]
    fn topsort_trivial() {
        let g = example_graph_trivial();
        let out = super::topsort(&g);
        check_ordering("topsort_straight", &g, &out, &[0, 2, 1]);
    }

    #[test]
    fn topsort_straight() {
        let g = init(&[(0, 1), (1, 2), (2, 3), (3, 4)]);
        let out = super::topsort(&g);
        check_ordering("topsort_straight", &g, &out, &[0, 1, 2, 3, 4]);
    }

    #[test]
    fn topsort_2() {
        let g = init(&[(0, 1), (2, 1), (1, 3), (4, 3)]);
        let out = super::topsort(&g);
        check_ordering("topsort_2", &g, &out, &[4, 2, 0, 1, 3]);
    }

    #[test]
    fn topsort_dag_example() {
        let g = graph_from(example_edges_dag);
        let expected = &[15, 14, 8, 9, 17, 16, 7, 10, 11, 12, 13, 6, 5, 0, 1, 4, 3, 2];
        let out = super::topsort(&g);
        check_ordering("topsort_dag_example", &g, &out, expected);
    }
}
