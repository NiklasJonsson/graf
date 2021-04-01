use std::collections::VecDeque;

mod map;
mod set;

pub use map::NodeMap;
pub use set::NodeSet;

#[derive(Debug, PartialEq, Eq)]
pub enum GraphType {
    Directed,
    Undirected,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct Node(usize);
pub type Cost = f32;

pub struct AdjacencyList {
    nodes: Vec<Vec<(Node, Cost)>>,
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
        self.nodes[a.0].push((b, c));
        if self.ty == GraphType::Undirected {
            assert!(self.has_directed_edge_unchecked(b, a));
            self.nodes[b.0].push((a, c));
        }
    }

    fn has_directed_edge_unchecked(&self, a: Node, b: Node) -> bool {
        return self.nodes[a.0].iter().find(|&&(x, _)| x == b).is_some();
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

    pub fn edges(&self, n: Node) -> impl Iterator<Item = &(Node, Cost)> {
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
            for &(child, _) in g.edges(n) {
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
            for &(child, _) in g.edges(n) {
                queue.push_back(child);
            }
        }
    }
}

pub type Path = Vec<(Node, Cost)>;

pub fn pathfind(g: &AdjacencyList, start: Node, end: Node) -> Option<Path> {
    if g.nodes.is_empty() || start == end {
        return None;
    }

    let mut queue = VecDeque::new();
    let mut visited = NodeSet::with_capacity(g.size());
    let mut parents = NodeMap::with_capacity(g.size());
    queue.push_back(start);
    visited.add(start);

    let mut found = false;
    while let Some(n) = queue.pop_front() {
        if found {
            break;
        }
        for &(child, cost) in g.edges(n) {
            if visited.has(child) {
                continue;
            }

            parents.insert(child, (n, cost));
            if child == end {
                found = true;
                break;
            }

            visited.add(child);
            queue.push_back(child);
        }
    }

    if !found {
        return None;
    }

    let mut child = end;
    let mut path = Path::new();
    loop {
        let (parent, cost) = *parents
            .get(&child)
            .expect(&format!("Expected {:?} to have a parent", child));
        path.push((child, cost));
        child = parent;
        if child == start {
            break;
        }
    }
    path.push((start, 0.0));
    path.reverse();
    Some(path)
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
