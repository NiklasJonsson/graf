use std::collections::VecDeque;

#[derive(Debug, PartialEq, Eq)]
pub enum GraphType {
    Directed,
    Undirected,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Node(usize);

// TODO: Bitset
pub struct NodeSet {
    v: Vec<bool>,
}

impl NodeSet {
    pub fn new() -> Self {
        Self { v: Vec::new() }
    }

    pub fn add(&mut self, n: Node) {
        let i = n.0;
        if i >= self.v.len() {
            self.v.resize(i + 1, false);
        }
        self.v[i] = true;
    }

    pub fn has(&self, n: Node) -> bool {
        let i = n.0;
        if i >= self.v.len() {
            false
        } else {
            self.v[i]
        }
    }

    pub fn remove(&mut self, n: Node) -> bool {
        let i = n.0;
        if i >= self.v.len() {
            false
        } else {
            let r = self.v[i];
            self.v[i] = false;
            r
        }
    }
}

pub struct AdjacencyList {
    nodes: Vec<Vec<Node>>,
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

    pub fn add_edge(&mut self, a: Node, b: Node) {
        assert!(self.is_valid(a) && self.is_valid(b));
        assert!(self.nodes[a.0].iter().find(|&&x| x == b).is_none());
        self.nodes[a.0].push(b);
        if self.ty == GraphType::Undirected {
            assert!(self.nodes[b.0].iter().find(|&&x| x == a).is_none());
            self.nodes[b.0].push(a);
        }
    }

    pub fn has_edge(&self, a: Node, b: Node) -> bool {
        if self.nodes[a.0].iter().find(|&&x| x == b).is_some() {
            return true;
        }

        if self.ty == GraphType::Undirected {
            return self.nodes[b.0].iter().find(|&&x| x == a).is_some();
        }

        false
    }

    pub fn edges(&self, n: Node) -> impl Iterator<Item = &Node> {
        self.nodes[n.0].iter()
    }

    pub fn nodes(&self) -> impl Iterator<Item = Node> {
        (0..self.nodes.len()).map(Node)
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
            for child in g.edges(n) {
                queue.push_back(*child);
            }
        }
    }
}

pub fn bfs(g: &AdjacencyList, mut visit: impl FnMut(Node)) {
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
        while let Some(n) = queue.pop_front() {
            if visited.has(n) {
                continue;
            }
            visited.add(n);
            visit(n);
            for child in g.edges(n) {
                queue.push_back(*child);
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use crate::{AdjacencyList, GraphType, Node};

    #[test]
    fn add_simple() {
        let mut g = AdjacencyList::new(GraphType::Directed);

        let a = g.add_node();
        let b = g.add_node();
        g.add_edge(a, b);
        assert!(g.has_edge(a, b));
    }

    fn example_edges() -> [(usize, usize); 6] {
        [(1, 4), (3, 6), (4, 1), (10, 19), (0, 19), (0, 4)]
    }

    fn init(edges: &[(usize, usize)]) -> AdjacencyList {
        let mut g = AdjacencyList::new(GraphType::Directed);

        let nodes: Vec<Node> = (0..20).map(|_| g.add_node()).collect();
        for e in edges.iter() {
            g.add_edge(nodes[e.0], nodes[e.1]);
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
