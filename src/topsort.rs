use crate::{AdjacencyList, Node};



pub fn topsort(g: &AdjacencyList) -> Vec<Node> {
    let mut out = vec![];

    let mut roots = crate::compute_roots(g);

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
    use crate::{AdjacencyList, GraphType, Node};

    fn example_graph_trivial() -> AdjacencyList {
        let mut g = AdjacencyList::new(GraphType::Directed);
        let ns: [Node; 3] = std::array::from_fn(|_| g.add_node());

        g.add_edge(ns[0], ns[1], 1.0);
        g.add_edge(ns[0], ns[2], 1.0);

        g
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
        let mut g = AdjacencyList::new(GraphType::Directed);

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
            assert!(false);
        }
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