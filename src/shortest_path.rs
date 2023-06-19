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

                if !queue.iter().any(|e| e.node == child) {
                    let estimated_end_cost = start_to_child_cost + heuristic(&child);
                    queue.push(edge(child, estimated_end_cost));
                }
            }
        }
    }

    None
}

