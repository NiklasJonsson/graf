use crate::{walk_backwards, AdjacencyList, Edge, Node, NodeMap, Path, Weight};

use std::collections::BinaryHeap;

struct ImmutableAdjacencyList {
    node_data: Vec<Edge>,
    node_info: Vec<NodeInfo>,
}

#[derive(Debug, Clone, Copy)]
struct NodeInfo {
    offset: usize,
    len: usize,
}

// Impl for converting to jagged array
fn lock_graph(g: &AdjacencyList) -> ImmutableAdjacencyList {
    let nodes: &[Vec<Edge>] = &g.nodes;
    let len: usize = nodes.iter().map(|inner| inner.iter().count()).sum();
    let mut node_data: Vec<Edge> = Vec::with_capacity(len);
    let mut node_info: Vec<NodeInfo> = Vec::with_capacity(nodes.len());

    for edges in nodes {
        let len = edges.len();
        let offset = node_data.len();
        node_data.extend_from_slice(&edges);
        node_info.push(NodeInfo { offset, len });
    }

    ImmutableAdjacencyList {
        node_data,
        node_info,
    }
}

impl ImmutableAdjacencyList {
    /// Return the outgoing edges from n
    fn edges(&self, n: Node) -> impl Iterator<Item = &Edge> {
        let NodeInfo { offset, len } = self.node_info[n.0];
        self.node_data[offset..offset + len].iter()
    }

    pub fn len(&self) -> usize {
        self.node_info.len()
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }
}

pub struct AStarAcceleration {
    graph: ImmutableAdjacencyList,
    node_cost: NodeMap<Weight>,
    parents: NodeMap<Edge>,
    queue: BinaryHeap<Edge>,
}

impl AStarAcceleration {
    pub fn new(g: &AdjacencyList) -> Self {
        let len = g.len();
        Self {
            graph: lock_graph(g),
            node_cost: NodeMap::with_capacity(len),
            parents: NodeMap::with_capacity(len),
            queue: BinaryHeap::with_capacity(len),
        }
    }

    pub fn clear_transients(&mut self) {
        self.node_cost.clear();
        self.parents.clear();
        self.queue.clear();
    }
}

pub trait HeuristicDistance {
    fn cost(&self, node: &Node) -> Weight;
}

/// Find the shortest path between two nodes
pub fn a_star(
    acc: &mut AStarAcceleration,
    start: Node,
    end: Node,
    heuristic: impl HeuristicDistance,
) -> Option<Path> {
    acc.clear_transients();

    let g = &acc.graph;
    if g.is_empty() || start == end {
        return None;
    }

    let node_cost: &mut NodeMap<Weight> = &mut acc.node_cost;
    let parents: &mut NodeMap<Edge> = &mut acc.parents;
    let queue: &mut BinaryHeap<Edge> = &mut acc.queue;
    node_cost.insert(start, 0.0);
    queue.push(Edge {
        node: start,
        weight: 0.0,
    });

    while let Some(Edge { node: cur, .. }) = queue.pop() {
        if cur == end {
            return walk_backwards(&start, &end, &parents);
        }

        for &Edge {
            node: child,
            weight: cost,
        } in g.edges(cur)
        {
            let start_to_child_cost = node_cost[cur] + cost;
            if !node_cost.has(&child) || start_to_child_cost < node_cost[child] {
                node_cost.insert(child, start_to_child_cost);
                parents.insert(
                    child,
                    Edge {
                        node: cur,
                        weight: cost,
                    },
                );

                if !queue.iter().any(|e| e.node == child) {
                    let estimated_end_cost = start_to_child_cost + heuristic.cost(&child);
                    queue.push(Edge {
                        node: child,
                        weight: estimated_end_cost,
                    });
                }
            }
        }
    }

    None
}
