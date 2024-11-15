use movingai::Coords2D;
use movingai::Map2D;
use movingai::MovingAiMap;
use std::cmp::Ordering;
use std::collections::{BinaryHeap, HashMap};

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

fn walk_backwards(
    start: Coords2D,
    goal: Coords2D,
    parents: &HashMap<Coords2D, (Coords2D, f64)>,
) -> Vec<Coords2D> {
    let mut child = goal;
    let mut path = Vec::new();
    loop {
        let parent = *parents.get(&child).unwrap();
        path.push(child);
        child = parent.0;
        if child == start {
            path.push(start);
            path.reverse();
            return path;
        }
    }
}

pub fn shortest_path(map: &MovingAiMap, start: Coords2D, goal: Coords2D) -> Vec<Coords2D> {
    let mut heap = BinaryHeap::new();
    let mut visited = Vec::<Coords2D>::new();
    let mut parents: HashMap<Coords2D, (Coords2D, f64)> = HashMap::new();

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
            return walk_backwards(start, goal, &parents);
        }

        if visited.contains(&current) {
            continue;
        }

        visited.push(current);

        for neigh in map.neighbors(current) {
            let new_h = distance(neigh, goal);
            let i = distance(neigh, current);

            match parents.entry(neigh) {
                std::collections::hash_map::Entry::Occupied(mut entry) => {
                    if entry.get_mut().1 > (g + i) {
                        entry.get_mut().1 = g + i;
                    }
                }
                std::collections::hash_map::Entry::Vacant(entry) => {
                    entry.insert((current, g + i));
                }
            }
            let next = SearchNode {
                f: g + i + new_h,
                g: g + i,
                h: new_h,
                current: neigh,
            };
            heap.push(next);
        }
    }

    panic!("No goal");
}
