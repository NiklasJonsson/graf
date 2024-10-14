use crate::Node;
// TODO: Bitset
#[derive(Default, Debug)]
pub struct NodeSet {
    v: Vec<bool>,
}

impl NodeSet {
    pub fn new() -> Self {
        Self { v: Vec::new() }
    }

    pub fn with_capacity(capacity: usize) -> Self {
        Self {
            v: Vec::with_capacity(capacity),
        }
    }

    /// Add a node to this set.
    /// Returns true if the node was not already in the set.
    pub fn add(&mut self, n: Node) -> bool {
        let i = n.0;
        if i >= self.v.len() {
            self.v.resize(i + 1, false);
        }
        let new = !self.v[i];
        self.v[i] = true;
        new
    }

    pub fn add_many(&mut self, nodes: &[Node]) -> usize {
        let mut count = 0;
        for n in nodes {
            let added = self.add(*n);
            if added {
                count += 1;
            }
        }
        count
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

    pub fn remove_many(&mut self, nodes: &[Node]) -> usize {
        let mut count = 0;
        for n in nodes {
            let removed = self.remove(*n);
            if removed {
                count += 1;
            }
        }
        count
    }

    pub fn to_vec(self) -> Vec<Node> {
        self.v
            .into_iter()
            .enumerate()
            .filter_map(|(i, x)| if x { Some(Node(i)) } else { None })
            .collect::<Vec<Node>>()
    }

    pub fn is_empty(&self) -> bool {
        if self.v.is_empty() {
            true
        } else {
            self.v.iter().all(|v| !v)
        }
    }

    pub fn size(&self) -> usize {
        self.v.iter().filter(|&&x| x).count()
    }

    pub fn clear(&mut self) {
        for v in &mut self.v {
            *v = false;
        }
    }
}

#[cfg(test)]
mod test {
    use crate::{Node, NodeSet};

    #[test]
    fn empty_new() {
        let set = NodeSet::new();
        let nodes: Vec<Node> = (0..10).map(Node).collect();
        for n in nodes {
            assert!(!set.has(n));
        }
    }

    #[test]
    fn empty_with_capacity() {
        let set = NodeSet::with_capacity(30);
        let nodes: Vec<Node> = (0..10).map(Node).collect();
        for n in nodes {
            assert!(!set.has(n));
        }
    }

    #[test]
    fn with_capacity_insert() {
        let mut set = NodeSet::with_capacity(30);
        let nodes: Vec<Node> = (0..10).map(Node).collect();
        let new = set.add(nodes[9]);
        assert!(new);
        assert!(set.has(nodes[9]));
        for &n in &nodes[0..9] {
            assert!(!set.has(n));
        }
        assert!(set.has(nodes[9]));
    }

    #[test]
    fn is_empty() {
        let mut set = NodeSet::with_capacity(30);
        let nodes: Vec<Node> = (0..10).map(Node).collect();
        assert!(set.is_empty());
        set.add(nodes[0]);
        assert!(!set.is_empty());

        set.add(nodes[0]);
        assert!(!set.is_empty());

        set.add(nodes[1]);
        set.add(nodes[2]);
        assert!(!set.is_empty());

        set.remove_many(&nodes[0..=2]);
        assert!(set.is_empty());
    }

    #[test]
    fn add_many() {
        let mut set = NodeSet::with_capacity(30);
        let nodes: Vec<Node> = (0..10).map(Node).collect();

        {
            let count = set.add_many(&[]);
            assert_eq!(count, 0);
        }

        {
            let count = set.add_many(&nodes[0..1]);
            assert_eq!(count, 1);
            assert!(set.has(nodes[0]));
        }

        {
            let count = set.add_many(&nodes[0..3]);
            assert_eq!(count, 2);
            assert!(set.has(nodes[0]));
            assert!(set.has(nodes[1]));
            assert!(set.has(nodes[2]));
        }
    }

    #[test]
    fn remove_many() {
        let mut set = NodeSet::with_capacity(30);
        let nodes: Vec<Node> = (0..10).map(Node).collect();

        {
            let count = set.remove_many(&[]);
            assert_eq!(count, 0)
        }

        {
            let count = set.remove_many(&nodes[0..10]);
            assert_eq!(count, 0)
        }

        set.add_many(&nodes[0..10]);

        {
            let count = set.remove_many(&nodes[0..1]);
            assert_eq!(count, 1);
            assert!(!set.has(nodes[0]));
        }

        {
            let count = set.remove_many(&nodes[0..3]);
            assert_eq!(count, 2);
            assert!(!set.has(nodes[0]));
            assert!(!set.has(nodes[1]));
            assert!(!set.has(nodes[2]));
        }
    }

    #[test]
    fn clear() {
        let mut set = NodeSet::with_capacity(30);
        let nodes: Vec<Node> = (0..10).map(Node).collect();
        assert!(set.is_empty());
        set.add(nodes[0]);
        set.clear();
        assert!(set.is_empty());

        set.add(nodes[0]);
        set.add(nodes[1]);
        assert!(!set.is_empty());
        set.clear();
        assert!(set.is_empty());

        for node in nodes {
            assert!(!set.has(node));
        }
    }

    #[test]
    fn simple() {
        let mut set = NodeSet::with_capacity(30);
        let nodes: Vec<Node> = (0..10).map(Node).collect();
        assert!(set.is_empty());
        for (i, &n) in nodes.iter().enumerate() {
            if i % 2 == 0 {
                let added = set.add(n);
                assert!(added);
                assert!(set.has(n));
            } else {
                assert!(!set.has(n));
            }
        }
        assert!(!set.is_empty());

        for (i, &n) in nodes.iter().enumerate() {
            if i % 2 == 0 {
                assert!(set.has(n));
            } else {
                assert!(!set.has(n));
            }
        }

        set.clear();
        assert!(set.is_empty());
    }
}
