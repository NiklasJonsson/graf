use crate::Node;
// TODO: Bitset
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
        set.add(nodes[9]);
        assert!(set.has(nodes[9]));
        for &n in &nodes[0..9] {
            assert!(!set.has(n));
        }
        assert!(set.has(nodes[9]));
    }

    #[test]
    fn simple() {
        let mut set = NodeSet::with_capacity(30);
        let nodes: Vec<Node> = (0..10).map(Node).collect();
        for (i, &n) in nodes.iter().enumerate() {
            if i % 2 == 0 {
                set.add(n);
                assert!(set.has(n));
            } else {
                assert!(!set.has(n));
            }
        }

        for (i, &n) in nodes.iter().enumerate() {
            if i % 2 == 0 {
                assert!(set.has(n));
            } else {
                assert!(!set.has(n));
            }
        }
    }
}
