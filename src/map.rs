use crate::Node;

#[derive(Default, Debug)]
pub struct NodeMap<T> {
    v: Vec<Option<T>>,
}

impl<T> std::ops::Index<Node> for NodeMap<T> {
    type Output = T;
    fn index(&self, n: Node) -> &T {
        if let Some(t) = self.get(&n) {
            t
        } else {
            panic!("Node not in map: {:?}", n);
        }
    }
}

impl<T> std::ops::Index<&Node> for NodeMap<T> {
    type Output = T;
    fn index(&self, n: &Node) -> &T {
        if let Some(t) = self.get(n) {
            t
        } else {
            panic!("Node not in map: {:?}", n);
        }
    }
}

impl<T> NodeMap<T> {
    pub fn new() -> Self {
        Self { v: Vec::new() }
    }

    pub fn with_capacity(cap: usize) -> Self {
        Self {
            v: Vec::with_capacity(cap),
        }
    }

    pub fn insert(&mut self, n: Node, t: T) {
        let i = n.0;
        if i >= self.v.len() {
            self.v.resize_with(i + 1, || None);
        }
        self.v[i] = Some(t);
    }

    pub fn has(&self, n: &Node) -> bool {
        let i = n.0;
        if i >= self.v.len() {
            false
        } else {
            self.v[i].is_some()
        }
    }

    pub fn get(&self, n: &Node) -> Option<&T> {
        if !self.has(n) {
            None
        } else {
            self.v[n.0].as_ref()
        }
    }

    pub fn get_mut(&mut self, n: &Node) -> Option<&mut T> {
        if !self.has(n) {
            None
        } else {
            self.v[n.0].as_mut()
        }
    }

    pub fn remove(&mut self, n: &Node) -> Option<T> {
        if !self.has(n) {
            None
        } else {
            self.v[n.0].take()
        }
    }
}
