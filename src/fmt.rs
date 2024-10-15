use crate::AdjacencyList as Graph;

mod dot {
    use crate::AdjacencyList as Graph;
    fn write_header(name: &str, out: &mut String) {
        out.push_str("digraph ");
        out.push_str(name);
        out.push_str(" {\n");
    }

    fn write_footer(out: &mut String) {
        out.push_str("\n}\n");
    }

    pub fn write(g: &Graph, out: &mut String) {
        write_header("G", out);

        for n in g.nodes() {
            out.push_str(&format!("{}\n", n));
        }

        for n in g.nodes() {
            for e in g.edges(n) {
                out.push_str(&format!(
                    "{} -> {} [label = \"{}\"];\n",
                    n, e.node, e.weight
                ));
            }
        }

        write_footer(out);
    }
}

pub fn to_dot(g: &Graph) -> String {
    let mut out = String::new();
    dot::write(g, &mut out);
    out
}
