# Project for learning graphs

graf is a generic graph library

bin/moving-ai-pathfind.rs is tool for experimenting with 2d game maps from the [moving ai game map data repo](https://movingai.com/benchmarks/grids.html)

Run like:

```sh
cargo run --release -- --maps data/dao-map/ data/dao-scen/arena.map.scen # Run a specific scenario file
cargo run --release -- --maps data/dao-map/ data/dao-scen/ # Run all scenario files
```

## Getting the data

```sh
nu script/download-data.nu
```

## TODO

* Update deps
* Fix remaining optimal path failures/diff. Test with BFS for shortest path and compare.
