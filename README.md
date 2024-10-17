# Project for learning graphs

graf is a generic graph library

bin/moving-ai-pathfind.rs is tool for experimenting with 2d game maps from the [moving ai game map data repo](https://movingai.com/benchmarks/grids.html)

Run like:

```sh
cargo run --release -- data/dao-scen data/dao-map
```

## Getting the data

```sh
# nushell
mkdir data
mkdir data/dao-scen
mkdir data/dao-map
http get https://movingai.com/benchmarks/dao/dao-scen.zip | save data/dao-scen.zip
http get https://movingai.com/benchmarks/dao/dao-map.zip | save data/dao-map.zip
tar -xf data/dao-scen.zip -C data/dao-scen
tar -xf data/dao-map.zip -C data/dao-map
```

## TODO

* Update deps
* Fix remaining optimal path failures/diff. Test with BFS for shortest path and compare.
