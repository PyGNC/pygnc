# Algorithms

Each directory in `algorithms/` should be a standalone python module.
It should not rely on imports from the `pygnc` module.
This makes the algorithm implementations independent of the rest of the system.
Parameters for the algorithms should be set when instantiating the algorithm class.