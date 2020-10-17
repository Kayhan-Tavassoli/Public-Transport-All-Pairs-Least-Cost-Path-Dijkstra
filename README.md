# Public-Transport-All-Pairs-Least-Cost-Path-Dijkstra
This is a modified Dijkstra algorithm that solves the All Pairs Least Cost Paths (APLCP) problem, factoring in costs (waiting time and fare) incurred conditionally at transfer nodes. 
A public transportation graph (Class TransitGraph) is defined which inherits from NetworkX’s Graph and holds additional methods and attributes that are necessary for calculating transfers and conditional costs.

This was first developed to prepare the inputs of a network design problem. 
Please cite doi.org/10.1016/j.jclepro.2019.119247 for reference.
