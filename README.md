# Public-Transport-All-Pairs-Least-Cost-Path-Dijkstra
When modeling weighted nodes with #NetworkX, the common practice is to shift these weights to the incoming links so as to obviate the need for further classes and keep the code simple. But how about cases wherein nodes are weighted conditionally, say, fares paid and delays incurred with each transfer on the public transportation network? Clearly, at a given transfer point, not all passengers change line and the route they take to travel between different stations varies depending on factors other than just distance and speed—i.e. value of time, fare, and waiting times experienced with each transfer that are disregarded by NetworkX’s built-in tools. You see relying solely on NetworkX could generate misleading results since many unlikely transfers would be erroneously modeled when analyzing public transportation networks.

This is a modified Dijkstra algorithm that solves the All Pairs Least Cost Paths (APLCP) problem, factoring in costs (waiting time and fare) incurred conditionally at transfer nodes. 
A public transportation graph (Class TransitGraph) is defined which inherits from NetworkX’s Graph and holds additional methods and attributes that are necessary for calculating transfers and conditional costs.

This was first developed to prepare the inputs of a network design problem. 

Please cite doi.org/10.1016/j.jclepro.2019.119247 for reference.
