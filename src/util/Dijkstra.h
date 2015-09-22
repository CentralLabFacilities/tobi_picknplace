#pragma once

#include <iostream>
#include <vector>
#include <string>
#include <list>
#include <map>

#include <limits>

#include <set>
#include <utility>
#include <algorithm>
#include <iterator>

struct Neighbor {
	int target;
	double weight;
	Neighbor(int arg_target, double arg_weight) :
			target(arg_target), weight(arg_weight) {
	}
};

struct Edge {
	std::string source;
	std::string target;
	double weight;
	Edge(const std::string &arg_source, const std::string &arg_target, double arg_weight) :
		source(arg_source), target(arg_target), weight(arg_weight) {
	}
};

typedef std::map<int, std::vector<Neighbor> > AdjacencyList;

class DijkstraAlgorithm {
public:

	std::vector<std::string> getShortestPath(const std::string &source, const std::string &target);

	void addEdge(Edge e);

private:
	void computePaths(int source, AdjacencyList &adjacency_list,
			std::vector<double> &min_distance, std::vector<int> &previous);

	std::list<int> getShortestPathTo(int vertex,
			const std::vector<int> &previous);

	AdjacencyList adjacency_list;

	static int idCounter;
	std::map<std::string, int> mappingNameId;
	std::map<int, std::string> mappingIdName;
};
