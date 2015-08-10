/*
 * Dijkstra.cpp
 *
 *  Created on: 12.07.2014
 *      Author: leon
 */

#include "Dijkstra.h"
#include <iostream>
#include <sstream>
#include <log4cxx/log4cxx.h>

using namespace std;
using namespace log4cxx;

const double max_weight = std::numeric_limits<double>::infinity();

int DijkstraAlgorithm::idCounter = 0;
LoggerPtr DijkstraAlgorithm::logger = Logger::getLogger("DijkstraAlgorithm");

list<int> DijkstraAlgorithm::getShortestPathTo(int vertex,
		const vector<int> &previous) {
	list<int> path;
	for (; vertex != -1; vertex = previous[vertex])
		path.push_front(vertex);
	return path;
}

void DijkstraAlgorithm::computePaths(int source,
		AdjacencyList &adjacency_list, vector<double> &min_distance,
		vector<int> &previous) {
	int n = adjacency_list.size();
	min_distance.clear();
	min_distance.resize(n, max_weight);
	min_distance[source] = 0;
	previous.clear();
	previous.resize(n, -1);
	set<pair<double, int> > vertex_queue;
	vertex_queue.insert(make_pair(min_distance[source], source));

	while (!vertex_queue.empty()) {
		double dist = vertex_queue.begin()->first;
		int u = vertex_queue.begin()->second;
		vertex_queue.erase(vertex_queue.begin());

		// Visit each edge exiting u
		const vector<Neighbor> &neighbors = adjacency_list[u];
		for (vector<Neighbor>::const_iterator neighbor_iter = neighbors.begin();
				neighbor_iter != neighbors.end(); neighbor_iter++) {
			int v = neighbor_iter->target;
			double weight = neighbor_iter->weight;
			double distance_through_u = dist + weight;
			if (distance_through_u < min_distance[v]) {
				vertex_queue.erase(make_pair(min_distance[v], v));

				min_distance[v] = distance_through_u;
				previous[v] = u;
				vertex_queue.insert(make_pair(min_distance[v], v));

			}

		}
	}
}

vector<string> DijkstraAlgorithm::getShortestPath(const string &source,
		const string &target) {
	LOG4CXX_DEBUG(logger, "calculating shortest path " << source << " -> " << target);
	if (mappingNameId.count(source) == 0) {
		LOG4CXX_ERROR(logger, "source \"" << source << "\" is not part of the graph!");
		return vector<string>();
	}
	if (mappingNameId.count(target) == 0) {
		LOG4CXX_ERROR(logger, "target \"" << target << "\" is not part of the graph!");
		return vector<string>();
	}
	int sourceId = mappingNameId[source];
	int targetId = mappingNameId[target];
	vector<double> min_distance;
	vector<int> previous;
	computePaths(sourceId, adjacency_list, min_distance, previous);
	list<int> pathIds = getShortestPathTo(targetId, previous);
	vector<string> path;
	stringstream ss;
	ss << "path: ";
	for (list<int>::iterator it = pathIds.begin(); it != pathIds.end(); ++it) {
		ss << mappingIdName[*it] << " ";
		path.push_back(mappingIdName[*it]);
	}
	ss << endl;
	LOG4CXX_DEBUG(logger, ss.str());
	return path;
}

void DijkstraAlgorithm::addEdge(Edge e) {

	if (mappingNameId.count(e.source) == 0) {
		mappingNameId[e.source] = idCounter;
		mappingIdName[idCounter] = e.source;
		idCounter++;
	}
	if (mappingNameId.count(e.target) == 0) {
		mappingNameId[e.target] = idCounter;
		mappingIdName[idCounter] = e.target;
		idCounter++;
	}

	adjacency_list[mappingNameId[e.source]].push_back(
			Neighbor(mappingNameId[e.target], e.weight));
	adjacency_list[mappingNameId[e.target]].push_back(
			Neighbor(mappingNameId[e.source], e.weight));
}
