#include "Graph.hpp"
#include "logger.h"

Graph::GraphUndirectedWeighted Graph::getGraph() { return graph_; }

/**
 * @brief Add edge to the graph
 *
 * It creates a new vertex only when no vertex with the same name (i.e. v1 or
 * v2) is present in the graph.
 *
 * @param v1 vertex one
 * @param v2 vertex two
 * @param weight edge weight
 */
void Graph::addEdge(const int v1, const int v2, const double weight) {
  // create a new vertex only when no vertex with the same name (i.e.
  // boost::vertex_name_t) is available, reuse vertices otherwise
  Vertex vertex1 = (idx_to_vertex_.find(v1) != idx_to_vertex_.end())
                       ? idx_to_vertex_[v1]
                       : boost::add_vertex(std::to_string(v1), graph_);
  Vertex vertex2 = (idx_to_vertex_.find(v2) != idx_to_vertex_.end())
                       ? idx_to_vertex_[v2]
                       : boost::add_vertex(std::to_string(v2), graph_);
  idx_to_vertex_[v1] = vertex1;
  idx_to_vertex_[v2] = vertex2;

  boost::add_edge(vertex1, vertex2, EdgeWeightProperty(weight), graph_);
}

/**
 * @brief Add vertex to the graph
 *
 * It creates a new vertex only when no vertex with the same name is present in
 * the graph.
 *
 * @param vertex vertex to be created
 */
void Graph::addVertex(const int vertex) {
  // create a new vertex only when no vertex with the same name (i.e.
  // boost::vertex_name_t) is available
  if (idx_to_vertex_.find(vertex) != idx_to_vertex_.end()) {
    LOG_WARNING << "Vertex {" << vertex
                << "} is not added because it exists in the graph";
  } else {
    Vertex created_vertex = boost::add_vertex(std::to_string(vertex), graph_);
    idx_to_vertex_[vertex] = created_vertex;
  }
}

/**
 * @brief Clear graph
 *
 */
void Graph::clearGraph() {
  graph_.clear();
  idx_to_vertex_.clear();
}

/**
 * @brief Get al connected components
 *
 * @return all connected components and a list of vertices belonging to each
 * component
 */
std::vector<std::vector<int>> Graph::connectedComponents() {
  std::vector<std::vector<int>> all_components;

  int num_vertices = boost::num_vertices(graph_);

  if (num_vertices == 0)
    return all_components;

  // get connected components with boost
  std::vector<int> component(num_vertices);
  int num_components = boost::connected_components(graph_, &component[0]);

  for (int component_idx = 0; component_idx < num_components; ++component_idx) {

    std::vector<int> cur_component;
    for (int vert_idx = 0; vert_idx < num_vertices; ++vert_idx) {
      if (component[vert_idx] == component_idx) {
        // get vertex name which was supplied by a user
        std::string vert_name = boost::get(boost::vertex_name, graph_,
                                           boost::vertex(vert_idx, graph_));

        cur_component.push_back(std::stoi(vert_name));
      }
    }
    all_components.push_back(cur_component);
  }

  return all_components;
}

/**
 * @brief Get a shortest path between two vertices
 *
 * Dijkstra Shortest Path algorithm on connected graph.
 * Inspired by https://siavashk.github.io/2019/01/23/boost-graph-library/
 *
 * @return vector of vertices belonging to the path
 */
std::vector<int> Graph::shortestPathBetween(const int v1, const int v2) {
  std::vector<int> vert_in_path;

  if (idx_to_vertex_.find(v1) == idx_to_vertex_.end() ||
      idx_to_vertex_.find(v2) == idx_to_vertex_.end()) {
    LOG_ERROR << "Vertices with names {" << v1 << "," << v2
              << "} are not in the graph";
    return vert_in_path;
  }

  std::vector<VertexProperties> p(boost::num_vertices(graph_));
  std::vector<int> d(boost::num_vertices(graph_));

  const int num_vertices = boost::num_vertices(graph_);
  std::vector<int> distances(num_vertices);
  std::vector<Vertex> p_map(num_vertices);

  auto distance_map =
      boost::predecessor_map(
          boost::make_iterator_property_map(
              p_map.begin(), boost::get(boost::vertex_index, graph_)))
          .distance_map(boost::make_iterator_property_map(
              distances.begin(), boost::get(boost::vertex_index, graph_)));

  boost::dijkstra_shortest_paths(graph_, idx_to_vertex_[v1], distance_map);

  vert_in_path = Graph::getPath(p_map, idx_to_vertex_[v1], idx_to_vertex_[v2]);

  return vert_in_path;
}

/**
 * @brief Utility function to build a path after Dijkstra run
 *
 * @return vector of vertices belonging to the path
 */
std::vector<int> Graph::getPath(const std::vector<Vertex> &p_map,
                                const Vertex &source,
                                const Vertex &destination) {
  std::vector<int> path;
  Vertex current = destination;
  while (current != source) {
    path.push_back(std::stoi(boost::get(boost::vertex_name, graph_, current)));
    current = p_map[current];
  }
  path.push_back(std::stoi(boost::get(boost::vertex_name, graph_, source)));

  std::reverse(path.begin(), path.end()); // return in v1->v2 order
  return path;
}