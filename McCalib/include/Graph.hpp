#include <unordered_map>

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/connected_components.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/graph_traits.hpp>

/**
 * @class Graph
 *
 * @brief Undirected weighted graph with connected components and shortest path
 * functionalities.
 *
 * Relies on boost. Reference:
 * https://www.boost.org/doc/libs/1_75_0/libs/graph/example/dijkstra-example.cpp
 */
class Graph final {
public:
  // edge weight
  typedef boost::property<boost::edge_weight_t, double> EdgeWeightProperty;
  // vertex descriptor
  typedef boost::property<boost::vertex_name_t, std::string> VertexProperties;

  // graph
  typedef boost::adjacency_list<boost::listS, boost::vecS, boost::undirectedS,
                                VertexProperties, EdgeWeightProperty>
      GraphUndirectedWeighted;

  typedef
      typename boost::graph_traits<GraphUndirectedWeighted>::vertex_descriptor
          Vertex;

  void addVertex(int vertex);
  void addEdge(int v1, int v2, double weight);
  GraphUndirectedWeighted getGraph();
  std::vector<std::vector<int>> connectedComponents();
  std::vector<int> shortestPathBetween(int v1, int v2);
  void clearGraph();

private:
  // map vertex index (supplied by a user) and actual vertex in the graph
  std::unordered_map<int, Vertex> idx_to_vertex_;

  GraphUndirectedWeighted graph_;

  std::vector<int> getPath(const std::vector<Vertex> &p_map,
                           const Vertex &source, const Vertex &destination);
};