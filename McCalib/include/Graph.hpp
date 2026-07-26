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

  /**
   * @brief Add a vertex if it does not already exist.
   *
   * @param vertex User-defined vertex id.
   */
  void addVertex(const int vertex);

  /**
   * @brief Add an undirected weighted edge between two vertices.
   *
   * Missing vertices are created automatically.
   *
   * @param v1 First vertex id.
   * @param v2 Second vertex id.
   * @param weight Edge weight.
   */
  void addEdge(const int v1, const int v2, const double weight);

  /**
   * @brief Return a copy of the underlying Boost graph object.
   *
   * @return Copy of the internal weighted undirected graph.
   */
  GraphUndirectedWeighted getGraph();

  /**
   * @brief Compute connected components.
   *
   * @return Vector of components, each component is a list of vertex ids.
   */
  std::vector<std::vector<int>> connectedComponents();

  /**
   * @brief Compute shortest path between two vertices.
   *
   * @param v1 Source vertex id.
   * @param v2 Destination vertex id.
   * @return Ordered list of vertex ids forming the shortest path.
   */
  std::vector<int> shortestPathBetween(const int v1, const int v2);

  /**
   * @brief Remove all vertices and edges.
   */
  void clearGraph();

private:
  // map vertex index (supplied by a user) and actual vertex in the graph
  std::unordered_map<int, Vertex> idx_to_vertex_;

  GraphUndirectedWeighted graph_;

  std::vector<int> getPath(const std::vector<Vertex> &p_map,
                           const Vertex &source, const Vertex &destination);
};
