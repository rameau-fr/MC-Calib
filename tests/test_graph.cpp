#include <boost/test/unit_test.hpp>

#include <Graph.hpp>

BOOST_AUTO_TEST_SUITE(CheckGraph)

BOOST_AUTO_TEST_CASE(CheckSingleVertexAdding) {
  Graph test_graph;

  test_graph.addVertex(1);

  int num_vertices = boost::num_vertices(test_graph.getGraph());
  int num_edges = boost::num_edges(test_graph.getGraph());

  BOOST_REQUIRE_EQUAL(num_vertices, 1);
  BOOST_REQUIRE_EQUAL(num_edges, 0);
}

BOOST_AUTO_TEST_CASE(CheckTwoVertexAdding) {
  Graph test_graph;

  test_graph.addVertex(1);
  test_graph.addVertex(20000);

  int num_vertices = boost::num_vertices(test_graph.getGraph());
  int num_edges = boost::num_edges(test_graph.getGraph());

  BOOST_REQUIRE_EQUAL(num_vertices, 2);
  BOOST_REQUIRE_EQUAL(num_edges, 0);
}

BOOST_AUTO_TEST_CASE(CheckDuplicateVertexAdding) {
  Graph test_graph;

  test_graph.addVertex(1);
  test_graph.addVertex(1);

  int num_vertices = boost::num_vertices(test_graph.getGraph());
  int num_edges = boost::num_edges(test_graph.getGraph());

  BOOST_REQUIRE_EQUAL(num_vertices, 1);
  BOOST_REQUIRE_EQUAL(num_edges, 0);
}

BOOST_AUTO_TEST_CASE(CheckSingleEdgeAdding) {
  Graph test_graph;

  test_graph.addEdge(1, 2, 10);

  int num_vertices = boost::num_vertices(test_graph.getGraph());
  int num_edges = boost::num_edges(test_graph.getGraph());

  BOOST_REQUIRE_EQUAL(num_vertices, 2);
  BOOST_REQUIRE_EQUAL(num_edges, 1);
}

BOOST_AUTO_TEST_CASE(CheckSingleEdgeAddingBetweenVertices) {
  Graph test_graph;

  test_graph.addVertex(1);
  test_graph.addVertex(2);
  test_graph.addEdge(1, 2, 10);

  int num_vertices = boost::num_vertices(test_graph.getGraph());
  int num_edges = boost::num_edges(test_graph.getGraph());

  BOOST_REQUIRE_EQUAL(num_vertices, 2);
  BOOST_REQUIRE_EQUAL(num_edges, 1);
}

BOOST_AUTO_TEST_CASE(CheckTwoEdgeConnectedAdding) {
  Graph test_graph;

  test_graph.addEdge(1, 2, 10);
  test_graph.addEdge(2, 3, 1);

  int num_vertices = boost::num_vertices(test_graph.getGraph());
  int num_edges = boost::num_edges(test_graph.getGraph());

  BOOST_REQUIRE_EQUAL(num_vertices, 3);
  BOOST_REQUIRE_EQUAL(num_edges, 2);
}

BOOST_AUTO_TEST_CASE(CheckTwoEdgeDisconnectedAdding) {
  Graph test_graph;

  test_graph.addEdge(1, 2, 10);
  test_graph.addEdge(3, 4, 1);

  int num_vertices = boost::num_vertices(test_graph.getGraph());
  int num_edges = boost::num_edges(test_graph.getGraph());

  BOOST_REQUIRE_EQUAL(num_vertices, 4);
  BOOST_REQUIRE_EQUAL(num_edges, 2);
}

BOOST_AUTO_TEST_CASE(CheckThreeEdgeConnectedAdding) {
  Graph test_graph;

  test_graph.addEdge(1, 2, 10);
  test_graph.addEdge(2, 3, 1);
  test_graph.addEdge(3, 1, 3);

  int num_vertices = boost::num_vertices(test_graph.getGraph());
  int num_edges = boost::num_edges(test_graph.getGraph());

  BOOST_REQUIRE_EQUAL(num_vertices, 3);
  BOOST_REQUIRE_EQUAL(num_edges, 3);
}

BOOST_AUTO_TEST_CASE(CheckThreeEdgeDisconnectedAdding) {
  Graph test_graph;

  test_graph.addEdge(1, 2, 10);
  test_graph.addEdge(3, 4, 1);
  test_graph.addEdge(5, 6, 2);

  int num_vertices = boost::num_vertices(test_graph.getGraph());
  int num_edges = boost::num_edges(test_graph.getGraph());

  BOOST_REQUIRE_EQUAL(num_vertices, 6);
  BOOST_REQUIRE_EQUAL(num_edges, 3);
}

BOOST_AUTO_TEST_CASE(CheckConnectedComponentsEmpty) {
  Graph test_graph;

  std::vector<std::vector<int>> connected_components =
      test_graph.connectedComponents();
  int num_cc = connected_components.size();

  BOOST_REQUIRE_EQUAL(num_cc, 0);
}

BOOST_AUTO_TEST_CASE(CheckConnectedComponentsDisconnected) {
  Graph test_graph;

  test_graph.addEdge(1, 10, 3);
  test_graph.addEdge(5, 16, 1);
  test_graph.addEdge(25, 34, 2);

  std::vector<std::vector<int>> connected_components =
      test_graph.connectedComponents();
  int num_cc = connected_components.size();

  BOOST_REQUIRE_EQUAL(num_cc, 3);

  std::vector<int> cc1 = {1, 10};
  std::vector<int> cc2 = {5, 16};
  std::vector<int> cc3 = {25, 34};

  BOOST_REQUIRE_EQUAL_COLLECTIONS(cc1.begin(), cc1.end(),
                                  connected_components[0].begin(),
                                  connected_components[0].end());

  BOOST_REQUIRE_EQUAL_COLLECTIONS(cc2.begin(), cc2.end(),
                                  connected_components[1].begin(),
                                  connected_components[1].end());

  BOOST_REQUIRE_EQUAL_COLLECTIONS(cc3.begin(), cc3.end(),
                                  connected_components[2].begin(),
                                  connected_components[2].end());
}

BOOST_AUTO_TEST_CASE(CheckConnectedComponentsDisconnectedNoEdges) {
  Graph test_graph;

  test_graph.addVertex(1);
  test_graph.addVertex(5);
  test_graph.addVertex(25);

  std::vector<std::vector<int>> connected_components =
      test_graph.connectedComponents();
  int num_cc = connected_components.size();

  BOOST_REQUIRE_EQUAL(num_cc, 3);

  std::vector<int> cc1 = {1};
  std::vector<int> cc2 = {5};
  std::vector<int> cc3 = {25};

  BOOST_REQUIRE_EQUAL_COLLECTIONS(cc1.begin(), cc1.end(),
                                  connected_components[0].begin(),
                                  connected_components[0].end());

  BOOST_REQUIRE_EQUAL_COLLECTIONS(cc2.begin(), cc2.end(),
                                  connected_components[1].begin(),
                                  connected_components[1].end());

  BOOST_REQUIRE_EQUAL_COLLECTIONS(cc3.begin(), cc3.end(),
                                  connected_components[2].begin(),
                                  connected_components[2].end());
}

BOOST_AUTO_TEST_CASE(CheckConnectedComponentsDisconnectedWithSingleEdge) {
  Graph test_graph;

  test_graph.addVertex(1);
  test_graph.addVertex(5);
  test_graph.addVertex(25);

  test_graph.addEdge(1, 10, 250);

  std::vector<std::vector<int>> connected_components =
      test_graph.connectedComponents();
  int num_cc = connected_components.size();

  BOOST_REQUIRE_EQUAL(num_cc, 3);

  std::vector<int> cc1 = {1, 10};
  std::vector<int> cc2 = {5};
  std::vector<int> cc3 = {25};

  BOOST_REQUIRE_EQUAL_COLLECTIONS(cc1.begin(), cc1.end(),
                                  connected_components[0].begin(),
                                  connected_components[0].end());

  BOOST_REQUIRE_EQUAL_COLLECTIONS(cc2.begin(), cc2.end(),
                                  connected_components[1].begin(),
                                  connected_components[1].end());

  BOOST_REQUIRE_EQUAL_COLLECTIONS(cc3.begin(), cc3.end(),
                                  connected_components[2].begin(),
                                  connected_components[2].end());
}

BOOST_AUTO_TEST_CASE(CheckConnectedComponentsConnected) {
  Graph test_graph;

  test_graph.addEdge(1, 10, 3);
  test_graph.addEdge(10, 16, 1);
  test_graph.addEdge(16, 34, 2);

  std::vector<std::vector<int>> connected_components =
      test_graph.connectedComponents();
  int num_cc = connected_components.size();

  BOOST_REQUIRE_EQUAL(num_cc, 1);

  std::vector<int> cc1 = {1, 10, 16, 34};

  BOOST_REQUIRE_EQUAL_COLLECTIONS(cc1.begin(), cc1.end(),
                                  connected_components[0].begin(),
                                  connected_components[0].end());
}

BOOST_AUTO_TEST_CASE(CheckConnectedComponentsMixed) {
  Graph test_graph;

  // cc1
  test_graph.addEdge(1, 10, 3);
  test_graph.addEdge(10, 16, 1);
  test_graph.addEdge(16, 34, 2);

  // cc2
  test_graph.addEdge(150, 3, 15);
  test_graph.addEdge(3, 180, 148);

  // cc3 -> cc1
  test_graph.addEdge(259, 34, 10050);
  test_graph.addEdge(259, 1000, 1);

  std::vector<std::vector<int>> connected_components =
      test_graph.connectedComponents();
  int num_cc = connected_components.size();

  BOOST_REQUIRE_EQUAL(num_cc, 2);

  std::vector<int> cc1 = {1, 10, 16, 34, 259, 1000};
  std::vector<int> cc2 = {150, 3, 180};

  BOOST_REQUIRE_EQUAL_COLLECTIONS(cc1.begin(), cc1.end(),
                                  connected_components[0].begin(),
                                  connected_components[0].end());

  BOOST_REQUIRE_EQUAL_COLLECTIONS(cc2.begin(), cc2.end(),
                                  connected_components[1].begin(),
                                  connected_components[1].end());
}

BOOST_AUTO_TEST_CASE(CheckShortestPathSingleEdge) {
  Graph test_graph;

  test_graph.addEdge(5, 2, 10);

  std::vector<int> answer = {5, 2};
  std::vector<int> path = test_graph.shortestPathBetween(5, 2);

  BOOST_REQUIRE_EQUAL_COLLECTIONS(path.begin(), path.end(), answer.begin(),
                                  answer.end());
}

BOOST_AUTO_TEST_CASE(CheckShortestPathNonExistentEdge) {
  Graph test_graph;

  test_graph.addEdge(5, 2, 10);

  std::vector<int> answer = {};
  std::vector<int> path = test_graph.shortestPathBetween(10, 2);

  BOOST_REQUIRE_EQUAL_COLLECTIONS(path.begin(), path.end(), answer.begin(),
                                  answer.end());
}

BOOST_AUTO_TEST_CASE(CheckShortestPathSingleCC) {
  Graph test_graph;

  test_graph.addEdge(0, 1, 4);
  test_graph.addEdge(0, 7, 8);
  test_graph.addEdge(1, 2, 8);
  test_graph.addEdge(2, 3, 7);
  test_graph.addEdge(3, 4, 9);
  test_graph.addEdge(4, 5, 10);
  test_graph.addEdge(5, 6, 2);
  test_graph.addEdge(6, 7, 1);
  test_graph.addEdge(7, 8, 7);
  test_graph.addEdge(6, 8, 6);
  test_graph.addEdge(8, 2, 2);

  std::vector<int> answer = {0, 7, 6, 5, 4};
  std::vector<int> path = test_graph.shortestPathBetween(0, 4);

  BOOST_REQUIRE_EQUAL(path.size(), answer.size());
  BOOST_REQUIRE_EQUAL_COLLECTIONS(path.begin(), path.end(), answer.begin(),
                                  answer.end());
}

BOOST_AUTO_TEST_SUITE_END()