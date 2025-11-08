#ifndef GRAPHBASE
#define GRAPHBASE

#include <memory>
#include <optional>
#include <string>
#include <vector>

template <typename T> class GraphBase {
public:
  struct Edge {
    std::string node1;
    std::string node2;
    int weight;
    std::shared_ptr<Edge> next;

    bool operator<(const Edge &other) const { return weight < other.weight; }
  };

  virtual ~GraphBase() = default;
  [[nodiscard]] virtual bool addEdge(const std::string &node1,
                                     const std::string &node2, int weight) = 0;

  virtual std::optional<std::vector<Edge>> getEdges() const = 0;

  virtual std::optional<T> getMinimumSpanningTree() const = 0;
};

#endif