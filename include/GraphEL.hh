#ifndef GRAPHEL
#define GRAPHEL
#include "GraphBase.hh"
#include <filesystem>
#include <memory>
#include <string>
#include <vector>

/**
 * @brief 用于Kruskal算法的边列表图类
 * @author Alkaid
 */
class GraphEL : public GraphBase<GraphEL> {
public:
  GraphEL() = default;

  static std::optional<GraphEL> readFromTOML(const std::filesystem::path &filePath);

  [[nodiscard]] bool writeToTOML(const std::filesystem::path &filePath) const;

  [[nodiscard]] bool writeToDot(const std::filesystem::path &filePath) const;

  [[nodiscard]] bool exportImage(const std::filesystem::path &imageFile, const std::string &format = "png") const;

  [[nodiscard]] bool addEdge(const std::string &node1, const std::string &node2,
                             int weight) override;

  std::optional<std::vector<Edge>> getEdges() const override;

  std::optional<GraphEL> getMinimumSpanningTree() const override;

private:
  struct Edge {
    std::string node1;
    std::string node2;
    int weight;
    std::shared_ptr<Edge> next;

    bool operator<(const Edge &other) const { return weight < other.weight; }
  };

  std::vector<Edge> edges;
};
#endif