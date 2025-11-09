#ifndef GRAPHAL
#define GRAPHAL

#include "GraphBase.hh"
#include <filesystem>
#include <iostream>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace fs = std::filesystem;

/**
 * @brief 用于Prim算法的邻接表图类
 * @author Alkaid
 */
class GraphALPrim : public GraphBase<GraphALPrim> {
public:
  GraphALPrim() = default;

  static std::optional<GraphALPrim> readFromTOML(const fs::path &filePath);

  [[nodiscard]] bool writeToTOML(const fs::path &filePath) const;

  [[nodiscard]] bool writeToDot(const fs::path &filePath) const;

  [[nodiscard]] bool addEdge(const std::string &node1, const std::string &node2,
                             int weight) override;

  [[nodiscard]] std::optional<std::vector<Edge>> getEdges() const override;

  std::optional<GraphALPrim> getMinimumSpanningTree() const override;

  [[nodiscard]] bool exportImage(const fs::path &imageFile,
                                 const std::string &format = "png") const;

  friend std::ostream &operator<<(std::ostream &os, const GraphALPrim &graph) {
    for (const auto &pair : graph.adjList) {
      const std::string &node = pair.first;
      const auto &neighbors = pair.second;
      os << node << " -> ";
      for (const auto &neighborPair : neighbors) {
        os << "(" << neighborPair.first << ", " << neighborPair.second << ") ";
      }
      os << std::endl;
    }
    return os;
  }
  [[nodiscard]] bool checkConnected() const;

private:
  std::unordered_map<std::string, std::unordered_map<std::string, int>> adjList;
};

#endif