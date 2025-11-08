#include "GraphAL.hh"
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <functional>
#include <limits>
#include <optional>
#include <queue>
#include <toml++/toml.hpp>
#include <unordered_set>

namespace fs = std::filesystem;

bool GraphAL::addEdge(const std::string &node1, const std::string &node2,
                      int weight) {
  std::unordered_map<std::string, int> neighbors = adjList[node1];
  if (neighbors.find(node2) != neighbors.end()) {
    std::cerr << node1 << " 和 " << node2 << " 之间的边已存在" << std::endl;
    return false;
  }
  adjList[node1][node2] = weight;
  adjList[node2][node1] = weight; // 无向图
  return true;
}

std::optional<GraphAL>
GraphAL::readFromTOML(const std::filesystem::path &filePath) {
  try {
    toml::table tbl = toml::parse_file(filePath.string());
    GraphAL graph;
    for (const auto &item : tbl) {
      const std::string &node1 = item.first.data();
      const toml::table &edges = *item.second.as_table();
      for (const auto &edge : edges) {
        const std::string &node2 = edge.first.data();
        int64_t temp = edge.second.as_integer()->get();
        int weight = static_cast<int>(temp);
        if (!graph.addEdge(node1, node2, weight)) {
          std::cerr << "添加边 " << node1 << " - " << node2 << " 失败"
                    << std::endl;
          return std::nullopt;
        }
      }
    }
    if (!graph.checkConnected()) {
      std::cerr << "图不连通" << std::endl;
      return std::nullopt;
    }
    return graph;
  } catch (const toml::parse_error &err) {
    std::cerr << "解析 TOML 文件时出错: " << err.description() << std::endl;
    return std::nullopt;
  }
}

bool GraphAL::writeToTOML(const fs::path &filePath) const {
  toml::table tbl;
  for (const auto &pair : adjList) {
    const std::string &node1 = pair.first;
    const auto &neighbors = pair.second;
    toml::table edgeTable;
    for (const auto &neighborPair : neighbors) {
      const std::string &node2 = neighborPair.first;
      if (node1 < node2) { // 只写入 node1 < node2 的边，避免重复
        int weight = neighborPair.second;
        edgeTable.insert(node2, weight);
      }
    }
    if (!edgeTable.empty()) {
      tbl.insert(node1, edgeTable);
    }
  }
  try {
    std::ofstream out(filePath);
    if (!out) {
      std::cerr << "无法打开文件写入: " << filePath << std::endl;
      return false;
    }
    out << tbl;
    out.close();
    return true;
  } catch (const std::exception &e) {
    std::cerr << "写入 TOML 文件时出错: " << e.what() << std::endl;
    return false;
  }
}

std::optional<std::vector<GraphBase<GraphAL>::Edge>> GraphAL::getEdges() const {
  std::vector<GraphBase<GraphAL>::Edge> edges;
  for (const auto &[node1, neighbors] : adjList) {
    for (const auto &[node2, weight] : neighbors) {
      if (node1 < node2) { // 避免重复边
        edges.push_back({node1, node2, weight, nullptr});
      }
    }
  }
  return edges;
}

bool GraphAL::writeToDot(const fs::path &filePath) const {
  try {
    std::ofstream out(filePath);
    if (!out) {
      std::cerr << "无法打开文件写入: " << filePath << std::endl;
      return false;
    }
    out << "graph G {" << std::endl;
    for (const auto &pair : adjList) {
      const std::string &node1 = pair.first;
      const auto &neighbors = pair.second;
      for (const auto &neighborPair : neighbors) {
        const std::string &node2 = neighborPair.first;
        if (node1 < node2) { // 只写入 node1 < node2 的边
          int weight = neighborPair.second;
          out << "    " << node1 << " -- " << node2 << " [label=\"" << weight
              << "\"];" << std::endl;
        }
      }
    }
    out << "}" << std::endl;
    out.close();
    return true;
  } catch (const std::exception &e) {
    std::cerr << "写入 DOT 文件时出错: " << e.what() << std::endl;
    return false;
  }
}

bool GraphAL::checkConnected() const {
  if (adjList.empty())
    return true;
  std::unordered_set<std::string> visited;
  std::function<void(const std::string &)> dfs = [&](const std::string &node) {
    visited.insert(node);
    for (const auto &neighbor : adjList.at(node)) {
      if (visited.find(neighbor.first) == visited.end()) {
        dfs(neighbor.first);
      }
    }
  };
  auto it = adjList.begin();
  dfs(it->first);
  return visited.size() == adjList.size();
}

std::optional<GraphAL> GraphAL::getMinimumSpanningTree() const {
  if (adjList.empty())
    return GraphAL();

  std::unordered_map<std::string, int> minEdge;
  for (const auto &p : adjList) {
    minEdge[p.first] = std::numeric_limits<int>::max();
  }
  std::unordered_map<std::string, std::string> parent;
  std::unordered_set<std::string> inMST;

  auto cmp = [](const std::pair<std::string, int> &a,
                const std::pair<std::string, int> &b) {
    return a.second > b.second;
  };
  std::priority_queue<std::pair<std::string, int>,
                      std::vector<std::pair<std::string, int>>, decltype(cmp)>
      pq(cmp);

  auto it = adjList.begin();
  std::string start = it->first;
  minEdge[start] = 0;
  pq.push({start, 0});

  while (!pq.empty()) {
    auto [u, cost] = pq.top();
    pq.pop();
    if (inMST.count(u)) {
      continue;
    }
    inMST.insert(u);
    for (const auto &[v, w] : adjList.at(u)) {
      if (!inMST.count(v) && w < minEdge[v]) {
        minEdge[v] = w;
        parent[v] = u;
        pq.push({v, w});
      }
    }
  }

  GraphAL mst;
  for (const auto &[v, p] : parent) {
    if (!mst.addEdge(p, v, minEdge[v])) {
      std::cerr << "添加边失败" << std::endl;
      return std::nullopt;
    }
  }
  return mst;
}

bool GraphAL::exportImage(const std::filesystem::path &imageFile,
                          const std::string &format) const {
  std::filesystem::path dotFile = imageFile;
  dotFile.replace_extension(".dot");
  if (!writeToDot(dotFile)) {
    return false;
  }
  std::string command =
      "dot -T" + format + " " + dotFile.string() + " -o " + imageFile.string();
  int result = std::system(command.c_str());
  if (result != 0) {
    std::cerr << "执行 dot 命令失败: " << command << std::endl;
    return false;
  }
  return true;
}
