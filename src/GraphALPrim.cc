#include "GraphALPrim.hh"
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

bool GraphALPrim::addEdge(const std::string &node1, const std::string &node2,
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

std::optional<GraphALPrim>
GraphALPrim::readFromTOML(const std::filesystem::path &filePath) {
  try {
    toml::table tbl = toml::parse_file(filePath.string());
    GraphALPrim graph;
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

bool GraphALPrim::writeToTOML(const fs::path &filePath) const {
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

std::optional<std::vector<GraphBase<GraphALPrim>::Edge>>
GraphALPrim::getEdges() const {
  std::vector<GraphBase<GraphALPrim>::Edge> edges;
  for (const auto &[node1, neighbors] : adjList) {
    for (const auto &[node2, weight] : neighbors) {
      if (node1 < node2) { // 避免重复边
        edges.push_back({node1, node2, weight});
      }
    }
  }
  return edges;
}

bool GraphALPrim::writeToDot(const fs::path &filePath) const {
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

bool GraphALPrim::checkConnected() const {
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

std::optional<GraphALPrim> GraphALPrim::getMinimumSpanningTree() const {
  if (adjList.empty()) {
    return std::nullopt;
  }

  // 存储每个节点的由算法选择的最新的最小边的权重
  std::unordered_map<std::string, int> minWeight;
  // 遍历每个键值对，初始化最小边权重为无穷大
  for (const auto &kv : adjList) {
    minWeight[kv.first] = std::numeric_limits<int>::max();
  }
  // 存储每个节点在最小生成树中的父节点
  std::unordered_map<std::string, std::string> parent;
  // 存储已加入最小生成树的节点
  std::unordered_set<std::string> inMST;

  // 优先队列的比较函数，按边权重从小到大排序
  auto cmp = [](const std::pair<std::string, int> &a,
                const std::pair<std::string, int> &b) {
    return a.second > b.second;
  };
  std::priority_queue<std::pair<std::string, int>,
                      std::vector<std::pair<std::string, int>>, decltype(cmp)>
      pq(cmp);

  // 初始化起始节点，设置其最小边权重为0
  auto it = adjList.begin();
  std::string start = it->first;
  minWeight[start] = 0;
  pq.push({start, 0});

  while (!pq.empty()) {
    // 弹出当前边权重最小的节点
    auto [nodeU, cost] = pq.top();
    pq.pop();

    // 如果节点已在最小生成树中，跳过
    if (inMST.count(nodeU)) {
      continue;
    }
    // 否则将节点加入最小生成树
    inMST.insert(nodeU);
    // 此时u已经是最小生成树的一部分，遍历它的邻节点
    // 遍历与u节点相连的节点，更新minEdge中它们与u的最小的权重
    for (const auto &[nodeV, weightV] : adjList.at(nodeU)) {
      // 确保v节点不在最小生成树中，且边权重小于当前记录的最小边权重
      if (!inMST.count(nodeV) && weightV < minWeight[nodeV]) {
        // 更新v节点到u的最小边权重
        minWeight[nodeV] = weightV;
        // 将u节点设置为v的父节点
        parent[nodeV] = nodeU;
        // 将与u节点相连的节点及其边权重加入优先队列
        pq.push({nodeV, weightV});
      }
    }
  }

  GraphALPrim mst;
  for (const auto &[nodeU, nodeV] : parent) {
    if (!mst.addEdge(nodeV, nodeU, minWeight[nodeU])) {
      std::cerr << "添加边失败" << std::endl;
      return std::nullopt;
    }
  }
  return mst;
}

bool GraphALPrim::exportImage(const std::filesystem::path &imageFile,
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
