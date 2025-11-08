#include "GraphEL.hh"
#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <unordered_map>
#include <toml++/toml.hpp>

bool GraphEL::addEdge(const std::string &node1, const std::string &node2,
                      int weight) {
  edges.push_back({node1, node2, weight, nullptr});
  return true;
}

std::optional<std::vector<GraphBase<GraphEL>::Edge>> GraphEL::getEdges() const {
  std::vector<GraphBase<GraphEL>::Edge> result;
  for (const auto& edge : edges) {
    result.push_back({edge.node1, edge.node2, edge.weight, nullptr});
  }
  return result;
}

std::optional<GraphEL> GraphEL::getMinimumSpanningTree() const {
  // Kruskal 算法
  auto edgesOpt = getEdges();
  if (!edgesOpt) return std::nullopt;
  auto allEdges = *edgesOpt;

  // 排序边
  std::sort(allEdges.begin(), allEdges.end());

  // Union-Find
  std::unordered_map<std::string, std::string> parent;
  for (const auto& edge : allEdges) {
    parent[edge.node1] = edge.node1;
    parent[edge.node2] = edge.node2;
  }

  auto find = [&](auto& self, const std::string& x) -> std::string& {
    if (parent[x] != x) parent[x] = self(self, parent[x]);
    return parent[x];
  };

  GraphEL mst;
  for (const auto& edge : allEdges) {
    std::string root1 = find(find, edge.node1);
    std::string root2 = find(find, edge.node2);
    if (root1 != root2) {
      parent[root1] = root2;
      std::string n1 = edge.node1 < edge.node2 ? edge.node1 : edge.node2;
      std::string n2 = edge.node1 < edge.node2 ? edge.node2 : edge.node1;
      if (!mst.addEdge(n1, n2, edge.weight)) {
        return std::nullopt;
      }
    }
  }
  return mst;
}

bool GraphEL::writeToTOML(const std::filesystem::path &filePath) const {
  auto edgesOpt = getEdges();
  if (!edgesOpt) return false;
  auto edges = *edgesOpt;

  std::unordered_map<std::string, toml::table> nodeTables;
  for (const auto& edge : edges) {
    if (edge.node1 < edge.node2) {
      nodeTables[edge.node1].insert(edge.node2, edge.weight);
    }
  }

  toml::table tbl;
  for (const auto& [node, edgeTable] : nodeTables) {
    tbl.insert(node, edgeTable);
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

bool GraphEL::writeToDot(const std::filesystem::path &filePath) const {
  auto edgesOpt = getEdges();
  if (!edgesOpt) return false;
  auto edges = *edgesOpt;

  try {
    std::ofstream out(filePath);
    if (!out) {
      std::cerr << "无法打开文件写入: " << filePath << std::endl;
      return false;
    }
    out << "graph G {" << std::endl;
    for (const auto& edge : edges) {
      if (edge.node1 < edge.node2) {
        out << "    " << edge.node1 << " -- " << edge.node2 << " [label=\"" << edge.weight << "\"];" << std::endl;
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

bool GraphEL::exportImage(const std::filesystem::path &imageFile, const std::string &format) const {
    std::filesystem::path dotFile = imageFile;
    dotFile.replace_extension(".dot");
    if (!writeToDot(dotFile)) {
        return false;
    }
    std::string command = "dot -T" + format + " " + dotFile.string() + " -o " + imageFile.string();
    int result = std::system(command.c_str());
    if (result != 0) {
        std::cerr << "执行 dot 命令失败: " << command << std::endl;
        return false;
    }
    return true;
}

std::optional<GraphEL> GraphEL::readFromTOML(const std::filesystem::path &filePath) {
  try {
    toml::table tbl = toml::parse_file(filePath.string());
    GraphEL graph;
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
    return graph;
  } catch (const toml::parse_error &err) {
    std::cerr << "解析 TOML 文件时出错: " << err.description() << std::endl;
    return std::nullopt;
  }
}