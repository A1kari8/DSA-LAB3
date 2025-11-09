#include "GraphALPrim.hh"
#include "GraphELKruskal.hh"
#include <filesystem>
#include <optional>
#include <toml++/toml.hpp>

namespace fs = std::filesystem;

int main() {
  fs::path exePath = fs::read_symlink("/proc/self/exe");
  fs::path workingDir = exePath.parent_path().parent_path();

  auto graphOpt = GraphALPrim::readFromTOML(workingDir / "mst_test.toml");
  if (!graphOpt) {
    std::cerr << "读取图失败" << std::endl;
    return 1;
  }
  const GraphALPrim &graph = *graphOpt;
  std::cout << graph << std::endl;
  fs::path originalImagePath = workingDir / "original_graph.png";
  if (!graph.exportImage(originalImagePath, "png")) {
    std::cerr << "导出原始图图片失败" << std::endl;
    return 1;
  }
  auto mstOptPrim = graph.getMinimumSpanningTree();
  if (!mstOptPrim) {
    std::cerr << "计算Prim最小生成树失败" << std::endl;
    return 1;
  }
  if (!mstOptPrim.value().writeToTOML(workingDir / "output_graph_prim.toml")) {
    std::cerr << "写入图失败" << std::endl;
    return 1;
  }
  fs::path mstImagePath = workingDir / "output_graph_prim.png";
  if (!mstOptPrim.value().exportImage(mstImagePath, "png")) {
    std::cerr << "导出图片失败" << std::endl;
    return 1;
  }



  
  auto graphELOpt = GraphELKruskal::readFromTOML(workingDir / "mst_test.toml");
  if (!graphELOpt) {
    std::cerr << "读取EL图失败" << std::endl;
    return 1;
  }
  const GraphELKruskal &graphEL = *graphELOpt;
  auto mstOptKruskal = graphEL.getMinimumSpanningTree();
  if (!mstOptKruskal) {
    std::cerr << "计算Kruskal最小生成树失败" << std::endl;
    return 1;
  }
  for (const auto &edge : mstOptKruskal.value().getEdges().value()) {
    std::cout << edge.node1 << " - " << edge.node2 << " : " << edge.weight
              << std::endl;
  }
  if (!mstOptKruskal.value().writeToTOML(workingDir /
                                         "output_graph_kruskal.toml")) {
    std::cerr << "写入Kruskal图失败" << std::endl;
    return 1;
  }
  if (!mstOptKruskal.value().exportImage(
          workingDir / "output_graph_kruskal.png", "png")) {
    std::cerr << "导出Kruskal图片失败" << std::endl;
    return 1;
  }
  return 0;
}