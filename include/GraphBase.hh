#ifndef GRAPHBASE
#define GRAPHBASE

#include <optional>
#include <string>
#include <vector>

template <typename T> class GraphBase {
public:
  /**
   * @brief 边结构体
   * @author Alkaid
   */
  struct Edge {
    std::string node1;
    std::string node2;
    int weight;

    bool operator<(const Edge &other) const { return weight < other.weight; }
  };

  virtual ~GraphBase() = default;

  /**
   * @brief 添加一条边
   * @param[in] node1         边的起始节点
   * @param[in] node2         边的结束节点
   * @param[in] weight        边的权重
   * @return true 成功
   * @return false 失败
   * @author Alkaid
   */
  [[nodiscard]] virtual bool addEdge(const std::string &node1,
                                     const std::string &node2, int weight) = 0;
  /**
   * @brief 获取所有边的vector，用于写入文件和绘图
   * @return std::optional<std::vector<Edge>>
   * @author Alkaid
   */
  virtual std::optional<std::vector<Edge>> getEdges() const = 0;

  /**
   * @brief 获取最小生成树
   * @return std::optional<T> 最小生成树图对象
   * @author Alkaid
   */
  virtual std::optional<T> getMinimumSpanningTree() const = 0;
};

#endif