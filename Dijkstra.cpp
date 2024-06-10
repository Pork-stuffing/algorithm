#include <iostream>
#include <vector>
#include <unordered_map>
#include <queue>
#include <limits>
#include <string>
#include <utility>
#include <algorithm>

using namespace std;

typedef unordered_map<string, unordered_map<string, double>> Graph;

struct Node {
    string vertex;
    double distance;
    bool operator>(const Node& other) const {
        return distance > other.distance;
    }
};

void dijkstra(const Graph& graph, const string& source, unordered_map<string, double>& dist, unordered_map<string, string>& prev) {
    // 初始化
    for (const auto& vertex : graph) {
        dist[vertex.first] = numeric_limits<double>::infinity();
        prev[vertex.first] = "";
    }
    dist[source] = 0;

    // 使用优先队列管理待处理的顶点
    priority_queue<Node, vector<Node>, greater<Node>> Q;
    Q.push({ source, 0 });

    while (!Q.empty()) {
        // 选择Q中距离最小的顶点u
        Node u = Q.top();
        Q.pop();

        if (u.distance > dist[u.vertex]) {
            continue;
        }

        // 对u的每个邻居v
        for (const auto& neighbor : graph.at(u.vertex)) {
            string v = neighbor.first;
            double alt = dist[u.vertex] + neighbor.second; // 计算u到v的距离

            if (alt < dist[v]) { // 如果这个距离比当前已知的距离更短
                dist[v] = alt; // 更新距离
                prev[v] = u.vertex; // 更新前驱节点
                Q.push({ v, alt }); // 将更新后的节点插入优先队列
            }
        }
    }
}

Graph createGraph() {
    Graph graph;
    int num_vertices, num_edges;
    cout << "输入图中的顶点数量: ";
    cin >> num_vertices;

    cout << "输入顶点名称: " << endl;
    for (int i = 0; i < num_vertices; ++i) {
        string vertex;
        cin >> vertex;
        graph[vertex] = unordered_map<string, double>();
    }

    cout << "输入图中的边数量: ";
    cin >> num_edges;

    cout << "输入边的起点、终点和权重:" << endl;
    for (int i = 0; i < num_edges; ++i) {
        string u, v;
        double weight;
        cin >> u >> v >> weight;
        graph[u][v] = weight;
    }

    return graph;
}

void printPath(const unordered_map<string, string>& previous, const string& vertex) {
    vector<string> path;
    string current = vertex;
    while (!current.empty()) {
        path.push_back(current);
        current = previous.at(current);
    }
    reverse(path.begin(), path.end());
    for (size_t i = 0; i < path.size(); ++i) {
        cout << path[i];
        if (i < path.size() - 1) {
            cout << " -> ";
        }
    }
    cout << endl;
}

int main() {
    Graph graph = createGraph();
    string start_vertex;
    cout << "输入起点: ";
    cin >> start_vertex;

    unordered_map<string, double> dist;
    unordered_map<string, string> prev;

    dijkstra(graph, start_vertex, dist, prev);

    cout << "最短距离和路径:" << endl;
    for (const auto& distance : dist) {
        if (distance.second == numeric_limits<double>::infinity()) {
            cout << distance.first << ": 无法到达" << endl;
        }
        else {
            cout << distance.first << ": " << distance.second << ", 路径: ";
            printPath(prev, distance.first);
            cout << endl;
        }
    }

    return 0;
}
