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
    // ��ʼ��
    for (const auto& vertex : graph) {
        dist[vertex.first] = numeric_limits<double>::infinity();
        prev[vertex.first] = "";
    }
    dist[source] = 0;

    // ʹ�����ȶ��й��������Ķ���
    priority_queue<Node, vector<Node>, greater<Node>> Q;
    Q.push({ source, 0 });

    while (!Q.empty()) {
        // ѡ��Q�о�����С�Ķ���u
        Node u = Q.top();
        Q.pop();

        if (u.distance > dist[u.vertex]) {
            continue;
        }

        // ��u��ÿ���ھ�v
        for (const auto& neighbor : graph.at(u.vertex)) {
            string v = neighbor.first;
            double alt = dist[u.vertex] + neighbor.second; // ����u��v�ľ���

            if (alt < dist[v]) { // ����������ȵ�ǰ��֪�ľ������
                dist[v] = alt; // ���¾���
                prev[v] = u.vertex; // ����ǰ���ڵ�
                Q.push({ v, alt }); // �����º�Ľڵ�������ȶ���
            }
        }
    }
}

Graph createGraph() {
    Graph graph;
    int num_vertices, num_edges;
    cout << "����ͼ�еĶ�������: ";
    cin >> num_vertices;

    cout << "���붥������: " << endl;
    for (int i = 0; i < num_vertices; ++i) {
        string vertex;
        cin >> vertex;
        graph[vertex] = unordered_map<string, double>();
    }

    cout << "����ͼ�еı�����: ";
    cin >> num_edges;

    cout << "����ߵ���㡢�յ��Ȩ��:" << endl;
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
    cout << "�������: ";
    cin >> start_vertex;

    unordered_map<string, double> dist;
    unordered_map<string, string> prev;

    dijkstra(graph, start_vertex, dist, prev);

    cout << "��̾����·��:" << endl;
    for (const auto& distance : dist) {
        if (distance.second == numeric_limits<double>::infinity()) {
            cout << distance.first << ": �޷�����" << endl;
        }
        else {
            cout << distance.first << ": " << distance.second << ", ·��: ";
            printPath(prev, distance.first);
            cout << endl;
        }
    }

    return 0;
}
