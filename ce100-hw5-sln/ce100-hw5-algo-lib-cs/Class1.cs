/**
* @file ce100-hw5-algo-lib-cs
* @author Rümeysa Er
* @date 27 May 2022
*
* @brief <b> HW-5 Functions </b>
*
* HW-5 Sample Lib Functions
*
* @see http://bilgisayar.mmf.erdogan.edu.tr/en/
*
*/

using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using ce100_hw5_algo_lib_cs;

namespace ce100_hw5_algo_lib_cs
{
    public class Class1
    {
       


        public class GraphCycleDetection
        {

            private readonly int V;
            private readonly List<List<int>> adj;

            public GraphCycleDetection(int V)
            {
                this.V = V;
                adj = new List<List<int>>(V);

                for (int i = 0; i < V; i++)
                    adj.Add(new List<int>());
            }

            /**
            * @name isCyclicUtil
            * @param [in] i [\b int]
            * @param [in] visited [\b bool[]]
            * @param [in] recStack [\b bool[]]
            * @retval [\b bool]
            * 
            * 
            * Depth First Traversal can be used to detect a cycle in a Graph. DFS for a connected graph produces a tree.
            * There is a cycle in a graph only if there is a back edge present in the graph.
            * A back edge is an edge that is from a node to itself (self-loop) or one of its ancestors in the tree produced by DFS. In the following graph, there are 3 back edges, marked with a cross sign.
            * We can observe that these 3 back edges indicate 3 cycles present in the graph.
            * 
            **/
            public bool IsCyclicUtil(int i, bool[] visited, bool[] recStack)
            {

                // Mark the current node as visited and
                // part of recursion stack
                if (recStack[i])
                    return true;

                if (visited[i])
                    return false;

                visited[i] = true;

                recStack[i] = true;
                List<int> children = adj[i];

                foreach (int c in children)
                    if (IsCyclicUtil(c, visited, recStack))
                        return true;

                recStack[i] = false;

                return false;
            }

            /**
            * @name  addEdge
            * @param [in] sou [\b int]
            * @param [in] dest [\b int]
            * @retval [\b void]
            * Simply remove them from the graph.
            * **/

            public void addEdge(int sou, int dest)
            {
                adj[sou].Add(dest);
            }

            public bool isCyclic()
            {
                bool[] visited = new bool[V];
                bool[] recStack = new bool[V];

                for (int i = 0; i < V; i++)
                    if (IsCyclicUtil(i, visited, recStack))
                        return true;

                return false;
            }
        }


        /** 
        // HERE I EXPLAIN THE CONTENTS OF THE CODE;
          
        // A class to represent a graph edge
        // Comparator function used for sorting edges
        // based on their weight
        // A class to represent
        // a subset for union-find

        // Creates a graph with V vertices and E edges  A utility function to find set of an element i
        // (uses path compression technique)
        // find root and make root as
        // parent of i(path compression)A function that does union of
        // two sets of x and y(uses union by rank)

        // Attach smaller rank tree under root of
        // high rank tree(Union by Rank)

        // If ranks are same, then make one as root
        // and increment its rank by one
        // The main function to construct MST
        // using Kruskal's algorithm
        // This will store the
        // resultant MST
        // Step 1: Sort all the edges in non-decreasing
        // order of their weight.If we are not allowed
        // to change the given graph, we can create
        // a copy of array of edges

        // Allocate memory for creating V subsets
        // Create V subsets with single elements

        // Index used to pick next edge

        // Number of edges to be taken is equal to V-1
        // Step 2: Pick the smallest edge.And increment
        // the index for next iteration

        // If including this edge doesn't cause cycle,
        // include it in result and increment the index
        // of result for next edge
        // Else discard the next_edge
         **/




        public class MinimumSpanningTree
        {

            public class Edge : IComparable<Edge>
            {
                public int src, dest, weight;

                public int CompareTo(Edge compareEdge)
                {
                    return this.weight
                           - compareEdge.weight;
                }
            }

            public class subset
            {
                public int parent, rank;
            };

            int V, E;
            public Edge[] edge;

            /**
         * @name minimum spanning tree
         * @param [in] v [\b int]
         * @param [in] e [\b int]
         * Given a connected and undirected graph,
         * a spanning tree of that graph is a subgraph that is a tree and connects all the vertices together.
         * A single graph can have many different spanning trees.
         * A minimum spanning tree (MST) or minimum weight spanning tree for a weighted, connected,
         * undirected graph is a spanning tree with a weight less than 
         * or equal to the weight of every other spanning tree. The weight of a spanning tree 
         * is the sum of weights given to each edge of the spanning tree.
         *
         * */



            public MinimumSpanningTree(int v, int e)
            {
                V = v;
                E = e;
                edge = new Edge[E];
                for (int i = 0; i < e; ++i)
                    edge[i] = new Edge();
            }

            /**
            * 
            * @name Union
            * @param [in] subsets [\b subset[]]
            * @param [in] i [\b int[]]
            * retval [\b int]
            * The Array.Find method is used to find the first matching element in an array.
            * If there is an item matching the condition, it returns the item's value. 
            * Returns null if any value does not match the condition.
            * 
            * **/
            int find(subset[] subsets, int i)
          {

              if (subsets[i].parent != i)
                  subsets[i].parent
                      = find(subsets, subsets[i].parent);

              return subsets[i].parent;
          }


          /**
          * 
          * @name Union
          * @param [in] subsets [\b subset[]]
          * @param [in] x [\b int]
          * @param [in] y [\b int]
          * retval [\b void]
          * Creates a new collection without including duplicates of data in two different collections.
          * The two collections in the example have the same values.
          * With Union, these collections are combined and duplicate values ​​are not found in the newly created collection.
          * 
          * **/

            void Union(subset[] subsets, int x, int y)
          {
              int xroot = find(subsets, x);
              int yroot = find(subsets, y);

              if (subsets[xroot].rank < subsets[yroot].rank)
                  subsets[xroot].parent = yroot;
              else if (subsets[xroot].rank > subsets[yroot].rank)
                  subsets[yroot].parent = xroot;

              else
              {
                  subsets[yroot].parent = xroot;
                  subsets[xroot].rank++;
              }
          }
          /**
          * 
          * @name KruskalMST
          * retval [\b string]
          * The main function to construct MST using Kruskal's algorithm
          * Sort all the edges in non-decreasing order of their weight. 
          * Pick the smallest edge. Check if it forms a cycle with the spanning tree formed so far.
          * If cycle is not formed, include this edge. Else, discard it. 
          * Repeat step#2 until there are (V-1) edges in the spanning tree.
          *
          * */
            public string KruskalMST()
            {

                Edge[] result = new Edge[V];
                string mst = "";
                int e = 0;
                int i
                    = 0; // An index variable, used for sorted edges
                for (i = 0; i < V; ++i)
                    result[i] = new Edge();

                Array.Sort(edge);

                // Allocate memory for creating V subsets
                subset[] subsets = new subset[V];
                for (i = 0; i < V; ++i)
                    subsets[i] = new subset();

                // Create V subsets with single elements
                for (int v = 0; v < V; ++v)
                {
                    subsets[v].parent = v;
                    subsets[v].rank = 0;
                }

                i = 0; // Index used to pick next edge

                // Number of edges to be taken is equal to V-1
                while (e < V - 1)
                {
                    // Step 2: Pick the smallest edge. And increment
                    // the index for next iteration
                    Edge next_edge = new Edge();
                    next_edge = edge[i++];

                    int x = find(subsets, next_edge.src);
                    int y = find(subsets, next_edge.dest);

                    // If including this edge doesn't cause cycle,
                    // include it in result and increment the index
                    // of result for next edge
                    if (x != y)
                    {
                        result[e++] = next_edge;
                        Union(subsets, x, y);
                    }
                    // Else discard the next_edge
                }
                mst += "(source --> destination : weight)\r\n";
                for (i = 0; i < e; ++i)
                {
                    mst += "(" + result[i].src + " --> " + result[i].dest + " : " + result[i].weight + ")\r\n";
                }

                return mst;
            }
        }






        public class SingleSourceShortestPath
        {
            public class Edge
            {
                public int src, dest, weight;
                public Edge()
                {
                    src = dest = weight = 0;
                }
            };

            int V, E;
            public Edge[] edge;

            // Creates a graph with V vertices and E edges

            /**
            * 
            * @name Single Source Shortest Path
            * @param [in] v [\b int]
            * @param [in] e [\b int]
            * Like other Dynamic Programming Problems, the algorithm calculates shortest paths in a bottom-up manner.
            * It first calculates the shortest distances which have at-most one edge in the path.
            * Then, it calculates the shortest paths with at-most 2 edges, and so on.
            * After the i-th iteration of the outer loop, the shortest paths with at most i edges are calculated.
            * There can be maximum |V| – 1 edges in any simple path, that is why the outer loop runs |v| – 1 times.
            * The idea is, assuming that there is no negative weight cycle,
            * if we have calculated shortest paths with at most i edges,
            * then an iteration over all edges guarantees to give shortest path with at-most (i+1) edges (Proof is simple,
            * you can refer this or MIT Video Lecture)
            *
            * */
            public SingleSourceShortestPath(int v, int e)
            {
                V = v;
                E = e;
                edge = new Edge[e];
                for (int i = 0; i < e; ++i)
                    edge[i] = new Edge();
            }

            // The main function that finds shortest distances from src
            // to all other vertices using Bellman-Ford algorithm. The
            // function also detects negative weight cycle

            /**
            * @name Bellman Ford
            * @param [in] graph [\b SingleSourceShortestPath]
            * @param [in] src [\b int]
            * retval [\b string]
            * Bellman-Ford works for such graphs.
            * Bellman-Ford is also simpler than Dijkstra and suites well for distributed systems. 
            *
            * */
            public string BellmanFord(SingleSourceShortestPath graph, int src)
            {
                string print = "";
                int V = graph.V, E = graph.E;
                int[] dist = new int[V];

                // Step 1: Initialize distances from src to all other
                // vertices as INFINITE
                for (int i = 0; i < V; ++i)
                    dist[i] = int.MaxValue;
                dist[src] = 0;

                // Step 2: Relax all edges |V| - 1 times. A simple
                // shortest path from src to any other vertex can
                // have at-most |V| - 1 edges
                for (int i = 1; i < V; ++i)
                {
                    for (int j = 0; j < E; ++j)
                    {
                        int u = graph.edge[j].src;
                        int v = graph.edge[j].dest;
                        int weight = graph.edge[j].weight;
                        if (dist[u] != int.MaxValue && dist[u] + weight < dist[v])
                            dist[v] = dist[u] + weight;
                    }
                }

                for (int j = 0; j < E; ++j)
                {
                    int u = graph.edge[j].src;
                    int v = graph.edge[j].dest;
                    int weight = graph.edge[j].weight;
                    
                }
                print+="(Vertex --> Distance from Source)\r\n";
                for (int i = 0; i < V; ++i)
                {
                    print+="("+ i + " --> " + dist[i]+ ")\r\n";
                }
                return print;
            }
        }




        public class MaxFlow
        {
            static readonly int V = 6;
            // Number of vertices in
            // graph

            /* Returns true if there is a path
            from source 's' to sink 't' in residual
            graph. Also fills parent[] to store the
            path */


            /**
            * @name bfs
            * @param [in] rGraph [\b int[]]
            * @param [in] s [\b int[]]
            * @param [in] t [\b int[]]
            * @param [in] parent [\b int[]]
            * retval [\b bool]
            * BFS can be used to find single source shortest path in an unweighted graph, 
            * because in BFS, we reach a vertex with minimum number of edges from a source vertex.
            * BFS is more suitable for searching vertices which are closer to the given source.
            *
            * */
            bool bfs(int[,] rGraph, int s, int t, int[] parent)
            {
                // Create a visited array and mark
                // all vertices as not visited
                bool[] visited = new bool[V];
                for (int i = 0; i < V; ++i)
                    visited[i] = false;

                // Create a queue, enqueue source vertex and mark
                // source vertex as visited
                List<int> queue = new List<int>();
                queue.Add(s);
                visited[s] = true;
                parent[s] = -1;

                // Standard BFS Loop
                while (queue.Count != 0)
                {
                    int u = queue[0];
                    queue.RemoveAt(0);

                    for (int v = 0; v < V; v++)
                    {
                        if (visited[v] == false
                            && rGraph[u, v] > 0)
                        {
                            // If we find a connection to the sink
                            // node, then there is no point in BFS
                            // anymore We just have to set its parent
                            // and can return true
                            if (v == t)
                            {
                                parent[v] = u;
                                return true;
                            }
                            queue.Add(v);
                            parent[v] = u;
                            visited[v] = true;
                        }
                    }
                }

                // We didn't reach sink in BFS starting from source,
                // so return false
                return false;
            }

            // Returns the maximum flow
            // from s to t in the given graph

            /**
            * @name Ford Fulkerson
            * @param [in] graph [\b int[]]
            * @param [in] s [\b int[]]
            * @param [in] t [\b int[]]
            * retval [\b int]
            * Residual Graph of a flow network is a graph which indicates additional possible flow. 
            * If there is a path from source to sink in residual graph, then it is possible to add flow.
            * Every edge of a residual graph has a value called residual capacity which is equal to 
            * original capacity of the edge minus current flow.
            * Residual capacity is basically the current capacity of the edge. 
            * 
            * */

            public int FordFulkerson(int[,] graph, int s, int t)
            {
                int u, v;

                // Create a residual graph and fill
                // the residual graph with given
                // capacities in the original graph as
                // residual capacities in residual graph

                // Residual graph where rGraph[i,j]
                // indicates residual capacity of
                // edge from i to j (if there is an
                // edge. If rGraph[i,j] is 0, then
                // there is not)
                int[,] rGraph = new int[V, V];

                for (u = 0; u < V; u++)
                    for (v = 0; v < V; v++)
                        rGraph[u, v] = graph[u, v];

                // This array is filled by BFS and to store path
                int[] parent = new int[V];

                int max_flow = 0; // There is no flow initially

                // Augment the flow while there is path from source
                // to sink
                while (bfs(rGraph, s, t, parent))
                {
                    // Find minimum residual capacity of the edhes
                    // along the path filled by BFS. Or we can say
                    // find the maximum flow through the path found.
                    int path_flow = int.MaxValue;
                    for (v = t; v != s; v = parent[v])
                    {
                        u = parent[v];
                        path_flow
                            = Math.Min(path_flow, rGraph[u, v]);
                    }

                    // update residual capacities of the edges and
                    // reverse edges along the path
                    for (v = t; v != s; v = parent[v])
                    {
                        u = parent[v];
                        rGraph[u, v] -= path_flow;
                        rGraph[v, u] += path_flow;
                    }

                    // Add path flow to overall flow
                    max_flow += path_flow;
                }

                // Return the overall flow
                return max_flow;
            }
        }
    }
}







