using Microsoft.VisualStudio.TestTools.UnitTesting;
using System;
using ce100_hw5_algo_lib_cs;
using static ce100_hw5_algo_lib_cs.Class1;

namespace ce100_hw5_algo_test_cs
{
    [TestClass]
    public class UnitTest1
    {
        [TestMethod]
        public void Graph_Cycle_Detection_Test_1()
        {
            GraphCycleDetection graph = new GraphCycleDetection(9);
            graph.addEdge(1, 2);
            graph.addEdge(1, 3);
            graph.addEdge(1, 4);
            graph.addEdge(1, 5);
            graph.addEdge(2, 6);
            graph.addEdge(2, 7);
            graph.addEdge(3, 8);

            bool result = false;
            Assert.AreEqual(result, graph.isCyclic());
        }

        [TestMethod]
        public void Graph_Cycle_Detection_Test_2()
        {
            GraphCycleDetection graph = new GraphCycleDetection(6);
            graph.addEdge(0, 1);
            graph.addEdge(1, 2);
            graph.addEdge(2, 0);
            graph.addEdge(3, 4);
            graph.addEdge(4, 5);

            bool result = true;
            Assert.AreEqual(result, graph.isCyclic());

        }

        [TestMethod]
        public void Graph_Cycle_Detection_Test_3()
        {
            GraphCycleDetection graph = new GraphCycleDetection(4);
            graph.addEdge(0, 1);
            graph.addEdge(0, 2);
            graph.addEdge(1, 2);
            graph.addEdge(2, 0);
            graph.addEdge(2, 3);

            bool result = true;
            Assert.AreEqual(result, graph.isCyclic());

        }


        [TestMethod]
        public void Minimum_Spanning_Tree_Test_1()
        {
            int V = 4; // Number of vertices in graph
            int E = 5; // Number of edges in graph
            MinimumSpanningTree graph = new MinimumSpanningTree(V, E);

            // add edge 0-1
            graph.edge[0].src = 0;
            graph.edge[0].dest = 1;
            graph.edge[0].weight = 10;

            // add edge 0-2
            graph.edge[1].src = 0;
            graph.edge[1].dest = 2;
            graph.edge[1].weight = 6;

            // add edge 0-3
            graph.edge[2].src = 0;
            graph.edge[2].dest = 3;
            graph.edge[2].weight = 5;

            // add edge 1-3
            graph.edge[3].src = 1;
            graph.edge[3].dest = 3;
            graph.edge[3].weight = 15;

            // add edge 2-3
            graph.edge[4].src = 2;
            graph.edge[4].dest = 3;
            graph.edge[4].weight = 4;

            string result = graph.KruskalMST();

            string expected = "(source --> destination : weight)\r\n" +
                              "(2 --> 3 : 4)\r\n" +
                              "(0 --> 3 : 5)\r\n" +
                              "(0 --> 1 : 10)\r\n";
            Assert.AreEqual(result, expected);
        }

        [TestMethod]
        public void Single_Source_Shortest_Path_Test2()
        {
            int V = 4; // Number of vertices in graph
            int E = 6; // Number of edges in graph

            SingleSourceShortestPath graph = new SingleSourceShortestPath(V, E);

            // add edge 1-3 (or A-B in above figure)
            graph.edge[0].src = 1;
            graph.edge[0].dest = 3;
            graph.edge[0].weight = -2;

            // add edge 1-3 (or A-C in above figure)
            graph.edge[1].src = 1;
            graph.edge[1].dest = 3;
            graph.edge[1].weight = 2;

            // add edge 0-2 (or B-C in above figure)
            graph.edge[2].src = 0;
            graph.edge[2].dest = 2;
            graph.edge[2].weight = 3;

            // add edge 1-2 (or B-D in above figure)
            graph.edge[3].src = 1;
            graph.edge[3].dest = 2;
            graph.edge[3].weight = 2;

            // add edge 3-1 (or B-E in above figure)
            graph.edge[4].src = 3;
            graph.edge[4].dest = 1;
            graph.edge[4].weight = 2;

            // add edge 2-3 (or D-C in above figure)
            graph.edge[5].src = 2;
            graph.edge[5].dest = 3;
            graph.edge[5].weight = 4;

            string result = graph.BellmanFord(graph, 0);
            string expected = "(Vertex --> Distance from Source)\r\n" +
                              "(0 --> 0)\r\n" +
                              "(1 --> 9)\r\n" +
                              "(2 --> 3)\r\n" +
                              "(3 --> 7)\r\n";

            Assert.AreEqual(result, expected);
        }

        [TestMethod]
        public void Single_Source_Shortest_Path_Test3()
        {
            int V = 4; // Number of vertices in graph
            int E = 6; // Number of edges in graph

            SingleSourceShortestPath graph = new SingleSourceShortestPath(V, E);

            // add edge 1-2 (or A-B in above figure)
            graph.edge[0].src = 1;
            graph.edge[0].dest = 2;
            graph.edge[0].weight = -2;

            // add edge 0-3 (or A-C in above figure)
            graph.edge[1].src = 0;
            graph.edge[1].dest = 3;
            graph.edge[1].weight = 2;

            // add edge 0-1 (or B-C in above figure)
            graph.edge[2].src = 0;
            graph.edge[2].dest = 1;
            graph.edge[2].weight = 3;

            // add edge 1-2 (or B-D in above figure)
            graph.edge[3].src = 1;
            graph.edge[3].dest = 2;
            graph.edge[3].weight = 2;

            // add edge 3-1 (or B-E in above figure)
            graph.edge[4].src = 3;
            graph.edge[4].dest = 1;
            graph.edge[4].weight = 2;

            // add edge 2-3 (or D-C in above figure)
            graph.edge[5].src = 2;
            graph.edge[5].dest = 3;
            graph.edge[5].weight = 4;

            string result = graph.BellmanFord(graph, 0);
            string expected = "(Vertex --> Distance from Source)\r\n" +
                              "(0 --> 0)\r\n" +
                              "(1 --> 3)\r\n" +
                              "(2 --> 1)\r\n" +
                              "(3 --> 2)\r\n";

            Assert.AreEqual(result, expected);
        }

        [TestMethod]
        public void Minimum_Spanning_Tree_Test_2()
        {
            int V = 3; // Number of vertices in graph
            int E = 4; // Number of edges in graph
            MinimumSpanningTree graph = new MinimumSpanningTree(V, E);

            // add edge 0-2
            graph.edge[0].src = 0;
            graph.edge[0].dest = 2;
            graph.edge[0].weight = 5;

            // add edge 1-3
            graph.edge[1].src = 1;
            graph.edge[1].dest = 3;
            graph.edge[1].weight = 6;

            // add edge 0-1
            graph.edge[2].src = 0;
            graph.edge[2].dest = 1;
            graph.edge[2].weight = 4;

            // add edge 1-3
            graph.edge[3].src = 1;
            graph.edge[3].dest = 3;
            graph.edge[3].weight = 17;


            string result = graph.KruskalMST();

            string expected = "(source --> destination : weight)\r\n" +
                              "(0 --> 1 : 4)\r\n" +
                              "(0 --> 2 : 5)\r\n";
            Assert.AreEqual(result, expected);
        }

        [TestMethod]
        public void Minimum_Spanning_Tree_Test_3()
        {
            int V = 4; // Number of vertices in graph
            int E = 3; // Number of edges in graph
            MinimumSpanningTree graph = new MinimumSpanningTree(V, E);

            // add edge 0-3
            graph.edge[0].src = 0;
            graph.edge[0].dest = 3;
            graph.edge[0].weight = 7;

            // add edge 1-2
            graph.edge[1].src = 1;
            graph.edge[1].dest = 2;
            graph.edge[1].weight = 8;

            // add edge 0-1
            graph.edge[2].src = 0;
            graph.edge[2].dest = 1;
            graph.edge[2].weight = 9;


            string result = graph.KruskalMST();

            string expected = "(source --> destination : weight)\r\n" +
                              "(0 --> 3 : 7)\r\n" +
                              "(1 --> 2 : 8)\r\n" +
                              "(0 --> 1 : 9)\r\n";
            Assert.AreEqual(result, expected);
        }


        [TestMethod]
        public void Single_Source_Shortest_Path_Test1()
        {
            int V = 5; // Number of vertices in graph
            int E = 8; // Number of edges in graph

            SingleSourceShortestPath graph = new SingleSourceShortestPath(V, E);

            // add edge 0-1 (or A-B in above figure)
            graph.edge[0].src = 0;
            graph.edge[0].dest = 1;
            graph.edge[0].weight = -1;

            // add edge 0-2 (or A-C in above figure)
            graph.edge[1].src = 0;
            graph.edge[1].dest = 2;
            graph.edge[1].weight = 4;

            // add edge 1-2 (or B-C in above figure)
            graph.edge[2].src = 1;
            graph.edge[2].dest = 2;
            graph.edge[2].weight = 3;

            // add edge 1-3 (or B-D in above figure)
            graph.edge[3].src = 1;
            graph.edge[3].dest = 3;
            graph.edge[3].weight = 2;

            // add edge 1-4 (or B-E in above figure)
            graph.edge[4].src = 1;
            graph.edge[4].dest = 4;
            graph.edge[4].weight = 2;

            // add edge 3-2 (or D-C in above figure)
            graph.edge[5].src = 3;
            graph.edge[5].dest = 2;
            graph.edge[5].weight = 5;

            // add edge 3-1 (or D-B in above figure)
            graph.edge[6].src = 3;
            graph.edge[6].dest = 1;
            graph.edge[6].weight = 1;

            // add edge 4-3 (or E-D in above figure)
            graph.edge[7].src = 4;
            graph.edge[7].dest = 3;
            graph.edge[7].weight = -3;

            string result = graph.BellmanFord(graph, 0);
            string expected = "(Vertex --> Distance from Source)\r\n" +
                              "(0 --> 0)\r\n" +
                              "(1 --> -1)\r\n" +
                              "(2 --> 2)\r\n" +
                              "(3 --> -2)\r\n" +
                              "(4 --> 1)\r\n";
            Assert.AreEqual(result, expected);
        }


        [TestMethod]
        public void Max_Flow_Test_1()
        {
            int[,] graph = new int[,]
            {
                    { 0, 16, 13, 0, 0, 0 }, { 0, 0, 10, 12, 0, 0 },
                    { 0, 4, 0, 0, 14, 0 },  { 0, 0, 9, 0, 0, 20 },
                    { 0, 0, 0, 7, 0, 4 },   { 0, 0, 0, 0, 0, 0 }
            };

            MaxFlow m = new MaxFlow();

            int result = 23;
            Assert.AreEqual(m.FordFulkerson(graph, 0, 5), result);
        }

        [TestMethod]
        public void Max_Flow_Test_2()
        {
            int[,] graph = new int[,]
            {
                    { 0, 10, 10, 0, 0, 0 }, { 0, 0, 2, 4, 8, 0 },
                    { 0, 0, 0, 0, 9, 0 },  { 0, 0, 0, 0, 0, 10 },
                    { 0, 0, 0, 6, 0, 10 },   { 0, 0, 0, 0, 0, 0 }
            };
            MaxFlow m = new MaxFlow();
            int result = 19;
            Assert.AreEqual(m.FordFulkerson(graph, 0, 5), result);
        }

        [TestMethod]
        public void Max_Flow_Test_3()
        {
            int[,] graph = new int[,]
            {
                    { 0, 7, 5, 0, 0, 0 }, { 0, 0, 5, 4, 0, 0 },
                    { 0, 0, 0, 6, 4, 0 },  { 0, 0, 0, 0, 2, 5 },
                    { 0, 0, 0, 0, 0, 6 },   { 0, 0, 0, 0, 0, 0 }
            };
            MaxFlow m = new MaxFlow();
            int result = 11;
            Assert.AreEqual(m.FordFulkerson(graph, 0, 5), result);
        }
    }
}







