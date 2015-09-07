using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.Reflection;
using System.Text;
using System.Threading.Tasks;
using Autodesk.DesignScript.Geometry;
using QuickGraph;
using QuickGraph.Algorithms;
using QuickGraph.Algorithms.ShortestPath;
using QuickGraph.Collections;
using QuickGraph.Graphviz;

namespace DynamoGraph
{
    public class Connector
    {
        internal TaggedEdge<object, Connector> TaggedEdge { get; private set; }

        private Connector(object from, object to, double weight)
        {
            this.TaggedEdge = new TaggedEdge<object, Connector>(from, to, this);
            this.Weight = weight;
        }

        /// <summary>
        /// Creates a connection between two objects
        /// </summary>
        /// <param name="from">The source object</param>
        /// <param name="to">The target object</param>
        /// <param name="weight">The weight of the connector</param>
        /// <param name="edgeData">Additional data to attach with this connector</param>
        /// <returns>Connector object</returns>
        public static Connector ConnectObjects(object from, object to, double weight, object edgeData)
        {
            return new Connector(from, to, weight){EdgeData = edgeData};
        }

        /// <summary>
        /// Creates a connection between two objects
        /// </summary>
        /// <param name="from">The source object</param>
        /// <param name="to">The target object</param>
        /// <param name="weight">The weight of the connector</param>
        /// <returns>Connector object</returns>
        public static Connector ConnectObjects(object from, object to, double weight = 1)
        {
            return new Connector(from, to, weight) { EdgeData = null };
        }

        /// <summary>
        /// Gets the source vertex
        /// </summary>
        public object Source { get { return TaggedEdge.Source; } }

        /// <summary>
        /// Gets the target vertex
        /// </summary>
        public object Target { get { return TaggedEdge.Target; } }

        /// <summary>
        /// Gets weight parameter for this connector
        /// </summary>
        public double Weight { get; private set; }

        /// <summary>
        /// Additional Data associated with this connector
        /// </summary>
        public object EdgeData { get; set; }

        /// <summary>
        /// Returns formatted string for this connector object
        /// </summary>
        /// <returns></returns>
        public override string ToString()
        {
            return TaggedEdge.ToString();
        }
    }

    public class Graph
    {
        protected UndirectedGraph<object, TaggedEdge<object, Connector>> graph;

        /// <summary>
        /// Default constructor, made protected
        /// </summary>
        protected Graph(){}

        /// <summary>
        /// Constructs a topology graph, to connect the vertex geometry with
        /// cooresponding curve geometry, so that users can do topology traversal.
        /// </summary>
        /// <param name="topology">BRep Topology</param>
        /// <returns>Graph</returns>
        public static Graph FromTopology(Topology topology)
        {
            if(null == topology)
                throw new ArgumentNullException("topology");

            var connectors =
                topology.Edges.Select(
                    e =>
                    {
                        var curve = e.CurveGeometry;
                        return Connector.ConnectObjects(e.StartVertex.PointGeometry, e.EndVertex.PointGeometry,
                            curve.Length, curve);
                    });

            return FromConnectors(connectors);
        }

        /// <summary>
        /// Creates a graph from a network of curves.
        /// </summary>
        /// <param name="curves">List of curves</param>
        /// <returns>Graph</returns>
        public static Graph FromCurves(IEnumerable<Curve> curves)
        {
            if(null == curves)
                throw new ArgumentNullException("curves");

            var connectors = curves.Select(c => Connector.ConnectObjects(c.StartPoint, c.EndPoint, c.Length, c));
            return FromConnectors(connectors);
        }

        /// <summary>
        /// Creates undirected graph using the given list of connectors.
        /// </summary>
        /// <param name="connectors">List of connectors for the graph.</param>
        /// <returns>Graph</returns>
        public static Graph FromConnectors(IEnumerable<Connector> connectors)
        {
            if (null == connectors)
                throw new ArgumentNullException("connectors");

            var connectorList = connectors as IList<Connector> ?? connectors.ToList();
            var edges = connectorList.Select(c => c.TaggedEdge);

            return new Graph { Connectors = connectorList, graph = edges.ToUndirectedGraph<object, TaggedEdge<object, Connector>>(false) };
        }

        /// <summary>
        /// Gets list of vertices in the graph
        /// </summary>
        public IEnumerable Vertices { get { return graph.Vertices; } }

        /// <summary>
        /// Gets number of vertices.
        /// </summary>
        public int VertexCount { get { return graph.VertexCount; } }

        /// <summary>
        /// 
        /// </summary>
        public IEnumerable<Connector> Connectors { get; private set; }

        /// <summary>
        /// Gets all connectors that connects the given vertex.
        /// </summary>
        /// <param name="vertex">Vertex object</param>
        /// <returns>List of adjacent connectors</returns>
        public IEnumerable<Connector> AdjacentConnectors(object vertex)
        {
            return graph.AdjacentEdges(vertex).Select(e => e.Tag);
        }

        /// <summary>
        /// Gets EdgeData of all the adjacent connectors on the given vertex.
        /// </summary>
        /// <param name="vertex">Vertex object</param>
        /// <returns>List of adjacent edges</returns>
        public IEnumerable AdjacentEdges(object vertex)
        {
            return graph.AdjacentEdges(vertex).Select(e => e.Tag.EdgeData);
        }

        /// <summary>
        /// Gets shortest path from given source to the given target object in
        /// this graph.
        /// </summary>
        /// <param name="source">Source object</param>
        /// <param name="target">Target object</param>
        /// <param name="directed"></param>
        /// <returns>List of objects from source to target to define the path.</returns>
        public IEnumerable GetShortestPath(object source, object target, bool directed = false)
        {
            TryFunc<object, IEnumerable<TaggedEdge<object, Connector>>> tryGetPathFunc = null;
            if (directed)
            {
                var directedGraph = Connectors.Select(c => c.TaggedEdge).ToAdjacencyGraph<object, TaggedEdge<object, Connector>>();
                tryGetPathFunc = directedGraph.ShortestPathsDijkstra(e => e.Tag.Weight, source);
            }
            else
            {
                tryGetPathFunc = graph.ShortestPathsDijkstra(e => e.Tag.Weight, source);
            }

            IEnumerable<TaggedEdge<object, Connector>> path;
            var vertices = new List<object>();

            if (!tryGetPathFunc(target, out path)) return vertices;

            vertices.Add(source);
            var currentVertex = source;
            foreach (var edge in path)
            {
                currentVertex = edge.GetOtherVertex(currentVertex);
                vertices.Add(currentVertex);
            }

            return vertices;
        }
    }

    internal static class Algorithms
    {
        /// <summary>
        /// Computes shortest path for the given network of curves from the given
        /// source point to the given target point. Both source and target should
        /// be one of the end points of a curve in the input network of curves. 
        /// </summary>
        /// <param name="curves">List of curves to define the network</param>
        /// <param name="source">Source/Start point</param>
        /// <param name="target">Target/End point</param>
        /// <returns>List of points on the shortest path from source to target.
        /// The list includes source and target points.</returns>
        public static IEnumerable GetShortestPath(IEnumerable<Curve> curves, Point source, Point target)
        {
            var connectors = new List<Connector>();
            connectors.AddRange(curves.Select(c => Connector.ConnectObjects(c.StartPoint, c.EndPoint, c.Length, c)));

            return GetShortestPath(connectors, source, target);
        }

        /// <summary>
        /// Computes shortes path for the given network of connections from the given
        /// source object to the given target object. Both the source and the target
        /// should be a source or a target of a connector in the input list of connectors. 
        /// </summary>
        /// <param name="connectors">List of connectors to define the network</param>
        /// <param name="source">Source object</param>
        /// <param name="target">Target object</param>
        /// <returns>List of objects connected to form the shortest path.</returns>
        public static IEnumerable GetShortestPath(List<Connector> connectors, object source, object target)
        {
            if(null == source)
                throw new ArgumentNullException("source");

            if (null == target)
                throw new ArgumentNullException("target");

            if(null== connectors)
                throw new ArgumentNullException("connectors");

            var graph = connectors.Select(c => c.TaggedEdge).ToAdjacencyGraph<object, TaggedEdge<object, Connector>>();

            var tryGetPaths = graph.ShortestPathsDijkstra(e => e.Tag.Weight, source);
            IEnumerable<TaggedEdge<object, Connector>> path;
            var vertices = new List<object>();

            if (!tryGetPaths(target, out path)) return vertices;
            
            vertices.Add(source);
            vertices.AddRange(path.Select(e => e.Target));

            return vertices;
        }
    }
}
