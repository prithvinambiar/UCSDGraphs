/**
 * @author UCSD MOOC development team and YOU
 * <p>
 * A class which reprsents a graph of geographic locations
 * Nodes in the graph are intersections between
 */
package roadgraph;


import geography.GeographicPoint;
import util.GraphLoader;

import java.util.*;
import java.util.function.Consumer;

/**
 * @author UCSD MOOC development team and YOU
 * <p>
 * A class which represents a graph of geographic locations
 * Nodes in the graph are intersections
 */
public class MapGraph {
    private final List<MapVertex> vertices;
    private int edgesCount;


    /**
     * Create a new empty MapGraph
     */
    public MapGraph() {
        this.vertices = new ArrayList<>();
        this.edgesCount = 0;
    }

    /**
     * Get the number of vertices (road intersections) in the graph
     *
     * @return The number of vertices in the graph.
     */
    public int getNumVertices() {
        return this.vertices.size();
    }

    /**
     * Return the intersections, which are the vertices in this graph.
     *
     * @return The vertices in this graph as GeographicPoints
     */
    public Set<GeographicPoint> getVertices() {
        Set<GeographicPoint> geographicPoints = new HashSet<>();
        for (MapVertex mapVertex : this.vertices) {
            geographicPoints.add(mapVertex.getLocation());
        }
        return geographicPoints;
    }

    /**
     * Get the number of road segments in the graph
     *
     * @return The number of edges in the graph.
     */
    public int getNumEdges() {
        return this.edgesCount;
    }


    /**
     * Add a node corresponding to an intersection at a Geographic Point
     * If the location is already in the graph or null, this method does
     * not change the graph.
     *
     * @param location The location of the intersection
     * @return true if a node was added, false if it was not (the node
     * was already in the graph, or the parameter is null).
     */
    public boolean addVertex(GeographicPoint location) {
        if (location == null) {
            return false;
        }
        if (this.findVertex(location) != null) return false;
        MapVertex mapVertex = new MapVertex(location);
        this.vertices.add(mapVertex);
        return true;
    }

    /**
     * Find the MapVertex given a geographic point
     * If the location is not found, this method returns null
     * @param location The location of the intersection
     * @return MapVertex if a vertex was found, null if it was not.
     */
    private MapVertex findVertex(GeographicPoint location) {
        for (MapVertex vertex : this.vertices) {
            if (vertex.getLocation().equals(location)) {
                return vertex;
            }
        }
        return null;
    }

    /**
     * Adds a directed edge to the graph from pt1 to pt2.
     * Precondition: Both GeographicPoints have already been added to the graph
     *
     * @param from     The starting point of the edge
     * @param to       The ending point of the edge
     * @param roadName The name of the road
     * @param roadType The type of the road
     * @param length   The length of the road, in km
     * @throws IllegalArgumentException If the points have not already been
     *                                  added as nodes to the graph, if any of the arguments is null,
     *                                  or if the length is less than 0.
     */
    public void addEdge(GeographicPoint from, GeographicPoint to, String roadName,
                        String roadType, double length) throws IllegalArgumentException {
        MapVertex fromVertex = this.findVertex(from);
        MapVertex toVertex = this.findVertex(to);

        if (fromVertex == null) {
            throw new IllegalArgumentException("The location " + from + " is not added to the graph");
        }
        if (toVertex == null) {
            throw new IllegalArgumentException("The location " + to + " is not added to the graph");
        }
        if (length < 0) {
            throw new IllegalArgumentException("The length cannot be less than 0");
        }

        MapEdge mapEdge = new MapEdge(fromVertex, toVertex, roadName, roadType);
        fromVertex.addEdge(mapEdge);
        this.edgesCount++;
    }


    /**
     * Find the path from start to goal using breadth first search
     *
     * @param start The starting location
     * @param goal  The goal location
     * @return The list of intersections that form the shortest (unweighted)
     * path from start to goal (including both start and goal).
     */
    public List<GeographicPoint> bfs(GeographicPoint start, GeographicPoint goal) {
        // Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {
        };
        return bfs(start, goal, temp);
    }

    /**
     * Find the path from start to goal using breadth first search
     *
     * @param start        The starting location
     * @param goal         The goal location
     * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
     * @return The list of intersections that form the shortest (unweighted)
     * path from start to goal (including both start and goal).
     */
    public List<GeographicPoint> bfs(GeographicPoint start,
                                     GeographicPoint goal, Consumer<GeographicPoint> nodeSearched) {
        // Initialization
        Map<MapVertex, MapVertex> parentMap = new HashMap<>();
        MapVertex startVertex = this.findVertex(start);
        MapVertex goalVertex = this.findVertex(goal);

        // Perform BFS
        boolean foundPath = performBFS(startVertex, goalVertex, parentMap, nodeSearched);

        // Return Path
        if (!foundPath) return null;
        return buildPath(startVertex, goalVertex, parentMap);
    }

    /**
     * Helper method which runs BFS on the graph and returns whether a path was
     * found between start and goal vertex
     * @param startVertex  The starting vertex
     * @param goalVertex   The ending vertex
     * @param parentMap    The map of vertex and its parent. This is needed to capture the path
     * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
     * @return true if a path exists between start and goal vertex, false otherwise
     */
    private boolean performBFS(MapVertex startVertex, MapVertex goalVertex,
                               Map<MapVertex, MapVertex> parentMap, Consumer<GeographicPoint> nodeSearched) {
        boolean foundPath = false;
        LinkedList<MapVertex> queue = new LinkedList<>();
        Set<MapVertex> visitedVertices = new HashSet<>();
        queue.addLast(startVertex);
        while (!queue.isEmpty()) {
            MapVertex next = queue.removeFirst();
            List<MapVertex> neighbours = next.findNeighbours();
            for (MapVertex neighbour : neighbours) {
                // If the neighbour was already visited, ignore the vertex
                if (visitedVertices.contains(neighbour)) {
                    continue;
                }
                queue.addLast(neighbour);
                parentMap.put(neighbour, next);
                if (neighbour.equals(goalVertex)) {
                    foundPath = true;
                    break;
                }
            }
            visitedVertices.add(next);
            nodeSearched.accept(next.getLocation());
            if (foundPath) {
                // Path was found between Start and Goal vertex, hence return true
                return true;
            }
        }
        // No path was found between Start and Goal vertex, hence return false
        return false;
    }

    /**
     * Builds a path from start vertex to goal vertex
     * @param startVertex The starting vertex
     * @param goalVertex  The ending vertex
     * @param parentMap   The map of vertex and its parent. This is needed to capture the path
     * @return The list of intersections that form the shortest (unweighted)
     * path from start to goal (including both start and goal).
     */
    private List<GeographicPoint> buildPath(MapVertex startVertex, MapVertex goalVertex, Map<MapVertex, MapVertex> parentMap) {
        LinkedList<GeographicPoint> path = new LinkedList<>();
        MapVertex currentVertex = goalVertex;
        path.addFirst(currentVertex.getLocation());
        // Till startVertex is reached continue adding parent of each vertex to path
        while (!currentVertex.equals(startVertex)) {
            currentVertex = parentMap.get(currentVertex);
            path.addFirst(currentVertex.getLocation());
        }
        return path;
    }


    /**
     * Find the path from start to goal using Dijkstra's algorithm
     *
     * @param start The starting location
     * @param goal  The goal location
     * @return The list of intersections that form the shortest path from
     * start to goal (including both start and goal).
     */
    public List<GeographicPoint> dijkstra(GeographicPoint start, GeographicPoint goal) {
        // Dummy variable for calling the search algorithms
        // You do not need to change this method.
        Consumer<GeographicPoint> temp = (x) -> {
        };
        return dijkstra(start, goal, temp);
    }

    /**
     * Find the path from start to goal using Dijkstra's algorithm
     *
     * @param start        The starting location
     * @param goal         The goal location
     * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
     * @return The list of intersections that form the shortest path from
     * start to goal (including both start and goal).
     */
    public List<GeographicPoint> dijkstra(GeographicPoint start,
                                          GeographicPoint goal, Consumer<GeographicPoint> nodeSearched) {
        // TODO: Implement this method in WEEK 4

        // Hook for visualization.  See writeup.
        //nodeSearched.accept(next.getLocation());

        return null;
    }

    /**
     * Find the path from start to goal using A-Star search
     *
     * @param start The starting location
     * @param goal  The goal location
     * @return The list of intersections that form the shortest path from
     * start to goal (including both start and goal).
     */
    public List<GeographicPoint> aStarSearch(GeographicPoint start, GeographicPoint goal) {
        // Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {
        };
        return aStarSearch(start, goal, temp);
    }

    /**
     * Find the path from start to goal using A-Star search
     *
     * @param start        The starting location
     * @param goal         The goal location
     * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
     * @return The list of intersections that form the shortest path from
     * start to goal (including both start and goal).
     */
    public List<GeographicPoint> aStarSearch(GeographicPoint start,
                                             GeographicPoint goal, Consumer<GeographicPoint> nodeSearched) {
        // TODO: Implement this method in WEEK 4

        // Hook for visualization.  See writeup.
        //nodeSearched.accept(next.getLocation());

        return null;
    }


    public static void main(String[] args) {
        System.out.print("Making a new map...");
        MapGraph firstMap = new MapGraph();
        System.out.print("DONE. \nLoading the map...");
        GraphLoader.loadRoadMap("data/testdata/simpletest.map", firstMap);
        GeographicPoint testStart = new GeographicPoint(1.0, 1.0);
        GeographicPoint testEnd = new GeographicPoint(8.0, -1.0);

        System.out.println(firstMap.bfs(testStart, testEnd));
        System.out.println("DONE.");

        // You can use this method for testing.


        /* Here are some test cases you should try before you attempt
         * the Week 3 End of Week Quiz, EVEN IF you score 100% on the
         * programming assignment.
         */
		/*
		MapGraph simpleTestMap = new MapGraph();
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", simpleTestMap);
		
		GeographicPoint testStart = new GeographicPoint(1.0, 1.0);
		GeographicPoint testEnd = new GeographicPoint(8.0, -1.0);
		
		System.out.println("Test 1 using simpletest: Dijkstra should be 9 and AStar should be 5");
		List<GeographicPoint> testroute = simpleTestMap.dijkstra(testStart,testEnd);
		List<GeographicPoint> testroute2 = simpleTestMap.aStarSearch(testStart,testEnd);
		
		
		MapGraph testMap = new MapGraph();
		GraphLoader.loadRoadMap("data/maps/utc.map", testMap);
		
		// A very simple test using real data
		testStart = new GeographicPoint(32.869423, -117.220917);
		testEnd = new GeographicPoint(32.869255, -117.216927);
		System.out.println("Test 2 using utc: Dijkstra should be 13 and AStar should be 5");
		testroute = testMap.dijkstra(testStart,testEnd);
		testroute2 = testMap.aStarSearch(testStart,testEnd);
		
		
		// A slightly more complex test using real data
		testStart = new GeographicPoint(32.8674388, -117.2190213);
		testEnd = new GeographicPoint(32.8697828, -117.2244506);
		System.out.println("Test 3 using utc: Dijkstra should be 37 and AStar should be 10");
		testroute = testMap.dijkstra(testStart,testEnd);
		testroute2 = testMap.aStarSearch(testStart,testEnd);
		*/


        /* Use this code in Week 3 End of Week Quiz */
		/*MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/maps/utc.map", theMap);
		System.out.println("DONE.");

		GeographicPoint start = new GeographicPoint(32.8648772, -117.2254046);
		GeographicPoint end = new GeographicPoint(32.8660691, -117.217393);
		
		
		List<GeographicPoint> route = theMap.dijkstra(start,end);
		List<GeographicPoint> route2 = theMap.aStarSearch(start,end);

		*/

    }

}
