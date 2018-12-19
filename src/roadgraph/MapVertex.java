package roadgraph;

import geography.GeographicPoint;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;

/**
 * @author Prithvi
 * <p>
 * A class which represents a geographic point as a vertex
 * It also captures list of out edges from this vertex
 * The geographic point represents an intersection
 */

class MapVertex {
    private final GeographicPoint geographicPoint;
    private final List<MapEdge> mapEdges;

    /**
     * Create a new MapVertex
     */
    MapVertex(GeographicPoint geographicPoint) {
        this.geographicPoint = geographicPoint;
        this.mapEdges = new ArrayList<>();
    }

    /**
     * Get geographic location
     * @return The GeographicPoint
     */
    GeographicPoint getLocation() {
        return this.geographicPoint;
    }

    /**
     * Add an outgoing edge
     * @param mapEdge Represents the edge
     */
    void addEdge(MapEdge mapEdge) {
        this.mapEdges.add(mapEdge);
    }

    /**
     * Returns all out neighbours for this vertex
     * @return The list of out neighbour vertices
     */
    List<MapVertex> findNeighbours() {
        List<MapVertex> neighbours = new ArrayList<>();
        for (MapEdge mapEdge : this.mapEdges) {
            neighbours.add(mapEdge.getDestination());
        }
        return neighbours;
    }

    /**
     * Returns {@code true} if the argument is equal to current vertex
     * and {@code false} otherwise.
     * @param o an object to be compared for equality
     * @return {@code true} if the argument is equal to current vertex
     * and {@code false} otherwise
     */
    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;
        MapVertex mapVertex = (MapVertex) o;
        return Objects.equals(this.geographicPoint, mapVertex.geographicPoint) && this.mapEdges.equals(mapVertex.mapEdges);
    }

    /**
     * Returns the hashcode for this <code>MapVertex</code>.
     * @return a hash code for this <code>MapVertex</code>.
     */
    @Override
    public int hashCode() {
        return Objects.hash(this.geographicPoint.hashCode());
    }
}
