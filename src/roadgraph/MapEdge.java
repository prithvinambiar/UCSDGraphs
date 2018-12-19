package roadgraph;

import java.util.Objects;

/**
 * @author Prithvi
 * <p>
 * A class which represents an edge between two vertices
 */
class MapEdge {
    private final MapVertex source;
    private final MapVertex destination;
    private final String edgeName;
    private final String edgeType;

    /**
     * Create a new MapEdge
     */
    MapEdge(MapVertex fromVertex, MapVertex sourceVertex, String roadName, String roadType) {
        this.source = fromVertex;
        this.destination = sourceVertex;
        this.edgeName = roadName;
        this.edgeType = roadType;
    }

    /**
     * Return destination vertex
     * @return Destination vertex
     */
    MapVertex getDestination() {
        return destination;
    }

    /**
     * Returns {@code true} if the argument is equal to current edge
     * and {@code false} otherwise.
     * @param o an object to be compared for equality
     * @return {@code true} if the argument is equal to current edge
     * and {@code false} otherwise
     */
    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;
        MapEdge mapEdge = (MapEdge) o;
        return Objects.equals(source, mapEdge.source) &&
                Objects.equals(destination, mapEdge.destination) &&
                Objects.equals(edgeName, mapEdge.edgeName) &&
                Objects.equals(edgeType, mapEdge.edgeType);
    }

    /**
     * Returns the hashcode for this <code>MapEdge</code>.
     * @return a hash code for this <code>MapEdge</code>.
     */
    @Override
    public int hashCode() {
        return Objects.hash(source, destination, edgeName, edgeType);
    }
}
