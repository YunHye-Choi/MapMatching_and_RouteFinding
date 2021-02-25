import java.util.ArrayList;

public class Prediction {
    private static ArrayList<Double> prediction = new ArrayList<Double>();

    public Double angleDifference(Link a, Link b){

        RoadNetwork roadNetwork = new RoadNetwork();

        double startAX = roadNetwork.getNode(a.getStartNodeID()).getCoordinate().getX();
        double startAY = roadNetwork.getNode(a.getStartNodeID()).getCoordinate().getY();
        double endAX = roadNetwork.getNode(a.getEndNodeID()).getCoordinate().getX();
        double endAY = roadNetwork.getNode(a.getEndNodeID()).getCoordinate().getY();

        double startBX = roadNetwork.getNode(b.getStartNodeID()).getCoordinate().getX();
        double startBY = roadNetwork.getNode(b.getStartNodeID()).getCoordinate().getY();
        double endBX = roadNetwork.getNode(b.getEndNodeID()).getCoordinate().getX();
        double endBY = roadNetwork.getNode(b.getEndNodeID()).getCoordinate().getY();

        Vector2D vectorA = new Vector2D(startAX-endAX,startAY-endAY);
        Vector2D vectorB = new Vector2D(startBX-endBX,startBY-endBY);

        Double difference = vectorA.getAngle(vectorB);

        return difference;
    }
}
