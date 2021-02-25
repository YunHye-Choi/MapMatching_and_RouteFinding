import javafx.util.Pair;

import java.util.ArrayList;
import java.util.List;

public class AdjacentNode {
    private Node node;
    private Link adjacentLink;
    private AdjacentNode nextNode;
    private int weight;

    public AdjacentNode(Node node){
        this.node=node;
        adjacentLink=null;
        nextNode=null;
        weight=0;
    }

    public AdjacentNode(Node node, Link adjacentLink){
        this.node=node;
        this.adjacentLink=adjacentLink;
        nextNode=null;
        weight=0;
    }

    public AdjacentNode(Node node, Link adjacentLink,AdjacentNode nextNode){
        this.node=node;
        this.adjacentLink=adjacentLink;
        this.nextNode=nextNode;
        weight=0;
    }

    public void setNextNode(AdjacentNode nextNode){
        this.nextNode=nextNode;
    }

    public AdjacentNode getNextNode(){return nextNode;}

    public Link getLink(){return adjacentLink;}

    public Node getNode() { return node; }

    // 윤혜가 추가한 출력서식 (그냥 확인용으로 쓰세용)

    @Override
    public String toString() {
        return "nextNode: "+ getNextNode().getNode().getNodeID() + ", "
                + "weight: "+ getNextNode().getLink().getWeight() + " | ";
    }

    public Link getAdjacentLink() {
        return adjacentLink;
    }

    public static ArrayList<AdjacentNode> ridAdjacentNode(RoadNetwork roadNetwork){
        ArrayList<AdjacentNode> heads = new ArrayList<>();
        for (int i = 0; i < roadNetwork.nodeArrayList.size(); i++) {
            AdjacentNode headNode = new AdjacentNode(roadNetwork.nodeArrayList.get(i));
            heads.add(headNode);

            List<Pair<Link, Integer>> adjacentLink = roadNetwork.getLink1(headNode.getNode().getNodeID());
            if (adjacentLink.size() == 0) continue;
            AdjacentNode ptr = headNode;
            for (int j = 0; j < adjacentLink.size(); j++) {
                AdjacentNode addNode = new AdjacentNode(roadNetwork.getNode(adjacentLink.get(j).getValue()), adjacentLink.get(j).getKey());
                ptr.setNextNode(addNode);
                ptr = ptr.getNextNode();
            }
        }
        return heads;
    }

}