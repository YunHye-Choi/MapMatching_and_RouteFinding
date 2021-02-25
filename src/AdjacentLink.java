import javafx.util.Pair;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.List;

public class AdjacentLink {
    private Link link;
    private AdjacentLink nextLink;
    private boolean head;
    private int num;

    public AdjacentLink(){
        this.link=null;
        this.nextLink=null;
        this.head=false;
        this.num=0;
    }

    public AdjacentLink(Link link, AdjacentLink nextLink, boolean head, int num){
        this.link=link;
        this.nextLink=nextLink;
        this.head=head;
        this.num=num;
    }

    public Link getLink() {
        return link;
    }

    public void setLink(Link link) {
        this.link = link;
    }

    public AdjacentLink getNextLink() {
        return nextLink;
    }

    public void setNextLink(AdjacentLink nextLink) {
        this.nextLink = nextLink;
    }

    public boolean isHead() {
        return head;
    }

    public void setHead(boolean head) {
        this.head = head;
    }

    public int getNum() {
        return num;
    }

    public void setNum(int num) {
        this.num = num;
    }

    public static ArrayList<AdjacentLink> ridAdjacentLink(RoadNetwork roadNetwork){
        ArrayList<AdjacentLink> lheads = new ArrayList<>();
        for (int i = 0; i < roadNetwork.linkArrayList.size(); i++) {
            AdjacentLink headLink = new AdjacentLink(roadNetwork.linkArrayList.get(i),null,true,0);
            lheads.add(headLink);

            ArrayList<Link> adjacentLink = roadNetwork.giveNodegetLink(headLink.getLink());
            if (adjacentLink.size() == 0) continue;
            headLink.setNum(adjacentLink.size());
            AdjacentLink ptr = headLink;
            for (int j = 0; j < adjacentLink.size(); j++) {
                AdjacentLink addLink = new AdjacentLink(adjacentLink.get(i),null,false,0);
                ptr.setNextLink(addLink);
                ptr = ptr.getNextLink();
            }
        }
        return lheads;
    }
}
