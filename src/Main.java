import javafx.util.Pair;

import java.io.*;
import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Scanner;

public class Main {
    private static Emission emission = new Emission();
    private static Transition transition = new Transition();
    private static int wSize = 3; //윈도사이즈는 3!!!!!!

    public static void main(String[] args) throws IOException, InterruptedException {
        System.out.println("===== [YSY] Map-matching PilotTest 1-2 =====");
        int testNo = 4; // 여기만 바꿔주면 됨 (1-세정, 2-유네, 3-유림, 4-가중치)
        FileIO fileIO = new FileIO(testNo);
        // 파일에서 읽어와 도로네트워크 생성
        RoadNetwork roadNetwork = fileIO.generateRoadNetwork();

        ///////////// Transition probability matrix 구하기 (yh_tp)////////////////
        int n = roadNetwork.getLinksSize();
        double [][] tp_matrix = new double[n][n];
        for (int i = 0; i < n;i++) {
            // 여기에서 link[i]가 몇개의 link와 맞닿아있는지 int 변수 선언해서 저장
            int m = roadNetwork.getLink(i).nextLinksNum(roadNetwork);
            // 알고리즘대로 tp 지정
            for (int j = 0; j < n; j++) {
                if (i == j) tp_matrix[i][j] = 0.5;
                else if (roadNetwork.getLink(i).isLinkNextTo(roadNetwork, j))
                    tp_matrix[i][j] = 1.0/m;
                else tp_matrix[i][j] = 0.0;
            }
        }

        // Link와 Node를 바탕으로 Adjacent List 구축
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

        /* 여기부터 dijkstra~ ShortestRoute -> dijkstra */
        //before get map matching algorithm
        ShortestRoute shortestPath = new ShortestRoute();
        //시작, 끝 노드 ID!
        int start = 0;
        int end = 34;

        ArrayList<Integer> dijkstra_route = shortestPath.dijkstra(roadNetwork, heads, start, end); //dijkstra
        ArrayList<Integer> aStart_route = shortestPath.astar(roadNetwork, heads, start, end); //width A*
        ArrayList<Integer> llf_route = shortestPath.longest_leg_first(roadNetwork, heads, start, end); //longest leg first! (llf)
        ArrayList<Integer> ft_route = shortestPath.fewest_turn(roadNetwork, heads, start, end); //fewest turn

        ArrayList<Integer> result_route = dijkstra_route;
        for (int i = 0; i < result_route.size(); i++) {
            System.out.println(result_route.get(i) + " ");
        }

        // GPS points와 routePoints를 저장할 ArrayList생성
        ArrayList<GPSPoint> gpsPointArrayList = new ArrayList<>();
        ArrayList<Point> routePointArrayList; // 실제 경로의 points!
        ArrayList<Candidate> matchingCandiArrayList = new ArrayList<>();

        // test 번호에 맞는 routePoints생성
        routePointArrayList = roadNetwork.routePoints(testNo, result_route);


        // window size만큼의 t-window, ... , t-1, t에서의 candidates의 arrayList
        ArrayList<ArrayList<Candidate>> arrOfCandidates = new ArrayList<>();
        ArrayList<GPSPoint> subGPSs = new ArrayList<>();
        // ArrayList<Point> subRPA = new ArrayList<>(); // 비터비 내부 보려면 이것도 주석 해제해야! (subRoadPointArrayList)
        // GPSPoints 생성
        int timestamp = 0;
        //System.out.println("여기부터 생성된 gps point~~");
        System.out.println("Fixed Sliding Window Viterbi (window size: 3)");
        for (Point point : routePointArrayList) {
            GPSPoint gpsPoint = new GPSPoint(timestamp, point);
            gpsPointArrayList.add(gpsPoint);
            timestamp++;
            //System.out.println(gpsPoint); //gps point 제대로 생성 되는지 확인차 넣음
            ArrayList<Candidate> candidates = new ArrayList<>();
            candidates.addAll(Candidate.findRadiusCandidate(gpsPointArrayList, matchingCandiArrayList,
                    gpsPoint.getPoint(), 20, roadNetwork, timestamp,emission,transition));

            /////////matching print/////////////
            //System.out.println("매칭완료 " + matchingPointArrayList.get(timestamp-1));

            //System.out.println();

            emission.Emission_Median(matchingCandiArrayList.get(timestamp-1));
            if(timestamp > 1){
                transition.Transition_Median(matchingCandiArrayList.get(timestamp-1));
            }
            //median값 저장

            ///////////// FSW VITERBI /////////////
            subGPSs.add(gpsPoint);
            arrOfCandidates.add(candidates);
            // subRPA.add(point); // 비터비 내부 보려면 이것도 주석 해제해야!
            if (subGPSs.size() == wSize) {
                FSWViterbi.generateMatched_yhtp(wSize, arrOfCandidates, tp_matrix); // 윤혜tp 비터비
                FSWViterbi.generateMatched_sjtp(wSize, arrOfCandidates, gpsPointArrayList, transition, timestamp, roadNetwork); // 세정tp로 비터비
                subGPSs.clear();
                arrOfCandidates.clear();
                // subRPA.clear(); // 비터비 내부 보려면 이것도 주석 해제해야!
                subGPSs.add(gpsPoint);
                arrOfCandidates.add(candidates);
                // subRPA.add(point); // 비터비 내부 보려면 이것도 주석 해제해야!
            }
            ///////////////////////////////////////
        }
        // yhtp 이용해서 구한 subpath 출력
        FSWViterbi.printSubpath_yhtp (wSize);

        // sjtp 이용해서 구한 subpath 출력
        FSWViterbi.printSubpath_sjtp (wSize);

        // origin->생성 gps-> yhtp 이용해서 구한 matched 출력 및 정확도 확인
        FSWViterbi.test_data2_yhtp(routePointArrayList, gpsPointArrayList);

        // origin->생성 gps-> sjtp 이용해서 구한 matched 출력 및 정확도 확인
        FSWViterbi.test_data2_sjtp(routePointArrayList, gpsPointArrayList);

        // 윤혜tp와 세정tp비교!
        FSWViterbi.compareYhtpAndSjtp();
    }

    public static Double coordDistanceofPoints(Point a, Point b) {
        return Math.sqrt(Math.pow(a.getX() - b.getX(), 2) + Math.pow(a.getY() - b.getY(), 2));
    }//유클리드 거리 구하기

    // EP클래스 가서 캔디데이트 마다 값 구하고 저장
    public static void calculationEP(Candidate cand, Point center, int timestamp) {
        cand.setEp(emission.Emission_pro(cand, center, cand.getPoint(), timestamp)); //ep 구하기
        return;
    }

    // TP클래스 가서 캔디데이트 마다 값 구하고 저장
    public static void calculationTP(Candidate cand, ArrayList<Candidate> matchingPointArrayList, Point center, ArrayList<GPSPoint> gpsPointArrayList, int timestamp,  RoadNetwork roadNetwork) {
        if (timestamp == 1 || timestamp == 2) {
            cand.setTp(0);
            return;
        }
        Candidate matching_pre = matchingPointArrayList.get(timestamp - 2);
        cand.setTp(transition.Transition_pro(gpsPointArrayList.get(timestamp - 2).getPoint(), center, matching_pre, cand, roadNetwork)); //tp 구하기
        return;

    }

    // 곱해진 eptp저장하고 후보들 중 가장 높은 eptp를 가지는 후보를 matchingPointArrayList에 저장하고
    // tp median과 ep median을 저장
    public static Candidate calculationEPTP(ArrayList<Candidate> resultCandidate, ArrayList<Candidate> matchingPointArrayList, int timestamp) {
        Candidate matchingCandidate = new Candidate();

        if (timestamp == 2 || timestamp == 1) {
            double min_ep = 0;
            for (int i = 0; i < resultCandidate.size(); i++) {
                if (i == 0) {
                    min_ep = resultCandidate.get(i).getEp();
                    matchingCandidate = resultCandidate.get(i);
                } else if (min_ep > resultCandidate.get(i).getEp()) {
                    min_ep = resultCandidate.get(i).getEp();
                    matchingCandidate = resultCandidate.get(i);
                }
            }
            matchingPointArrayList.add(matchingCandidate);

            return matchingCandidate;
        }

        double maximum_tpep = 0;

        for(int i=0; i < resultCandidate.size(); i++){
            double tpep=0;
            tpep = resultCandidate.get(i).getEp() * resultCandidate.get(i).getTp();
            resultCandidate.get(i).setTpep(tpep);

            if(maximum_tpep < tpep){
                maximum_tpep = tpep;
                matchingCandidate = resultCandidate.get(i);

            }
        }
        matchingPointArrayList.add(matchingCandidate);

        return matchingCandidate;
    }


}