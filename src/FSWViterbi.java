import java.util.ArrayList;
import java.util.Arrays;

public class FSWViterbi {
    private static ArrayList<Candidate[]> subpaths_yhtp;
    private static ArrayList<Candidate> matched_yhtp;

    private static ArrayList<Candidate[]> subpaths_sjtp;
    private static ArrayList<Candidate> matched_sjtp;

    public static void generateMatched_yhtp (int wSize, ArrayList<ArrayList<Candidate>> arrOfCandidates, ArrayList<Point> routePointArrayList,
             ArrayList<GPSPoint> gpsPointArrayList, double [][] tp_matrix) {
        subpaths_yhtp = new ArrayList<>();
        // arrOfCandidates를 순회하며 찾은 path의 마지막을 matching_success에 추가하는 loop
        // t는 timestamp를 의미
        // subpath 생성 및 matched arraylist에 저장
        matched_yhtp = new ArrayList<>();
        for (int t = wSize-1; t < arrOfCandidates.size(); t+=wSize-1) {
            double maximum_prob = 0;
            Candidate[] subpath = new Candidate[wSize-1]; // path의 길이를 t로 설정

            // 현재 candidates와 다음 candidates로 가는 t.p와 e.p곱 중 최대 값을 가지는 curr와 그 index를 maximum_tpep[현재]에 저장
            for (int i = t - wSize + 1; i < t; i++) { // i moves in window
                ArrayList<Candidate> curr_candidates = arrOfCandidates.get(i);
                ArrayList<Candidate> next_candidates = arrOfCandidates.get(i+1);
                System.out.println("☆origin point:" + routePointArrayList.get(i));
                System.out.println("☆GPS point: " + gpsPointArrayList.get(i));
                // 다음 candidate를 하나씩 순회
                for (Candidate nc : next_candidates) {
                    maximum_prob = 0;

                    System.out.println("  nc: "+nc.getPoint()+"/ ep: "+nc.getEp());
                    // 현재 candidate를 하나씩 순회하며
                    for (Candidate cc : curr_candidates) {
                        double tp = tp_matrix[cc.getInvolvedLink().getLinkID()][nc.getInvolvedLink().getLinkID()];
                        double prob = tp * nc.getEp(); /*cc->nc로의 tp구해야하고 */

                        System.out.println("    cc: "+cc.getPoint()+"/ ep: "+cc.getEp()+ "/ tp: " + tp + "/ prob: "+nc.getEp()*tp);

                        if (i == t - wSize + 1) { // window내 window의 시작 부분
                            if(maximum_prob < prob * cc.getEp()) { // 최대의 acc_prob를 갱신하며 이전전
                                maximum_prob = prob * cc.getEp();// window의 시작부분이므로 현재의 ep * 다음의 ep * 현재->다음의tp를 Acc_prob에 축적한다
                                nc.setPrev_index(curr_candidates.indexOf(cc));
                                nc.setAcc_prob(maximum_prob);
                                //System.out.println("    MAX!");
                            }
                        }
                        else { // window 내 그 외의 부분
                            if(maximum_prob < prob * cc.getAcc_prob()) {
                                maximum_prob = prob * cc.getAcc_prob(); // 현재의 acc_prob * 다음의 ep * 현재->다음의 tp를 Acc_prob에 축적한다
                                nc.setPrev_index(curr_candidates.indexOf(cc));
                                nc.setAcc_prob(maximum_prob);
                                //System.out.println("    MAX!");
                            }
                        }
                    }
                }
                if (t > arrOfCandidates.size() - wSize + 1) {
                    //wSize = arrOfCandidates.size() - t + 1;
                    break;
                }
            }

            // 마지막 candidates 중 acc_prob가 가장 높은 것 max_last_candi에 저장
            Candidate max_last_candi = new Candidate();
            double max_prob = 0;
            for(Candidate candidate : arrOfCandidates.get(t)) {
                if (max_prob < candidate.getAcc_prob()) {
                    max_prob = candidate.getAcc_prob();
                    max_last_candi = candidate;
                }
            }
            // max_last_candi를 시작으로 back tracing하여 subpath구하기
            Candidate tempCandi = arrOfCandidates.get(t-1).get(max_last_candi.getPrev_index());
            subpath[wSize-2] = tempCandi;
            int _t = t-2;
            for (int j = wSize-3; j>=0; j--) {
                tempCandi = arrOfCandidates.get(_t--).get(tempCandi.getPrev_index());
                subpath[j] = tempCandi;
            }

            // 생성된 subpath를 subpaths에 추가가
            subpaths_yhtp.add(subpath);
            ArrayList<Candidate> subpathArrayList = new ArrayList<Candidate>(Arrays.asList(subpath));
            // subpath를 모두 매칭!!
            matched_yhtp.addAll(subpathArrayList);
            if (t > arrOfCandidates.size() - wSize + 1) {
                break;
            }
        }
    }
    public static void printSubpath_yhtp (int wSize) {
        // subpath 출력
        int t = wSize-2;
        for (Candidate[] subpath : subpaths_yhtp) {
            System.out.print(t + "] ");
            for (int  i=0;i<subpath.length;i++) {
                System.out.print("["+subpath[i].getInvolvedLink().getLinkID() + "]");
                if (i!=subpath.length-1)
                    System.out.print(" ㅡ ");
            }
            System.out.println(); t++;
        }
    }
    public static void test_data2_yhtp (ArrayList<Point> routePointArrayList, ArrayList<GPSPoint> gpsPointArrayList) {
        // origin->생성 gps->matched 출력*
        double success_sum= 0;
        System.out.println("[Origin]\t->\t[GPS]\t->\t[Matched]");
        System.out.println("HERE!!:" + matched_yhtp.size());
        for(int i = 0; i< matched_yhtp.size() ; i++){
            System.out.println(i +" [" + routePointArrayList.get(i) + "] -> ["
                    + gpsPointArrayList.get(i).getPoint() + "] -> ["
                    + matched_yhtp.get(i).getPoint() + ", id: "
                    + matched_yhtp.get(i).getInvolvedLink().getLinkID()+ "]");

            if (i >=0 && i <= 19 && matched_yhtp.get(i).getInvolvedLink().getLinkID() == 0){
                success_sum ++;
            } else if (i >= 20 && i <=40 && matched_yhtp.get(i).getInvolvedLink().getLinkID() == 3) {
                success_sum ++;
            } else if (i >= 41 && i <= 61 && matched_yhtp.get(i).getInvolvedLink().getLinkID() == 13) {
                success_sum ++;
            } else if (i >= 62 && i <= 82 && matched_yhtp.get(i).getInvolvedLink().getLinkID() == 25) {
                success_sum ++;
            } else if (i >= 83 && i <= 103 && matched_yhtp.get(i).getInvolvedLink().getLinkID() == 46) {
                success_sum ++;
            } else if (i >= 104 && i <= 124 && matched_yhtp.get(i).getInvolvedLink().getLinkID() == 48) {
                success_sum ++;
            } else if (i >= 125 && i <= 145 && matched_yhtp.get(i).getInvolvedLink().getLinkID() == 52) {
                success_sum ++;
            } else if (i >= 146 && i <= 165 && matched_yhtp.get(i).getInvolvedLink().getLinkID() == 58) {
                success_sum ++;
            }

        }
        System.out.println("Success prob = "+(100*(success_sum/(double) matched_yhtp.size()))/* + "%"*/);
        System.out.println(" Total: "+ matched_yhtp.size() +"\n Succeed: "+success_sum+ "\n Failed: "+(matched_yhtp.size()-success_sum));

    }
}
