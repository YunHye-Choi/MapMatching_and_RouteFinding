import java.util.ArrayList;
import java.util.Arrays;

public class FSWViterbi {
    private static final ArrayList<Candidate[]> subpaths_yhtp = new ArrayList<>();
    private static final ArrayList<Candidate> matched_yhtp = new ArrayList<>();
    private static double correctness_yhtp;

    private static final ArrayList<Candidate[]> subpaths_sjtp = new ArrayList<>();
    private static final ArrayList<Candidate> matched_sjtp = new ArrayList<>();
    private static double correctness_sjtp;

    // gps 받아올때마다 FSW비터비로 매칭하는 메서드 -윤혜tp
    public static void generateMatched_yhtp(int wSize, ArrayList<ArrayList<Candidate>> arrOfCandidates,
                                            /*ArrayList<GPSPoint> subGPSs,*/ double[][] tp_matrix) {
        // arrOfCandidates를 순회하며 찾은 path의 마지막을 matching_success에 추가하는 loop
        // t는 timestamp를 의미
        // subpath 생성 및 matched arraylist에 저장
        // 현재 candidates와 다음 candidates로 가는 t.p와 e.p곱 중 최대 값을 가지는 curr와 그 index를 maximum_tpep[현재]에 저장
        double maximum_prob = 0;
        Candidate[] subpath = new Candidate[wSize-1]; // path의 길이를 t로 설정
        for (int i = 0; i < wSize - 1; i++) { // i moves in window
            ArrayList<Candidate> curr_candidates = arrOfCandidates.get(i);
            ArrayList<Candidate> next_candidates = arrOfCandidates.get(i+1);
            //System.out.println("☆origin point:" + subRPA.get(i+1));// 테스트 하려면 메서드 인자에 subGPSs추가해야함
            //System.out.println("☆GPS point: " + subGPSs.get(i));// 테스트 하려면 메서드 인자에 subRPA추가해야함
            // 다음 candidate를 하나씩 순회
            for (Candidate nc : next_candidates) {
                maximum_prob = 0;

                //System.out.println("  nc: " + nc.getPoint() + "/ ep: " + nc.getEp());
                // 현재 candidate를 하나씩 순회하며
                for (Candidate cc : curr_candidates) {
                    double tp = tp_matrix[cc.getInvolvedLink().getLinkID()][nc.getInvolvedLink().getLinkID()];
                    double prob = tp * nc.getEp();

                    //System.out.println("    cc: " + cc.getPoint() + "/ ep: " + cc.getEp() + "/ tp: " + tp + "/ prob: " + nc.getEp() * tp);

                    if (i == 0) { // window내 window의 시작 부분
                        if (maximum_prob < prob * cc.getEp()) { // 최대의 acc_prob를 갱신하며 이전전
                            maximum_prob = prob * cc.getEp();// window의 시작부분이므로 현재의 ep * 다음의 ep * 현재->다음의tp를 Acc_prob에 축적한다
                            nc.setPrev_index(curr_candidates.indexOf(cc));
                            nc.setAcc_prob(maximum_prob);
                            //System.out.println("    MAX!");
                        }
                    } else { // window 내 그 외의 부분
                        if (maximum_prob < prob * cc.getAcc_prob()) {
                            maximum_prob = prob * cc.getAcc_prob(); // 현재의 acc_prob * 다음의 ep * 현재->다음의 tp를 Acc_prob에 축적한다
                            nc.setPrev_index(curr_candidates.indexOf(cc));
                            nc.setAcc_prob(maximum_prob);
                            //System.out.println("    MAX!");
                        }
                    }
                }
            }/*
            if (t > arrOfCandidates.size() - wSize + 1) {
                //wSize = arrOfCandidates.size() - t + 1;
                break;
            }*/
        }

        // 마지막 candidates 중 acc_prob가 가장 높은 것 max_last_candi에 저장
        Candidate max_last_candi = new Candidate();
        double max_prob = 0;
        for (Candidate candidate : arrOfCandidates.get(wSize - 1)) {
            if (max_prob < candidate.getAcc_prob()) {
                max_prob = candidate.getAcc_prob();
                max_last_candi = candidate;
            }
        }
        // max_last_candi를 시작으로 back tracing하여 subpath구하기
        Candidate tempCandi = arrOfCandidates.get(wSize - 2).get(max_last_candi.getPrev_index());
        subpath[subpath.length - 1] = tempCandi;
        //int i = subpath.length - 1;
        for (int j = subpath.length - 2; j >= 0; j--) {
            subpath[j] = tempCandi;
            tempCandi = arrOfCandidates.get(j).get(tempCandi.getPrev_index());
        }

        // 생성된 subpath를 subpaths에 추가가
        subpaths_yhtp.add(subpath);
        ArrayList<Candidate> subpathArrayList = new ArrayList<>(Arrays.asList(subpath));
        // subpath를 모두 매칭!!
        matched_yhtp.addAll(subpathArrayList);
    }
    // subpath출력 메서드 (테스트용) -윤혜tp
    public static void printSubpath_yhtp(int wSize) {
        // subpath 출력
        int t = wSize - 2;
        for (Candidate[] subpath : subpaths_yhtp) {
            System.out.print(t + "] ");
            for (int i = 0; i < subpath.length; i++) {
                System.out.print("[" + subpath[i].getInvolvedLink().getLinkID() + "]");
                if (i != subpath.length - 1)
                    System.out.print(" ㅡ ");
            }
            System.out.println();
            t++;
        }
    }

    // pilottest1-2에서만 사용가능한 메서드 (테스트용) -윤혜tp
    public static void test_data2_yhtp(ArrayList<Point> routePointArrayList, ArrayList<GPSPoint> gpsPointArrayList) {
        // origin->생성 gps->matched 출력*
        double success_sum = 0;
        System.out.println("[Origin]\t->\t[GPS]\t->\t[Matched]");
        System.out.println("HERE!!:" + matched_yhtp.size());
        for (int i = 0; i < matched_yhtp.size(); i++) {
            System.out.println(i + " [" + routePointArrayList.get(i) + "] -> ["
                    + gpsPointArrayList.get(i).getPoint() + "] -> ["
                    + matched_yhtp.get(i).getPoint() + ", id: "
                    + matched_yhtp.get(i).getInvolvedLink().getLinkID() + "]");

            if (i >= 0 && i <= 19 && matched_yhtp.get(i).getInvolvedLink().getLinkID() == 0) {
                success_sum++;
            } else if (i >= 20 && i <= 40 && matched_yhtp.get(i).getInvolvedLink().getLinkID() == 3) {
                success_sum++;
            } else if (i >= 41 && i <= 61 && matched_yhtp.get(i).getInvolvedLink().getLinkID() == 13) {
                success_sum++;
            } else if (i >= 62 && i <= 82 && matched_yhtp.get(i).getInvolvedLink().getLinkID() == 25) {
                success_sum++;
            } else if (i >= 83 && i <= 103 && matched_yhtp.get(i).getInvolvedLink().getLinkID() == 46) {
                success_sum++;
            } else if (i >= 104 && i <= 124 && matched_yhtp.get(i).getInvolvedLink().getLinkID() == 48) {
                success_sum++;
            } else if (i >= 125 && i <= 145 && matched_yhtp.get(i).getInvolvedLink().getLinkID() == 52) {
                success_sum++;
            } else if (i >= 146 && i <= 165 && matched_yhtp.get(i).getInvolvedLink().getLinkID() == 58) {
                success_sum++;
            }

        }
        correctness_yhtp = (100 * (success_sum / (double) matched_yhtp.size()));
        System.out.println("Correctness(yhtp) = " + correctness_yhtp);
        System.out.println(" Total: " + matched_yhtp.size() + "\n Succeed: " + success_sum + "\n Failed: " + (matched_yhtp.size() - success_sum));

    }

    // gps 받아올때마다 FSW비터비로 매칭하는 메서드 -세정tp
    public static void generateMatched_sjtp(int wSize, ArrayList<ArrayList<Candidate>> arrOfCandidates,
                                            ArrayList<GPSPoint> gpsPointArrayList, Transition transition, int timeStamp, RoadNetwork roadNetwork) {
        // arrOfCandidates를 순회하며 찾은 path의 마지막을 matching_success에 추가하는 loop
        // t는 timestamp를 의미
        // subpath 생성 및 matched arraylist에 저장
        // 현재 candidates와 다음 candidates로 가는 t.p와 e.p곱 중 최대 값을 가지는 curr와 그 index를 maximum_tpep[현재]에 저장
        double maximum_prob = 0;
        Candidate[] subpath = new Candidate[wSize-1]; // path의 길이를 t로 설정
        for (int i = 0; i < wSize - 1; i++) { // i moves in window
            ArrayList<Candidate> curr_candidates = arrOfCandidates.get(i);
            ArrayList<Candidate> next_candidates = arrOfCandidates.get(i+1);
            //System.out.println("☆origin point:" + subRPA.get(i+1)); // 테스트 하려면 메서드 인자에 subRPA추가해야함
            //System.out.println("☆GPS point: " + subGPSs.get(i)); // 테스트 하려면 메서드 인자에 subGPS추가해야함
            // 다음 candidate를 하나씩 순회
            for (Candidate nc : next_candidates) {
                maximum_prob = 0;

                //System.out.println("  nc: " + nc.getPoint() + "/ ep: " + nc.getEp());
                // 현재 candidate를 하나씩 순회하며
                for (Candidate cc : curr_candidates) {
                    double tp = transition.Transition_pro(gpsPointArrayList.get(timeStamp-1).getPoint(), gpsPointArrayList.get(timeStamp-3).getPoint(), cc, nc, roadNetwork);
                    double prob = tp * nc.getEp();

                    //System.out.println("    cc: " + cc.getPoint() + "/ ep: " + cc.getEp() + "/ tp: " + tp + "/ prob: " + nc.getEp() * tp);

                    if (i == 0) { // window내 window의 시작 부분
                        if (maximum_prob < prob * cc.getEp()) { // 최대의 acc_prob를 갱신하며 이전전
                            maximum_prob = prob * cc.getEp();// window의 시작부분이므로 현재의 ep * 다음의 ep * 현재->다음의tp를 Acc_prob에 축적한다
                            nc.setPrev_index(curr_candidates.indexOf(cc));
                            nc.setAcc_prob(maximum_prob);
                            //System.out.println("    MAX!");
                        }
                    } else { // window 내 그 외의 부분
                        if (maximum_prob < prob * cc.getAcc_prob()) {
                            maximum_prob = prob * cc.getAcc_prob(); // 현재의 acc_prob * 다음의 ep * 현재->다음의 tp를 Acc_prob에 축적한다
                            nc.setPrev_index(curr_candidates.indexOf(cc));
                            nc.setAcc_prob(maximum_prob);
                            //System.out.println("    MAX!");
                        }
                    }
                }
            }/*
            if (t > arrOfCandidates.size() - wSize + 1) {
                //wSize = arrOfCandidates.size() - t + 1;
                break;
            }*/
        }

        // 마지막 candidates 중 acc_prob가 가장 높은 것 max_last_candi에 저장
        Candidate max_last_candi = new Candidate();
        double max_prob = 0;
        for (Candidate candidate : arrOfCandidates.get(wSize - 1)) {
            if (max_prob < candidate.getAcc_prob()) {
                max_prob = candidate.getAcc_prob();
                max_last_candi = candidate;
            }
        }
        // max_last_candi를 시작으로 back tracing하여 subpath구하기
        Candidate tempCandi = arrOfCandidates.get(wSize - 2).get(max_last_candi.getPrev_index());
        subpath[subpath.length - 1] = tempCandi;
        //int i = subpath.length - 1;
        for (int j = subpath.length - 2; j >= 0; j--) {
            subpath[j] = tempCandi;
            tempCandi = arrOfCandidates.get(j).get(tempCandi.getPrev_index());
        }

        // 생성된 subpath를 subpaths에 추가가
        subpaths_sjtp.add(subpath);
        ArrayList<Candidate> subpathArrayList = new ArrayList<>(Arrays.asList(subpath));
        // subpath를 모두 매칭!!
        matched_sjtp.addAll(subpathArrayList);
    }
    // subpath출력 메서드 (테스트용) -세정tp
    public static void printSubpath_sjtp(int wSize) {
        // subpath 출력
        int t = wSize - 2;
        for (Candidate[] subpath : subpaths_sjtp) {
            System.out.print(t + "] ");
            for (int i = 0; i < subpath.length; i++) {
                System.out.print("[" + subpath[i].getInvolvedLink().getLinkID() + "]");
                if (i != subpath.length - 1)
                    System.out.print(" ㅡ ");
            }
            System.out.println();
            t++;
        }
    }

    // pilottest1-2에서만 사용가능한 메서드 (테스트용) -세정tp
    public static void test_data2_sjtp(ArrayList<Point> routePointArrayList, ArrayList<GPSPoint> gpsPointArrayList) {
        // origin->생성 gps->matched 출력*
        double success_sum = 0;
        System.out.println("[Origin]\t->\t[GPS]\t->\t[Matched]");
        System.out.println("HERE!!:" + matched_sjtp.size());
        for (int i = 0; i < matched_sjtp.size(); i++) {
            System.out.println(i + " [" + routePointArrayList.get(i) + "] -> ["
                    + gpsPointArrayList.get(i).getPoint() + "] -> ["
                    + matched_sjtp.get(i).getPoint() + ", id: "
                    + matched_sjtp.get(i).getInvolvedLink().getLinkID() + "]");

            if (i >= 0 && i <= 19 && matched_sjtp.get(i).getInvolvedLink().getLinkID() == 0) {
                success_sum++;
            } else if (i >= 20 && i <= 40 && matched_sjtp.get(i).getInvolvedLink().getLinkID() == 3) {
                success_sum++;
            } else if (i >= 41 && i <= 61 && matched_sjtp.get(i).getInvolvedLink().getLinkID() == 13) {
                success_sum++;
            } else if (i >= 62 && i <= 82 && matched_sjtp.get(i).getInvolvedLink().getLinkID() == 25) {
                success_sum++;
            } else if (i >= 83 && i <= 103 && matched_sjtp.get(i).getInvolvedLink().getLinkID() == 46) {
                success_sum++;
            } else if (i >= 104 && i <= 124 && matched_sjtp.get(i).getInvolvedLink().getLinkID() == 48) {
                success_sum++;
            } else if (i >= 125 && i <= 145 && matched_sjtp.get(i).getInvolvedLink().getLinkID() == 52) {
                success_sum++;
            } else if (i >= 146 && i <= 165 && matched_sjtp.get(i).getInvolvedLink().getLinkID() == 58) {
                success_sum++;
            }

        }
        correctness_sjtp = (100 * (success_sum / (double) matched_sjtp.size()));
        System.out.println("Correctness(sjtp) = " + correctness_sjtp);
        System.out.println(" Total: " + matched_sjtp.size() + "\n Succeed: " + success_sum + "\n Failed: " + (matched_sjtp.size() - success_sum));
    }

    // yhtp와 sjtp 비교용 메서드 (테스트용)
    public static void compareYhtpAndSjtp () {
        System.out.println("========= Compare correctness =========\nCorrectness(yhtp) = " + correctness_yhtp);
        System.out.println("Correctness(sjtp) = " + correctness_sjtp+"\n=======================================");
    }
}
