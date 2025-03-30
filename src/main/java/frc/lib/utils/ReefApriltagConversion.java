package frc.lib.utils;

public class ReefApriltagConversion {
    public ReefApriltagConversion() {

    }

    public static double convertApriltagToAngle(int apriltag) {
        double angle = 0;

        if (apriltag == 6 || apriltag == 19) {
            angle = -30;
        } else if (apriltag == 7 || apriltag == 18) {
            angle = 0;
        } else if (apriltag == 8 || apriltag == 17) {
            angle = 30;
        } else if (apriltag == 9 || apriltag == 22) {
            angle = 120;
        } else if (apriltag == 10 || apriltag == 21) {
            angle = 180;
        } else if (apriltag == 11 || apriltag == 20) {
            angle = -120;
        }
        System.out.println(angle);
        return angle;
    }
}
