package org.firstinspires.ftc.teamcode;

public class BasicOdometry {
    int prevX, prevY, prevTheta;
    int currentX, currentY, currentTheta;
    private void setCurrentX(int x) {
        prevX = currentX;
        currentX = x;
    }
    private void setCurrentY(int y) {
        prevY = currentY;
        currentY = y;
    }
    private void setCurrentTheta(int theta) {
        prevTheta = currentTheta;
        currentTheta = theta;
    }
    public int[] getPose(int x, int y, int theta) {
        int[] pose = {};
        return pose;
    }
}