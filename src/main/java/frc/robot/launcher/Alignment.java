package frc.robot.launcher;

import frc.robot.limelight.Distance;
import frc.robot.limelight.LimeLight;

class Alignment {
    private Distance distance;
    private LimeLight limeLight;
    private int pipeline;

    private double velocity;
    private double maxDisplacement;

    Alignment(Distance distance, LimeLight limeLight, int pipeline, double velocity) {
        this.distance = distance;
        this.limeLight = limeLight;
        this.pipeline = pipeline;
        this.velocity = velocity;

        maxDisplacement = getDistance(Math.atan(Math.pow(velocity, 2) / 9.81));
    }

    private double getTime(double theta) {
        if (theta < 0) {
            return -1;
        }
        if (theta > 90) {
            return -1;
        }

        return Math.sqrt(2 * distance.getHeight() / 9.81 + Math.pow(velocity * Math.sin(theta) / 9.81, 2))
                + velocity * Math.sin(theta) / 9.81;
    }

    private double getErrorAngle() {
        limeLight.selectPipeline(pipeline);
        if (limeLight.getTv() == 0) {
            return -1;
        }

        return distance.getAngle(limeLight.getTy());
    }

    private double getDistance(double theta) {
        if (theta < 0) {
            return -1;
        }
        if (theta > 90) {
            return -1;
        }

        return velocity * Math.cos(theta) * getTime(theta);
    }

    public double getAngle() {
        double theta = getErrorAngle();
        if (theta < 0) {
            return -1;
        }
        if (theta > 90) {
            return -1;
        }

        if (getDistance(theta) > maxDisplacement) {
            return -1;
        }

        return Math.acos(getDistance(theta) / (velocity * getTime(theta)));
    }
}