package frc.robot.launcher;

import frc.robot.limelight.Distance;
import frc.robot.limelight.LimeLight;

class Alignment {
    private Pivot pivot;
    private Distance distance;
    private LimeLight limeLight;
    private int pipeline;

    private double velocity;
    private double maxDisplacement;

    Alignment(Pivot pivot, Distance distance, LimeLight limeLight, int pipeline, double velocity) {
        this.pivot = pivot;
        this.distance = distance;
        this.limeLight = limeLight;
        this.pipeline = pipeline;
        this.velocity = velocity;

        maxDisplacement = velocity * Math.cos(45) * getTime(45);
    }

    public double getTime(double theta) {
        if (theta < 0) {
            return -1
        }
        if (theta > 90) {
            return -1
        }
        return Math.sqrt(2*-9.81*distance.getHeight() + Math.pow(velocity * Math.sin(45), 2)) + velocity * Math.sin(theta);
    }

    public double getAngle() {
        limeLight.selectPipeline(pipeline);
        if (limeLight.getTv() == 0) {
            return -1;
        }

        displacement = distance.getDistance(limeLight.getTy());

    }
}