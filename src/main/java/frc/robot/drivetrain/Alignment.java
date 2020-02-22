package frc.robot.drivetrain;

import frc.robot.limelight.*;

class Alignment {
    private LimeLight limeLight;
    private int pipeline;

    Alignment(int pipeline) {
        limeLight = new LimeLight();

        this.pipeline = pipeline;
    }

    public boolean targetFound() {
        limeLight.selectPipeline(pipeline);

        if (limeLight.getTv() == 0) {
            return false;
        }
        return true;
    }

    public double getError() {
        if (targetFound()) {
            return -limeLight.getTx();
        }
        return -1;
    }
}