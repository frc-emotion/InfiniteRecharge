package frc.robot.drivetrain;

import frc.robot.limelight.*;

class Alignment {
    private LimeLight limeLight;
    private int pipeline;

    Alignment(int pipeline) {
        limeLight = new LimeLight();
        
        this.pipeline = pipeline;
    }

    public double getError() {
        limeLight.selectPipeline(pipeline);

        if (limeLight.getTv() == 0) {
            return -1;
        }

        return limeLight.getTx();
    }
}