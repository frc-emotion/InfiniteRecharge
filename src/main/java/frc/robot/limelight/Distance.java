package frc.robot.limelight;

public class Distance {
    private double mountingHeight, refrenceHeight;  // Height relative to the ground in meters
    private double mountingAngle; // Angle relative to ground plane in degrees

    Distance(double mountingHeight, double mountingAngle, double refrenceHeight) {
        this.mountingHeight = mountingHeight;
        this.mountingAngle = mountingAngle;
        this.refrenceHeight = refrenceHeight;
    }

    public double getDistance(double ty) {
        return (refrenceHeight - mountingHeight) / Math.tan(ty + mountingAngle);
    }

    public double getHeight() {
        return refrenceHeight - mountingHeight;
    }
}