package frc.robot;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.geometry.Pose2d;

class AutoOptions {
    private ArrayList<Character> validColors; // List of valid color values
    private Pose2d currentStartingLocation; // Specific starting location for this match

    AutoOptions(ArrayList<ArrayList<Pose2d>> startingLocations, ArrayList<Character> validColors) {
        this.validColors = validColors;

        // Determine starting location from FMS
        Alliance currentAlliance = DriverStation.getInstance().getAlliance();

        switch (DriverStation.getInstance().getLocation()) {
        case 1:
            if (currentAlliance.equals(Alliance.Blue)) {
                currentStartingLocation = startingLocations.get(0).get(0);
            } else {
                currentStartingLocation = startingLocations.get(1).get(0);
            }
        case 2:
            if (currentAlliance.equals(Alliance.Blue)) {
                currentStartingLocation = startingLocations.get(0).get(1);
            } else {
                currentStartingLocation = startingLocations.get(1).get(1);
            }
        case 3:
            if (currentAlliance.equals(Alliance.Blue)) {
                currentStartingLocation = startingLocations.get(0).get(2);
            } else {
                currentStartingLocation = startingLocations.get(1).get(2);
            }
        default:
            return;
        }
    }

    public Pose2d getStartingLocation() {
        return currentStartingLocation;
    }

    public char getRequestedColor() {
        String gameData = DriverStation.getInstance().getGameSpecificMessage();
        if (gameData.length() > 0) {
            if (validColors.contains(gameData.charAt(0))) {
                return gameData.charAt(0);
            }
        }
        return 'N';
    }
}