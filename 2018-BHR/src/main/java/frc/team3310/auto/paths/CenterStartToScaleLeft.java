package frc.team3310.auto.paths;

import java.util.ArrayList;

import frc.team3310.auto.PathBuilder;
import frc.team3310.auto.PathBuilder.Waypoint;
import frc.team3310.auto.PathContainer;
import frc.team3310.utility.control.Path;
import frc.team3310.utility.math.RigidTransform2d;
import frc.team3310.utility.math.Rotation2d;
import frc.team3310.utility.math.Translation2d;


public class CenterStartToScaleLeft implements PathContainer {
    
    @Override
    public Path buildPath() {
        ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        sWaypoints.add(new Waypoint(19,157,0,0));
        sWaypoints.add(new Waypoint(40,157,20,75));
        sWaypoints.add(new Waypoint(115,282,60,75));
        sWaypoints.add(new Waypoint(210,239,20,75,  "raiseElevator"));
        sWaypoints.add(new Waypoint(250,213,20,75));
        sWaypoints.add(new Waypoint(272,210,0,75));

        return PathBuilder.buildPathFromWaypoints(sWaypoints);
    }
    
    @Override
    public RigidTransform2d getStartPose() {
        return new RigidTransform2d(new Translation2d(19, 157), Rotation2d.fromDegrees(180.0)); 
    }

    @Override
    public boolean isReversed() {
        return true; 
    }
}