package frc.team3310.auto.paths;

import java.util.ArrayList;

import frc.team3310.auto.PathBuilder;
import frc.team3310.auto.PathBuilder.Waypoint;
import frc.team3310.auto.PathContainer;
import frc.team3310.utility.control.Path;
import frc.team3310.utility.math.RigidTransform2d;
import frc.team3310.utility.math.Rotation2d;
import frc.team3310.utility.math.Translation2d;


public class ScaleRightToSwitchRight implements PathContainer {
    
    @Override
    public Path buildPath() {
        ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        sWaypoints.add(new Waypoint(287,85,0,40));
        sWaypoints.add(new Waypoint(234,91,0,40));
        sWaypoints.add(new Waypoint(214,94,0,20));

        return PathBuilder.buildPathFromWaypoints(sWaypoints);
    }
    
    @Override
    public RigidTransform2d getStartPose() {
        return new RigidTransform2d(new Translation2d(287, 85), Rotation2d.fromDegrees(180.0)); 
    }

    @Override
    public boolean isReversed() {
        return false; 
    }
}