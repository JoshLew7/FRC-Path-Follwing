package frc.team3310.auto.test;

import java.util.ArrayList;

import frc.team3310.auto.PathBuilder;
import frc.team3310.auto.PathBuilder.Waypoint;
import frc.team3310.auto.PathContainer;
import frc.team3310.utility.control.Path;
import frc.team3310.utility.math.RigidTransform2d;
import frc.team3310.utility.math.Rotation2d;
import frc.team3310.utility.math.Translation2d;

public class RightTurnRadiusStart90 implements PathContainer {
    
    @Override
    public Path buildPath() {
        ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        sWaypoints.add(new Waypoint(60,100,0,0));
        sWaypoints.add(new Waypoint(60,60,30,20));
        sWaypoints.add(new Waypoint(20,60,0,20));

        return PathBuilder.buildPathFromWaypoints(sWaypoints);
    }
    
    @Override
    public RigidTransform2d getStartPose() {
        return new RigidTransform2d(new Translation2d(60, 100), Rotation2d.fromDegrees(-90.0)); 
    }

    @Override
    public boolean isReversed() {
        return false; 
    }
}