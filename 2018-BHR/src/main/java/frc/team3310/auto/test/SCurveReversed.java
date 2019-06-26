package frc.team3310.auto.test;

import java.util.ArrayList;

import frc.team3310.auto.PathBuilder;
import frc.team3310.auto.PathBuilder.Waypoint;
import frc.team3310.auto.PathContainer;
import frc.team3310.utility.control.Path;
import frc.team3310.utility.math.RigidTransform2d;
import frc.team3310.utility.math.Rotation2d;
import frc.team3310.utility.math.Translation2d;

public class SCurveReversed implements PathContainer {
    
    @Override
    public Path buildPath() {
        ArrayList<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        sWaypoints.add(new Waypoint( 20,  0,  0,  0));
        sWaypoints.add(new Waypoint( 60,  0, 20, 20));
        sWaypoints.add(new Waypoint(100, 15, 20, 20));
        sWaypoints.add(new Waypoint(130, 15,  0, 20));

        return PathBuilder.buildPathFromWaypoints(sWaypoints);
    }
    
    @Override
    public RigidTransform2d getStartPose() {
        return new RigidTransform2d(new Translation2d(20, 0), Rotation2d.fromDegrees(180.0)); 
    }

    @Override
    public boolean isReversed() {
        return true; 
    }
	// WAYPOINT_DATA: [{"position":{"x":20,"y":60},"speed":0,"radius":0,"comment":""},{"position":{"x":60,"y":60},"speed":20,"radius":20,"comment":""},{"position":{"x":100,"y":75},"speed":20,"radius":20,"comment":""},{"position":{"x":130,"y":75},"speed":20,"radius":0,"comment":""}]
	// IS_REVERSED: true
	// FILE_NAME: SCurveReversed
}