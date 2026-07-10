package org.sciborgs1155.robot.climb;

import org.sciborgs1155.robot.Robot;


public class Climb implements AutoCloseable {
    private final ClimbIO hardware;

    public static Climb create() {
        return Robot.isReal() ? new Climb(new RealClimb()) : new Climb(new SimClimb());
    }

    public Climb(ClimbIO hardware) {
        this.hardware = hardware;
    }



    }


    

