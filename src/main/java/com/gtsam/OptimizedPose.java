package com.gtsam;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N6;
import edu.wpi.first.networktables.TimestampedObject;

/** This class represents the optimization solution of the GTSAM algorithm for a given timestep, dont worry, i will comment this out*/
public class OptimizedPose {
    
    private Pose3d optimizedPose;
    private Pose3d latencyCompensatedPose;
    private Vector<N6> stdDevs;
    private Vector<N3> stdDevs2d;
    private double timestamp;
    public OptimizedPose(TimestampedObject<Pose3d> optimizedPose, Transform3d odometryDelta, double[] stdDevs) {
        this.optimizedPose = optimizedPose.value;
        latencyCompensatedPose = this.optimizedPose.transformBy(odometryDelta);
        this.stdDevs = VecBuilder.fill(stdDevs[3], stdDevs[4], stdDevs[5], stdDevs[0], stdDevs[1], stdDevs[2]);
        stdDevs2d = VecBuilder.fill(stdDevs[3], stdDevs[4], stdDevs[2]);
        timestamp = optimizedPose.timestamp / 1e6;
    }

    public Pose3d getLatencyCompensatedPose() {
        return latencyCompensatedPose;
    }

    public Pose3d getLastResultPose() {
        return optimizedPose;
    }

    public Vector<N6> getStdDevs() {
        return stdDevs;
    }

    public Vector<N3> getStdDevs2d() {
        return stdDevs2d;
    }

    public double getTimestamp() {
        return timestamp;
    }  


}
