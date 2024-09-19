package com.gtsam.detections;

import java.util.List;

public class TagDetection {
    public final int id;
    public final List<TargetCorner> corners;

    public TagDetection(int id, List<TargetCorner> corners) {
        this.id = id;
        this.corners = corners;
    }

    public static final TagDetectionStruct struct = new TagDetectionStruct();
}