package com.gtsam.detections;

import java.util.Objects;

public class TargetCorner {  
    public double x;
    public double y;

    public TargetCorner(double cx, double cy) {
        this.x = cx;
        this.y = cy;
    }

    public TargetCorner() {
        this(0, 0);
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;
        TargetCorner that = (TargetCorner) o;
        return Double.compare(that.x, x) == 0 && Double.compare(that.y, y) == 0;
    }

    @Override
    public int hashCode() {
        return Objects.hash(x, y);
    }

    @Override
    public String toString() {
        return "(" + x + "," + y + ')';
    }
}