/*
 * Copyright (C) Photon Vision.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

package com.gtsam;

import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Num;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.*;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;
import org.ejml.simple.SimpleMatrix;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfInt;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.Point;
import org.opencv.core.Point3;
import org.opencv.core.Rect;
import org.opencv.core.RotatedRect;
import org.opencv.imgproc.Imgproc;


public final class OpenCVHelp {

    // Creating a cscore object is sufficient to load opencv, per
    // https://www.chiefdelphi.com/t/unsatisfied-link-error-when-simulating-java-robot-code-using-opencv/426731/4
    private static CvSink dummySink = null;

    public static void forceLoadOpenCV() {
        if (dummySink != null) return;
        dummySink = new CvSink("ignored");
        dummySink.close();
    }

    public static Mat matrixToMat(SimpleMatrix matrix) {
        var mat = new Mat(matrix.getNumRows(), matrix.getNumCols(), CvType.CV_64F);
        mat.put(0, 0, matrix.getDDRM().getData());
        return mat;
    }

    public static Matrix<Num, Num> matToMatrix(Mat mat) {
        double[] data = new double[(int) mat.total() * mat.channels()];
        var doubleMat = new Mat(mat.rows(), mat.cols(), CvType.CV_64F);
        mat.convertTo(doubleMat, CvType.CV_64F);
        doubleMat.get(0, 0, data);
        return new Matrix<>(new SimpleMatrix(mat.rows(), mat.cols(), true, data));
    }

}