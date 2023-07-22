package org.firstinspires.ftc.teamcode.commandBased.classes;

import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.DecompositionSolver;
import org.apache.commons.math3.linear.LUDecomposition;
import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealMatrix;
import org.firstinspires.ftc.teamcode.commandBased.classes.util.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.commandBased.classes.util.geometry.Vector2d;

import java.util.ArrayList;
import java.util.List;

abstract class TwoWheelLocalizerAbstract {

    private Pose2d poseEstimate;
    private List<Double> lastWheelPositions;
    private double lastHeading;
    private Pose2d poseVelocity;
    private DecompositionSolver forwardSolver;

    public TwoWheelLocalizerAbstract(List<Pose2d> wheelPoses) {
        if (wheelPoses.size() != 2) {
            throw new IllegalArgumentException("Requires two wheel poses");
        }

        Array2DRowRealMatrix inverseMatrix = new Array2DRowRealMatrix(3, 3);

        for (int i = 0; i <= 1; i++) {
            Vector2d positionVector = wheelPoses.get(i).getVector();
            Vector2d orientationVector = wheelPoses.get(i).headingVec();
            inverseMatrix.setEntry(i, 0, orientationVector.getX());
            inverseMatrix.setEntry(i, 1, orientationVector.getY());
            inverseMatrix.setEntry(
                    i,
                    2,
                    positionVector.getX() * orientationVector.getY() -
                            positionVector.getY() * orientationVector.getX()
            );
        }
        inverseMatrix.setEntry(2, 2, 1.0);

        LUDecomposition decomp = new LUDecomposition(inverseMatrix);
        forwardSolver = decomp.getSolver();

        if (!forwardSolver.isNonSingular()) {
            throw new IllegalArgumentException("The specified configuration cannot support full localization");
        }
    }

    private Pose2d calculatePoseDelta(List<Double> wheelDeltas, double headingDelta) {
        double[][] deltas = new double[][]{{wheelDeltas.get(0), wheelDeltas.get(1), headingDelta}};

        RealMatrix rawPoseDelta = forwardSolver.solve(
                MatrixUtils.createRealMatrix(
                        deltas
                ).transpose()
        );

        return new Pose2d(
                rawPoseDelta.getEntry(0, 0),
                rawPoseDelta.getEntry(1, 0),
                rawPoseDelta.getEntry(2, 0)
        );
    }

    public void update() {
        List<Double> wheelPositions = getWheelPositions();
        double heading = getHeading();
        if (lastWheelPositions != null) {
            List<Double> wheelDeltas = new ArrayList<>();
            for (int i = 0; i <= wheelPositions.size(); i++) {
                wheelDeltas.add(wheelPositions.get(i) - lastWheelPositions.get(i));
            }
            double headingDelta = Angle.normDelta(heading - lastHeading);
            Pose2d robotPoseDelta = calculatePoseDelta(wheelDeltas, headingDelta);
            poseEstimate = Kinematics.relativeOdometryUpdate(poseEstimate, robotPoseDelta);
        }

        List<Double> wheelVelocities = getWheelVelocities();
        double headingVelocity = getHeadingVelocity();
        poseVelocity = calculatePoseDelta(wheelVelocities, headingVelocity);
        lastWheelPositions = wheelPositions;
        lastHeading = heading;
    }

    abstract List<Double> getWheelPositions();

    abstract List<Double> getWheelVelocities();

    abstract double getHeading();

    abstract double getHeadingVelocity();

}
