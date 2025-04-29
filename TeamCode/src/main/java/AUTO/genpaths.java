package AUTO;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;
@Autonomous(name = "Generated Paths Auto", group = "Auto")

public class genpaths  extends OpMode{

    private Follower follower;
    private Timer pathTimer;
    private int pathState;
    public static PathBuilder builder = new PathBuilder();
    private final Pose startPose = new Pose(4.853932584269662, 8.537549407114623, Math.toRadians(0));  // Starting position
    public PathChain line1, line2, line3, line4, line5, line6, line7;

    public  void buildPaths (){


          PathChain line1 = builder
                .addPath(
                        new BezierLine(
                                new Point(4.854, 2.225, Point.CARTESIAN),
                                new Point(42.472, 2.022, Point.CARTESIAN)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();

          PathChain line2 = builder
                .addPath(
                        new BezierCurve(
                                new Point(42.472, 2.022, Point.CARTESIAN),
                                new Point(5.056, 5.663, Point.CARTESIAN),
                                new Point(4.854, 129.034, Point.CARTESIAN)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();

          PathChain line3 = builder
                .addPath(
                        new BezierLine(
                                new Point(4.854, 129.034, Point.CARTESIAN),
                                new Point(43.079, 12.337, Point.CARTESIAN)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();

          PathChain line4 = builder
                .addPath(
                        new BezierCurve(
                                new Point(43.079, 12.337, Point.CARTESIAN),
                                new Point(16.382, 33.169, Point.CARTESIAN),
                                new Point(8.292, 129.438, Point.CARTESIAN)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();

          PathChain line5 = builder
                .addPath(
                        new BezierLine(
                                new Point(8.292, 129.438, Point.CARTESIAN),
                                new Point(43.685, 22.652, Point.CARTESIAN)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();

          PathChain line6 = builder
                .addPath(
                        new BezierCurve(
                                new Point(43.685, 22.652, Point.CARTESIAN),
                                new Point(56.427, 57.438, Point.CARTESIAN),
                                new Point(13.753, 133.281, Point.CARTESIAN)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();

          PathChain line7 = builder
                .addPath(
                        new BezierLine(
                                new Point(13.753, 133.281, Point.CARTESIAN),
                                new Point(6.472, 4.449, Point.CARTESIAN)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0: // Move from start to scoring position
                follower.followPath(line1);
                setPathState(1);
                break;

            case 1: // Wait until the robot is near the scoring position
                if (!follower.isBusy()) {
                    follower.followPath(line2, true);
                    setPathState(2);
                }
                break;

            case 2: // Wait until the robot is near the first sample pickup position
                if (!follower.isBusy()) {
                    follower.followPath(line3, true);
                    setPathState(3);
                }
                break;

            case 3: // Wait until the robot returns to the scoring position
                if (!follower.isBusy()) {
                    follower.followPath(line4, true);
                    setPathState(4);
                }
                break;

            case 4: // Wait until the robot is near the second sample pickup position
                if (!follower.isBusy()) {
                    follower.followPath(line5, true);
                    setPathState(5);
                }
                break;

            case 5: // Wait until the robot returns to the scoring position
                if (!follower.isBusy()) {
                    follower.followPath(line6, true);
                    setPathState(6);
                }
                break;

            case 6: // Wait until the robot is near the third sample pickup position
                if (!follower.isBusy()) {
                    follower.followPath(line7, true);
                    setPathState(7);
                }
                break;

            case 7: // Wait until the robot returns to the scoring position
                if (!follower.isBusy()) {
                    setPathState(-1); // End the autonomous routine
                }
                break;

                }

        }

    public void setPathState(int pState) {
    pathState = pState;
            pathTimer.resetTimer();
        }

        @Override
    public void init() {
        pathTimer = new Timer();
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);
         buildPaths();
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();
        telemetry.addData("Path State", pathState);
        telemetry.addData("Position", follower.getPose().toString());
        telemetry.update();
    }
}
