package org.firstinspires.ftc.teamcode.auto;

import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.leftFrontMotorName;
import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.leftRearMotorName;
import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.rightFrontMotorName;
import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.rightRearMotorName;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.config.Intake;
import org.firstinspires.ftc.teamcode.config.Slide;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.opencv.core.Mat;

@Config
@Autonomous(name = "FiveClip")
public class FiveClip extends LinearOpMode {

    Follower follower;
    Slide slide;
    Intake intake;
    private DcMotorEx leftFront;
    private DcMotorEx leftRear;
    private DcMotorEx rightFront;
    private DcMotorEx rightRear;

    private Pose startPose = new Pose(9.244, 65.422);
    private Pose highRung = new Pose(39.500, 65.422);

    private double backDistance = 30;
    private Pose block1 = new Pose(44.444,26.667);
    private Pose block2 = new Pose(44.444, 16.167);
    private Pose block3 = new Pose(44.444, 6.22);
    private Pose grabPoint = new Pose(15, 40.000, Math.PI/2);

    private Pose grabPoint2 = new Pose(10, 40.000, Math.PI/2);

    private Pose grabSide = new Pose(15.26, 18.244);


    PathChain toRung, pushBlocks, toClip2, toGrab3, toClip3, toGrab4, toClip4, toGrab5, toClip5, toPark;

    @Override
    public void runOpMode() throws InterruptedException {

        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);

        Slide slide = new Slide(hardwareMap);
        Intake intake = new Intake(hardwareMap);

        toRung = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(startPose),
                        new Point(highRung)
                ))
                .setTangentHeadingInterpolation()
                .build();

        pushBlocks = follower.pathBuilder()
                //to block 1
                .addPath(new BezierCurve(
                        new Point(highRung),
                        new Point(18.133, 18.489, Point.CARTESIAN),
                        new Point(59.378,47.111,Point.CARTESIAN),
                        new Point(block1.getX() + 10.5, block1.getY() + 2,Point.CARTESIAN)))
                .setConstantHeadingInterpolation(highRung.getHeading())
                //push block 1
                .addPath(new BezierLine(
                        new Point(block1.getX() + 10.5, block1.getY() + 2,Point.CARTESIAN),
                        new Point(backDistance, 26.844, Point.CARTESIAN)))
                .setTangentHeadingInterpolation()
                .setReversed(true)
                //to block 2
                .addPath(new BezierCurve(
                        new Point(backDistance, 26.844, Point.CARTESIAN),
                        new Point(58.000, 29.333, Point.CARTESIAN),
                        new Point(block2.getX() + 12.5, block2.getY() + 4, Point.CARTESIAN)))
                .setConstantHeadingInterpolation(highRung.getHeading())
                //push block 2
                .addPath(new BezierLine(
                        new Point(block2.getX() + 12.5, block2.getY() + 4,Point.CARTESIAN),
                        new Point(backDistance, 20.167, Point.CARTESIAN)))
                .setTangentHeadingInterpolation()
                .setReversed(true)
                //to block 3
                .addPath(new BezierCurve(
                        new Point(backDistance, 20.167, Point.CARTESIAN),
                        new Point(59.743, 21.167, Point.CARTESIAN),
                        new Point(block3.getX() + 12.5, block3.getY() + 7, Point.CARTESIAN)))
                .setConstantHeadingInterpolation(highRung.getHeading())
                //push block 3
                .addPath(new BezierLine(
                        new Point(block3.getX() + 12.5, block3.getY() + 7,Point.CARTESIAN),
                        new Point(backDistance, 13.167, Point.CARTESIAN)))
                .setTangentHeadingInterpolation()
                .setReversed(true)
                .addPath(new BezierLine(
                        new Point(backDistance, 13.167, Point.CARTESIAN),
                        new Point(grabSide)
                ))
                .setLinearHeadingInterpolation(highRung.getHeading(), 3*Math.PI/2)
                .addPath(new BezierLine(
                        new Point(grabSide),
                        new Point(grabSide.getX(), grabSide.getY()-6)
                ))
                .setTangentHeadingInterpolation()
                .build();

        toClip2 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(grabSide.getX(), grabSide.getY()-5),
                        new Point(24, 69, Point.CARTESIAN),
                        new Point(highRung.getX()-1.2, highRung.getY()+0.5)
                ))
                .setLinearHeadingInterpolation(3*Math.PI/2, highRung.getHeading())
                .build();

        toGrab3 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(highRung.getX(), highRung.getY()+0.5),
                        new Point(grabPoint)
                ))
                .setLinearHeadingInterpolation(highRung.getHeading(), Math.PI)
                .addPath(new BezierLine(
                        new Point(grabPoint),
                        new Point(grabPoint2)
                ))
                .setTangentHeadingInterpolation()
                .build();

        toClip3 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(grabPoint2),
                        new Point(highRung.getX(), highRung.getY()+1)
                ))
                .setLinearHeadingInterpolation(Math.PI, highRung.getHeading())
                .build();

        toGrab4 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(highRung.getX(), highRung.getY()+1),
                        new Point(grabPoint)
                ))
                .setLinearHeadingInterpolation(highRung.getHeading(), Math.PI)
                .addPath(new BezierLine(
                        new Point(grabPoint),
                        new Point(grabPoint2)
                ))
                .setTangentHeadingInterpolation()
                .build();

        toClip4 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(grabPoint2),
                        new Point(highRung.getX(), highRung.getY()+1.5)
                ))
                .setLinearHeadingInterpolation(Math.PI, highRung.getHeading())
                .build();

        toGrab5 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(highRung.getX(), highRung.getY()+1.5),
                        new Point(grabPoint)
                ))
                .setLinearHeadingInterpolation(highRung.getHeading(), Math.PI)
                .addPath(new BezierLine(
                        new Point(grabPoint),
                        new Point(grabPoint2)
                ))
                .setTangentHeadingInterpolation()
                .build();

        toClip5 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(grabPoint2),
                        new Point(highRung.getX(), highRung.getY()+2)
                ))
                .setLinearHeadingInterpolation(Math.PI, highRung.getHeading())
                .build();

        toPark = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(highRung.getX(), highRung.getY()+2),
                        new Point(grabPoint2.getX(), grabPoint2.getY()-10)
                ))
                .setConstantHeadingInterpolation(highRung.getHeading())
                .build();


        waitForStart();
        if (isStopRequested()) return;
        
        Actions.runBlocking(
                new ParallelAction(
                        slide.slideToPosition(),
                        intake.autoSetWrist(),
                        intake.autoIntake(),
                        intake.ClawMovement(),
                        intake.ClawClosed(),
                        follower.followerUpdate(),
                        new SequentialAction(
                                intake.autoSetWristHide(),
                                intake.ClawClosed(),
                                new ParallelAction(
                                        follower.follow(toRung),
                                        slide.HighBar()
                                ),
                                slide.UnClip(),
                                new SleepAction(0.1),
                                intake.ClawOpen(),
                                new ParallelAction(
                                        slide.Base(),
                                        follower.follow(pushBlocks)
                                ),
                                intake.ClawClosed(),
                                new SleepAction(0.5),
                                new ParallelAction(
                                        slide.HighBar(),
                                        follower.follow(toClip2)
                                ),
                                slide.UnClip(),
                                new SleepAction(0.1),
                                intake.ClawOpen(),
                                new ParallelAction(
                                        slide.Base(),
                                        follower.follow(toGrab3)
                                ),
                                intake.ClawClosed(),
                                new SleepAction(0.175),
                                new ParallelAction(
                                        slide.HighBar(),
                                        follower.follow(toClip3)
                                ),
                                slide.UnClip(),
                                new SleepAction(0.1),
                                intake.ClawOpen(),
                                new ParallelAction(
                                        slide.Base(),
                                        follower.follow(toGrab4)
                                ),
                                intake.ClawClosed(),
                                new SleepAction(0.175),
                                new ParallelAction(
                                        slide.HighBar(),
                                        follower.follow(toClip4)
                                ),
                                slide.UnClip(),
                                new SleepAction(0.1),
                                intake.ClawOpen(),
                                new ParallelAction(
                                        slide.Base(),
                                        follower.follow(toGrab5)
                                ),
                                intake.ClawClosed(),
                                new SleepAction(0.175),
                                new ParallelAction(
                                        slide.HighBar(),
                                        follower.follow(toClip5)
                                ),
                                slide.UnClip(),
                                new SleepAction(0.1),
                                intake.ClawOpen(),
                                new ParallelAction(
                                        slide.Base(),
                                        follower.follow(toPark)
                                )
                        )
                )
        );


    }
}
