package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.config.Config;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;

import org.firstinspires.ftc.teamcode.config.Intake;
import org.firstinspires.ftc.teamcode.config.Slide;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

@Config
@Autonomous(name = "YellowBasket")
public class YellowBasket extends LinearOpMode {

    Follower follower;
    Slide slide;
    Intake intake;
    private Pose startPose = new Pose(9.244, 104.889);
    private Pose basketApproach = new Pose(26,118);
    private Pose basketScore = new Pose(19,125);

    private double blockXLine = 41.00;
    private double blockXLine2 = 45.000;
    private double blockXLine3 = 45.000;
    PathChain toRung, toFirst, toFirsthalf, toScoreFirst, toSecond, toSecondhalf, toScoreSecond, toThird, toThirdhalf,toScoreThird, toPark;
    @Override
    public void runOpMode() throws InterruptedException {

        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);

        Slide slide = new Slide(hardwareMap);
        Intake intake = new Intake(hardwareMap);

        toRung = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(startPose),
                        new Point(30, 100, Point.CARTESIAN),
                        new Point(basketApproach)
                ))
                .setTangentHeadingInterpolation()
                .addPath(new BezierLine(
                        new Point(basketApproach),
                        new Point(basketScore)
                ))
                .setTangentHeadingInterpolation()
                .build();

        toFirst = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(basketScore),
                        new Point(2, 122, Point.CARTESIAN),
                        new Point(47, 102.5, Point.CARTESIAN),
                        new Point(blockXLine, 111.222, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(startPose.getHeading(), Math.PI/2)
                .addPath(new BezierLine(
                        new Point(blockXLine, 111.222, Point.CARTESIAN),
                        new Point(blockXLine, 113.222, Point.CARTESIAN)
                ))
                .build();

        toFirsthalf = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(blockXLine, 111.222, Point.CARTESIAN),
                        new Point(blockXLine, 113.222, Point.CARTESIAN)
                ))
                .setTangentHeadingInterpolation()
                .build();


        toScoreFirst = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(42.200, 113.222, Point.CARTESIAN),
                        new Point(basketApproach)
                ))
                .setLinearHeadingInterpolation(Math.PI/2, 3*Math.PI/4)
                .addPath(new BezierLine(
                        new Point(basketApproach),
                        new Point(basketScore)
                ))
                .setTangentHeadingInterpolation()
                .build();

        toSecond = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(basketScore),
                        new Point(44.800, 100, Point.CARTESIAN),
                        new Point(blockXLine2, 114, Point.CARTESIAN)
                ))
                .setConstantHeadingInterpolation(Math.PI/2)
                .addPath(new BezierLine(
                        new Point(blockXLine2, 114, Point.CARTESIAN),
                        new Point(blockXLine2, 123, Point.CARTESIAN)
                ))
                .setTangentHeadingInterpolation()
                .build();

        toSecondhalf = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(blockXLine2, 114, Point.CARTESIAN),
                        new Point(blockXLine2, 123, Point.CARTESIAN)
                ))
                .setTangentHeadingInterpolation()
                .build();

        toScoreSecond = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(blockXLine2, 123, Point.CARTESIAN),
                        new Point(basketApproach)
                ))
                .setLinearHeadingInterpolation(Math.PI/2, 3*Math.PI/4)
                .addPath(new BezierLine(
                        new Point(basketApproach),
                        new Point(basketScore)
                ))
                .setTangentHeadingInterpolation()
                .build();

        toThird = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(basketScore),
                        new Point(47.289, 110.489, Point.CARTESIAN),
                        new Point(blockXLine3, 120, Point.CARTESIAN)
                ))
                .setConstantHeadingInterpolation(Math.PI/2)
                .addPath(new BezierLine(
                        new Point(blockXLine3, 120, Point.CARTESIAN),
                        new Point(blockXLine3, 130, Point.CARTESIAN)
                ))
                .build();

        toThirdhalf = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(blockXLine3, 120, Point.CARTESIAN),
                        new Point(blockXLine3, 130, Point.CARTESIAN)
                ))
                .setTangentHeadingInterpolation()
                .build();

        toScoreThird = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(blockXLine3, 130, Point.CARTESIAN),
                        new Point(basketApproach)
                ))
                .setLinearHeadingInterpolation(Math.PI/2, 3*Math.PI/4)
                .addPath(new BezierLine(
                        new Point(basketApproach),
                        new Point(basketScore)
                ))
                .setTangentHeadingInterpolation()
                .build();

        toPark = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(basketScore),
                        new Point(64.000, 125.156, Point.CARTESIAN),
                        new Point(62.244, 90.644, Point.CARTESIAN)
                ))
                .build();


        waitForStart();
        if (isStopRequested()) return;

        follower.setMaxPower(0.75);

        Actions.runBlocking(
                new ParallelAction(
                        slide.slideToPosition(),
                        intake.autoSetWrist(),
                        intake.autoIntake(),
                        intake.ClawMovement(),
                        intake.ClawClosed(),
                        follower.followerUpdate(),
                        new SequentialAction(
                                intake.autoSetWristDrop(),
                                intake.ClawClosed(),
                                new SleepAction(1),
                                new ParallelAction(
                                        follower.follow(toRung),
                                        slide.HighBasket(),
                                        intake.autoSetWristDrop()
                                ),
                                new SleepAction(0.5),
                                intake.autoCycleWheelOut(),
                                new SleepAction(0.5),
                                intake.autoCycleWheelStop(),
                                new ParallelAction(
                                        slide.Base(),
                                        follower.follow(toFirst),
                                        intake.autoSetWristFlat(),
                                        intake.autoCycleWheelIn()
                                ),
                                follower.follow(toFirsthalf),
                                intake.autoCycleWheelStop(),
                                new ParallelAction(
                                        follower.follow(toScoreFirst),
                                        intake.autoSetWristDrop(),
                                        slide.HighBasket()
                                ),
                                new SleepAction(0.5),
                                intake.autoCycleWheelOut(),
                                new SleepAction(0.5),
                                intake.autoCycleWheelStop(),
                                new ParallelAction(
                                        follower.follow(toSecond),
                                        slide.Base(),
                                        intake.autoCycleWheelIn(),
                                        intake.autoSetWristFlat()
                                ),
                                follower.follow(toSecondhalf),
                                intake.autoCycleWheelStop(),
                                new ParallelAction(
                                        follower.follow(toScoreSecond),
                                        intake.autoSetWristDrop(),
                                        slide.HighBasket()
                                ),
                                intake.autoCycleWheelOut(),
                                new SleepAction(0.5),
                                intake.autoCycleWheelStop(),
                                new ParallelAction(
                                        follower.follow(toThird),
                                        slide.Base(),
                                        intake.autoSetWristFlat(),
                                        intake.autoCycleWheelIn()
                                ),
                                follower.follow(toThirdhalf),
                                intake.autoCycleWheelStop(),
                                new ParallelAction(
                                        follower.follow(toScoreThird),
                                        slide.HighBasket(),
                                        intake.autoSetWristDrop()
                                ),
                                intake.autoCycleWheelOut(),
                                new SleepAction(0.5),
                                intake.autoCycleWheelStop(),
                                new ParallelAction(
                                        follower.follow(toPark),
                                        slide.HighBar(),
                                        intake.autoSetWristDrop()
                                )
                        )
                )
        );


    }
}
