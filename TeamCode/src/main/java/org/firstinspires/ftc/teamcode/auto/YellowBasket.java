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
    private Pose startPose = new Pose(9.244, 80.889);
    private Pose basketApproach = new Pose(16,128);
    private Pose basketScore = new Pose(12,132);
    PathChain toRung, toFirst, toScoreFirst, toSecond, toScoreSecond, toThird, toScoreThird, toPark;
    @Override
    public void runOpMode() throws InterruptedException {

        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);

        Slide slide = new Slide(hardwareMap);
        Intake intake = new Intake(hardwareMap);

        toRung = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(startPose),
                        new Point(basketApproach)
                ))
                .setLinearHeadingInterpolation(startPose.getHeading(), 3*Math.PI/2)
                .addPath(new BezierLine(
                        new Point(basketApproach),
                        new Point(basketScore)
                ))
                .setTangentHeadingInterpolation()
                .build();

        toFirst = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(basketScore),
                        new Point(16.711, 131.378, Point.CARTESIAN),
                        new Point(47.822, 92.444, Point.CARTESIAN),
                        new Point(44.800, 118.222, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(startPose.getHeading(), Math.PI/2)
                .build();

        toScoreFirst = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(44.800, 118.222, Point.CARTESIAN),
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
                        new Point(44.800, 87.289, Point.CARTESIAN),
                        new Point(45.511, 127.467, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(3*Math.PI/2, Math.PI/2)
                .build();

        toScoreSecond = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(45.511, 127.467, Point.CARTESIAN),
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
                        new Point(47.289, 90.489, Point.CARTESIAN),
                        new Point(45.689, 138.489, Point.CARTESIAN)
                ))
                .build();

        toScoreThird = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(45.689, 138.489, Point.CARTESIAN),
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
                        new Point(57.244, 87.644, Point.CARTESIAN)
                ))
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
                                        slide.HighBasket()
                                ),
                                intake.autoCycleWheelOut(),
                                new SleepAction(0.5),
                                intake.autoCycleWheelStop(),
                                new ParallelAction(
                                        slide.Base(),
                                        follower.follow(toFirst),
                                        intake.autoCycleWheelIn()
                                ),
                                intake.autoCycleWheelStop(),
                                new ParallelAction(
                                        follower.follow(toScoreFirst),
                                        slide.HighBasket()
                                ),
                                intake.autoCycleWheelOut(),
                                new SleepAction(0.5),
                                intake.autoCycleWheelStop(),
                                new ParallelAction(
                                        follower.follow(toSecond),
                                        slide.Base(),
                                        intake.autoCycleWheelIn()
                                ),
                                intake.autoCycleWheelStop(),
                                new ParallelAction(
                                        follower.follow(toScoreSecond),
                                        slide.HighBasket()
                                ),
                                intake.autoCycleWheelOut(),
                                new SleepAction(0.5),
                                intake.autoCycleWheelStop(),
                                new ParallelAction(
                                        follower.follow(toThird),
                                        slide.Base(),
                                        intake.autoCycleWheelIn()
                                ),
                                intake.autoCycleWheelStop(),
                                new ParallelAction(
                                        follower.follow(toScoreThird),
                                        slide.HighBasket()
                                ),
                                intake.autoCycleWheelOut(),
                                new SleepAction(0.5),
                                intake.autoCycleWheelStop(),
                                new ParallelAction(
                                        follower.follow(toPark),
                                        slide.HighBar()
                                )
                        )
                )
        );


    }
}
