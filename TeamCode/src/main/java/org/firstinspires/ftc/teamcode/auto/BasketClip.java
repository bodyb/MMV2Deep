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

import java.time.format.TextStyle;

@Config
@Autonomous(name = "BasketClip")
public class BasketClip extends LinearOpMode {

    Follower follower;
    Slide slide;
    Intake intake;
    private Pose startPose = new Pose(9.244, 80.889);
    private Pose highRung = new Pose(39.500, 80.889);
    private Pose basketApproach = new Pose(24,120);
    private Pose basketScore = new Pose(20,124);

    private double blockXLine = 42.200;
    private double blockXLine2 = 45.800;
    private double blockXLine3 = 47.800;
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
                        new Point(highRung)
                ))
                .setTangentHeadingInterpolation()
                .build();

        toFirst = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(highRung),
                        new Point(2, 122, Point.CARTESIAN),
                        new Point(47, 102.5, Point.CARTESIAN),
                        new Point(42.200, 111.222, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(startPose.getHeading(), Math.PI/2)
                .addPath(new BezierLine(
                        new Point(42.200, 111.222, Point.CARTESIAN),
                        new Point(42.200, 113.222, Point.CARTESIAN)
                ))
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
                        new Point(blockXLine2, 118, Point.CARTESIAN)
                ))
                .setConstantHeadingInterpolation(Math.PI/2)
                .addPath(new BezierLine(
                        new Point(blockXLine2, 118, Point.CARTESIAN),
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
                        new Point(blockXLine3, 126, Point.CARTESIAN)
                ))
                .setConstantHeadingInterpolation(Math.PI/2)
                .addPath(new BezierLine(
                        new Point(blockXLine3, 126, Point.CARTESIAN),
                        new Point(blockXLine3, 130, Point.CARTESIAN)
                ))
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
                                        follower.follow(toFirst),
                                        intake.autoSetWristFlat(),
                                        intake.autoCycleWheelIn()
                                ),
                                intake.autoCycleWheelStop(),
                                new ParallelAction(
                                        follower.follow(toScoreFirst),
                                        intake.autoSetWristDrop(),
                                        slide.HighBasket()
                                ),
                                intake.autoCycleWheelOut(),
                                new SleepAction(0.5),
                                intake.autoCycleWheelStop(),
                                new ParallelAction(
                                        follower.follow(toSecond),
                                        slide.Base(),
                                        intake.autoCycleWheelIn(),
                                        intake.autoSetWristFlat()
                                ),
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
