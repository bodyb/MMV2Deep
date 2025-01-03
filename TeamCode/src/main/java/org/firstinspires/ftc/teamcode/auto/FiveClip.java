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
import org.firstinspires.ftc.teamcode.config.IntakeV2;
import org.firstinspires.ftc.teamcode.config.Pivot;
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
    IntakeV2 intake;
    Pivot pivot;
    private Pose startPose = new Pose(9.244, 65.422);
    private Pose highRung = new Pose(39.500, 65.422);
    PathChain toRung, pushBlocks, toClip2, toGrab3, toClip3, toGrab4, toClip4, toGrab5, toClip5, toPark;

    @Override
    public void runOpMode() throws InterruptedException {

        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);

        Slide slide = new Slide(hardwareMap);
        IntakeV2 intake = new IntakeV2(hardwareMap);

        toRung = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(startPose),
                        new Point(highRung)
                ))
                .setTangentHeadingInterpolation()
                .build();

        waitForStart();
        if (isStopRequested()) return;
        
        Actions.runBlocking(
                new ParallelAction(
                        slide.slideToPosition(),
                        intake.moveClaw(),
                        intake.moveWrist(),
                        pivot.pivotToPosition(),
                        follower.followerUpdate(),
                        new SequentialAction(
                                follower.follow(toRung),
                                pivot.setPivot(pivot.pivotDrop),
                                slide.HighBar(),
                                intake.setClaw(intake.closeClaw)
                        )
                )
        );


    }
}
