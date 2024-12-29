package org.firstinspires.ftc.teamcode.teleop;

import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.leftFrontMotorName;
import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.leftRearMotorName;
import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.rightFrontMotorName;
import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.rightRearMotorName;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.config.BetterBoolGamepad;
import org.firstinspires.ftc.teamcode.config.Intake;
import org.firstinspires.ftc.teamcode.config.IntakeV2;
import org.firstinspires.ftc.teamcode.config.Pivot;
import org.firstinspires.ftc.teamcode.config.Slide;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;

@Config
@TeleOp(name = "MMMM", group = "Drive")
public class MMMM extends OpMode {

    Follower follower;
    Slide slide;
    IntakeV2 intake;
    Pivot pivot;
    private DcMotorEx leftFront;
    private DcMotorEx leftRear;
    private DcMotorEx rightFront;
    private DcMotorEx rightRear;

    BetterBoolGamepad bGamepad1, bGamepad2;
    public double speedMod = 0.5;
    public double speedModTwo = 0.25;
    public double speedModTurn = 0.5;

    public double speedModSlide = 1;

    public static int intakeCycleDirection = 0;
    public static int flipDrive = -1;

    @Override
    public void init() {

        follower = new Follower(hardwareMap);

        leftFront = hardwareMap.get(DcMotorEx.class, leftFrontMotorName);
        leftRear = hardwareMap.get(DcMotorEx.class, leftRearMotorName);
        rightRear = hardwareMap.get(DcMotorEx.class, rightRearMotorName);
        rightFront = hardwareMap.get(DcMotorEx.class, rightFrontMotorName);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Slide slide = new Slide(hardwareMap);
        IntakeV2 intake = new IntakeV2(hardwareMap);
        Pivot pivot = new Pivot(hardwareMap);

        follower.startTeleopDrive();

        bGamepad2 = new BetterBoolGamepad(gamepad2);
        bGamepad1 = new BetterBoolGamepad(gamepad1);

    }

    @Override
    public void loop() {
        Slide slide = new Slide(hardwareMap);
        IntakeV2 intake = new IntakeV2(hardwareMap);

        if (gamepad1.right_bumper) {
            speedMod = 0.25;
            speedModTurn = 0.25;
        }
        else {
            if (gamepad1.right_trigger > 0.5) speedMod = gamepad1.right_trigger;
            else if (speedMod < 0.5) {
                speedMod = 0.5;
            }
            if (gamepad1.left_trigger > 0.5) speedModTurn = gamepad1.left_trigger;
            else if (speedModTurn < 0.5) {
                speedModTurn = 0.5;
            }
        }

        speedMod = gamepad1.right_bumper ? 0.25 : (gamepad1.right_trigger>0.5 ? gamepad1.right_trigger : 0.5);

        follower.setTeleOpMovementVectors(
                        gamepad1.right_stick_y * speedMod * flipDrive,
                        gamepad1.right_stick_x * speedMod * flipDrive,
                        gamepad1.left_stick_x * speedModTurn * flipDrive
        );
        follower.update();

        //Davids Control
        //Cycler
        if (gamepad2.right_trigger >= 0.25) {intakeCycleDirection = 1;}
        else if ((gamepad2.right_trigger < 0.25)) {intakeCycleDirection = 0;}
        intake.cycleCycler(intakeCycleDirection);

        //Wrist / Gate
        if (gamepad2.x) {intake.teleWrist(intake.wristIntake);}
        if (gamepad2.b) {intake.teleGate(intake.openGate);}
        else {intake.teleGate(intake.closeGate);}

        //Claw
        if (gamepad2.left_trigger >= 0.25) {intake.claw.setPosition(intake.openClaw);}
        else if (gamepad2.left_trigger < 0.25) {intake.claw.setPosition(intake.closeClaw);}

        //Pivot
        if (!gamepad2.dpad_down && !gamepad2.dpad_up) {
            pivot.setManual(gamepad2.right_stick_y);
            pivot.setTargetPivotPosition(pivot.rightPivot.getCurrentPosition());
        }
        else if (gamepad2.dpad_down && !gamepad2.dpad_up) {
            pivot.setTargetPivotPosition(pivot.pivotHover);
        }
        else if (!gamepad2.dpad_down && gamepad2.dpad_up) {
            pivot.setTargetPivotPosition(pivot.pivotDrop);
        }
        pivot.pivotPositionTele();

        //Slide
        if (!gamepad2.y && !gamepad2.a) {
            slide.setCentralLiftPower(-gamepad2.left_stick_y);
            slide.setTargetPosition(slide.slide.getTargetPosition());
        } else if (gamepad2.y && !gamepad2.a) {
            slide.setTargetPosition(slide.slideIntake);
        } else if (!gamepad2.y && gamepad2.a) {
            slide.setTargetPosition(slide.base);
            intake.teleWrist(intake.wristHide);
        }
        slide.setCentralLift();

        telemetry.addData("Speedmod:", speedMod);
        telemetry.addData("SpeedmodTwo:", speedModTwo);
        telemetry.addData("Flip Drive:", flipDrive);
        telemetry.addData("intakeCycleDirection:", intakeCycleDirection);
        telemetry.addData("Slide Position:", slide.slide.getCurrentPosition());
        telemetry.addData("Pivot Position:", pivot.rightPivot.getCurrentPosition());
        telemetry.update();
    }


}

