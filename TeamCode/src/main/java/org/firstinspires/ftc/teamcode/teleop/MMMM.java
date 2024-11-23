package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.drive.config.BetterBoolGamepad;
import org.firstinspires.ftc.teamcode.drive.config.Intake;
import org.firstinspires.ftc.teamcode.drive.config.Lift;
import org.firstinspires.ftc.teamcode.drive.config.Slide;

@Config
@TeleOp(name = "MMMM", group = "Drivetrain")
public class MMMM extends OpMode {

    MecanumDrive drive;
    Lift lift;
    Slide slide;
    Intake intake;
    BetterBoolGamepad bGamepad1, bGamepad2;
    public double speedMod = 0.5;
    public double speedModTwo = 0.25;
    public double speedModTurn = 0.5;

    public int intakeCycleDirection = 1;
    public Servo shaft;
    public DcMotor centralLift;

    public CRServo intakeWheel;
    public static int flipDrive = -1;

    @Override
    public void init() {
        drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));

        Lift lift = new Lift(hardwareMap);
        Slide slide = new Slide(hardwareMap);
        Intake intake = new Intake(hardwareMap);

        bGamepad2 = new BetterBoolGamepad(gamepad2);
        bGamepad1 = new BetterBoolGamepad(gamepad1);
    }

    @Override
    public void loop() {

        Lift lift = new Lift(hardwareMap);
        Slide slide = new Slide(hardwareMap);
        Intake intake = new Intake(hardwareMap);

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
        if (bGamepad1.a()) flipDrive = -1;
        if (bGamepad1.b()) flipDrive = 1;
        if (gamepad2.dpad_left) {intake.claw.setPosition(intake.clawPosition);}

        speedMod = gamepad1.right_bumper ? 0.25 : (gamepad1.right_trigger>0.5 ? gamepad1.right_trigger : 0.5);

        drive.setDrivePowers(
                new PoseVelocity2d(new Vector2d(
                        gamepad1.right_stick_y * speedMod * flipDrive,
                        gamepad1.right_stick_x * speedMod * flipDrive),
                        gamepad1.left_stick_x * speedModTurn * flipDrive
                )
        );


        if (gamepad2.right_bumper && !gamepad2.left_bumper) {intakeCycleDirection = 1;}
        if (!gamepad2.right_bumper && gamepad2.left_bumper) {intakeCycleDirection = -1;}
        if (!gamepad2.right_bumper && !gamepad2.left_bumper) {intakeCycleDirection = 0;}

        if (gamepad2.dpad_left && !gamepad2.dpad_right) {intake.setClaw(Intake.clawClosed);}
        if (!gamepad2.dpad_left && gamepad2.dpad_right) {intake.setClaw(Intake.clawOpen);}
        if (!gamepad2.dpad_left && !gamepad2.dpad_right) {intake.setClaw(intake.claw.getPosition());}

        //Davids Control
        //if (gamepad2.right_trigger >= 0.25) {intakeCycleDirection = 1;}
        //if (gamepad2.left_trigger >= 0.25) {intakeCycleDirection = -1;}
        //if (gamepad2.right_trigger <= 0.25 && gamepad2.left_trigger <= 0.25) {intakeCycleDirection = 0;}

        intake.cycleIntake(intakeCycleDirection);

        //slide.setCentralLift(-gamepad2.left_stick_y);
        intake.cycleGearRack(-gamepad2.right_stick_y);

        if(gamepad2.y) {intake.setWrist(intake.wristDrop);}
        if (gamepad2.a) {intake.setWrist(intake.wristFlat);}
        if (gamepad2.b) {intake.setWrist(intake.wristBack);}


        if (Math.abs(gamepad2.left_stick_y) < 0) {
            if (gamepad2.dpad_right) {
                slide.HighBasket();
            }
            if (gamepad2.dpad_left) {
                slide.Base();
            }
        }
        else {
            lift.setCentralLift(-gamepad2.left_stick_y);
            slide.setTargetPosition(slide.slide.getCurrentPosition());
        }

        //if (gamepad2.dpad_up && !gamepad2.dpad_down){slide.sethangLift(1);}
        //if (!gamepad2.dpad_up && gamepad2.dpad_down) {slide.sethangLift(-1);}
        //if (!gamepad2.dpad_up && !gamepad2.dpad_down) {slide.sethangLift(0);}

        slide.slideToPosition();

        telemetry.addData("Speedmod:", speedMod);
        telemetry.addData("SpeedmodTwo:", speedModTwo);
        telemetry.addData("Flip Drive:", flipDrive);
        telemetry.addData("intakeCycleDirection:", intakeCycleDirection);
        telemetry.addData("slide Position:", slide.slide.getCurrentPosition());
        telemetry.update();
    }


}

