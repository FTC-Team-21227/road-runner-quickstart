package org.firstinspires.ftc.teamcode.autons;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "AUTON2025REDLEFT_3")
public class AUTON2025REDLEFT_3 extends LinearOpMode{
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(0, 72, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        ARM1 arm1 = new ARM1(hardwareMap);
        ARM2 arm2 = new ARM2(hardwareMap);
        CLAW claw = new CLAW(hardwareMap);
        INTAKE_ANGLE intake_angle = new INTAKE_ANGLE(hardwareMap);

        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .lineToX(10)
                .waitSeconds(1);
        TrajectoryActionBuilder tab2 = drive.actionBuilder(new Pose2d(10, 72, 0))
                .strafeTo(new Vector2d(5,72))
                ;
        TrajectoryActionBuilder tab3 = drive.actionBuilder(new Pose2d(5, 72, 0))
                .waitSeconds(1)
                .strafeTo(new Vector2d(5, 117))
                .strafeTo(new Vector2d(15,117));
        TrajectoryActionBuilder tab4 = drive.actionBuilder(new Pose2d(15, 117, 0))
                .turnTo(Math.toRadians(135)) //face basket
                .strafeTo(new Vector2d(5, 125));
        ;
        TrajectoryActionBuilder tab5 = drive.actionBuilder(new Pose2d(5, 125, Math.toRadians(135)))
                .turnTo(Math.toRadians(0))//get second sample
                .strafeTo(new Vector2d(15,125));
        ;
        TrajectoryActionBuilder tab6 = drive.actionBuilder(new Pose2d(15, 125, Math.toRadians(0)))
                .turnTo(Math.toRadians(135)) //face basket
                .strafeTo(new Vector2d(5, 125));
        ;
        TrajectoryActionBuilder tab7 = drive.actionBuilder(new Pose2d(15,125,Math.toRadians(135)))
                .turnTo(Math.toRadians(20))//get third sample
                .strafeTo(new Vector2d(20,125));                ;
        TrajectoryActionBuilder tab8 = drive.actionBuilder(new Pose2d(20,125,Math.toRadians(125)))
                .turnTo(Math.toRadians(135)) //face basket
                .strafeTo(new Vector2d(5, 125));
                ;
        TrajectoryActionBuilder tab9 = drive.actionBuilder(new Pose2d(5,125,Math.toRadians(135)))//face basket
                .strafeTo(new Vector2d(10, 120))
                .waitSeconds(2)
                .strafeTo(new Vector2d(5,125));
        ;

        claw.closeClaw();
        intake_angle.RotatePosition0();

        waitForStart();

        Action firstTrajectory = tab1.build();
        Action secondTrajectory = tab2.build();
        Action thirdTrajectory = tab3.build();
        Action fourthTrajectory = tab4.build();
        Action fifthTrajectory = tab5.build();
        Action sixthTrajectory = tab6.build();
        Action seventhTrajectory = tab7.build();
        Action eighthTrajectory = tab8.build();
        Action ninthTrajectory = tab9.build();

        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                                intake_angle.RotatePosition0(),
                                arm1.liftRungUp(),
                                arm2.liftRungUp(),
                                claw.closeClaw(),
                                firstTrajectory
                        ),
                        arm1.hookSpecimen(),
                        //arm2.hookSpecimen(),
                        secondTrajectory,
                        claw.openClaw(),
                        new ParallelAction(
                                arm1.LiftFloorDown(),
                                arm2.liftFloorUp(),
                                thirdTrajectory
                        ),
                        claw.closeClaw(),
                        new ParallelAction(
                            arm1.liftBucketUp(),
                            arm2.liftBucketUp()
                        ),
                        new ParallelAction(
                                fourthTrajectory
                        ),
                        claw.openClaw(),
                        new ParallelAction(
                                fifthTrajectory,
                                arm1.LiftFloorDown(),
                                arm2.liftFloorUp()
                        ),
                        claw.closeClaw(),
                        new ParallelAction(
                                arm1.liftBucketUp(),
                                arm2.liftBucketUp(),
                                sixthTrajectory
                        ),
                        claw.openClaw(),
                        new ParallelAction(
                                arm1.LiftFloorDown(),
                                arm2.liftFloorUp(),
                                seventhTrajectory
                        ),
                        claw.closeClaw(),
                        new ParallelAction(
                                arm1.liftBucketUp(),
                                arm2.liftBucketUp(),
                                eighthTrajectory
                        ),
                        claw.openClaw(),
                        new ParallelAction(
                            ninthTrajectory,
                            arm2.liftDown(),
                            arm1.liftDown()
                        )
                        //         trajectoryActionCloseOut
                )
        );
    }
}
