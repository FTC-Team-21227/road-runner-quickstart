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
                .strafeTo(new Vector2d(0, 120))
                .strafeTo(new Vector2d(10, 120));
                //.lineToX(5);
        TrajectoryActionBuilder tab4 = drive.actionBuilder(initialPose)
                .turn(Math.toRadians(225)) //face basket
                .strafeTo(new Vector2d(115, 117))
        ;
        TrajectoryActionBuilder tab5 = drive.actionBuilder(initialPose)
                .turn(Math.toRadians(235)) //get second sample
        ;
        TrajectoryActionBuilder tab6 = drive.actionBuilder(initialPose)
                .turn(Math.toRadians(125)) //face basket
        ;
        TrajectoryActionBuilder tab7 = drive.actionBuilder(initialPose)
                .lineToX(10)
                ;
        TrajectoryActionBuilder tab8 = drive.actionBuilder(initialPose)
                .lineToX(10)
                ;
        TrajectoryActionBuilder tab9 = drive.actionBuilder(initialPose)
                .lineToX(10)
                ;
        TrajectoryActionBuilder tab10 = drive.actionBuilder(initialPose)
                        .waitSeconds(1)
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
        Action tenthTrajectory = tab10.build();

        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                                intake_angle.RotatePosition0(),
                                arm1.liftRungUp(),
                                arm2.liftRungUp(),
                                claw.closeClaw(),
                                firstTrajectory
                        ),
                        tenthTrajectory,
                        arm1.hookSpecimen(),
                        //arm2.hookSpecimen(),
                        secondTrajectory,
                        tenthTrajectory,
                        claw.openClaw(),
                        tenthTrajectory,
                        new ParallelAction(
                                arm1.liftDown(),
                                arm2.liftFloorUp(),
                                thirdTrajectory
                        ),
                        claw.closeClaw()
//                        new ParallelAction(
//                                arm1.liftBucketUp(),
//                                arm2.liftBucketUp(),
//                                fourthTrajectory
//                        ),
//                        claw.openClaw(),
//                        new ParallelAction(
//                                fifthTrajectory,
//                                arm1.liftDown(),
//                                arm2.liftFloorUp()
//                        ),
//                        claw.closeClaw(),
//                        new ParallelAction(
//                                arm1.liftBucketUp(),
//                                arm2.liftBucketUp(),
//                                sixthTrajectory
//                        ),
//                        claw.openClaw(),
//                        new ParallelAction(
//                                arm1.liftDown(),
//                                arm2.liftFloorUp(),
//                                seventhTrajectory
//                        ),
//                        claw.closeClaw(),
//                        new ParallelAction(
//                                arm1.liftBucketUp(),
//                                arm2.liftBucketUp(),
//                                eighthTrajectory
//                        ),
//                        claw.openClaw(),
//                        ninthTrajectory,
//                        new ParallelAction(
//                            arm2.liftDown(),
//                            arm1.liftDown()
//                        )
                        //         trajectoryActionCloseOut
                )
        );
    }
}
