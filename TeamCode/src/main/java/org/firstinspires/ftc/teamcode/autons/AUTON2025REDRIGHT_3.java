package org.firstinspires.ftc.teamcode.autons;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous(name = "AUTON2025REDRIGHT_3")
public class AUTON2025REDRIGHT_3 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(0, 55, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        ARM1 arm1 = new ARM1(hardwareMap);
        ARM2 arm2 = new ARM2(hardwareMap);
        CLAW claw = new CLAW(hardwareMap);
        INTAKE_ANGLE intake_angle = new INTAKE_ANGLE(hardwareMap);

        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(10,60))
                .waitSeconds(1);
        TrajectoryActionBuilder tab2 = drive.actionBuilder(new Pose2d(10, 60, 0))
                .strafeTo(new Vector2d(5,60))
                ;
        TrajectoryActionBuilder tab3 = drive.actionBuilder(new Pose2d(5, 60, 0))
                .strafeTo(new Vector2d(0, 30))
                .strafeTo(new Vector2d(45,30))
                .strafeTo(new Vector2d(45, 20))
                .strafeTo(new Vector2d(5,20))
                .strafeTo(new Vector2d(45,20))
                .strafeTo(new Vector2d(45,12))
                .strafeTo(new Vector2d(5,12));
        TrajectoryActionBuilder tab4 = drive.actionBuilder(new Pose2d(5, 12, 0))
                .strafeTo(new Vector2d(5,30))
                .turnTo(Math.toRadians(-90));
        TrajectoryActionBuilder tab5 = drive.actionBuilder(new Pose2d(5, 30, -90))
                .strafeTo(new Vector2d(5,25));
        TrajectoryActionBuilder tab6 = drive.actionBuilder(new Pose2d(5, 25, -90))
                .turnTo(Math.toRadians(0))
                .strafeTo(new Vector2d(5,72));
        TrajectoryActionBuilder tab7 = drive.actionBuilder(new Pose2d(5, 72, 0))
                .strafeTo(new Vector2d(10,72));
        TrajectoryActionBuilder tab8 = drive.actionBuilder(new Pose2d(10, 72, 0))
                .strafeTo(new Vector2d(5,72))
                .strafeTo(new Vector2d(0, 10));

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



        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                                intake_angle.RotatePosition0(),
                                arm1.liftRungUp(),
                                arm2.liftRungUp(),
                                claw.closeClaw(),
                                firstTrajectory
                        ),
                        //arm1.hookSpecimen(),
                        //arm2.hookSpecimen(),
                        secondTrajectory,
                        claw.openClaw(),
                        new ParallelAction(
                                arm1.liftDown(),
                                arm2.liftDown(),
                                thirdTrajectory
                        ),
                        fourthTrajectory,
                        new ParallelAction(
                            arm1.liftWallUp(),
                            arm2.liftWallUp()
                        ),
                        fifthTrajectory,
                        claw.closeClaw(),
                        new ParallelAction(
                            arm1.liftRungUp(),
                            arm2.liftRungUp(),
                            sixthTrajectory
                        ),
                        seventhTrajectory,
                        arm1.hookSpecimen(),
                        new ParallelAction(
                            eighthTrajectory,
                            arm1.liftDown(),
                            arm2.liftDown()
                        )
                )
        );
    }
}
