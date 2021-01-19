package org.firstinspires.ftc.teamcode.modes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.DifferentialOdometry;
import com.arcrobotics.ftclib.purepursuit.Path;
import com.arcrobotics.ftclib.purepursuit.Waypoint;
import com.arcrobotics.ftclib.purepursuit.types.PathType;
import com.arcrobotics.ftclib.purepursuit.waypoints.EndWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.GeneralWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.StartWaypoint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.GoBILDABase;

@Autonomous()
public class Kinematics extends LinearOpMode {

    // define our trackwidth
    static final double TRACKWIDTH = 13.7;

    // convert ticks to inches
    static final double TICKS_TO_INCHES = 15.3;

    GoBILDABase robot = new GoBILDABase();

    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);

        // create our odometry
        DifferentialOdometry diffOdom = new DifferentialOdometry(
                () -> robot.LF.getCurrentPosition() * TICKS_TO_INCHES,
                () -> robot.RF.getCurrentPosition() * TICKS_TO_INCHES,
                TRACKWIDTH
        );

        // update the initial position
        diffOdom.updatePose(new Pose2d(1, 2, new Rotation2d(0)));

        FtcDashboard dashboard = FtcDashboard.getInstance();
        TelemetryPacket packet = new TelemetryPacket();

        // control loop
        while (!isStopRequested()) {
            /* implementation */

//            packet.fieldOverlay()
//                    .setFill("blue")
//                    .fillRect(-20, -20, 40, 40);

            dashboard.sendTelemetryPacket(packet);

            telemetry.addData("x", diffOdom.getPose().getX());
            telemetry.addData("y", diffOdom.getPose().getY());
            telemetry.update();

            // update the position
            diffOdom.updatePose();
        }
    }
}
