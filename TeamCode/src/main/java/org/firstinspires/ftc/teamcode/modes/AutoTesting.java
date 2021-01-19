package org.firstinspires.ftc.teamcode.modes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.GoBILDABase;

@Autonomous()
public class AutoTesting extends OpMode {

    GoBILDABase robot = new GoBILDABase();

    @Override
    public void init() {
        robot.init(hardwareMap);

        robot.setBaseRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.setBaseRunMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    int step = 0;

    @Override
    public void loop() {

        switch (step){
            case 0:
                if(robot.moveX(50)){
                    step++;
                }
                break;
            case 1:
                if(robot.moveX(-50)){
                    step++;
                }
                break;
            case 2:
                if(robot.moveY(50)){
                    step++;
                }
                break;
            case 3:
                if(robot.moveY(-50)){
                    step++;
                }
                break;
            default:
                robot.setAllVelocity(0);
        }


        telemetry.addData("curr", robot.LF.getCurrentPosition());
        telemetry.addData("target", robot.LF.getTargetPosition());
    }
}
