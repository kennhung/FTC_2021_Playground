package org.firstinspires.ftc.teamcode.modes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;

@TeleOp()
public class EncMeas extends OpMode {
    DcMotorImplEx m;

    @Override
    public void init() {
        m = hardwareMap.get(DcMotorImplEx.class, "m1");
    }

    @Override
    public void loop() {
        telemetry.addData("enc", m.getCurrentPosition());
        telemetry.update();
    }
}
