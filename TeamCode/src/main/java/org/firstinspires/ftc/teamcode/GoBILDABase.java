package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

public class GoBILDABase {

    static final double wheelDiameter = 9.6;

    public BNO055IMU imu;
    public DcMotorImplEx LF, LB, RF, RB;

    double targetAngle;

    public void init(HardwareMap hardwareMap) {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        targetAngle = imu.getAngularOrientation().firstAngle;

        LF = hardwareMap.get(DcMotorImplEx.class, "m1");
        LB = hardwareMap.get(DcMotorImplEx.class, "m2");
        RF = hardwareMap.get(DcMotorImplEx.class, "m3");
        RB = hardwareMap.get(DcMotorImplEx.class, "m4");

        LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        LF.setDirection(DcMotorSimple.Direction.REVERSE);
        LB.setDirection(DcMotorSimple.Direction.REVERSE);

        setBaseRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // default pid
        setPIDCoefficients(new PIDCoefficients(15, 4, 0));
    }

    public void setBaseRunMode(DcMotor.RunMode r) {
        if (r == DcMotor.RunMode.RUN_TO_POSITION) {
            LF.setTargetPosition(0);
            LB.setTargetPosition(0);
            RF.setTargetPosition(0);
            RB.setTargetPosition(0);
        }

        LF.setMode(r);
        LB.setMode(r);
        RF.setMode(r);
        RB.setMode(r);
    }

    public double getVelPower(double speed) {
        return speed * 2500;
    }

    public boolean isMotorReachTarget(DcMotorEx m) {
        return m.getCurrentPosition() - m.getTargetPosition() < m.getTargetPositionTolerance();
    }

    public int translateDist(double dist) {
        return (int) Math.floor((dist / (wheelDiameter * Math.PI) * 537.6) + 0.5);
    }

    public boolean moveX(double dist) {
        int target = translateDist(dist);

        LF.setPower(0.5);
        LB.setPower(0.5);
        RF.setPower(0.5);
        RB.setPower(0.5);

        LF.setTargetPosition(target);
        LB.setTargetPosition(target);
        RF.setTargetPosition(target);
        RB.setTargetPosition(target);

        return isMotorReachTarget(LF) && isMotorReachTarget(LB) && isMotorReachTarget(RF) && isMotorReachTarget(RB);
    }

    public boolean moveY(double dist) {
        int target = translateDist(dist);

        LF.setPower(0.5);
        LB.setPower(0.5);
        RF.setPower(0.5);
        RB.setPower(0.5);

        LF.setTargetPosition(target);
        LB.setTargetPosition(-target);
        RF.setTargetPosition(-target);
        RB.setTargetPosition(target);

        return isMotorReachTarget(LF) && isMotorReachTarget(LB) && isMotorReachTarget(RF) && isMotorReachTarget(RB);
    }

    public double getAngle() {
        // change this to use different angle, order is ZYX.
        return imu.getAngularOrientation().firstAngle;
    }



    public boolean turn(double newTargetAngle) {
        targetAngle = newTargetAngle;

        double deltaAngle = getAngle() - targetAngle;



        return Math.abs(deltaAngle) < 10;
    }

    public void teleop(Gamepad gamepad) {
        LF.setVelocity(getVelPower(-gamepad.left_stick_y + gamepad.left_stick_x + gamepad.right_stick_x));
        LB.setVelocity(getVelPower(-gamepad.left_stick_y - gamepad.left_stick_x + gamepad.right_stick_x));
        RF.setVelocity(getVelPower(-gamepad.left_stick_y - gamepad.left_stick_x - gamepad.right_stick_x));
        RB.setVelocity(getVelPower(-gamepad.left_stick_y + gamepad.left_stick_x - gamepad.right_stick_x));
    }

    public void setAllVelocity(double speed) {
        LF.setVelocity(speed);
        LB.setVelocity(speed);
        RF.setVelocity(speed);
        RB.setVelocity(speed);
    }

    public void setPIDCoefficients(PIDCoefficients pidCoefficients) {
        LF.setPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidCoefficients);
        LB.setPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidCoefficients);
        RF.setPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidCoefficients);
        RB.setPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidCoefficients);
    }
}
