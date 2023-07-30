package org.firstinspires.ftc.teamcode.commandBased;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.commandBased.subsystems.AllSubsystems;
import org.firstinspires.ftc.teamcode.commandBased.subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.commandBased.subsystems.Subsystems;

import java.util.List;

public class RobotHardware {
    public DcMotorEx fL;
    public DcMotorEx fR;
    public DcMotorEx rL;
    public DcMotorEx rR;

    public DcMotorEx eleL;
    public DcMotorEx eleR;

    public BNO055IMU imu;

    private List<LynxModule> hubs;


    private static RobotHardware instance = null;

    public boolean enabled;

    private HardwareMap hwMap;

    public static RobotHardware getInstance() {
        if (instance == null) {
            instance = new RobotHardware();
        }
        instance.enabled = true;
        return instance;
    }

    public void init(HardwareMap hwMap) {
        this.hwMap = hwMap;

        hubs = hwMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }


        eleL = hwMap.get(DcMotorEx.class, "eleL");
        eleR = hwMap.get(DcMotorEx.class, "eleR");

        eleL.setDirection(DcMotorSimple.Direction.REVERSE);
        eleL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        eleR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        eleL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        eleR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        eleL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        eleR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void loop(ElevatorSubsystem subsystems) {
        //subsystems.getDrive().periodic();
    }

    public void read(ElevatorSubsystem subsystems) {
        try {
            subsystems.read();
        } catch (Exception e) {

        }
    }

    public void write(ElevatorSubsystem subsystems) {
        subsystems.write();
    }

    public void clearBulkCache() {
        for (LynxModule hub : hubs) {
            hub.clearBulkCache();
        }
    }
}
