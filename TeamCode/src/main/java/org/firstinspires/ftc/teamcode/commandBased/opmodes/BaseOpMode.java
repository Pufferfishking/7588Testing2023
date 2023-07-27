package org.firstinspires.ftc.teamcode.commandBased.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.RollingAverage;

import org.firstinspires.ftc.teamcode.commandBased.Constants;
import org.firstinspires.ftc.teamcode.commandBased.classes.misc.CommandSchedulerEx;
import org.firstinspires.ftc.teamcode.commandBased.classes.triggers.TriggerCommand;
import org.firstinspires.ftc.teamcode.commandBased.classes.GamepadTrigger;
import org.firstinspires.ftc.teamcode.commandBased.classes.TriggerGamepadEx;
import org.firstinspires.ftc.teamcode.commandBased.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.commandBased.subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.commandBased.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.commandBased.subsystems.RotatorSubsystem;

import java.util.List;

@TeleOp
public class BaseOpMode extends CommandOpMode {

    protected ElevatorSubsystem elevatorSS;
    protected ArmSubsystem armSS;
    protected RotatorSubsystem rotatorSS;
    protected IntakeSubsystem intakeSS;

    protected GamepadEx driver;
    protected GamepadEx operator;
    protected TriggerGamepadEx driverEx;
    protected TriggerGamepadEx operatorEx;

    protected MultipleTelemetry tele;

    protected RevBlinkinLedDriver blinkin;
    protected VoltageSensor batteryVoltageSensor;

    protected List<LynxModule> hubs;

    protected double loopTime;
    protected RollingAverage loopAvg;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();

        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule module : hubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        elevatorSS = new ElevatorSubsystem(hardwareMap);
        armSS = new ArmSubsystem(hardwareMap);
        rotatorSS = new RotatorSubsystem(hardwareMap);
        intakeSS = new IntakeSubsystem(hardwareMap);

        driver = new GamepadEx(gamepad1);
        operator = new GamepadEx(gamepad2);
        driverEx = new TriggerGamepadEx(gamepad1, driver);
        operatorEx = new TriggerGamepadEx(gamepad2, operator);

        tele = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        blinkin = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
        blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.AQUA);

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        loopAvg = new RollingAverage(50);
    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();
        CommandSchedulerEx.getInstance().run();

        for (LynxModule module : hubs) {
            module.clearBulkCache();
        }

        tad("loop time", System.currentTimeMillis() - loopTime);
        tad("loop avg", loopAvg.getAverage());
        loopAvg.addNumber((int) (System.currentTimeMillis() - loopTime));
        loopTime = System.currentTimeMillis();

        if (Constants.DEBUG_ELE) {
            tad("ele pos", elevatorSS.getElePos());
            tad("ele profile target", elevatorSS.getEleProfileTarget());
            tad("ele final target", elevatorSS.getEleTarget());
            tad("ele power", elevatorSS.getElePower());
        }

        if (Constants.DEBUG_ARM) {
            tad("arm final encoder target", armSS.getArmTargetEnc());
            tad("arm final angle target", armSS.getArmTargetAngle());
            tad("arm profile target", armSS.getArmProfileTarget());
            tad("arm target", armSS.getArmTargetEnc());
            tad("arm pos", armSS.getArmPos());
            tad("arm power", armSS.getArmPower());
            tad("arm angle", armSS.getArmAngle());
            tad("arm velocity", armSS.getArmVelocity());
            tad("arm acceleration", armSS.getArmAcceleration());
            tad("arm KF", armSS.getCoeffs()[6]);
            tad("arm disabled", armSS.isDisabled());
        }

        if (Constants.DEBUG_ROTATOR) {
            tad("rotator pos", rotatorSS.getPosition());
            tad("rotator usFrame", rotatorSS.getPWMRange()[0]);
            tad("rotator usPulseLower", rotatorSS.getPWMRange()[1]);
            tad("rotator usPulseUpper", rotatorSS.getPWMRange()[2]);
            tad("rotator current", rotatorSS.getAverageCurrent());
        }

        if (Constants.DEBUG_INTAKE) {
            tad("intake power", intakeSS.getPower());
            tad("intake current", intakeSS.getServoBusCurrent());
            tad("intake avg current", intakeSS.getAverageCurrent());
        }

        if (Constants.DEBUG_COMMANDS) {
            for (TriggerCommand command : CommandSchedulerEx.getCommands()) {
//                if (command.isScheduled()) {
//                    tad("Scheduled", command.getName());
//                } else
                if (command.isFinished()) {
                    tad("Finished", command.getName());
                }
//                else if (command.isTriggered()) {
//                    tad("Triggered", command.getName());
                }

            tad("test", 1);
        }

        tele.update();
    }

    protected double getVoltage() {
        return batteryVoltageSensor.getVoltage();
    }

    protected void tal() {
        tele.addLine();
    }
    protected void tal(String caption) {
        tele.addLine(caption);
    }
    protected void tad(String caption, Object value) {
        tele.addData(caption, value);
    }

    protected GamepadButton gp1(GamepadKeys.Button button) {
        return driver.getGamepadButton(button);
    }

    protected GamepadTrigger gp1(GamepadKeys.Trigger trigger) {
        return driverEx.getGamepadTrigger(trigger);
    }

    protected Trigger gp1(GamepadKeys.Button button, int layer) {
        if (layer == 1) {
            return driver.getGamepadButton(button)
                    .and(gp1(Constants.CONTROL_LAYER_2).negate())
                    .and(gp1(Constants.CONTROL_LAYER_3).negate());
        } else if (layer == 2) {
            return driver.getGamepadButton(button)
                    .and(gp1(Constants.CONTROL_LAYER_2))
                    .and(gp1(Constants.CONTROL_LAYER_3).negate());
        } else if (layer == 3) {
            return driver.getGamepadButton(button)
                    .and(gp1(Constants.CONTROL_LAYER_2).negate())
                    .and(gp1(Constants.CONTROL_LAYER_3));
        } else {
            return driver.getGamepadButton(button);
        }
    }

    protected Trigger gp1(GamepadKeys.Trigger trigger, int layer) {
        if (layer == 1) {
            return driverEx.getGamepadTrigger(trigger)
                    .and(gp1(Constants.CONTROL_LAYER_2).negate())
                    .and(gp1(Constants.CONTROL_LAYER_3).negate());
        } else if (layer == 2) {
            return driverEx.getGamepadTrigger(trigger)
                    .and(gp1(Constants.CONTROL_LAYER_2))
                    .and(gp1(Constants.CONTROL_LAYER_3).negate());
        } else if (layer == 3) {
            return driverEx.getGamepadTrigger(trigger)
                    .and(gp1(Constants.CONTROL_LAYER_2).negate())
                    .and(gp1(Constants.CONTROL_LAYER_3));
        } else {
            return driverEx.getGamepadTrigger(trigger);
        }
    }

    protected GamepadButton gp2(GamepadKeys.Button button) {
        return operator.getGamepadButton(button);
    }

    protected GamepadTrigger gp2(GamepadKeys.Trigger trigger) {
        return operatorEx.getGamepadTrigger(trigger);
    }

    protected Trigger gp2(GamepadKeys.Button button, int layer) {
        if (layer == 1) {
            return operator.getGamepadButton(button)
                    .and(gp1(Constants.CONTROL_LAYER_2).negate())
                    .and(gp1(Constants.CONTROL_LAYER_3).negate());
        } else if (layer == 2) {
            return operator.getGamepadButton(button)
                    .and(gp1(Constants.CONTROL_LAYER_2))
                    .and(gp1(Constants.CONTROL_LAYER_3).negate());
        } else if (layer == 3) {
            return operator.getGamepadButton(button)
                    .and(gp1(Constants.CONTROL_LAYER_2).negate())
                    .and(gp1(Constants.CONTROL_LAYER_3));
        } else {
            return operator.getGamepadButton(button);
        }
    }

    protected Trigger gp2(GamepadKeys.Trigger trigger, int layer) {
        if (layer == 1) {
            return operatorEx.getGamepadTrigger(trigger)
                    .and(gp1(Constants.CONTROL_LAYER_2).negate())
                    .and(gp1(Constants.CONTROL_LAYER_3).negate());
        } else if (layer == 2) {
            return operatorEx.getGamepadTrigger(trigger)
                    .and(gp1(Constants.CONTROL_LAYER_2))
                    .and(gp1(Constants.CONTROL_LAYER_3).negate());
        } else if (layer == 3) {
            return operatorEx.getGamepadTrigger(trigger)
                    .and(gp1(Constants.CONTROL_LAYER_2).negate())
                    .and(gp1(Constants.CONTROL_LAYER_3));
        } else {
            return operatorEx.getGamepadTrigger(trigger);
        }
    }


    protected Trigger gp2l1(GamepadKeys.Button button) {
        return operator.getGamepadButton(button).and(gp1(Constants.CONTROL_LAYER_2));
    }

    protected Trigger gp2l1(GamepadKeys.Trigger trigger) {
        return operatorEx.getGamepadTrigger(trigger).and(gp1(Constants.CONTROL_LAYER_2));
    }

    protected Trigger gp2l2(GamepadKeys.Button button) {
        return operator.getGamepadButton(button).and(gp1(Constants.CONTROL_LAYER_3));
    }

    protected Trigger gp2l2(GamepadKeys.Trigger trigger) {
        return operatorEx.getGamepadTrigger(trigger).and(gp1(Constants.CONTROL_LAYER_3));
    }
}
