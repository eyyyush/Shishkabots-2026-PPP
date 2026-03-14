package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import util.Logger;

public class IntakePivotSubsystem extends SubsystemBase {
    private static final int PIVOT_CAN_ID = 14;

    // Tune these using the absolute encoder value shown on Shuffleboard.
    private static final double LOWER_POSITION = 0.991;
    private static final double UPPER_POSITION = 0.705;
    private static final double POSITION_TOLERANCE = 0.01;
    private static final double PIVOT_POWER_UP = 0.30;
    private static final double PIVOT_POWER_DOWN = -0.20;
    private static final int PIVOT_CURRENT_LIMIT = 40;

    private final SparkMax pivotMotor;
    private final AbsoluteEncoder absoluteEncoder;

    private double targetPosition = UPPER_POSITION;
    private boolean movingToUpper = true;
    private boolean nextToggleGoesToLower = false;
    private boolean enabled = false;
    private boolean lastEnabled = false;
    private int toggleCount = 0;

    public IntakePivotSubsystem() {
        pivotMotor = new SparkMax(PIVOT_CAN_ID, MotorType.kBrushless);
        absoluteEncoder = pivotMotor.getAbsoluteEncoder();

        SparkMaxConfig pivotConfig = new SparkMaxConfig();
        pivotConfig
            .idleMode(IdleMode.kBrake)
            .inverted(false)
            .smartCurrentLimit(PIVOT_CURRENT_LIMIT)
            .openLoopRampRate(0.05);

        pivotMotor.configure(
            pivotConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );

        stop();
        Logger.log("Intake pivot subsystem initialized on CAN 14");
    }

    public void togglePosition() {
        double currentPosition = getAbsolutePosition();

        movingToUpper = !nextToggleGoesToLower;
        targetPosition = nextToggleGoesToLower ? LOWER_POSITION : UPPER_POSITION;
        nextToggleGoesToLower = !nextToggleGoesToLower;
        enabled = true;
        toggleCount++;

        Logger.log(
            String.format(
                "Intake pivot toggle requested. Current=%.3f Target=%.3f",
                currentPosition,
                targetPosition
            )
        );
        Logger.log(
            String.format(
                "Intake pivot status on toggle. BusV=%.2f Applied=%.2f OutputCurrent=%.2f Enabled=%s",
                pivotMotor.getBusVoltage(),
                pivotMotor.getAppliedOutput(),
                pivotMotor.getOutputCurrent(),
                enabled
            )
        );
    }

    public void stop() {
        pivotMotor.stopMotor();
        enabled = false;
    }

    public double getAbsolutePosition() {
        return absoluteEncoder.getPosition();
    }

    public boolean isAtTarget() {
        return Math.abs(getAbsolutePosition() - targetPosition) <= POSITION_TOLERANCE;
    }

    public boolean isEnabled() {
        return enabled;
    }

    @Override
    public void periodic() {
        double currentPosition = getAbsolutePosition();

        SmartDashboard.putNumber("IntakePivot/AbsolutePosition", currentPosition);
        SmartDashboard.putNumber("IntakePivot/TargetPosition", targetPosition);
        SmartDashboard.putBoolean("IntakePivot/Moving", enabled);
        SmartDashboard.putBoolean("IntakePivot/MovingToUpper", movingToUpper);
        SmartDashboard.putNumber("IntakePivot/AppliedOutput", pivotMotor.getAppliedOutput());
        SmartDashboard.putNumber("IntakePivot/ToggleCount", toggleCount);

        if (!enabled) {
            lastEnabled = false;
            return;
        }

        if (!lastEnabled) {
            Logger.log(
                String.format(
                    "Intake pivot enabled. Current=%.3f Target=%.3f BusV=%.2f Applied=%.2f OutputCurrent=%.2f",
                    currentPosition,
                    targetPosition,
                    pivotMotor.getBusVoltage(),
                    pivotMotor.getAppliedOutput(),
                    pivotMotor.getOutputCurrent()
                )
            );
            lastEnabled = true;
        }

        if (isAtTarget()) {
            stop();
            return;
        }

        pivotMotor.set(MathUtil.clamp(movingToUpper ? PIVOT_POWER_UP : PIVOT_POWER_DOWN, -1.0, 1.0));
    }
}
