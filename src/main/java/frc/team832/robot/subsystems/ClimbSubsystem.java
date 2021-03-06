package frc.team832.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.SlewRateLimiter;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.*;
import frc.team832.lib.driverstation.dashboard.DashboardManager;
import frc.team832.lib.driverstation.dashboard.DashboardWidget;
import frc.team832.lib.motorcontrol.NeutralMode;
import frc.team832.lib.motorcontrol2.vendor.CANSparkMax;
import frc.team832.lib.motors.Motor;
import frc.team832.lib.power.GrouchPDP;
import frc.team832.lib.power.impl.SmartMCAttachedPDPSlot;
import frc.team832.lib.util.OscarMath;
import frc.team832.robot.Constants;
import frc.team832.robot.utilities.state.ShooterCalculations;

public class ClimbSubsystem extends SubsystemBase {
    public final boolean initSuccessful;

    private final Solenoid climbLock;
    private final CANSparkMax winchMotor, deployMotor;
    private final SmartMCAttachedPDPSlot winchSlot, deploySlot;

    private double climbPower = 0;
    private double extendTarget = Constants.ClimberValues.Retract;

    private ProfiledPIDController extendPID = new ProfiledPIDController(Constants.ClimberValues.ExtendkP, Constants.ClimberValues.ExtendkI, Constants.ClimberValues.ExtendkD, Constants.ClimberValues.ExtendConstraints);
    private SlewRateLimiter climbRamp = new SlewRateLimiter(Constants.ClimberValues.ClimbRateLimit);

    private final NetworkTableEntry dashboard_isSafe, dashboard_deployTarget, dashboard_deployPosition;

    public ClimbSubsystem(GrouchPDP pdp) {
        DashboardManager.addTab(this);

        climbLock = new Solenoid(Constants.PneumaticsValues.PCM_MODULE_NUM, Constants.PneumaticsValues.CLIMB_LOCK_SOLENOID_ID);

        winchMotor = new CANSparkMax(Constants.ClimberValues.WINCH_CAN_ID, Motor.kNEO);
        deployMotor = new CANSparkMax(Constants.ClimberValues.DEPLOY_CAN_ID, Motor.kNEO550);

        winchSlot = pdp.addDevice(Constants.ClimberValues.WINCH_PDP_PORT, winchMotor);
        deploySlot = pdp.addDevice(Constants.ClimberValues.DEPLOY_PDP_PORT, deployMotor);

        winchMotor.wipeSettings();
        deployMotor.wipeSettings();

        deployMotor.rezeroSensor();

        winchMotor.setNeutralMode(NeutralMode.kCoast);
        deployMotor.setNeutralMode(NeutralMode.kBrake);

        winchMotor.setInverted(false);
        winchMotor.setSensorPhase(true);

        deployMotor.setInverted(false);
        deployMotor.setSensorPhase(true);

        winchMotor.limitInputCurrent(40);
        deployMotor.limitInputCurrent(30);

        dashboard_isSafe = DashboardManager.addTabItem(this, "Is Safe", false, DashboardWidget.BooleanBox);
        dashboard_deployTarget = DashboardManager.addTabItem(this, "Deploy Target Pos", 0);
        dashboard_deployPosition = DashboardManager.addTabItem(this, "Deploy Actual Pos", 0);

        initSuccessful = winchMotor.getCANConnection() && deployMotor.getCANConnection();
    }

    public void periodic() {
        runExtendPID();
        runClimbPID();
        dashboard_isSafe.setBoolean(isWinchSafe());
    }

    public void unwindWinch() {
        climbPower = -0.25;
    }

    public void windWinch() {
        climbPower = 0.75;
    }

    public boolean isWinchSafe() {
        return deployMotor.getSensorPosition() > Constants.ClimberValues.MinExtend;
    }

    public void retractHook() {
        setTargetPosition(Constants.ClimberValues.Retract);
    }

    public void stopExtend() {
        deployMotor.set(0);
    }

    public void adjustHook(double slider) {
        double targetPos = OscarMath.clipMap(slider, -1, 1, Constants.ClimberValues.MinExtend, Constants.ClimberValues.MaxExtend);
        setTargetPosition(targetPos);
    }

    private void setTargetPosition(double pos) {
        extendTarget = pos;
    }

    private void runExtendPID() {
        deployMotor.set(extendPID.calculate(deployMotor.getSensorPosition(), extendTarget));
    }

    private void runClimbPID() {
        winchMotor.set(climbRamp.calculate(climbPower));
    }

    public void stopClimb() {
        climbPower = 0;
    }

    public void lockClimb() {
        climbLock.set(false);
    }

    public void unlockClimb() { climbLock.set(true); }

    public void zeroDeploy() {
        deployMotor.rezeroSensor();
    }



    // Commands:
    public class StartClimbGroup extends SequentialCommandGroup {
        public StartClimbGroup(boolean climbUp) {
            addRequirements(ClimbSubsystem.this);
            addCommands(
                    new InstantCommand(ClimbSubsystem.this::unlockClimb, ClimbSubsystem.this),
                    new WaitCommand(0.2),
                    new InstantCommand(climbUp ? ClimbSubsystem.this::windWinch : ClimbSubsystem.this::unwindWinch, ClimbSubsystem.this)
            );
        }
    }

    public class StopClimbGroup extends SequentialCommandGroup {
        public StopClimbGroup(){
            addRequirements(ClimbSubsystem.this);
            addCommands(
                    new InstantCommand(ClimbSubsystem.this::stopClimb),
                    new WaitCommand(0.2),
                    new InstantCommand(ClimbSubsystem.this::lockClimb)
            );
        }
    }
}
