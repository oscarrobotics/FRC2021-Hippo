package frc.team832.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.controller.LinearQuadraticRegulator;
import edu.wpi.first.wpilibj.estimator.KalmanFilter;
import edu.wpi.first.wpilibj.system.LinearSystem;
import edu.wpi.first.wpilibj.system.LinearSystemLoop;
import edu.wpi.first.wpilibj.system.plant.DCMotor;
import edu.wpi.first.wpilibj.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.Nat;
import edu.wpi.first.wpiutil.math.VecBuilder;
import edu.wpi.first.wpiutil.math.numbers.N1;
import edu.wpi.first.wpiutil.math.numbers.N2;
import frc.team832.lib.driverstation.dashboard.DashboardManager;
import frc.team832.lib.logging.writers.ArmStateSpaceLogWriter;
import frc.team832.lib.motorcontrol.NeutralMode;
import frc.team832.lib.motorcontrol2.vendor.CANSparkMax;
import frc.team832.lib.motors.Motor;
import frc.team832.lib.power.GrouchPDP;
import frc.team832.lib.power.PDPSlot;
import frc.team832.lib.sensors.REVThroughBorePWM;
import frc.team832.lib.util.OscarMath;
import frc.team832.robot.Constants;
import frc.team832.robot.utilities.state.ShooterCalculations;

public class TurretSubsystem extends SubsystemBase {

    public final boolean initSuccessful;

    private double turretTargetDeg;
    private boolean isVision;
    private double turretFF = 0;

    private final CANSparkMax turretMotor;
    private final REVThroughBorePWM encoder;
    private final PDPSlot pdpSlot;

    private NetworkTableEntry dashboard_turretPos, dashboard_turretPow, dashboard_turretTarget;

    private final ArmStateSpaceLogWriter turretSSLogger = new ArmStateSpaceLogWriter("Turret");

    public TurretSubsystem(GrouchPDP pdp) {
        setName("Turret");
        DashboardManager.addTab(this);
        turretMotor = new CANSparkMax(Constants.TurretValues.TURRET_MOTOR_CAN_ID, Motor.kNEO550);
        encoder = new REVThroughBorePWM(Constants.TurretValues.TURRET_ENCODER_DIO_CHANNEL);
        pdpSlot = pdp.addDevice(Constants.TurretValues.TURRET_PDP_SLOT, turretMotor);

        turretMotor.wipeSettings();

        turretMotor.limitInputCurrent(25);
        turretMotor.setNeutralMode(NeutralMode.kBrake);

        dashboard_turretPos = DashboardManager.addTabItem(this, "Position", 0.0);
        dashboard_turretPow = DashboardManager.addTabItem(this, "Power", 0.0);
        dashboard_turretTarget = DashboardManager.addTabItem(this, "Target", 0.0);

        initSuccessful = turretMotor.getCANConnection();
    }

    private final TrapezoidProfile.Constraints m_constraints =
            new TrapezoidProfile.Constraints(
                    Units.degreesToRadians(360),
                    Units.degreesToRadians(1440)); // Max arm speed and acceleration.

    private TrapezoidProfile.State m_lastProfiledReference = new TrapezoidProfile.State();


    // The plant holds a state-space model of our arm. This system has the following properties:
    //
    // States: [position, velocity], in radians and radians per second.
    // Inputs (what we can "put in"): [voltage], in volts.
    // Outputs (what we can measure): [position], in radians.
    private final LinearSystem<N2, N1, N1> m_turretPlant =
            LinearSystemId.createSingleJointedArmSystem(DCMotor.getNEO(2), Constants.TurretValues.kTurretMOI, Constants.TurretValues.turretGearing);

    // The observer fuses our encoder data and voltage inputs to reject noise.
    private final KalmanFilter<N2, N1, N1> m_observer =
            new KalmanFilter<>(
                    Nat.N2(),
                    Nat.N1(),
                    m_turretPlant,
                    VecBuilder.fill(0.015, 0.17), // How accurate we
                    // think our model is, in radians and radians/sec
                    VecBuilder.fill(0.01), // How accurate we think our encoder position
                    // data is. In this case we very highly trust our encoder position reading.
                    0.020);

    // A LQR uses feedback to create voltage commands.
    private final LinearQuadraticRegulator<N2, N1, N1> m_controller =
            new LinearQuadraticRegulator<>(
                    m_turretPlant,
                    VecBuilder.fill(Units.degreesToRadians(1.0), Units.degreesToRadians(10.0)), // qelms.
                    // Position and velocity error tolerances, in radians and radians per second. Decrease
                    // this to more heavily penalize state excursion, or make the controller behave more
                    // aggressively. In this example we weight position much more highly than velocity, but
                    // this can be tuned to balance the two.
                    VecBuilder.fill(12.0), // relms. Control effort (voltage) tolerance. Decrease this to more
                    // heavily penalize control effort, or make the controller less aggressive. 12 is a good
                    // starting point because that is the (approximate) maximum voltage of a battery.
                    0.020); // Nominal time between loops. 0.020 for TimedRobot, but can be
                    // lower if using notifiers.

    // The state-space loop combines a controller, observer, feedforward and plant for easy control.
    private final LinearSystemLoop<N2, N1, N1> m_loop =
            new LinearSystemLoop<>(m_turretPlant, m_controller, m_observer, 12.0, 0.020);

    public void updateControlLoops() {
        if (RobotState.isEnabled()) {
            double turretTargetRadians = Units.degreesToRadians(turretTargetDeg);
            double targetStateSpaceEffortVolts = calculateStateSpace(turretTargetRadians);
            setVoltage(targetStateSpaceEffortVolts);
        }
    }

    private double calculateStateSpace(double targetRadians) {
        TrapezoidProfile.State goal;
        goal = new TrapezoidProfile.State(targetRadians, 0);
        m_lastProfiledReference = (new TrapezoidProfile(m_constraints, goal, m_lastProfiledReference)).calculate(Constants.TurretValues.ControlLoopPeriod);
        m_loop.setNextR(m_lastProfiledReference.position, m_lastProfiledReference.velocity);

        // Correct our Kalman filter's state vector estimate with encoder data.
        m_loop.correct(VecBuilder.fill(Units.degreesToRadians(getDegrees())));

        // Update our LQR to generate new voltage commands and use the voltages to predict the next state with out Kalman filter.
        m_loop.predict(Constants.TurretValues.ControlLoopPeriod);

        // Send the new calculated voltage to the motors.
        // voltage = duty cycle * battery voltage, so
        // duty cycle = voltage / battery voltage
        double nextVoltage = m_loop.getU(0);
        if(RobotState.isEnabled()) {
            turretSSLogger.logSystemState(
                    m_loop,
                    Units.rotationsPerMinuteToRadiansPerSecond(turretMotor.getSensorVelocity() / Constants.TurretValues.turretGearing),
                    Units.degreesToRadians(getDegrees())
            );
        }
        return nextVoltage;
    }

    public boolean atTargetAngle() {
        return OscarMath.withinEpsilon(3, turretTargetDeg, getDegrees());
    }

    public void setTurretTargetDegrees(double pos, boolean visionMode) {
        isVision = visionMode;
        turretTargetDeg = pos;
    }

    public void trackTarget(double spindexerRPM) {
        updateFF(spindexerRPM);
        setTurretTargetDegrees(ShooterCalculations.visionYaw + ((spindexerRPM / 30.0) * Math.signum(spindexerRPM)) + getDegrees(), true);
    }

    double getRotations() {
        return OscarMath.round(encoder.get(), 3);
    }

    double getDegrees() {
        return Constants.TurretValues.convertRotationsToDegrees(getRotations());
    }

    public void setForward() { setTurretTargetDegrees(Constants.TurretValues.TurretCenterVisionPosition, false); }

    public void setForward(boolean isVision) { setTurretTargetDegrees(Constants.TurretValues.TurretCenterVisionPosition, isVision); }

    public void setIntake() {
        setTurretTargetDegrees(Constants.TurretValues.IntakeOrientationDegrees, false);
    }

    private void updateFF(double spindexerRPM) {
        turretFF =  spindexerRPM * Constants.TurretValues.FFMultiplier;
    }

    @Override
    public void periodic() {
        updateDashboardData();
    }

    private void setVoltage(double voltage){
        turretMotor.set(voltage / turretMotor.getInputVoltage());
    }

    public void updateDashboardData() {
//        dashboard_turretPos.setDouble(getDegrees());
        dashboard_turretPow.setDouble(turretMotor.getOutputVoltage());
//        dashboard_turretTarget.setDouble(turretTargetDeg);
    }
}
