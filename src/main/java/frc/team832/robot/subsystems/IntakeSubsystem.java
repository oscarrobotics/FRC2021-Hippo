package frc.team832.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team832.lib.driverstation.dashboard.DashboardManager;
import frc.team832.lib.motorcontrol2.vendor.CANTalonFX;
import frc.team832.lib.power.GrouchPDP;
import frc.team832.lib.util.OscarMath;
import frc.team832.robot.Constants.IntakeValues;
import frc.team832.robot.Constants.PneumaticsValues;

public class IntakeSubsystem extends SubsystemBase {

    public final boolean initSuccessful;

    private final CANTalonFX intakeMotor = new CANTalonFX(IntakeValues.INTAKE_MOTOR_CAN_ID);
    private final Solenoid intakePistons = new Solenoid(PneumaticsValues.PCM_MODULE_NUM, PneumaticsValues.INTAKE_SOLENOID_ID);

    public IntakeSubsystem(GrouchPDP pdp) {
        setName("Intake");
        DashboardManager.addTab(this);

        initSuccessful = intakeMotor.getCANConnection();
    }

    public void setPower(double power) {
        intakeMotor.set(power);
    }

    public void stop() {
        intakeMotor.set(0);
    }

    public void stopAll() {
        retractIntake();
        intakeMotor.set(0);
    }

    public void extendIntake() {
        intakePistons.set(true);
    }

    public void retractIntake() {
        intakePistons.set(false);
    }

    public void intake(double power) {
        extendIntake();
        OscarMath.clip(power, 0, 1);
        intakeMotor.set(power);
    }

    public void setPosition(boolean state) {
        intakePistons.set(state);
    }
}
