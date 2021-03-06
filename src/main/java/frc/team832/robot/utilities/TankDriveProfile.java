package frc.team832.robot.utilities;

import frc.team832.lib.drive.SmartDiffDrive;
import frc.team832.lib.driverinput.oi.DriveAxesSupplier;
import frc.team832.lib.driverinput.oi.DriverOI;
import frc.team832.lib.driverinput.oi.SticksDriverOI;
import frc.team832.lib.util.OscarMath;
import frc.team832.robot.Constants;



public class TankDriveProfile {
	public final boolean useFF, useCloseLoop;
	public SmartDiffDrive.LoopMode loopMode;
	public double leftPower, rightPower;

	private final DriverOI oi;

	public TankDriveProfile (DriverOI oi, boolean useFF, boolean useCloseLoop) {
		this.oi = oi;
		this.useFF = useFF;
		this.useCloseLoop = useCloseLoop;
		if (useCloseLoop) loopMode = SmartDiffDrive.LoopMode.VELOCITY;
		else loopMode = SmartDiffDrive.LoopMode.PERCENTAGE;
	}

	public void calculateTankSpeeds() {
		boolean isPreciseRotate = false;
		boolean isRotate = false;
		boolean driveStraight = false;

		if (oi instanceof SticksDriverOI) {
			var sticksOI = (SticksDriverOI)oi;
			isPreciseRotate = sticksOI.rightStick.two.get();
			isRotate = sticksOI.rightStick.trigger.get();
			driveStraight = sticksOI.leftStick.trigger.get();
		}

		if (driveStraight) {
			StoreDriveSpeeds straight = getTankStraightProfile();
			if (isPreciseRotate || isRotate) {
				StoreDriveSpeeds rotate;
				if (isPreciseRotate) {
					rotate = getTankPreciseRotateProfile();
				} else {
					rotate = getTankRotateProfile();
				}
				setSpeeds(straight.leftPow + rotate.leftPow, straight.rightPow + rotate.rightPow);
			} else {
				setSpeeds(straight.leftPow, straight.rightPow);
			}
		} else {
			if (isPreciseRotate) {
				StoreDriveSpeeds rotateOnCenter = getTankRotateOnCenterProfile();
				setSpeeds(rotateOnCenter.leftPow, rotateOnCenter.rightPow);
			} else {
				StoreDriveSpeeds profile = getTankNormalProfile();
				setSpeeds(profile.leftPow, profile.rightPow);
			}
		}
	}

	public StoreDriveSpeeds getTankStraightProfile() {
		DriveAxesSupplier axes = oi.getGreenbergDriveAxes();
		double power;

		power = getNormalPower(-axes.getLeftY());

		return new StoreDriveSpeeds(power, power);
	}

	public StoreDriveSpeeds getTankNormalProfile() {
		DriveAxesSupplier axes = oi.getGreenbergDriveAxes();
		double rightPow, leftPow;

		rightPow = getNormalPower(-axes.getRightY());
		leftPow = getNormalPower(-axes.getLeftY());

		return new StoreDriveSpeeds(leftPow, rightPow);
	}

	public StoreDriveSpeeds getTankRotateProfile() {
		DriveAxesSupplier axes = oi.getGreenbergDriveAxes();
		double rightPow, leftPow;
		double powerMultiplier = OscarMath.clipMap(Math.abs(axes.getLeftY()), 0, 1, Constants.DrivetrainValues.StickRotateMultiplier, Constants.DrivetrainValues.StickRotateMultiplier * 2);

		rightPow = -OscarMath.signumPow(axes.getRightX() * powerMultiplier, 2);
		leftPow = OscarMath.signumPow(axes.getRightX() * powerMultiplier, 2);

		return new StoreDriveSpeeds(leftPow, rightPow);
	}

	public StoreDriveSpeeds getTankPreciseRotateProfile() {
		DriveAxesSupplier axes = oi.getGreenbergDriveAxes();
		double rightPow, leftPow;

		rightPow = OscarMath.signumPow(axes.getRotation() * Constants.DrivetrainValues.StickRotateMultiplier, 2);
		leftPow = -OscarMath.signumPow(axes.getRotation() * Constants.DrivetrainValues.StickRotateMultiplier, 2);

		return new StoreDriveSpeeds(leftPow, rightPow);
	}

	public StoreDriveSpeeds getTankRotateOnCenterProfile() {
		DriveAxesSupplier axes = oi.getGreenbergDriveAxes();
		double rotation;

		rotation = OscarMath.signumPow(axes.getRotation() * Constants.DrivetrainValues.StickRotateOnCenterMultiplier, 3);

		return new StoreDriveSpeeds(-rotation, rotation);
	}

	private double getNormalPower(double stick) {
		double exponent = -9.5 * (Math.abs(stick) - 0.7);
		double power = 1.05 / (1 + (Math.pow(2.718281828459045, exponent))) + 0.005;
		return power * Math.signum(stick) * Constants.DrivetrainValues.StickDriveMultiplier;
	}

	private void setSpeeds(double leftPower, double rightPower) {
		this.leftPower = leftPower;
		this.rightPower = rightPower;
	}

	public class StoreDriveSpeeds {
		public double leftPow, rightPow;

		public StoreDriveSpeeds (double leftPow, double rightPow) {
			this.leftPow = leftPow;
			this.rightPow = rightPow;
		}
	}
}
