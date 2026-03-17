/*   MIT License
 *   Copyright (c) [2025] [Base 10 Assets, LLC]
 *
 *   Permission is hereby granted, free of charge, to any person obtaining a copy
 *   of this software and associated documentation files (the "Software"), to deal
 *   in the Software without restriction, including without limitation the rights
 *   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *   copies of the Software, and to permit persons to whom the Software is
 *   furnished to do so, subject to the following conditions:

 *   The above copyright notice and this permission notice shall be included in all
 *   copies or substantial portions of the Software.

 *   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *   SOFTWARE.
 */

package frc.robot.subsystems;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Iterator;
import java.util.stream.Stream;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.RobotController;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Millimeters;
import static edu.wpi.first.units.Units.Radians;

// THIS CODE IS NOT THREAD SAFE

public class GoBildaPinpointFRCDriver {

    private static final float goBILDA_SWINGARM_POD = 13.26291192f; //ticks-per-mm for the goBILDA Swingarm Pod
    private static final float goBILDA_4_BAR_POD    = 19.89436789f; //ticks-per-mm for the goBILDA 4-Bar Pod

    private final int CRC_SIZE = 1;

    private final byte CRC_INITIAL_VALUE   = (byte)0x90;
    private final byte CRC_POLYNOMIAL_VALUE = (byte)0x31;
    private final byte CRC_FINAL_XOR_VALUE  = (byte)0x00;

    //i2c address of the device NOTE: only kMXP is stable on the RoboRio!
    private final I2C i2c = new I2C(I2C.Port.kMXP, 0x31);

    /**
     * Captures the length of each type of register used on the device. Aside from BULK_READ all registers are 4 bytes long
     */
    private enum RegisterType {
        INT32(4),
        FLOAT(4),
        GENERIC(4),
        BULK(40);

        private final int length;

        RegisterType(int length){
            this.length = length;
        }
    }
    
    private int updateCount=0;
    private int numRegisters=0;

        /**
     * Register map, including register address and register type
     */
    public class Register {
        private final int address;
        private final RegisterType type;
        private float fvalue;
        private int ivalue;
        private int lastUpdate=-1;

        public void set(float fvalue) {
            lastUpdate=updateCount;
            this.fvalue = fvalue;
        }

        public void set(int fvalue) {
            lastUpdate=updateCount;
            this.ivalue = fvalue;
        }

        public float getf() {
            if (type != RegisterType.FLOAT) throw new RuntimeException("Attempt to read non-float register as a float.");
            if (lastUpdate != updateCount && !isInBulkReadScope(this)) readRegister(this);
            return this.fvalue;
        }

        public int geti() {
            if (type != RegisterType.INT32) throw new RuntimeException("Attempt to read non-integer register as an integer.");
            if (lastUpdate != updateCount && !isInBulkReadScope(this)) readRegister(this);
            return this.ivalue;
        }

        Register(int address, RegisterType type){
            this.address = address;
            this.type = type;
            numRegisters++;
        }
    }

    public Register DEVICE_ID       = new Register (1, RegisterType.INT32);
    public Register DEVICE_VERSION  = new Register (2, RegisterType.INT32);
    public Register DEVICE_STATUS   = new Register (3, RegisterType.INT32);
    public Register DEVICE_CONTROL  = new Register (4, RegisterType.INT32);
    public Register LOOP_TIME       = new Register (5, RegisterType.INT32);
    public Register X_ENCODER_VALUE = new Register (6, RegisterType.INT32);
    public Register Y_ENCODER_VALUE = new Register (7, RegisterType.INT32);
    public Register X_POSITION      = new Register (8, RegisterType.FLOAT);
    public Register Y_POSITION      = new Register (9, RegisterType.FLOAT);
    public Register H_ORIENTATION   = new Register (10, RegisterType.FLOAT);
    public Register X_VELOCITY      = new Register (11, RegisterType.FLOAT);
    public Register Y_VELOCITY      = new Register (12, RegisterType.FLOAT);
    public Register H_VELOCITY      = new Register (13, RegisterType.FLOAT);
    public Register MM_PER_TICK     = new Register (14, RegisterType.FLOAT);
    public Register X_POD_OFFSET    = new Register (15, RegisterType.FLOAT);
    public Register Y_POD_OFFSET    = new Register (16, RegisterType.FLOAT);
    public Register YAW_SCALAR      = new Register (17, RegisterType.FLOAT);
    public Register BULK_READ       = new Register (18, RegisterType.BULK);
    public Register QUATERNION_W    = new Register (19, RegisterType.FLOAT);
    public Register QUATERNION_X    = new Register (20, RegisterType.FLOAT);
    public Register QUATERNION_Y    = new Register (21, RegisterType.FLOAT);
    public Register QUATERNION_Z    = new Register (22, RegisterType.FLOAT);
    public Register PITCH           = new Register (23, RegisterType.FLOAT);
    public Register ROLL            = new Register (24, RegisterType.FLOAT);
    public Register SET_BULK_READ   = new Register (25, RegisterType.INT32);
    
    private Register[] bulkReadScope = {
            DEVICE_STATUS,
            LOOP_TIME,
            X_ENCODER_VALUE,
            Y_ENCODER_VALUE,
            X_POSITION,
            Y_POSITION,
            H_ORIENTATION,
            X_VELOCITY,
            Y_VELOCITY,
            H_VELOCITY
    };

    /**
     * Device Status enum that captures the current fault condition of the device
     */
    public enum DeviceStatus{
        NOT_READY                (0),
        READY                    (1),
        CALIBRATING              (1 << 1),
        FAULT_X_POD_NOT_DETECTED (1 << 2),
        FAULT_Y_POD_NOT_DETECTED (1 << 3),
        FAULT_NO_PODS_DETECTED   (1 << 2 | 1 << 3),
        FAULT_IMU_RUNAWAY        (1 << 4),
        FAULT_BAD_WRITE_CRC      (1 << 7),
        FAULT_BAD_READ           (1 << 8);

        private final int status;

        DeviceStatus(int status){
            this.status = status;
        }
    }

    public enum EncoderDirection{
        FORWARD,
        REVERSED;
    }

    public enum GoBildaOdometryPods {
        goBILDA_SWINGARM_POD,
        goBILDA_4_BAR_POD;
    }

    /**
     * The kind of error correction used on the I²C communication from the device.
     * NONE: This does not check the data, and passes it directly to the user.
     * CRC: This uses CRC8 error detection to catch incorrect reads. - Only supported by devices with V3 firmware or newer.
     * LOCAL_TEST: Is "Controller only" validation that ensures that the data is !NAN, is not all zeros, and is a reasonable number.
     */
    public enum ErrorDetectionType {
        NONE,
        CRC,
        LOCAL_TEST,
    }

    ByteBuffer buffer = ByteBuffer.allocate(numRegisters*RegisterType.GENERIC.length+CRC_SIZE).order(ByteOrder.LITTLE_ENDIAN); // 100 bytes for read/write

    public GoBildaPinpointFRCDriver() {
        int deviceVersion = getDeviceVersion();
        if(deviceVersion == 1 || deviceVersion == 2){
            throw new RuntimeException("As a mater of safety, old pinpoints are not supported due to lack of CRC.");
        }
    }


    /** Writes an int to the i2c device
     @param reg the register to write the int to
     @param i the integer to write to the register
     */
    private void writeInt(final Register reg, int i) {        
        buffer.put(0, (byte) reg.address); // Put Register Address first
        buffer.putInt(1, i);          // Put Float data
        
        // Send all 5 bytes at once
        i2c.writeBulk(buffer.array(), 5);
    }  
    
    /**
     * Reads an int from a register of the i2c device
     * @param reg the register to read from
     * @return returns an int that contains the value stored in the read register
     */
    @SuppressWarnings("unused")
    private int readInt(Register reg){
        i2c.read(reg.address, 4, buffer.array());
        return buffer.getInt(0);
    }

    /**
     * Reads a float from a register
     * @param reg the register to read
     * @return the float value stored in that register
     */
    @SuppressWarnings("unused")
    private float readFloat(Register reg){
        i2c.read(reg.address, 4, buffer.array());
        return buffer.getFloat(0);
    }

    /**
     * Writes a float to a register on the i2c device
     * @param reg the register to write to
     * @param f the float to write
     */
    private void writeFloat(Register reg, float f) {
        buffer.put(0, (byte) reg.address); // Put Register Address first
        buffer.putFloat(1, f);          // Put Float data
        
        // Send all 5 bytes at once
        i2c.writeBulk(buffer.array(), 5);
    }

    /**
     * Looks up the DeviceStatus enum corresponding with an int value
     * @param s int to lookup
     * @return the Odometry Computer state
     */
    private DeviceStatus lookupStatus (int s){
        if ((s & DeviceStatus.CALIBRATING.status) != 0){
            return DeviceStatus.CALIBRATING;
        }
        boolean xPodDetected = (s & DeviceStatus.FAULT_X_POD_NOT_DETECTED.status) == 0;
        boolean yPodDetected = (s & DeviceStatus.FAULT_Y_POD_NOT_DETECTED.status) == 0;

        if(!xPodDetected  && !yPodDetected){
            return DeviceStatus.FAULT_NO_PODS_DETECTED;
        }
        if (!xPodDetected){
            return DeviceStatus.FAULT_X_POD_NOT_DETECTED;
        }
        if (!yPodDetected){
            return DeviceStatus.FAULT_Y_POD_NOT_DETECTED;
        }
        if ((s & DeviceStatus.FAULT_IMU_RUNAWAY.status) != 0){
            return DeviceStatus.FAULT_IMU_RUNAWAY;
        }
        if ((s & DeviceStatus.READY.status) != 0){
            return DeviceStatus.READY;
        }
        if ((s & DeviceStatus.FAULT_BAD_READ.status) != 0){
            return DeviceStatus.FAULT_BAD_READ;
        }
        else {
            return DeviceStatus.NOT_READY;
        }
    }

    /**
     * Reads a register and saves the data found there to the local variable. Uses either CRC or Local error detection.
     * @param register register to read
     */
    @SuppressWarnings("incomplete-switch")
    public void readRegister(Register register){

        i2c.read(register.address, RegisterType.GENERIC.length+CRC_SIZE, buffer.array());

        switch (register.type) {
            case INT32:
                if(checkCRC(buffer.array(), RegisterType.INT32)) {
                    register.set(buffer.getInt(0));
                } else {
                    DEVICE_STATUS.set(DeviceStatus.FAULT_BAD_READ.status);
                }
                break;
            case FLOAT:
                if(checkCRC(buffer.array(), RegisterType.FLOAT)) {
                    register.set(buffer.getFloat(0));
                } else {
                    DEVICE_STATUS.set(DeviceStatus.FAULT_BAD_READ.status);
                }
                break;
            case BULK:
                update();
                break;
        }
    }


    /**
     * checks a given byteArray[] for a valid CRC data signature by comparing a calculated CRC to the one received in the read.
     * @param byteArray data to validate.
     * @param registerType The kind of register validated. Can be FLOAT, INT32, or BULK.
     * @return true if CRC validates the data. False otherwise.
     */

    private boolean checkCRC(byte[] byteArray, RegisterType registerType){
        int readLength;
        
        if(registerType == RegisterType.BULK) readLength = bulkReadScope.length*RegisterType.GENERIC.length;
        else readLength = RegisterType.GENERIC.length;

        if(computeCRC8(byteArray, readLength) == byteArray[(readLength+CRC_SIZE)-1]){
            return true;
        } else{
            DEVICE_STATUS.set(DeviceStatus.FAULT_BAD_READ.status);
            return false;
        }
    }

    /**
     * Computes the correct CRC8 for a byteArray.
     * @param byteArray data to check
     * @return byte to compare against received CRC.
     */
    private byte computeCRC8(byte[] byteArray, int length) {
        byte crc = CRC_INITIAL_VALUE;
        
        for (int index=0; index<length; index++) {
            byte b=byteArray[index];
            crc ^= b;
            for (int i = 0; i < 8; i++) {
                if ((crc & 0x80) != 0) {
                    crc = (byte) ((crc << 1) ^ CRC_POLYNOMIAL_VALUE);
                } else {
                    crc <<= 1;
                }
            }
        }
        return (byte)(crc ^ CRC_FINAL_XOR_VALUE);
    }

    private long fpgaBulkReadTimestamp;
    long getFpgaBulkReadTimestamp() {return fpgaBulkReadTimestamp;}

    private long fpgaBulkReadLatency;
    long getFpgaBulkReadLatency() {return fpgaBulkReadLatency;}

    /**
     * Call this once per loop to read new data from the Odometry Computer. Data will only update once this is called.
     * On devices with firmware V3 or above, the registers read by this function can be changed via .setBulkReadScope.
     */
    @SuppressWarnings("incomplete-switch")
    public boolean update() {
        updateCount++;

        long readTime = RobotController.getFPGATime();    
        i2c.read(BULK_READ.address, (bulkReadScope.length*RegisterType.GENERIC.length)+CRC_SIZE, buffer.array());
        long finishedTime = RobotController.getFPGATime();    
        if(!checkCRC(buffer.array(), RegisterType.BULK)) {
            return false;
        }
        fpgaBulkReadTimestamp=readTime;
        fpgaBulkReadLatency=finishedTime-readTime;
        for (int i = 0; i < bulkReadScope.length; i++) {
            int index = i*RegisterType.GENERIC.length;
            switch(bulkReadScope[i].type) {
                case INT32:
                    bulkReadScope[i].set(buffer.getInt(index));
                    break;
                case FLOAT:
                    bulkReadScope[i].set(buffer.getFloat(index));
                    break;
            }
        }

        return true;
    }

    /**
     * Only supported on V3 firmware and above. This configures the registers that are read in bulk
     * when .update() is called. Use this to minimize read times based on your unique application.
     * @param registers An array of registers, add registers that you need data from frequently.
     */
    public void setBulkReadScope(Register... registers){
        bulkReadScope = registers.clone();

        Stream<Register> reg = Arrays.stream(registers).distinct();
        ArrayList<Byte> arrayList = new ArrayList<>(registers.length);

        Iterator<Register> iter = reg.iterator();
        while (iter.hasNext()) {
            arrayList.add((byte) iter.next().address);
        }

        buffer.put(0, (byte) SET_BULK_READ.address); // Put Register Address first
        for (int i = 0; i < arrayList.size(); i++){
            buffer.put(i+1, (byte) arrayList.get(i));
        }
        i2c.writeBulk(buffer.array(), arrayList.size()+1);
    }

    private boolean isInBulkReadScope(Register register) {
        for (Register bulkRegister : bulkReadScope) {
            if (bulkRegister == register) {
                return true;
            }
        }
        return false;
    }

    /**
     * Sets the odometry pod positions relative to the point that the odometry computer tracks around.<br><br>
     * The most common tracking position is the center of the robot. <br> <br>
     * The X pod offset refers to how far sideways from the tracking point the X (forward) odometry pod is. Left of the center is a positive number, right of center is a negative number. <br>
     * the Y pod offset refers to how far forwards from the tracking point the Y (strafe) odometry pod is. forward of center is a positive number, backwards is a negative number.<br>
     * @param xOffset how sideways from the center of the robot is the X (forward) pod? Left increases
     * @param yOffset how far forward from the center of the robot is the Y (Strafe) pod? forward increases
     * @param distanceUnit the unit of distance used for offsets.
     */
    public void setOffsets(double xOffset, double yOffset, DistanceUnit distanceUnit){
        writeFloat(X_POD_OFFSET, (float) distanceUnit.of(xOffset).in(Millimeters));
        writeFloat(Y_POD_OFFSET, (float) distanceUnit.of(yOffset).in(Millimeters));
    }

    /**
     * Recalibrates the Odometry Computer's internal IMU. <br><br>
     * <strong> Robot MUST be stationary </strong> <br><br>
     * Device takes a large number of samples, and uses those as the gyroscope zero-offset. This takes approximately 0.25 seconds.
     */
    public void recalibrateIMU(){writeInt(DEVICE_CONTROL,1<<0);}

    /**
     * Resets the current position to 0,0,0 and recalibrates the Odometry Computer's internal IMU. <br><br>
     * <strong> Robot MUST be stationary </strong> <br><br>
     * Device takes a large number of samples, and uses those as the gyroscope zero-offset. This takes approximately 0.25 seconds.
     */
    public void resetPosAndIMU(){writeInt(DEVICE_CONTROL,1<<1);}

    /**
     * Can reverse the direction of each encoder.
     * @param xEncoder FORWARD or REVERSED, X (forward) pod should increase when the robot is moving forward
     * @param yEncoder FORWARD or REVERSED, Y (strafe) pod should increase when the robot is moving left
     */
    public void setEncoderDirections(EncoderDirection xEncoder, EncoderDirection yEncoder){
        if (xEncoder == EncoderDirection.FORWARD){
            writeInt(DEVICE_CONTROL,1<<5);
        }
        if (xEncoder == EncoderDirection.REVERSED) {
            writeInt(DEVICE_CONTROL,1<<4);
        }

        if (yEncoder == EncoderDirection.FORWARD){
            writeInt(DEVICE_CONTROL,1<<3);
        }
        if (yEncoder == EncoderDirection.REVERSED){
            writeInt(DEVICE_CONTROL,1<<2);
        }
    }

    /**
     * If you're using goBILDA odometry pods, the ticks-per-mm values are stored here for easy access.<br><br>
     * @param pods goBILDA_SWINGARM_POD or goBILDA_4_BAR_POD
     */
    public void setEncoderResolution(GoBildaOdometryPods pods){
        if (pods == GoBildaOdometryPods.goBILDA_SWINGARM_POD) {
            writeFloat(MM_PER_TICK, goBILDA_SWINGARM_POD);
        }
        if (pods == GoBildaOdometryPods.goBILDA_4_BAR_POD){
            writeFloat(MM_PER_TICK, goBILDA_4_BAR_POD);
        }
    }

    /**
     * Sets the encoder resolution in ticks per distance of the odometry pods. <br>
     * You can find this number by dividing the counts-per-revolution of your encoder by the circumference of the wheel.
     * @param ticks_per_unit should be somewhere between 10 ticks/mm and 100 ticks/mm a goBILDA Swingarm pod is ~13.26291192
     * @param distanceUnit unit used for distance
     */
    public void setEncoderResolution(double ticks_per_unit, DistanceUnit distanceUnit){
        double resolution = ticks_per_unit/distanceUnit.of(1).in(Millimeters);
        writeFloat(MM_PER_TICK, (float) resolution);
    }

    /**
     * Tuning this value should be unnecessary.<br>
     * The goBILDA Odometry Computer has a per-device tuned yaw offset already applied when you receive it.<br><br>
     * This is a scalar that is applied to the gyro's yaw value. Increasing it will mean it will report more than one degree for every degree the sensor fusion algorithm measures. <br><br>
     * You can tune this variable by rotating the robot a large amount (10 full turns is a good starting place) and comparing the amount that the robot rotated to the amount measured.
     * Rotating the robot exactly 10 times should measure 3600°. If it measures more or less, divide moved amount by the measured amount and apply that value to the Yaw Offset.<br><br>
     * If you find that to get an accurate heading number you need to apply a scalar of more than 1.05, or less than 0.95, your device may be bad. Please reach out to tech@gobilda.com
     * @param yawOffset A scalar for the robot's heading.
     */
    public void setYawScalar(double yawOffset){
        writeFloat(YAW_SCALAR, (float) yawOffset);
    }

    /**
     * Send a position that the Pinpoint should use to track your robot relative to. You can use this to
     * update the estimated position of your robot with new external sensor data, or to run a robot
     * in field coordinates. <br><br>
     * This overrides the current position. <br><br>
     * <strong>Using this feature to track your robot's position in field coordinates:</strong> <br>
     * When you start your code, send a Pose2D that describes the starting position on the field of your robot. <br>
     * Say you're on the red alliance, your robot is against the wall and closer to the audience side,
     * and the front of your robot is pointing towards the center of the field.
     * You can send a setPosition with something like -600mm x, -1200mm Y, and 90 degrees. The pinpoint would then always
     * keep track of how far away from the center of the field you are. <br><br>
     * <strong>Using this feature to update your position with additional sensors: </strong><br>
     * Some robots have a secondary way to locate their robot on the field. This is commonly
     * Apriltag localization in FTC, but it can also be something like a distance sensor.
     * Often these external sensors are absolute (meaning they measure something about the field)
     * so their data is very accurate. But they can be slower to read, or you may need to be in a very specific
     * position on the field to use them. In that case, spend most of your time relying on the Pinpoint
     * to determine your location. Then when you pull a new position from your secondary sensor,
     * send a setPosition command with the new position. The Pinpoint will then track your movement
     * relative to that new, more accurate position.
     * @param pos a Pose2D describing the robot's new position.
     */
    public void setPosition(Pose2d pos){
        writeFloat(X_POSITION, (float) Meters.of(pos.getX()).in(Millimeters));
        writeFloat(Y_POSITION, (float) Meters.of(pos.getY()).in(Millimeters));
        writeFloat(H_ORIENTATION, (float) pos.getRotation().getRadians());
    }

    /**
     * Send a position that the Pinpoint should use to track your robot relative to.
     * You can use this to update the estimated position of your robot with new external
     * sensor data, or to run a robot in field coordinates.
     * @param posX the new X position you'd like the Pinpoint to track your robot relive to.
     * @param distanceUnit the unit for posX
     */
    public void setPosX(double posX, DistanceUnit distanceUnit){
        writeFloat(X_POSITION, (float) Meters.of(posX).in(Millimeters));
    }

    /**
     * Send a position that the Pinpoint should use to track your robot relative to.
     * You can use this to update the estimated position of your robot with new external
     * sensor data, or to run a robot in field coordinates.
     * @param posY the new Y position you'd like the Pinpoint to track your robot relive to.
     * @param distanceUnit the unit for posY
     */
    public void setPosY(double posY, DistanceUnit distanceUnit){
        writeFloat(Y_POSITION, (float) Meters.of(posY).in(Millimeters));
    }

    /**
     * Send a heading that the Pinpoint should use to track your robot relative to.
     * You can use this to update the estimated position of your robot with new external
     * sensor data, or to run a robot in field coordinates.
     * @param heading the new heading you'd like the Pinpoint to track your robot relive to.
     * @param angleUnit Radians or Degrees
     */
    public void setHeading(double heading, AngleUnit angleUnit) {
        writeFloat(H_ORIENTATION, (float) angleUnit.of(heading).in(Radians));
    }

    /**
     * Checks the deviceID of the Odometry Computer. Should return 2.
     * @return 2 if device is functional.
     */
    private int deviceID=0;
    public int getDeviceID() {
        if (deviceID == 0) deviceID = DEVICE_ID.geti();
        return deviceID;
    }

    /**
     * @return the firmware version of the Odometry Computer
     */
    private int deviceVersion=0;
    public int getDeviceVersion() {
        if(deviceVersion==0) {
            deviceVersion = DEVICE_VERSION.geti();
            if(deviceVersion == 1 || deviceVersion == 2){
                deviceVersion = 1;
            }
            else if (deviceVersion == 3){
                deviceVersion = 2;
            }
            else deviceVersion = 0;
        }
        return deviceVersion;
    }

    /**
     * @return a scalar that the IMU measured heading is multiplied by. This is tuned for each unit
     * and should not need adjusted.
     */
    public float getYawScalar() {
        return YAW_SCALAR.getf();
    }

    /**
     * Device Status stores any faults the Odometry Computer may be experiencing. These faults include:
     * @return one of the following states:<br>
     * NOT_READY - The device is currently powering up. And has not initialized yet. RED LED<br>
     * READY - The device is currently functioning as normal. GREEN LED<br>
     * CALIBRATING - The device is currently recalibrating the gyro. RED LED<br>
     * FAULT_NO_PODS_DETECTED - the device does not detect any pods plugged in. PURPLE LED <br>
     * FAULT_X_POD_NOT_DETECTED - The device does not detect an X pod plugged in. BLUE LED <br>
     * FAULT_Y_POD_NOT_DETECTED - The device does not detect a Y pod plugged in. ORANGE LED <br>
     * FAULT_BAD_READ - Aa bad I²C read has been detected, the result reported is a
     * duplicate of the last good read.
     */
    public DeviceStatus getDeviceStatus() {
        return lookupStatus(DEVICE_STATUS.geti());
    }

    /**
     * Checks the Odometry Computer's most recent loop time.<br><br>
     * If values less than 500, or more than 1100 are commonly seen here, there may be something wrong with your device. Please reach out to tech@gobilda.com
     * @return loop time in microseconds (1/1,000,000 seconds)
     */
    public int getLoopTime(){
        return LOOP_TIME.geti();
    }

    /**
     * Checks the Odometry Computer's most recent loop frequency.<br><br>
     * If values less than 900, or more than 2000 are commonly seen here, there may be something wrong with your device. Please reach out to tech@gobilda.com
     * @return Pinpoint Frequency in Hz (loops per second),
     */
    public double getFrequency(){
        int loopTime=LOOP_TIME.geti();
        if (loopTime != 0){
            return 1000000.0/loopTime;
        }
        else {
            return 0;
        }
    }

    /**
     * @return the raw value of the X (forward) encoder in ticks
     */
    public int getEncoderX(){
        return X_ENCODER_VALUE.geti();
    }

    /**
     * @return the raw value of the Y (strafe) encoder in ticks
     */
    public int getEncoderY(){
        return Y_ENCODER_VALUE.geti();
    }


    /**
     * @return the estimated X (forward) position of the robot in specified unit
     * @param distanceUnit the unit that the estimated position will return in
     */
    public double getPosX(DistanceUnit distanceUnit){
        return Millimeters.of(X_POSITION.getf()).in(distanceUnit);
    }

    /**
     * @return the estimated Y (Strafe) position of the robot in specified unit
     * @param distanceUnit the unit that the estimated position will return in
     */
    public double getPosY(DistanceUnit distanceUnit){
        return Millimeters.of(Y_POSITION.getf()).in(distanceUnit);
    }

    /**
     * @return the unnormalized estimated H (heading) position of the robot in specified unit
     * unnormalized heading is not constrained from -180° to 180°. It will continue counting
     * multiple rotations.
     */
    public double getHeading(AngleUnit angleUnit){
        return Radians.of(H_ORIENTATION.getf()).in(angleUnit);
    }


    /**
     * @return the estimated X (forward) velocity of the robot in specified unit/sec
     */
    public double getVelX(DistanceUnit distanceUnit){
        return Millimeters.of(X_VELOCITY.getf()).in(distanceUnit);
    }


    /**
     * @return the estimated Y (strafe) velocity of the robot in specified unit/sec
     */
    public double getVelY(DistanceUnit distanceUnit){
        return Millimeters.of(Y_VELOCITY.getf()).in(distanceUnit);

    }

    /**
     * @return the estimated H (heading) velocity of the robot in specified unit/sec
     */
    public double getHeadingVelocity(AngleUnit angleUnit){
        return Radians.of(H_VELOCITY.getf()).in(angleUnit);
    }

    /**
     * @return the user-set offset for the X (forward) pod in specified unit
     */
    public double getXOffset(DistanceUnit distanceUnit){
        return Millimeters.of(X_POD_OFFSET.getf()).in(distanceUnit);
    }

    /**
     * @return the user-set offset for the Y (strafe) pod
     */
    public double getYOffset(DistanceUnit distanceUnit){
        return Millimeters.of(Y_POD_OFFSET.getf()).in(distanceUnit);
    }

    /**
     * @return a Pose2D containing the estimated position of the robot
     */
    public Pose2d getPosition() {
        return new Pose2d(
            getPosX(Meters),
            getPosY(Meters),
            new Rotation2d(getHeading(Radians))
            );
    }

    /**
     * @return a Quaternion which describes the 3d orientation of the device.
     */
    public Quaternion getQuaternion(){
        int deviceVersion = getDeviceVersion();
        if(deviceVersion == 1 || deviceVersion == 2){
            throw new RuntimeException("Quaternion output is not supported on this device firmware");
        } else {
            return new Quaternion(QUATERNION_W.getf(), QUATERNION_X.getf(), QUATERNION_Y.getf(), QUATERNION_Z.getf());
        }
    }

    /**
     * @return the current pitch of the device in the specified AngleUnit.
     */

    public double getPitch(AngleUnit angleUnit){
        return Radians.of(PITCH.getf()).in(angleUnit);
    }

    /**
     * @return the current roll of the device in the specified AngleUnit.
     */
    public double getRoll(AngleUnit angleUnit){
        return Radians.of(ROLL.getf()).in(angleUnit);
    }
}
