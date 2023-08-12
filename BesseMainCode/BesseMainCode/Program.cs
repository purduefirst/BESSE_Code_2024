using Microsoft.SPOT;
using System;
using System.Threading;
using System.IO.Ports;
using CTRE.Phoenix;
using CTRE.Phoenix.Controller;
using CTRE.Phoenix.MotorControl;
using CTRE.Phoenix.MotorControl.CAN;

namespace BesseMainCode
{
    /*
     * Channel Ranges:
     *      1 (Right Joystick y-value): 1811 - 172
     *      2 (Right Joystick x-value): 1811 - 172
     *      3 (Left Joystick y-value): 1811 - 172 (left joystick values were being funky might be a physical issue with it)
     *      4 (Left Joystick x-value): 1811 - 172
     *      5 (Top Right Switch - 3 states):
     *                                    - 999 (top)
     *                                    - 992 (middle)
     *                                    - 984 (bottom)
    */
    public class Program
    {
        // Drive Motors (Flacon 500)
        static TalonSRX rightLeader = new TalonSRX(3);
        static TalonSRX rightFollower1 = new TalonSRX(4);
        static TalonSRX rightFollower2 = new TalonSRX(5);
        static TalonSRX leftLeader = new TalonSRX(0);
        static TalonSRX leftFollower1 = new TalonSRX(1);
        static TalonSRX leftFollower2 = new TalonSRX(2);

        // Decorative Wheel Motors (Snow Blower motor)
        static VictorSPX leftMotor = new VictorSPX(11);
        static VictorSPX rightMotor = new VictorSPX(10);

        // Drone Controlle Receiver Signal Input
        static System.IO.Ports.SerialPort _uart = new SerialPort(CTRE.HERO.IO.Port1.UART, 100000);

        static byte[] buffer = new byte[25];
        static int[] _channels = new int[19];

        static byte idx = 0;
        static byte lost = 0;
        static byte b = 0;
        static byte errors = 0;
        static int count = 1;
        static double currentLeft = 0;
        static double currentRight = 0;

        static bool failsafe = false;
        static bool stop = false;

        public static void Main() {
            _uart.Open();

            SetMotorStates();

            // loop which runs the entire time on the HERO board
            while (true) {
                //keeps the motors happy
                Watchdog.Feed(); 

                //if (_uart.BytesToRead > 0) { //need to see later could break shit
                    // decode the singal from the receiver
                    Decode();

                    //Test();
                //}
            }
        }
        
        //this method needs to be called within decode(), breaks if called in the main method
        static void ArcadeDrive(double rotate, double drive) {
            // creates deadzones
            if (System.Math.Abs(rotate) < 0.07)
            {
                rotate = 0;
            }

            if (System.Math.Abs(drive) < 0.07)
            {
                drive = 0;
            }

            if (System.Math.Abs(drive) > 0.4)
            {
                drive = 0.4;
            }


            if (System.Math.Abs(rotate) > 0.4)
            {
                rotate = 0.4;
            }

            double maximum = System.Math.Max(System.Math.Abs(rotate), System.Math.Abs(drive));
            double total = drive + rotate;
            double difference = drive - rotate;

            if(System.Math.Abs(drive) < 1.00 || System.Math.Abs(drive) < 1.00) {
                if (drive >= 0)
                {
                    if (rotate >= 0)
                    {
                        leftLeader.Set(ControlMode.PercentOutput, maximum);
                        leftFollower1.Set(ControlMode.PercentOutput, maximum);
                        leftFollower2.Set(ControlMode.PercentOutput, maximum);

                        rightLeader.Set(ControlMode.PercentOutput, difference);
                        rightFollower1.Set(ControlMode.PercentOutput, difference);
                        rightFollower2.Set(ControlMode.PercentOutput, difference);
                    }
                    else
                    {
                        leftLeader.Set(ControlMode.PercentOutput, total);
                        leftFollower1.Set(ControlMode.PercentOutput, total);
                        leftFollower2.Set(ControlMode.PercentOutput, total);

                        rightLeader.Set(ControlMode.PercentOutput, maximum);
                        rightFollower1.Set(ControlMode.PercentOutput, maximum);
                        rightFollower2.Set(ControlMode.PercentOutput, maximum);
                    }
                }
                else
                {
                    if (rotate >= 0)
                    {
                        leftLeader.Set(ControlMode.PercentOutput, total);
                        leftFollower1.Set(ControlMode.PercentOutput, total);
                        leftFollower2.Set(ControlMode.PercentOutput, total);

                        rightLeader.Set(ControlMode.PercentOutput, -maximum);
                        rightFollower1.Set(ControlMode.PercentOutput, -maximum);
                        rightFollower2.Set(ControlMode.PercentOutput, -maximum);
                    }
                    else
                    {
                        leftLeader.Set(ControlMode.PercentOutput, -maximum);
                        leftFollower1.Set(ControlMode.PercentOutput, -maximum);
                        leftFollower2.Set(ControlMode.PercentOutput, -maximum);

                        rightLeader.Set(ControlMode.PercentOutput, difference);
                        rightFollower1.Set(ControlMode.PercentOutput, difference);
                        rightFollower2.Set(ControlMode.PercentOutput, difference);
                    }
                }
            }
        }

        //this method needs to be called within decode(), breaks if called in the main method
        static void TankDrive(double left, double right) {
            // creates deadzones
            if (System.Math.Abs(left) < 0.07) {
                left = 0;
            }

            if (System.Math.Abs(right) < 0.07) {
                right = 0;
            }
            
            if(System.Math.Abs(left - currentLeft) < 0.16)
            {
                currentLeft = left;
            } else
            {
                left = currentLeft;
            }

            if (System.Math.Abs(right - currentRight) < 0.25)
            {
                currentRight = right;
            }
            else
            {
                right = currentRight;
            }

            Debug.Print("Current Left: " + currentLeft);
            Debug.Print("New Left: " + left);



            /*if (System.Math.Abs(left) > 0.07) { 
                left = 0;
            }   
       

            if (System.Math.Abs(right) > 0.07) {
                right = 0;
            }*/


            
            leftLeader.Set(ControlMode.PercentOutput, left);
            leftFollower1.Set(ControlMode.PercentOutput, left);
            leftFollower2.Set(ControlMode.PercentOutput, left);

            rightLeader.Set(ControlMode.PercentOutput, -right);
            rightFollower1.Set(ControlMode.PercentOutput, -right);
            rightFollower2.Set(ControlMode.PercentOutput, -right);
            
        }

        static void RunFakeWheels() {
            if ((int)_channels.GetValue(5) == 999) {
                leftMotor.Set(ControlMode.PercentOutput, -0.8);
                rightMotor.Set(ControlMode.PercentOutput, 0.8);
            } else if ((int)_channels.GetValue(5) == 992) {
                leftMotor.Set(ControlMode.PercentOutput, 0);
                rightMotor.Set(ControlMode.PercentOutput, 0);
            } else if ((int)_channels.GetValue(5) == 984) {
                leftMotor.Set(ControlMode.PercentOutput, 0.8);
                rightMotor.Set(ControlMode.PercentOutput, -0.8);
            } else {
                leftMotor.Set(ControlMode.PercentOutput, 0);
                rightMotor.Set(ControlMode.PercentOutput, 0);
            }
        }

        static void KillSwitch()
        {
            if ((int)_channels.GetValue(6) == 992)
            {
                stop = false;
            }
            else
            {
                stop = true;
            }
        }

        static void Test() {
            leftLeader.SetNeutralMode(NeutralMode.Coast);
            leftFollower1.SetNeutralMode(NeutralMode.Coast);
            leftFollower2.SetNeutralMode(NeutralMode.Coast);
              
            rightLeader.SetNeutralMode(NeutralMode.Coast);
            rightFollower1.SetNeutralMode(NeutralMode.Coast);
            rightFollower2.SetNeutralMode(NeutralMode.Coast);

            //leftLeader.Set(ControlMode.PercentOutput, 0.1);
            //leftFollower1.Set(ControlMode.PercentOutput, 0.1);
            //leftFollower2.Set(ControlMode.PercentOutput, 0.1);

            //rightLeader.Set(ControlMode.PercentOutput, 0.1);
            //rightFollower1.Set(ControlMode.PercentOutput, 0.1);
            //rightFollower2.Set(ControlMode.PercentOutput, 0.1);

            leftMotor.Set(ControlMode.PercentOutput, 0.8);
            rightMotor.Set(ControlMode.PercentOutput, 0.8);
        }

        // sets the motors to the states defined below
        static void SetMotorStates() {
            //set all left drive motors to brake
            leftLeader.SetNeutralMode(NeutralMode.Brake);
            leftFollower1.SetNeutralMode(NeutralMode.Brake);
            leftFollower2.SetNeutralMode(NeutralMode.Brake);

            //set all right drive motors to brake
            rightLeader.SetNeutralMode(NeutralMode.Brake);
            rightFollower1.SetNeutralMode(NeutralMode.Brake);
            rightFollower2.SetNeutralMode(NeutralMode.Brake);
        }

        // decodes the signal from the drone controller receiver
        static void Decode() {
            b = (byte)_uart.ReadByte();

            if (idx == 0 && b != 0x0F) {
                // Do nothing
            } else {
                // Read until end byte is received
                buffer[idx++] = b;
            }

            // Check that all 25 bytes have been read
            if (idx == 25) {
                idx = 0;

                // Check that there is a stop byte
                if (buffer[24] != 0x00) {
                    errors++;
                } else {
                    // Decode received packet
                    // Decode SBUS packet
                    // 25 byte packet received is little endian so we have to swap some bytes.
                    _channels[1] = ((buffer[1] | buffer[2] << 8) & 0x07FF);
                    _channels[2] = ((buffer[2] >> 3 | buffer[3] << 5) & 0x07FF);
                    _channels[3] = ((buffer[3] >> 6 | buffer[4] << 2 | buffer[5] << 10) & 0x07FF);
                    _channels[4] = ((buffer[5] >> 1 | buffer[6] << 7) & 0x07FF);

                    _channels[5] = ((buffer[6] >> 4 | buffer[7] << 4) & 0x07FF);
                    _channels[6] = ((buffer[7] >> 7 | buffer[8] << 1 | buffer[9] << 9) & 0x07FF);
                    _channels[7] = ((buffer[9] >> 2 | buffer[10] << 6) & 0x07FF);
                    _channels[8] = ((buffer[10] >> 5 | buffer[11] << 3) & 0x07FF);

                    _channels[9] = ((buffer[12] | buffer[13] << 8) & 0x07FF);
                    _channels[10] = ((buffer[13] >> 3 | buffer[14] << 5) & 0x07FF);
                    _channels[11] = ((buffer[14] >> 6 | buffer[15] << 2 | buffer[16] << 10) & 0x07FF);
                    _channels[12] = ((buffer[16] >> 1 | buffer[17] << 7) & 0x07FF);

                    _channels[13] = ((buffer[17] >> 4 | buffer[18] << 4) & 0x07FF);
                    _channels[14] = ((buffer[18] >> 7 | buffer[19] << 1 | buffer[20] << 9) & 0x07FF);
                    _channels[15] = ((buffer[20] >> 2 | buffer[21] << 6) & 0x07FF);
                    _channels[16] = ((buffer[21] >> 5 | buffer[22] << 3) & 0x07FF);

                    _channels[17] = (((buffer[23]) & 0x0001) == 1) ? 2047 : 0;
                    _channels[18] = (((buffer[23] >> 1) & 0x0001) == 1) ? 2047 : 0;

                    // Check for failsafe from receiver
                    failsafe = (((buffer[23] >> 3) & 0x0001) == 1) ? true : false;

                    // Handle incomplete packages
                    if (((buffer[23] >> 2) & 0x0001) == 1) {
                        lost++;
                    }
                }

                //if (count % 20 == 0)
                //{

                    //RAW CHANNEL INPUT
                    //Debug.Print("RIGHT STICK (UP-DOWN):" + _channels.GetValue(1).ToString());
                    //Debug.Print("RIGHT STICK (LEFT-RIGHT):" + _channels.GetValue(2).ToString());
                    //Debug.Print("LEFT STICK (UP-DOWN):" + _channels.GetValue(3).ToString());
                    //Debug.Print("LEFT STICK (LEFT-RIGHT):" + _channels.GetValue(4).ToString());
                    //Debug.Print("Wheel Switch:" + _channels.GetValue(5).ToString());
                    //Debug.Print("Kill Switch:" + _channels.GetValue(6).ToString());

                //CONVERTED CHANNEL INPUT
                //Debug.Print("RIGHT STICK (UP-DOWN):" + ((((double)(int)_channels.GetValue(1) - 172) / (1811 - 172)) * (1 - -1) + -1));
                //Debug.Print("RIGHT STICK (LEFT-RIGHT):" + ((((double)(int)_channels.GetValue(2) - 172) / (1815 - 165)) * (1 - -1) + -1));
                //Debug.Print("LEFT STICK (UP-DOWN):" + ((((double)(int)_channels.GetValue(3) - 172) / (1820 - 165)) * (1 - -1) + -1));
                //Debug.Print("LEFT STICK (LEFT-RIGHT):" + ((((double)(int)_channels.GetValue(4) - 172) / (1811 - 172)) * (1 - -1) + -1));
                //    if ((int)_channels.GetValue(5) == 999)
                //    {
                //        Debug.Print("Wheel Switch: Run Wheels Forward");
                //    }
                //    else if ((int)_channels.GetValue(5) == 992)
                //    {
                //        Debug.Print("Wheel Switch: Stop Wheels");
                //    }
                //    else if ((int)_channels.GetValue(5) == 984)
                //    {
                //        Debug.Print("Wheel Switch: Run Wheels Backward");
                //    }
                //    else
                //    {
                //        Debug.Print("Wheel Switch: Stop Wheels");
                //    }

                //FAILSAFE
                Debug.Print("failsafe: " + failsafe);

                //count = 1;
                //}
                //count++;

                KillSwitch();
                //Debug.Print("STOP BOOLEAN: " + stop);
                // Use the left joystick (y-value) for moving the robot forward or backwards and the right joystic (x-value) for turning robot left or right
                // Inputs below are being converted form their channel ranges to the input range of -1 to 1 using the following equation:
                // ((Input - Old Min)/(Old Max - Old Min)) * (New Max - New Min) + New Min
                //ArcadeDrive(((((double)(int)_channels.GetValue(2) - 172) / (1815 - 165)) * (1 - -1) + -1),
                //((((double)(int)_channels.GetValue(1) - 172) / (1825 - 165)) * (1 - -1) + -1));

                
                if(stop)
                {
                    leftLeader.Set(ControlMode.PercentOutput, 0);
                    leftFollower1.Set(ControlMode.PercentOutput, 0);
                    leftFollower2.Set(ControlMode.PercentOutput, 0);

                    rightLeader.Set(ControlMode.PercentOutput, 0);
                    rightFollower1.Set(ControlMode.PercentOutput, 0);
                    rightFollower2.Set(ControlMode.PercentOutput, 0);

                    leftMotor.Set(ControlMode.PercentOutput, 0);
                    rightMotor.Set(ControlMode.PercentOutput, 0);

                    currentLeft = 0;
                    currentRight = 0;
                } else
                {
                    TankDrive(((((double)(int)_channels.GetValue(3) - 165) / (1820 - 165)) * (1 - -1) + -1),
                    ((((double)(int)_channels.GetValue(1) - 165) / (1820 - 165)) * (1 - -1) + -1));
                }

                //Test();

                // Based on what the swtich on the drone controller is, the method will run the fake wheels forwards, backwards, or stop them
                //RunFakeWheels();
            }
        }
    }
}
