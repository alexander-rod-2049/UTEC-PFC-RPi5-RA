#!/usr/bin/env python3
import sys
import pandas as pd
import matplotlib.pyplot as plt
import signal

def plot_magnetometer(csv_file):
    try:
        data = pd.read_csv(csv_file)
        if data.empty:
            print(f"No data to plot in {csv_file}")
            return

        # Figure 1: Orientation
        fig, axs = plt.subplots(3, 1, figsize=(10, 10))
        fig.suptitle('Orientation Data (Magnetometer + IMU)')

        # Roll
        axs[0].plot(data['timestamp'], data['roll_deg'], label='Roll (deg)', color='r')
        axs[0].set_ylabel('Degrees')
        axs[0].legend()
        axs[0].grid(True)

        # Pitch
        axs[1].plot(data['timestamp'], data['pitch_deg'], label='Pitch (deg)', color='g')
        axs[1].set_ylabel('Degrees')
        axs[1].legend()
        axs[1].grid(True)

        # Yaw
        axs[2].plot(data['timestamp'], data['yaw_deg'], label='Yaw (deg)', color='b')
        axs[2].set_ylabel('Degrees')
        axs[2].set_xlabel('Time (s)')
        axs[2].legend()
        axs[2].grid(True)

        plt.tight_layout()

        # Figure 2: Velocity & Linear Acceleration
        if 'lin_accel_x' in data.columns and 'vel_x' in data.columns:
            fig2, axs2 = plt.subplots(2, 1, figsize=(10, 8))
            fig2.suptitle('Velocity & Linear Acceleration (World Frame)')

            # Linear Acceleration
            axs2[0].plot(data['timestamp'], data['lin_accel_x'], label='Lin Ax')
            axs2[0].plot(data['timestamp'], data['lin_accel_y'], label='Lin Ay')
            axs2[0].plot(data['timestamp'], data['lin_accel_z'], label='Lin Az')
            axs2[0].set_ylabel('Lin Accel (m/s^2)')
            axs2[0].legend()
            axs2[0].grid(True)
            
            # Velocity
            axs2[1].plot(data['timestamp'], data['vel_x'], label='Vx')
            axs2[1].plot(data['timestamp'], data['vel_y'], label='Vy')
            axs2[1].plot(data['timestamp'], data['vel_z'], label='Vz')
            axs2[1].set_ylabel('Velocity (m/s)')
            axs2[1].set_xlabel('Time (s)')
            axs2[1].legend()
            axs2[1].grid(True)
            
            plt.tight_layout()

        plt.show() # Blocking show
    except Exception as e:
        print(f"Error plotting magnetometer data: {e}")

def plot_gps(csv_file):
    try:
        data = pd.read_csv(csv_file)
        if data.empty:
            print(f"No data to plot in {csv_file}")
            return
            
        # Filter out 0,0 points if any
        data = data[(data['lat'] != 0) & (data['lon'] != 0)]
        
        if data.empty:
             print("No valid GPS data points found.")
             return

        # Show Plot 2: Relative Progress X, Y, Z (if available) - This will be the main interactive plot
        if 'x' in data.columns and 'y' in data.columns and 'z' in data.columns:
            fig, axs = plt.subplots(2, 2, figsize=(12, 10))
            fig.suptitle('GPS Relative Progress (Start = 0,0,0)')

            # Top-Down View (X vs Y)
            axs[0, 0].plot(data['x'], data['y'], 'r-o', label='Path Top-Down')
            axs[0, 0].set_title('Top-Down Path (X vs Y)')
            axs[0, 0].set_xlabel('X (meters East)')
            axs[0, 0].set_ylabel('Y (meters North)')
            axs[0, 0].grid(True)
            axs[0, 0].axis('equal')
            axs[0, 0].legend()

            # X vs Time
            axs[0, 1].plot(data['timestamp'], data['x'], 'b-', label='X')
            axs[0, 1].set_title('X Progress (East)')
            axs[0, 1].set_xlabel('Time (s)')
            axs[0, 1].set_ylabel('Meters')
            axs[0, 1].grid(True)
            axs[0, 1].legend()

            # Y vs Time
            axs[1, 0].plot(data['timestamp'], data['y'], 'g-', label='Y')
            axs[1, 0].set_title('Y Progress (North)')
            axs[1, 0].set_xlabel('Time (s)')
            axs[1, 0].set_ylabel('Meters')
            axs[1, 0].grid(True)
            axs[1, 0].legend()

            # Z vs Time
            axs[1, 1].plot(data['timestamp'], data['z'], 'k-', label='Z')
            axs[1, 1].set_title('Z Progress (Altitude Change)')
            axs[1, 1].set_xlabel('Time (s)')
            axs[1, 1].set_ylabel('Meters')
            axs[1, 1].grid(True)
            axs[1, 1].legend()

            plt.tight_layout()
            plt.show()
        else:
            plt.figure(figsize=(10, 8))
            plt.plot(data['lon'], data['lat'], 'b-o', label='GPS Path')
            plt.title('GPS Path')
            plt.xlabel('Longitude')
            plt.ylabel('Latitude')
            plt.grid(True)
            plt.legend()
            plt.axis('equal')
            plt.show()

    except Exception as e:
        print(f"Error plotting GPS data: {e}")

if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("Usage: plotter.py <type> <csv_file>")
        sys.exit(1)

    plot_type = sys.argv[1]
    file_path = sys.argv[2]
    
    # Handle Ctrl+C gracefully
    signal.signal(signal.SIGINT, signal.SIG_DFL)

    print(f"Plotting {plot_type} from {file_path}")
    if plot_type == 'mag':
        try:
            data = pd.read_csv(file_path)
            if not data.empty:
                # 1. Orientation Plot
                fig, axs = plt.subplots(3, 1, figsize=(10, 10))
                fig.suptitle('Orientation Data (Complementary Filter)')
                
                axs[0].plot(data['timestamp'], data['roll_deg'], label='Roll (deg)', color='r')
                axs[0].set_ylabel('Degrees')
                axs[0].legend()
                axs[0].grid(True)

                axs[1].plot(data['timestamp'], data['pitch_deg'], label='Pitch (deg)', color='g')
                axs[1].set_ylabel('Degrees')
                axs[1].legend()
                axs[1].grid(True)

                axs[2].plot(data['timestamp'], data['yaw_deg'], label='Yaw (deg)', color='b')
                axs[2].set_ylabel('Degrees')
                axs[2].set_xlabel('Time (s)')
                axs[2].legend()
                axs[2].grid(True)

                plt.tight_layout()
                output_file = file_path.replace('.csv', '_orientation.png')
                plt.savefig(output_file)
                print(f"Orientation graph saved to {output_file}")
                
                # 2. Raw Data Plot (Accel/Gyro/Mag)
                fig2, axs2 = plt.subplots(3, 1, figsize=(10, 10))
                fig2.suptitle('Raw Sensor Data')
                
                # Accel
                if 'accel_x' in data.columns:
                    axs2[0].plot(data['timestamp'], data['accel_x'], label='Ax')
                    axs2[0].plot(data['timestamp'], data['accel_y'], label='Ay')
                    axs2[0].plot(data['timestamp'], data['accel_z'], label='Az')
                    axs2[0].set_ylabel('Accel (m/s^2)')
                    axs2[0].legend()
                    axs2[0].grid(True)
                
                # Gyro
                axs2[1].plot(data['timestamp'], data['gyro_x'], label='Gx')
                axs2[1].plot(data['timestamp'], data['gyro_y'], label='Gy')
                axs2[1].plot(data['timestamp'], data['gyro_z'], label='Gz')
                axs2[1].set_ylabel('Gyro (rad/s)')
                axs2[1].legend()
                axs2[1].grid(True)
                
                # Mag
                axs2[2].plot(data['timestamp'], data['mag_x'], label='Mx')
                axs2[2].plot(data['timestamp'], data['mag_y'], label='My')
                axs2[2].plot(data['timestamp'], data['mag_z'], label='Mz')
                axs2[2].set_ylabel('Mag (Tesla)')
                axs2[2].set_xlabel('Time (s)')
                axs2[2].legend()
                axs2[2].grid(True)
                
                plt.tight_layout()
                output_file_raw = file_path.replace('.csv', '_raw.png')
                plt.savefig(output_file_raw)
                print(f"Raw data graph saved to {output_file_raw}")

                # 3. Velocity & Linear Acceleration Plot
                fig3, axs3 = plt.subplots(2, 1, figsize=(10, 10))
                fig3.suptitle('Velocity & Linear Acceleration (World Frame)')

                # Linear Acceleration
                if 'lin_accel_x' in data.columns:
                    axs3[0].plot(data['timestamp'], data['lin_accel_x'], label='Lin Ax')
                    axs3[0].plot(data['timestamp'], data['lin_accel_y'], label='Lin Ay')
                    axs3[0].plot(data['timestamp'], data['lin_accel_z'], label='Lin Az')
                    axs3[0].set_ylabel('Lin Accel (m/s^2)')
                    axs3[0].legend()
                    axs3[0].grid(True)
                
                # Velocity
                if 'vel_x' in data.columns:
                    axs3[1].plot(data['timestamp'], data['vel_x'], label='Vx')
                    axs3[1].plot(data['timestamp'], data['vel_y'], label='Vy')
                    axs3[1].plot(data['timestamp'], data['vel_z'], label='Vz')
                    axs3[1].set_ylabel('Velocity (m/s)')
                    axs3[1].set_xlabel('Time (s)')
                    axs3[1].legend()
                    axs3[1].grid(True)

                plt.tight_layout()
                output_file_vel = file_path.replace('.csv', '_vel.png')
                plt.savefig(output_file_vel)
                print(f"Velocity graph saved to {output_file_vel}")

        except Exception as e:
            print(f"Error saving mag plot: {e}")
            
        plot_magnetometer(file_path) # Show plot
        
    elif plot_type == 'gps':
        try:
            data = pd.read_csv(file_path)
            if not data.empty:
                data = data[(data['lat'] != 0) & (data['lon'] != 0)]
                if not data.empty:
                    # Plot 1: GPS Path (Lat vs Lon)
                    plt.figure(figsize=(10, 8))
                    plt.plot(data['lon'], data['lat'], 'b-o', label='GPS Path')
                    plt.title('GPS Path (Global)')
                    plt.xlabel('Longitude')
                    plt.ylabel('Latitude')
                    plt.grid(True)
                    plt.legend()
                    plt.axis('equal')
                    
                    output_file = file_path.replace('.csv', '.png')
                    plt.savefig(output_file)
                    print(f"Global Path Graph saved to {output_file}")

                    # Plot 2: Relative Progress X, Y, Z (if available)
                    if 'x' in data.columns and 'y' in data.columns and 'z' in data.columns:
                        fig, axs = plt.subplots(2, 2, figsize=(12, 10))
                        fig.suptitle('GPS Relative Progress (Start = 0,0,0)')

                        # Top-Down View (X vs Y)
                        axs[0, 0].plot(data['x'], data['y'], 'r-o', label='Path Top-Down')
                        axs[0, 0].set_title('Top-Down Path (X vs Y)')
                        axs[0, 0].set_xlabel('X (meters East)')
                        axs[0, 0].set_ylabel('Y (meters North)')
                        axs[0, 0].grid(True)
                        axs[0, 0].axis('equal')
                        axs[0, 0].legend()

                        # X vs Time
                        axs[0, 1].plot(data['timestamp'], data['x'], 'b-', label='X')
                        axs[0, 1].set_title('X Progress (East)')
                        axs[0, 1].set_xlabel('Time (s)')
                        axs[0, 1].set_ylabel('Meters')
                        axs[0, 1].grid(True)
                        axs[0, 1].legend()

                        # Y vs Time
                        axs[1, 0].plot(data['timestamp'], data['y'], 'g-', label='Y')
                        axs[1, 0].set_title('Y Progress (North)')
                        axs[1, 0].set_xlabel('Time (s)')
                        axs[1, 0].set_ylabel('Meters')
                        axs[1, 0].grid(True)
                        axs[1, 0].legend()

                        # Z vs Time
                        axs[1, 1].plot(data['timestamp'], data['z'], 'k-', label='Z')
                        axs[1, 1].set_title('Z Progress (Altitude Change)')
                        axs[1, 1].set_xlabel('Time (s)')
                        axs[1, 1].set_ylabel('Meters')
                        axs[1, 1].grid(True)
                        axs[1, 1].legend()

                        plt.tight_layout()
                        output_file_xyz = file_path.replace('.csv', '_xyz.png')
                        plt.savefig(output_file_xyz)
                        print(f"XYZ Progress Graph saved to {output_file_xyz}")

        except Exception as e:
            print(f"Error saving gps plot: {e}")
            
        plot_gps(file_path) # Show plot
    else:
        print(f"Unknown plot type: {plot_type}")
