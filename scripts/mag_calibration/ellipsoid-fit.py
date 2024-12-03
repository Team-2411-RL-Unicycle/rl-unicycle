import importlib.resources as pkg_resources
import sys
import time

import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
from scipy import linalg

from rluni.icm20948.imu_lib import ICM20948

READ_DELAY = 0.02
PLOT_FREQUENCY = 500
CALIBRATION_FILE = "magnetometer_calibration"


class Magnetometer(object):
    """Magnetometer class with calibration capabilities.

    Parameters
    ----------

    F : float (optional)
        Expected earth magnetic field intensity, default=1.
    """

    def __init__(self, F=53.0, calibration_file=CALIBRATION_FILE):
        with pkg_resources.path(
            "rluni.configs.imu", "default.yaml"
        ) as config_file_path:
            imu_config_path = str(config_file_path)

        self.imu = ICM20948(config_file=imu_config_path)

        # initialize values
        self.F = F
        self.b = np.zeros([3, 1])
        self.A_1 = np.eye(3)

        self.calibration_file = calibration_file

        # Try to load existing calibration data
        self.load_calibration()

    def read(self):
        """Get a sample.

        Returns
        -------
        s : list
            The sample in uT, [x,y,z] (corrected if performed calibration).
        """

        mx, my, mz, flag = self.imu.read_magnetometer()

        s = np.array([mx, my, mz]).reshape(3, 1)
        s = np.dot(self.A_1, s - self.b)
        return [s[0, 0], s[1, 0], s[2, 0]]

    def calibrate(self):
        """Performs calibration."""

        print("Collecting samples (Ctrl-C to stop and perform calibration)")

        try:
            s = []
            n = 0
            while True:
                mx, my, mz, flag = self.imu.read_magnetometer(calibrated=False)
                s.append([mx, my, mz])
                n += 1
                sys.stdout.write("\rTotal: %d" % n)
                sys.stdout.flush()
                time.sleep(READ_DELAY)

                # Update the plot every 10 samples
                if n % PLOT_FREQUENCY == 0:
                    self.plot_data(
                        np.array(s), "calibration_points.png", "Raw Magnetometer Data"
                    )

        except KeyboardInterrupt:
            pass

        # ellipsoid fit
        s = np.array(s).T
        M, n_vec, d = self.__ellipsoid_fit(s)

        # calibration parameters
        # note: some implementations of sqrtm return complex type, taking real
        M_1 = linalg.inv(M)
        self.b = -np.dot(M_1, n_vec)
        self.A_1 = np.real(
            self.F / np.sqrt(np.dot(n_vec.T, np.dot(M_1, n_vec)) - d) * linalg.sqrtm(M)
        )

        # Save the calibration data
        self.save_calibration()
        print("Calibration completed and data saved.")

    def verify_calibration(self):
        """Collects calibrated data for verification with live visualization."""

        print("Collecting calibrated samples for verification (press Ctrl+C to stop)")
        s_calibrated = []
        n = 0

        try:
            while True:
                s = self.read()
                s_calibrated.append(s)
                n += 1
                sys.stdout.write("\rTotal samples collected: %d" % n)
                sys.stdout.flush()
                time.sleep(READ_DELAY)

                # Update the plot every 10 samples
                if n % PLOT_FREQUENCY == 0:
                    self.plot_data(
                        np.array(s_calibrated),
                        "calibrated_points.png",
                        "Calibrated Magnetometer Data",
                    )

        except KeyboardInterrupt:
            print("\nData collection for verification stopped.")

    def save_calibration(self):
        """Saves the calibration data (A_1 and b) to a file as separate NumPy arrays."""
        np.savez(self.calibration_file, A_1=self.A_1, b=self.b)
        print(f"Calibration data saved to {self.calibration_file}.npz")

    def load_calibration(self):
        """Loads the calibration data (A_1 and b) from a file if available."""
        try:
            with np.load(f"{self.calibration_file}.npz") as data:
                self.A_1 = data["A_1"]
                self.b = data["b"]
            print(f"Loaded calibration data from {self.calibration_file}.npz")
        except FileNotFoundError:
            print("No calibration data file found. Proceeding without calibration.")

    def plot_data(self, data, filename, title):
        """Plots the 3D data points and their projections onto XY, XZ, YZ planes.

        Parameters
        ----------
        data : numpy array
            The data points to plot, shape (N, 3).
        filename : str
            The filename to save the plot.
        title : str
            The title of the plot.
        """
        x = data[:, 0]
        y = data[:, 1]
        z = data[:, 2]

        # Create a figure with 2x2 subplots
        fig = plt.figure(figsize=(12, 10))

        # 3D Scatter plot
        ax1 = fig.add_subplot(221, projection="3d")
        ax1.scatter(x, y, z, c="b", marker="o", s=5)
        ax1.set_title(title)
        # Add subtitle with the number of points
        ax1.text2D(0.05, 0.95, f"n = {data.shape[0]}", transform=ax1.transAxes)
        ax1.set_xlabel("X (uT)")
        ax1.set_ylabel("Y (uT)")
        ax1.set_zlabel("Z (uT)")
        # Auto-scale the axes
        max_range = (
            np.array([x.max() - x.min(), y.max() - y.min(), z.max() - z.min()]).max()
            / 2.0
        )
        mid_x = (x.max() + x.min()) * 0.5
        mid_y = (y.max() + y.min()) * 0.5
        mid_z = (z.max() + z.min()) * 0.5
        ax1.set_xlim(mid_x - max_range, mid_x + max_range)
        ax1.set_ylim(mid_y - max_range, mid_y + max_range)
        ax1.set_zlim(mid_z - max_range, mid_z + max_range)

        # XY Projection
        ax2 = fig.add_subplot(222)
        ax2.scatter(x, y, c="r", marker="o", s=5)
        ax2.set_xlabel("X (uT)")
        ax2.set_ylabel("Y (uT)")
        ax2.set_title("XY Projection")
        ax2.set_aspect("equal", "box")
        ax2.grid(True)
        ax2.set_xlim(mid_x - max_range, mid_x + max_range)
        ax2.set_ylim(mid_y - max_range, mid_y + max_range)

        # XZ Projection
        ax3 = fig.add_subplot(223)
        ax3.scatter(x, z, c="g", marker="o", s=5)
        ax3.set_xlabel("X (uT)")
        ax3.set_ylabel("Z (uT)")
        ax3.set_title("XZ Projection")
        ax3.set_aspect("equal", "box")
        ax3.grid(True)
        ax3.set_xlim(mid_x - max_range, mid_x + max_range)
        ax3.set_ylim(mid_z - max_range, mid_z + max_range)

        # YZ Projection
        ax4 = fig.add_subplot(224)
        ax4.scatter(y, z, c="m", marker="o", s=5)
        ax4.set_xlabel("Y (uT)")
        ax4.set_ylabel("Z (uT)")
        ax4.set_title("YZ Projection")
        ax4.set_aspect("equal", "box")
        ax4.grid(True)
        ax4.set_xlim(mid_y - max_range, mid_y + max_range)
        ax4.set_ylim(mid_z - max_range, mid_z + max_range)

        plt.tight_layout()
        plt.savefig(filename)
        plt.close(fig)

    def __ellipsoid_fit(self, s):
        """Estimate ellipsoid parameters from a set of points.

        Parameters
        ----------
        s : array_like
          The samples (M,N) where M=3 (x,y,z) and N=number of samples.

        Returns
        -------
        M, n, d : array_like, array_like, float
          The ellipsoid parameters M, n, d.

        Fits to the equation h_m^T M h_m + h_m^T n + d = 0, where h_m = [mx, my, mz] magnetic readings
        h_true = A^(h_m - b) is the true magnetic field

        Soft Iron: A = [F / sqrt(n^T M^-1 n - d)] * sqrtm(M)
        Hard Iron: b = -M^-1 n

        References
        ----------
        .. [1] Qingde Li; Griffiths, J.G., "Least squares ellipsoid specific
           fitting," in Geometric Modeling and Processing, 2004.
           Proceedings, vol., no., pp.335-340, 2004
        """

        # D (samples)
        D = np.array(
            [
                s[0] ** 2.0,
                s[1] ** 2.0,
                s[2] ** 2.0,
                2.0 * s[1] * s[2],
                2.0 * s[0] * s[2],
                2.0 * s[0] * s[1],
                2.0 * s[0],
                2.0 * s[1],
                2.0 * s[2],
                np.ones_like(s[0]),
            ]
        )

        # S, S_11, S_12, S_21, S_22 (eq. 11)
        S = np.dot(D, D.T)
        S_11 = S[:6, :6]
        S_12 = S[:6, 6:]
        S_21 = S[6:, :6]
        S_22 = S[6:, 6:]

        # C (Eq. 8, k=4)
        C = np.array(
            [
                [-1, 1, 1, 0, 0, 0],
                [1, -1, 1, 0, 0, 0],
                [1, 1, -1, 0, 0, 0],
                [0, 0, 0, -4, 0, 0],
                [0, 0, 0, 0, -4, 0],
                [0, 0, 0, 0, 0, -4],
            ]
        )

        # v_1 (eq. 15, solution)
        E = np.dot(linalg.inv(C), S_11 - np.dot(S_12, np.dot(linalg.inv(S_22), S_21)))

        E_w, E_v = np.linalg.eig(E)

        v_1 = E_v[:, np.argmax(E_w)]
        if v_1[0] < 0:
            v_1 = -v_1

        # v_2 (eq. 13, solution)
        v_2 = np.dot(np.dot(-np.linalg.inv(S_22), S_21), v_1)

        a, b, c, f, g, h = v_1
        p, q, r, d = v_2

        # quadric-form parameters
        M = np.array([[a, h, g], [h, b, f], [g, f, c]])
        n = np.array([[p], [q], [r]])

        return M, n, d


if __name__ == "__main__":
    try:
        # F in Vancouver, Canada is about 53uT
        mag = Magnetometer(F=53.0)
        mag.calibrate()
        mag.verify_calibration()
    except KeyboardInterrupt:
        pass
