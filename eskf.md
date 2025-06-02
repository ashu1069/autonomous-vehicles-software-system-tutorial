# Explanation & Implementation


The standard Kalman filter works optimally for linear systems with Gaussian noise. However, autonomous driving systems often involve nonlinear dynamics (e.g., vehicle motion models at high speeds or with significant steering) and nonlinear measurement models (e.g., relating camera images to vehicle pose).

The Error State Kalman Filter (ESKF) is a powerful extension that addresses these nonlinearities, particularly when dealing with orientation represented by rotation matrices. The key idea is to perform the prediction step in the full nonlinear state space but perform the correction step on a small error state. This keeps the corrections small and the linearization around the current estimate more accurate.

### True State, Nominal State, and Error State

- True State (x): The actual, unknown state of the system
- Nominal State (x’): Our best estimate of the true state; evolves according to the (nonlinear) system dynamics.
- Error State ($\delta x$): The small difference between the true state and the nominal state

$$
x = x' \oplus \delta x
$$

### Nonlinear Systems and Measurement Models

- Nonlinear system dynamics: $x_k=f(x_{k-1},u_k,w_k)$
- Nonlinear measurement model: $z_k = h(x_k,v_k)$

## ESKF Algorithm

### Prediction Step

- **Propagate the nominal state**: We use the nonlinear system dynamics to predict the nominal state: 
$$x^{'}_{k|k-1}=f(x^{'}_{k-1|k-1},u_k,0)$$
 Here, we have ignored the process noise as it’s accounted for in the error state covariance.
- **Linearize the system dynamics**: We linearize the nonlinear function $f$ around the nominal state, 
$$x^{'}_{k-1|k-1}$$
to obtain the Jacobian matrix $F_k$ (this was Process Model matrix for the standard Kalman Filter—1-D configuration):

$$
\delta x_k \approx F_k\delta x_{k-1}+G_kw_k
$$

where $G_k=\delta f/\delta w$ is the Jacobian of the process noise.

- **Predict the error state covariance**: We propagate the error state covariance $P_{k-1|k-1}$:

$$
P_{k|k-1}=F_kP_{k-1|k-1}F^T_k+G_kQ_kG^T_k
$$

### Update Step

- **Linearize the measurement model**: We linearize the nonlinear measurement function h around the predicted nominal state $x^{'}_{k|k-1}$ to obtain the Jacobian matrix $H_k$: 
where 
$$δzk=zk−h(x^k∣k−1,0)$$ is the measurement residual.

$$
\delta z_k \approx H_k\delta x_{k|k-1}+v_k
$$

where 
$\delta z_k=z_k-h(x^{'}_{k|k-1},0)$ 
is the measurement residual.

- **Compute the Kalman Gain**:

$$
K_k=P_{k|k-1}H^T_k(H_kP_{k|k-1}H^T_k+R_k)^{-1}
$$

- **Update the error state**:

$$
\delta x^{'}_k=K_k\delta z_k=K_k(z_k-h(x^{'}_{k|k-1},0))
$$

- **Update the nominal state**: We correct the nominal state using the estimated error state: 
For additive states, this is simply addition. For orientation, this involves updating the rotation based on the small error rotation.

$$
x^{'}_{k|k}=x^{'}_{k|k-1}\oplus \delta x^{'}_k
$$

- **Update the error state covariance**:

$$
P_{k|k}=(I-K_kH_k)P_{k|k-1}
$$

- **Reset the error state**: After updating the nominal state, we reset the error state to zero, as its effect has been incorporated into the nominal state. This is crucial for the ESKF approach: $\(\delta x^{'}_k=0\)$.

```python
class ESKF:
    """
    Implements an Error State Kalman Filter for systems with nonlinear dynamics,
    particularly useful for orientation estimation.  This implementation
    assumes that the state vector is composed of [position, velocity, orientation].
    Orientation is represented as a 3-element rotation vector (angle-axis).
    """

    def __init__(self, dt, process_noise_vars, measurement_noise_vars, initial_state, initial_covariance):
        """
        Initializes the Error State Kalman Filter.

        Args:
            dt (float): Time step.
            process_noise_vars (dict): Dictionary containing variances for 'position', 'velocity', and 'orientation'.
            measurement_noise_vars (dict): Dictionary containing variances for 'position' and 'orientation' measurements.
            initial_state (numpy.ndarray): Initial nominal state [position (3x1), velocity (3x1), orientation (3x1)].
            initial_covariance (numpy.ndarray): Initial error state covariance matrix (9x9).
        """
        self.dt = dt

        # Nominal state: [position, velocity, orientation (rotation vector)]
        self.nominal_x = initial_state  # [pos, vel, ori]  (3x1, 3x1, 3x1)
        self.P = initial_covariance  # Error state covariance (9x9)

        # Process noise covariance (for error state)
        self.Q = np.diag([
            process_noise_vars['position'] * dt**3 / 3,  # position x
            process_noise_vars['position'] * dt**3 / 3,  # position y
            process_noise_vars['position'] * dt**3 / 3,  # position z
            process_noise_vars['velocity'] * dt,      # velocity x
            process_noise_vars['velocity'] * dt,      # velocity y
            process_noise_vars['velocity'] * dt,      # velocity z
            process_noise_vars['orientation'] * dt,    # orientation x
            process_noise_vars['orientation'] * dt,    # orientation y
            process_noise_vars['orientation'] * dt     # orientation z
        ])

        # Measurement noise covariance
        self.R_position = np.diag([measurement_noise_vars['position']] * 3)  # position (3x3)
        self.R_orientation = np.diag([measurement_noise_vars['orientation']] * 3) # orientation (3x3)

        # Identity matrix
        self.I = np.eye(9)

    def skew_symmetric(self, v):
        """
        Computes the skew-symmetric matrix of a 3D vector.

        Args:
            v (numpy.ndarray): 3D vector (3x1).

        Returns:
            numpy.ndarray: Skew-symmetric matrix (3x3).
        """
        return np.array([
            [0, -v[2, 0], v[1, 0]],
            [v[2, 0], 0, -v[0, 0]],
            [-v[1, 0], v[0, 0], 0]
        ])

    def exp_so3(self, omega):
        """
        Computes the exponential map of so(3) (rotation vector to rotation matrix).

        Args:
            omega (numpy.ndarray): Rotation vector (3x1).

        Returns:
            numpy.ndarray: Rotation matrix (3x3).
        """
        theta = np.linalg.norm(omega)
        if theta < 1e-6:  # Handle small rotations to avoid division by zero
            R = np.eye(3) + self.skew_symmetric(omega)
        else:
            omega_hat = self.skew_symmetric(omega)
            R = np.eye(3) + (np.sin(theta) / theta) * omega_hat + \
                ((1 - np.cos(theta)) / theta**2) * omega_hat @ omega_hat
        return R

    def log_so3(self, R):
        """
        Computes the matrix logarithm of a rotation matrix (rotation matrix to rotation vector).

        Args:
            R (numpy.ndarray): Rotation matrix (3x3).

        Returns:
            numpy.ndarray: Rotation vector (3x1).
        """
        theta = np.arccos(np.clip((np.trace(R) - 1) / 2, -1.0, 1.0))  # Clip to handle potential numerical issues
        if theta < 1e-6:
            return np.zeros((3, 1))
        else:
            return (theta / (2 * np.sin(theta))) * np.array([
                [R[2, 1] - R[1, 2]],
                [R[0, 2] - R[2, 0]],
                [R[1, 0] - R[0, 1]]
            ])

    def propagate_nominal_state(self, u):
        """
        Propagates the nominal state using the nonlinear system dynamics.
        This is a simplified example.  A real vehicle model would be more complex.

        Args:
            u (numpy.ndarray): Control input [acceleration (3x1), angular_velocity (3x1)].
        """
        acc = u[:3]  # Linear acceleration
        omega = u[3:] # Angular velocity

        # Position update
        self.nominal_x[0:3] = self.nominal_x[0:3] + self.nominal_x[3:6] * self.dt + 0.5 * acc * self.dt**2
        # Velocity update
        self.nominal_x[3:6] = self.nominal_x[3:6] + acc * self.dt
        # Orientation update (using exponential map)
        R_prev = self.exp_so3(self.nominal_x[6:9])
        R_new = R_prev @ self.exp_so3(omega * self.dt)
        self.nominal_x[6:9] = self.log_so3(R_new)

    def compute_F(self):
        """
        Computes the Jacobian of the state transition function (F matrix)
        linearized around the nominal state.

        Returns:
            numpy.ndarray: Jacobian F (9x9).
        """
        F = np.eye(9)
        # Position transition depends on velocity
        F[0:3, 3:6] = self.dt * np.eye(3)
        # Orientation transition depends on angular velocity.  This is a simplified
        # linearization.  A more accurate version would involve the skew-symmetric
        # matrix of the angular velocity.
        F[0:3, 6:9] = -self.dt * self.skew_symmetric(self.nominal_x[3:6])
        F[6:9,6:9] = self.exp_so3(-self.skew_symmetric(self.nominal_x[6:9]) @ np.array(np.eye(3)) * self.dt)
        return F

    def predict(self, u):
        """
        Predicts the nominal state and error state covariance.

        Args:
            u (numpy.ndarray): Control input [acceleration (3x1), angular_velocity (3x1)].
        """
        # 1. Propagate nominal state
        self.propagate_nominal_state(u)

        # 2. Linearize system dynamics to get F
        F = self.compute_F()

        # 3. Predict error state covariance
        self.P = F @ self.P @ F.T + self.Q

    def update(self, z_position, z_orientation):
        """
        Updates the nominal state and error state covariance based on measurements.

        Args:
            z_position (numpy.ndarray): Position measurement (3x1).
            z_orientation (numpy.ndarray): Orientation measurement (3x1) (rotation vector).
        """
        # Expected measurements based on nominal state
        h_position = self.nominal_x[0:3]
        h_orientation = self.nominal_x[6:9]

        # Measurement Jacobians
        H_position = np.zeros((3, 9))
        H_position[:, 0:3] = np.eye(3)  # Position measurement Jacobian

        H_orientation = np.zeros((3,9))
        H_orientation[:,6:9] = np.eye(3)

        # Innovations (measurement residuals)
        y_position = z_position - h_position
        y_orientation = z_orientation - h_orientation

        # Kalman Gain for position
        S_position = H_position @ self.P @ H_position.T + self.R_position
        K_position = self.P @ H_position.T @ np.linalg.inv(S_position)

        # Kalman Gain for orientation
        S_orientation = H_orientation @ self.P @ H_orientation.T + self.R_orientation
        K_orientation = self.P @ H_orientation.T @ np.linalg.inv(S_orientation)
        # Update error state
        delta_x_position = K_position @ y_position
        delta_x_orientation = K_orientation @ y_orientation

        delta_x = delta_x_position + delta_x_orientation

        # Update nominal state (composition of error state and nominal state)
        self.nominal_x[0:3] = self.nominal_x[0:3] + delta_x[0:3] # position
        self.nominal_x[3:6] = self.nominal_x[3:6] + delta_x[3:6] # velocity

        # Orientation update (using exponential map)
        R_nom = self.exp_so3(self.nominal_x[6:9])
        R_delta = self.exp_so3(delta_x[6:9])
        R_new = R_nom @ R_delta
        self.nominal_x[6:9] = self.log_so3(R_new)

        # Update error state covariance
        H = H_position + H_orientation
        self.P = (self.I - K_position @ H_position - K_orientation @ H_orientation) @ self.P

        # Reset error state
        # (In practice, we don't explicitly store a separate error state,
        #  but conceptually, it's set to zero here)

    def get_nominal_state(self):
        """Returns the current nominal state estimate."""
        return self.nominal_x

    def get_covariance(self):
        """Returns the current error state covariance matrix."""
        return self.P

```