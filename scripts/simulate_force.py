import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches

class EarthmovingForceSim:
    def __init__(self):
        # --- Soil Properties ---
        self.gamma = 1513.4     # Density (kg/m^3)
        self.c = 138.8          # Cohesion (Pa)
        self.c_a = 184.0        # Adhesion (Pa)
        self.phi = 0.78         # Internal Friction Angle (rad) ~45 deg
        self.delta = 0.63       # External Friction Angle (rad) ~36 deg

        # Bekker Parameters
        self.k_coh = 1274.0     # kc
        self.k_fri = 177.4      # k_phi
        self.n = 1.21           # Exponent n

        # --- Machine Specs ---
        self.b = 0.05           # Bucket cutting edge thickness (m)
        self.w = 1.89815        # Bucket width (m)
        self.h = 1.89815        # Bucket height (m)
        self.L = 1.8            # Bucket floor length (m) - Estimated
        # FIXED: Use self.gamma instead of hardcoded 1513.4
        self.max_payload = self.gamma * 1/2 * self.w * self.h * self.L
        self.g = 9.81

        # --- Pile Geometry ---
        self.pile_x_start = 2.0
        self.pile_height = 8.0
        self.alpha_pile = np.deg2rad(30) 

        # --- State ---
        self.current_payload_mass = 0.0

    def get_tip_location(self, x_pivot, y_pivot, theta):
        """
        Calculates Tip position given Pivot position and Bucket Angle.
        Assuming Pivot is at the 'Back-Bottom' of the bucket.
        """
        x_tip = x_pivot + self.L * np.cos(theta)
        y_tip = y_pivot + self.L * np.sin(theta)
        return x_tip, y_tip

    def get_pile_depth(self, x_tip, y_tip):
        """ Calculates perpendicular penetration depth 'd' """
        if y_tip > self.pile_height:
            return 0.0
        
        # X coordinate of the pile face at height y
        x_surface = self.pile_x_start + (y_tip / np.tan(self.alpha_pile))
        
        dx = x_tip - x_surface
        d = dx * np.sin(self.alpha_pile)
        
        if d < 0: return 0.0
        return d

    def calculate_n_factors(self, alpha, rho, beta):
        phi = self.phi
        delta = self.delta
        
        term_cot = 1.0 / np.tan(beta + phi)
        denom_part = np.cos(rho + delta) + np.sin(rho + delta) * term_cot
        
        term_A = (1.0 / np.tan(beta)) - np.tan(alpha)
        term_B = np.cos(alpha) + np.sin(alpha) * term_cot
        N_gamma = (term_A * term_B) / (2 * denom_part)
        
        num_c = 1.0 + (1.0 / np.tan(beta)) * term_cot
        N_c = num_c / denom_part
        
        num_a = 1.0 - (1.0 / np.tan(rho)) * term_cot
        N_a = num_a / denom_part
        
        N_q = term_B / denom_part
        
        return N_gamma, N_c, N_a, N_q

    def find_critical_beta(self, alpha, rho):
        start_deg = 45 - np.rad2deg(self.phi)/2
        best_beta = np.deg2rad(start_deg)
        min_N = 1e9
        
        # Search range (Passive failure)
        for b_deg in range(10, 50, 2):
            b_rad = np.deg2rad(b_deg)
            try:
                Ng, _, _, _ = self.calculate_n_factors(alpha, rho, b_rad)
                if 0 < Ng < min_N:
                    min_N = Ng
                    best_beta = b_rad
            except:
                continue
        return best_beta

    def calculate_fee_part(self, d, rho, beta):
        """ Returns: Components of resistance. """
        N_gamma, N_c, N_a, N_q = self.calculate_n_factors(self.alpha_pile, rho, beta)
        
        # 1. Weight Term
        f_w_fee = (d**2) * self.w * self.gamma * self.g * N_gamma
        
        # 2. Cohesion Term
        f_c = self.c * self.w * d * N_c
        
        # 3. Adhesion Term
        f_a_wedge = self.c_a * self.w * d * N_a
        
        # 4. Penetration Term
        P = (self.k_coh / self.b + self.k_fri) * (d ** self.n)
        f_pen = self.w * self.b * P
        
        # 5. Adhesion Term (Blade Surface)
        L_t = d / np.sin(rho)
        f_a_blade = self.c_a * self.w * L_t
        
        return f_w_fee, f_c, f_a_wedge, f_a_blade, f_pen, N_q, P

    def calculate_weight_part(self, d, rho, beta):
        """ Calculates the Pure Weight of the wedge. """
        N_pure_w = np.sin(rho+beta) / (2*np.sin(beta)*np.sin(self.alpha_pile)*np.cos(self.alpha_pile-rho))
        weight_pure = (d**2) * self.w * self.gamma * self.g * N_pure_w
        return weight_pure

    def calculate_forces(self, pivot_trajectory):
        res_fee_weight = []
        res_pure_weight = []
        res_surcharge = []
        res_penetration = []
        res_cohension = []
        res_total = []
        
        tip_x_list = []
        tip_y_list = []
        
        self.current_payload_mass = 0.0
        
        for i in range(len(pivot_trajectory)):
            x_p, y_p, theta = pivot_trajectory[i]
            
            x_t, y_t = self.get_tip_location(x_p, y_p, theta)
            tip_x_list.append(x_t)
            tip_y_list.append(y_t)
            
            d = self.get_pile_depth(x_t, y_t)
            
            # --- Check Phase ---
            if d <= 0.01:
                # Carrying Phase (No Cut)
                holding_force = self.current_payload_mass * self.g
                
                res_fee_weight.append(0)
                res_pure_weight.append(0)
                res_surcharge.append(holding_force)
                res_penetration.append(0)
                # FIXED: Added append for cohesion list to keep array lengths consistent
                res_cohension.append(0) 
                res_total.append(holding_force)
                continue
            
            # --- Cutting Phase ---
            rho = self.alpha_pile - theta
            if rho < np.deg2rad(10): rho = np.deg2rad(10)
            
            beta = self.find_critical_beta(self.alpha_pile, rho)
            
            # Part A: FEE
            f_w_fee, f_c, f_a_wedge, f_a_blade, f_pen, N_q, P = self.calculate_fee_part(d, rho, beta)
            
            # Part B: Weight
            pure_weight = self.calculate_weight_part(d, rho, beta)
            
            # Accumulate Payload
            self.current_payload_mass += (pure_weight/self.g) * 0.1
            if self.current_payload_mass > self.max_payload * 1.5:
                self.current_payload_mass = self.max_payload * 1.5
            # W_load = self.current_payload_mass * self.g
            W_load = pure_weight

            # Surcharge
            f_q = W_load * N_q
            
            # Combine
            F_i = f_w_fee + f_c + f_a_wedge + f_q
            
            # Resolve
            F_T = f_pen + (F_i * np.sin(self.delta)) + f_a_blade
            F_N = F_i * np.cos(self.delta)
            
            f_tot = np.sqrt(F_T**2 + F_N**2)
            
            # Store
            res_fee_weight.append(f_w_fee)
            res_pure_weight.append(pure_weight)
            res_surcharge.append(f_q)
            res_penetration.append(f_pen)
            res_cohension.append(f_c + f_a_blade + f_a_wedge)
            res_total.append(f_tot)
            
        return (np.array(res_fee_weight), np.array(res_pure_weight), 
                np.array(res_surcharge), np.array(res_cohension), np.array(res_penetration), 
                np.array(res_total), np.array(tip_x_list), np.array(tip_y_list))

# --- Define Trajectory ---
# User's requested trajectory
x_traj = np.concatenate([np.linspace(1.5, 5.0, 30), np.linspace(5.0, 8.5, 40), np.linspace(8.5, 8.5, 30)])
y_traj = np.concatenate([np.ones(30)*0.1, np.linspace(0.1, (3.5*np.tan(np.deg2rad(30))) , 40), np.linspace((3.5*np.tan(np.deg2rad(30))),  5.0, 30)])
theta_traj = np.concatenate([np.ones(30)*0.0, np.linspace(0.0, 0.523598776, 40), np.linspace(0.523598776, 0.98, 30)])

trajectory = list(zip(x_traj, y_traj, theta_traj))

# --- Run ---
sim = EarthmovingForceSim()
f_w_fee, f_w_pure, f_q, f_c, f_p, f_tot, tip_x, tip_y = sim.calculate_forces(trajectory)

# --- Visualization ---
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 8))

# Plot 1: Geometry
pile_x = [sim.pile_x_start, sim.pile_x_start + (sim.pile_height / np.tan(sim.alpha_pile))]
pile_y = [0.0, sim.pile_height]
ax1.fill_between(pile_x, pile_y, color='orange', alpha=0.3, label='Pile')
ax1.plot(x_traj, y_traj, 'k--', linewidth=2, label='Joint Path')
ax1.plot(tip_x, tip_y, 'b.-', markersize=2, label='Tip Path')

def draw_buckets(ax, pivot_x, pivot_y, theta, step=10):
    base_shape = np.array([[0, sim.h], [0, 0], [sim.L, 0]])
    for i in range(0, len(pivot_x), step):
        px, py, th = pivot_x[i], pivot_y[i], theta[i]
        c, s = np.cos(th), np.sin(th)
        R = np.array([[c, -s], [s, c]])
        rot_shape = np.dot(base_shape, R.T) + [px, py]
        ax.plot(rot_shape[:,0], rot_shape[:,1], 'r-', alpha=0.5)
        ax.add_patch(patches.Polygon(rot_shape, closed=False, facecolor='gray', alpha=0.2))

draw_buckets(ax1, x_traj, y_traj, theta_traj)
ax1.set_title('Bucket Trajectory')
ax1.axis('equal')
ax1.legend()
ax1.grid(True)

# Plot 2: Forces
time = np.arange(len(f_tot))
ax2.plot(time, f_w_fee, 'g--', label='Weight (FEE)')
ax2.plot(time, f_w_pure, 'm--', label='Pure Wedge Mass')
ax2.plot(time, f_q, 'r--', label='Surcharge')
ax2.plot(time, f_c, color='orange', label='Cohesion+Adhesion')
ax2.plot(time, f_p, color='purple', label='Penetration')
ax2.plot(time, f_tot, 'k-', linewidth=3, label='Total Force')

ax2.set_title('Excavation Forces')
ax2.set_xlabel('Step')
ax2.set_ylabel('Force (N)')
ax2.legend()
ax2.grid(True)

plt.tight_layout()
plt.show()